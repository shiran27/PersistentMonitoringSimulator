classdef CollabEDRHCPolicy < Policy
    % Collaborative event-driven receding horizon control (Collab-ED-RHC) for persistent
    % monitoring on networks: W. Getts and S. Welikala, "Collaborative persistent monitoring"
    % (in preparation). It generalizes ED-RHC (EDRHCPolicy; S. Welikala and C. G. Cassandras,
    % Automatica 127 (2021) 109519) in four ways:
    %   1. Agents may share targets. ED-RHC's no-sharing rule is dropped, and every target cost
    %      follows the Lindley recursion of Lemma G (CollabTargetCost.lindley), which holds for
    %      any number of agents at a target, any order of arrivals and departures, and A >= B.
    %   2. Every agent announces its plan: its planned departure time and tentative next target.
    %      These plans form an occupancy schedule per target (arrivals and departures), so the
    %      future visits of other agents are predicted from their plans instead of from the
    %      projections dR/dt = A of ED-RHC. Agents in transit stay at their next target until
    %      the end of the planning horizon (Lemma D2 of the paper).
    %   3. Coordination ("decide and announce", no replies):
    %        coordination = "A"   the agents at the leader's target plan jointly (their departure
    %                             times, next targets and the common horizon end tf) as a best
    %                             response to all other announced plans; 1-hop cost set
    %        coordination = "B"   as "A", and in the same solve the group may shift the departure
    %                             times of the agents residing at neighboring targets, provided no
    %                             such agent's 1-hop score gets worse than without any shift
    %                             (individual rationality, IR); the neighbors adopt the new times
    %                             without replying; 2-hop cost set (default)
    %   4. Terminal correction (terminal = "hhat", default): the RHCP objective is the corrected
    %      ratio [J + hhat(X(tf)) - hhat(X(t))] / (tf - t), where hhat sums, over the targets of
    %      the cost set, the cost of a revisit model (wait delta_k, then cleared by the minimum
    %      clearing group) counted up to the mission end (CollabTargetCost.terminal). delta_k is
    %      the time to the earliest announced arrival (deltaRule "plans", default) or, without
    %      one, the target's mean unattended time so far; deltaRule "history" always uses the
    %      latter. terminal = "none" gives ED-RHC's ratio J / (tf - t).
    %
    % Events (paper Sec. VI): E1 arrival, E2 deactivation (R_i reaches 0 with agents present),
    % E3 departure (a planned departure time is reached), E4 commitment at a neighbor (an agent
    % departs toward or from a neighbor of the target), E5 start. The agents at the target solve
    % RHCP-R at E1, E2, E4 and E5, and RHCP-D at E3 (the leader departs now). Shifts never
    % trigger events. With one agent and terminal = "none", RHCP-R and RHCP-D are RHCP1-2 and
    % RHCP3 of ED-RHC with alpha = 0.5, case by case.
    %
    % Each RHCP is solved numerically: the discrete choices (next targets of the group) are
    % enumerated; for each, the continuous decisions (departure times and tf) are searched on a
    % grid (one agent) or by sampling (a group) and refined with fmincon. With coordination "B"
    % the shifts are then optimized from the no-shift solution Pi0 under the IR constraints
    % (fmincon); a solution is adopted only if it lowers the objective, so
    % J(Pi*) <= J(Pi0) at every event.
    % Parameters are properties, set in runTrial with 'Policy_<name>'. missionEnd (the mission
    % end T) truncates the horizon and hhat; runTrial passes 'Policy_missionEnd'.
    properties
        coordination = "B"   % "A" (joint planning of co-located agents) or "B" (plus IR shifts)
        terminal = "hhat"    % "hhat" (corrected ratio) or "none" (ED-RHC's ratio)
        deltaRule = "plans"  % "plans" (announced arrival, else history) or "history"
        sharing = true       % false: no target sharing (covered next targets excluded)
        H = 250              % upper bound on the planning horizon tf - t [s]
        missionEnd = Inf     % mission end T [s] (truncates tf and hhat)
        gridSize = 15        % grid points per variable (one agent) before fmincon
        groupSamples = 150   % random samples per assignment for a group of 2 or more agents
        sampleSeed = 1       % seed of the samples' own stream (mt19937ar)
        retryTime = 1        % wait before a leader with no feasible plan tries again [s]
        traceDecisions = false
        decisions = zeros(0, 13)  % [t, leader, event, target i, group size, next j, d_a, tf,
                                  %  shifted agents, J(Pi0), J(Pi*), members, cpu seconds]
        stats = struct('solves', 0, 'events', zeros(1, 5), 'cpu', 0, 'shiftEvents', 0, 'shifts', 0)
    end

    properties (Access = private)
        sched        % per agent: status (1 resident, 2 transit), target, arrive, depart, next
        positive     % per target: R_i > 0 at the last check (for E2)
        pending      % per target: an E4 event waits for its residents
        gapSum       % per target: total unattended time so far
        gapCount     % per target: number of unattended periods so far
        lastLeft     % per target: time the last agent left (NaN: never)
        gapInit      % per target: initial unattended-time estimate (largest transit into it)
        lengths      % containers.Map: edge (i, j) -> path length
        options      % fmincon options
        optionsIR    % fmincon options with nonlinear constraints
    end

    properties (Constant)
        agentAssumption = "plans transit"
    end

    properties (Constant, Access = private)
        tol = 1e-6
    end

    methods
        function obj = CollabEDRHCPolicy(opts)
            arguments
                opts.Coordination string {mustBeMember(opts.Coordination, ["A", "B"])} = "B"
                opts.Terminal string {mustBeMember(opts.Terminal, ["hhat", "none"])} = "hhat"
            end
            obj.coordination = opts.Coordination;
            obj.terminal = opts.Terminal;
            obj.options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
                'MaxFunctionEvaluations', 400, 'StepTolerance', 1e-9);
            obj.optionsIR = obj.options;
            obj.resetReservations();
        end

        function resetReservations(obj)
            % Called by ScenarioModel on every reset: forget plans, events and statistics
            obj.sched = [];
            obj.positive = []; obj.pending = [];
            obj.gapSum = []; obj.gapCount = []; obj.lastLeft = []; obj.gapInit = [];
            obj.lengths = containers.Map('KeyType', 'double', 'ValueType', 'double');
            obj.decisions = zeros(0, 13);
            obj.stats = struct('solves', 0, 'events', zeros(1, 5), 'cpu', 0, 'shiftEvents', 0, 'shifts', 0);
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            % Called when the agent has no path and no dwell left: at the start (E5), on arrival
            % (E1), and when its planned departure time is reached (E3)
            obj.ensureInit(model, simTime);
            a = agent.index;
            s = obj.sched(a);
            if s.status == 2
                obj.markArrival(a, curIdx, simTime, model);
                kind = 1;
            elseif isnan(s.depart)
                kind = 5;
            else
                kind = 3;
            end
            cmd = obj.groupSolve(kind, agent, model, adj, curIdx, simTime);
        end

        function onStep(obj, agent, model, simTime)
            % E2 (deactivation) and E4 (commitment at a neighbor) for an agent dwelling at a target
            if isempty(obj.sched), return; end
            a = agent.index;
            if a > numel(obj.sched) || obj.sched(a).status ~= 1 || agent.dwellRemaining <= 0
                return;
            end
            i = obj.sched(a).target;
            kind = 0;
            if model.targets(i).R <= obj.tol
                if obj.positive(i), kind = 2; end
                obj.positive(i) = false;
            else
                obj.positive(i) = true;
            end
            if kind == 0 && obj.pending(i), kind = 4; end
            if kind == 0, return; end
            obj.pending(i) = false;
            obj.groupSolve(kind, agent, model, model.buildAdjacency(), i, simTime);
        end
    end

    methods (Access = private)
        function ensureInit(obj, model, simTime)
            % At the first call of a run every agent is a resident of its start target, with no plan
            nA = numel(model.agents);
            if ~isempty(obj.sched) && numel(obj.sched) >= max([model.agents.index]), return; end
            nT = numel(model.targets);
            obj.sched = repmat(struct('status', 1, 'target', 0, 'arrive', simTime, ...
                'depart', NaN, 'next', 0), 1, max([model.agents.index]));
            for k = 1:nA
                ag = model.agents(k);
                obj.sched(ag.index).target = ag.current_target_idx;
            end
            obj.positive = [model.targets.R] > obj.tol;
            obj.pending = false(1, nT);
            obj.gapSum = zeros(1, nT); obj.gapCount = zeros(1, nT);
            obj.lastLeft = nan(1, nT);
            startTargets = [obj.sched.target];
            obj.lastLeft(~ismember(1:nT, startTargets)) = simTime;   % unattended from the start
            adj = model.buildAdjacency();
            speed = mean(arrayfun(@(ag) ag.state.maxSpeed, model.agents));
            obj.gapInit = zeros(1, nT);
            for k = 1:nT
                nb = find(adj(:, k) | adj(k, :)');
                y = arrayfun(@(j) obj.edgeLength(model, j, k), nb);
                if isempty(y), y = 0; end
                obj.gapInit(k) = max(y) / speed;
            end
        end

        function markArrival(obj, a, i, t, model)
            % Agent a arrives at target i: it is a resident from now on (updates the target's
            % unattended-time history and the E2 flag)
            if obj.residentCount(i) == 0 && ~isnan(obj.lastLeft(i))
                obj.gapSum(i) = obj.gapSum(i) + (t - obj.lastLeft(i));
                obj.gapCount(i) = obj.gapCount(i) + 1;
            end
            obj.sched(a) = struct('status', 1, 'target', i, 'arrive', t, 'depart', t, 'next', 0);
            obj.positive(i) = model.targets(i).R > obj.tol;
            obj.pending(i) = false;   % the arrival solve re-plans every resident of i
        end

        function n = residentCount(obj, i)
            n = sum([obj.sched.status] == 1 & [obj.sched.target] == i);
        end

        function g = gapEstimate(obj, k)
            % Mean unattended time of target k so far (rule (i)); the initial estimate before any
            if obj.gapCount(k) > 0
                g = obj.gapSum(k) / obj.gapCount(k);
            else
                g = obj.gapInit(k);
            end
        end

        function y = edgeLength(obj, model, i, j)
            key = min(i, j) * 1e5 + max(i, j);
            if isKey(obj.lengths, key)
                y = obj.lengths(key);
            else
                e = model.findEdge(i, j);
                if isempty(e)
                    y = Inf;
                else
                    y = sum(vecnorm(diff(e.curvePoints, 1, 1), 2, 2));
                end
                obj.lengths(key) = y;
            end
        end

        function r = rho(obj, model, agent, i, j)
            % Transit time of the agent on edge (i, j): length / maxSpeed, or the agent's own pace
            y = obj.edgeLength(model, i, j);
            r = y / agent.state.maxSpeed;
            if isfinite(y) && r > 0
                e = model.findEdge(i, j);
                r = max(r, agent.shortestTransit(e.curvePoints));
            end
        end

        function cmd = groupSolve(obj, kind, leader, model, adj, i, t)
            % Solves RHCP-R (kind 1, 2, 4, 5) or RHCP-D (kind 3) for the agents at target i, led
            % by the agent leader, and applies the result. Returns the leader's move command (for
            % plan) or struct()
            cpu = tic;
            cmd = struct();
            obj.stats.events(kind) = obj.stats.events(kind) + 1;
            obj.stats.solves = obj.stats.solves + 1;
            agents = model.agents;
            byIndex = zeros(1, numel(obj.sched));
            byIndex([agents.index]) = 1:numel(agents);
            T = obj.missionEnd;
            Hm = min(obj.H, T - t);

            % --- the group S at i (leader first), the members M at neighboring targets
            st = [obj.sched.status]; tg = [obj.sched.target];
            S = leader.index;
            if obj.sharing
                coRes = find(st == 1 & tg == i);
                S = [S, coRes(coRes ~= leader.index)];
            end
            nbrs = find(adj(i, :));
            choices = nbrs;
            if ~obj.sharing   % ED-RHC's covering: leave out neighbors another agent is at or heading to
                coveredBy = arrayfun(@(j) any(tg == j & (1:numel(tg)) ~= leader.index & st > 0), choices);
                choices = choices(~coveredBy);
            end
            M = [];
            if obj.coordination == "B"
                cand = find(st == 1 & ismember(tg, nbrs));
                M = cand(arrayfun(@(c) isfinite(obj.sched(c).depart) && obj.sched(c).depart > t + obj.tol ...
                    && obj.sched(c).next > 0, cand));
            end
            % cost set: the closed neighborhood of i (A), plus those of the members' targets (B)
            K = [i, nbrs];
            for c = M
                K = [K, obj.sched(c).target, find(adj(obj.sched(c).target, :))]; %#ok<AGROW>
            end
            K = unique(K, 'stable');
            nK = numel(K);
            kpos = zeros(1, numel(model.targets)); kpos(K) = 1:nK;

            if isempty(choices) || Hm <= obj.tol
                % no next target can be planned: stay, and try again after retryTime
                obj.setDwell(leader, t, obj.retryTime, 0);
                obj.stats.cpu = obj.stats.cpu + toc(cpu);
                return;
            end

            % --- target data of the cost set
            D = struct();
            D.t = t; D.T = T;
            D.R = arrayfun(@(k) max(model.targets(k).R, 0), K);
            D.R(D.R < obj.tol) = 0;
            D.A = arrayfun(@(k) model.targets(k).growthRateAt(t, model.uncertaintyMode), K);
            D.B = arrayfun(@(k) model.targets(k).B, K);
            D.nLow = floor(D.A ./ D.B) + 1;
            D.gap = arrayfun(@(k) obj.gapEstimate(k), K);
            D.useH = obj.terminal == "hhat";
            D.plans = obj.deltaRule == "plans";
            % member score sets (positions in K)
            D.memberSets = cell(1, numel(M));
            for m = 1:numel(M)
                D.memberSets{m} = kpos([obj.sched(M(m)).target, find(adj(obj.sched(M(m)).target, :))]);
            end

            % --- fixed occupancy: every agent other than S and M, by its announced plan
            % events: [target position, var (0: constant), time or offset, +1 arrival / -1 departure]
            fixedPresent = zeros(1, nK);
            fixedEv = zeros(0, 4);
            for b = 1:numel(obj.sched)
                if ismember(b, S) || ismember(b, M) || byIndex(b) == 0, continue; end
                sb = obj.sched(b);
                ag = agents(byIndex(b));
                if sb.status == 1
                    p = kpos(sb.target);
                    dep = sb.depart;
                    if isnan(dep), dep = Inf; end
                    if p > 0
                        fixedPresent(p) = fixedPresent(p) + 1;
                        if isfinite(dep), fixedEv(end+1, :) = [p, 0, dep, -1]; end %#ok<AGROW>
                    end
                    if sb.next > 0 && isfinite(dep) && kpos(sb.next) > 0
                        fixedEv(end+1, :) = [kpos(sb.next), 0, dep + obj.rho(model, ag, sb.target, sb.next), 1]; %#ok<AGROW>
                    end
                elseif sb.status == 2 && kpos(sb.target) > 0
                    fixedEv(end+1, :) = [kpos(sb.target), 0, sb.arrive, 1]; %#ok<AGROW>
                end
            end
            % members: present at their target until their (variable in "B") departure, then
            % arrive at their tentative next target
            nS = numel(S); nM = numel(M);
            memberEv = zeros(0, 4);
            memberPresent = zeros(1, nK);
            for m = 1:nM
                c = M(m); sc = obj.sched(c); ag = agents(byIndex(c));
                p = kpos(sc.target);
                memberPresent(p) = memberPresent(p) + 1;
                v = nS + 1 + m;                          % variable index of its departure time
                memberEv(end+1, :) = [p, v, 0, -1]; %#ok<AGROW>
                if kpos(sc.next) > 0
                    memberEv(end+1, :) = [kpos(sc.next), v, obj.rho(model, ag, sc.target, sc.next), 1]; %#ok<AGROW>
                end
            end
            dCur = zeros(1, nM);
            for m = 1:nM, dCur(m) = obj.sched(M(m)).depart; end

            % hhat(X(t)) is a constant of the solve: the occupancy at t and the arrivals announced
            % by the agents outside the group
            D.present0 = fixedPresent + memberPresent;
            D.present0(kpos(i)) = D.present0(kpos(i)) + nS;
            D.h0 = obj.hhatNow(D, [fixedEv; memberEv], [zeros(1, nS + 1), dCur]);

            % --- enumerate the next targets of the group (the leader must visit one)
            g = cell(1, nS);
            [g{:}] = ndgrid(1:numel(choices));
            combos = cell2mat(cellfun(@(x) x(:), g, 'UniformOutput', false));
            results = struct('cost', {}, 'z', {}, 'tmpl', {}, 'phi', {});
            for q = 1:size(combos, 1)
                phi = choices(combos(q, :));
                rhoS = arrayfun(@(k) obj.rho(model, agents(byIndex(S(k))), i, phi(k)), 1:nS);
                if rhoS(1) > Hm - obj.tol, continue; end
                % group events: departure from i (var k) and arrival at phi(k) (var k + rho)
                groupEv = zeros(0, 4);
                for k = 1:nS
                    groupEv(end+1, :) = [kpos(i), k, 0, -1]; %#ok<AGROW>
                    groupEv(end+1, :) = [kpos(phi(k)), k, rhoS(k), 1]; %#ok<AGROW>
                end
                tmpl = obj.template(D, [fixedEv; memberEv; groupEv]);
                tmpl.rhoS = rhoS; tmpl.dCur = dCur; tmpl.nS = nS; tmpl.nM = nM; tmpl.Hm = Hm;
                tmpl.fixLeader = kind == 3;
                [cost, z] = obj.solveGroup(D, tmpl);
                results(end+1) = struct('cost', cost, 'z', z, 'tmpl', tmpl, 'phi', phi); %#ok<AGROW>
            end
            if isempty(results)   % no next target can be reached within the horizon
                obj.setDwell(leader, t, obj.retryTime, 0);
                obj.stats.cpu = obj.stats.cpu + toc(cpu);
                return;
            end
            [~, order] = sort([results.cost]);
            best = results(order(1));
            J0 = best.cost;
            Jstar = J0;
            shifted = 0;
            % --- coordination "B": departure-time shifts of the members under IR
            if nM > 0
                % IR baseline: the members' scores under Pi0 (the best response), for every
                % assignment of next targets searched
                [~, num0] = obj.evaluate(D, best.tmpl, best.z);
                score0 = cellfun(@(s) sum(num0(s)), D.memberSets) / (best.z(nS + 1) - t);
                for r = order(1:min(2, numel(order)))
                    [c2, z2, ok] = obj.solveShifts(D, results(r).tmpl, results(r).z, score0);
                    if ok && c2 < Jstar - 1e-9
                        Jstar = c2; best = results(r); best.z = z2;
                    end
                end
                if Jstar < J0 - 1e-9
                    moved = abs(best.z(nS + 2:end) - dCur) > 1e-3;
                    shifted = sum(moved);
                end
            end

            % --- apply: the group's plans, the leader's move (RHCP-D), the members' new departures
            z = best.z; phi = best.phi;
            tf = z(nS + 1);
            fromPlan = any(kind == [1 3 5]);   % plan() can move the leader now; onStep cannot
            for k = nS:-1:1
                ag = agents(byIndex(S(k)));
                d = z(k);
                if k == 1 && fromPlan && (kind == 3 || d <= t + obj.tol)
                    cmd = obj.depart(ag, model, i, phi(1), best.tmpl.rhoS(1), t);
                else
                    % a dwell of 0 makes the model call plan at the next step (E3, RHCP-D)
                    obj.setDwell(ag, t, max(d - t, 0), phi(k));
                end
            end
            if shifted > 0
                for m = 1:nM
                    ag = agents(byIndex(M(m)));
                    obj.setDwell(ag, t, max(z(nS + 1 + m) - t, 0), obj.sched(M(m)).next);
                end
                obj.stats.shiftEvents = obj.stats.shiftEvents + 1;
                obj.stats.shifts = obj.stats.shifts + shifted;
            end
            secs = toc(cpu);
            obj.stats.cpu = obj.stats.cpu + secs;
            if obj.traceDecisions
                obj.decisions(end+1, :) = [t, leader.index, kind, i, nS, phi(1), z(1), tf, ...
                    shifted, J0, Jstar, nM, secs];
            end
        end

        function cmd = depart(obj, agent, model, i, j, rho, t)
            % The leader leaves i for j now: the move command, the schedule, E4 at both ends
            cmd = struct('kind', "move", 'targetIdx', j);
            if isprop(agent, 'transitTime')
                agent.transitTime = rho;
                agent.transitProfile = ""; agent.transitLimits = [Inf Inf];
            end
            agent.dwellRemaining = 0;
            obj.sched(agent.index) = struct('status', 2, 'target', j, 'arrive', t + rho, ...
                'depart', NaN, 'next', 0);
            if obj.residentCount(i) == 0, obj.lastLeft(i) = t; end
            % E4 for the residents of the neighbors of both ends and of j (i's residents just
            % re-planned with the leader)
            adj = model.buildAdjacency();
            hit = unique([find(adj(i, :)), j, find(adj(j, :))]);
            hit = hit(hit ~= i);
            occupied = arrayfun(@(k) obj.residentCount(k) > 0, hit);
            obj.pending(hit(occupied)) = true;
        end

        function setDwell(obj, agent, t, duration, next)
            agent.dwellRemaining = duration;
            obj.sched(agent.index).depart = t + duration;
            obj.sched(agent.index).next = next;
        end

        function tmpl = template(~, D, ev)
            % Occupancy template of the cost set: per target, its agents present at t and its
            % events [var, time or offset, +-1]; times are z(var) + offset (var 0: constant)
            nK = numel(D.R);
            tmpl.present = D.present0;
            tmpl.ev = cell(1, nK);
            for p = 1:nK
                tmpl.ev{p} = ev(ev(:, 1) == p, 2:4);
            end
        end

        function [J, num] = evaluate(obj, D, tmpl, Z)
            % Objective of candidate plans Z (rows: [d_S, tf, d_M]): the (corrected) ratio J and
            % the numerator per target num (rows x targets)
            C = size(Z, 1);
            tf = Z(:, tmpl.nS + 1);
            nK = numel(D.R);
            num = zeros(C, nK);
            for p = 1:nK
                e = tmpl.ev{p};
                if isempty(e)
                    times = zeros(C, 0); sg = zeros(C, 0);
                else
                    times = repmat(e(:, 2)', C, 1);
                    hasVar = e(:, 1)' > 0;
                    times(:, hasVar) = times(:, hasVar) + Z(:, e(hasVar, 1)');
                    sg = repmat(e(:, 3)', C, 1);
                end
                [Jk, Rf] = CollabTargetCost.lindley(D.R(p), D.A(p), D.t, tf, ...
                    D.B(p) * tmpl.present(p), times, D.B(p) * sg);
                num(:, p) = Jk;
                if D.useH
                    % next visit: the earliest arrival announced after tf (rule "plans"), else
                    % the target's mean unattended time
                    delta = D.gap(p) + zeros(C, 1);
                    if D.plans
                        arr = times; arr(sg < 0 | times <= tf) = Inf;
                        nextArr = min([arr, Inf(C, 1)], [], 2);
                        has = isfinite(nextArr);
                        delta(has) = nextArr(has) - tf(has);
                    end
                    num(:, p) = num(:, p) + obj.hhat(D, p, Rf, delta, D.T - tf) - D.h0(p);
                end
            end
            J = sum(num, 2) ./ max(tf - D.t, obj.tol);
        end

        function h = hhat(~, D, p, R, delta, remaining)
            % hhat^T of target p at uncertainty R with its next visit delta later, up to the
            % remaining mission time. The revisit model does not credit the agents present at the
            % evaluation time (the plan does not say how long they stay): the target waits delta
            % at its growth rate A, then its minimum clearing group clears it at nLow*B - A. The
            % relative value subtracts the target's average cost rate times the time to that
            % clearing; the rate of the revisit model is A * gap / 2.
            A = D.A(p); B = D.B(p); nLow = D.nLow(p);
            lam = A * D.gap(p) / 2;
            h = CollabTargetCost.terminal(R, A, nLow * B - A, delta, max(remaining, 0), lam);
        end

        function h0 = hhatNow(obj, D, ev, zNow)
            % hhat(X(t)) per target: the arrivals announced so far (ev rows:
            % [target position, var, time or offset, +-1]; zNow: the current value of each var)
            nK = numel(D.R);
            h0 = zeros(1, nK);
            if ~D.useH, return; end
            for p = 1:nK
                e = ev(ev(:, 1) == p, 2:4);
                arr = [];
                if ~isempty(e)
                    times = e(:, 2)';
                    hasVar = e(:, 1)' > 0;
                    times(hasVar) = times(hasVar) + zNow(e(hasVar, 1));
                    arr = times(e(:, 3)' > 0 & times > D.t);
                end
                delta = D.gap(p);
                if D.plans && ~isempty(arr), delta = min(arr) - D.t; end
                h0(p) = obj.hhat(D, p, D.R(p), delta, D.T - D.t);
            end
        end

        function [cost, z] = solveGroup(obj, D, tmpl)
            % Step 1 (Pi0 for one assignment of next targets): the group's departure times and tf
            % with the members' plans fixed. Parameters: x1 = d_a - t, x2 = tf - (d_a + rho_a),
            % and for every other group agent f_k in [0, 1] with d_k = t + f_k (tf - t).
            nS = tmpl.nS; Hm = tmpl.Hm; r1 = tmpl.rhoS(1);
            span = Hm - r1;
            g = obj.gridSize;
            if nS == 1
                % dense grid on the triangle x1 + x2 <= span (the cost is piecewise smooth, with
                % kinks where a target's uncertainty reaches 0)
                if tmpl.fixLeader
                    X = [zeros(4 * g, 1), linspace(0, span, 4 * g)'];
                else
                    [a, b] = meshgrid(linspace(0, span, 2 * g));
                    X = [a(:), b(:)];
                    X = X(sum(X, 2) <= span + obj.tol, :);
                end
            else
                % deterministic samples (P1: identical solutions for identical data) from an own
                % stream: the same on parallel workers, whose default generator differs, and the
                % caller's rng state is left as is
                s = RandStream('mt19937ar', 'Seed', obj.sampleSeed);
                n = obj.groupSamples;
                a = rand(s, n, 1) * span; b = rand(s, n, 1) .* (span - a);
                if tmpl.fixLeader, a(:) = 0; end
                F = rand(s, n, nS - 1);
                F(1:min(n, 10), :) = 1;                       % the others stay
                F(11:min(n, 20), :) = 0;                      % the others leave now
                F(21:min(n, 30), :) = repmat(a(21:min(n, 30)) ./ max(a(21:min(n, 30)) + b(21:min(n, 30)) + r1, obj.tol), 1, nS - 1);
                X = [a, b, F];                                % ... or leave with the leader
            end
            J = obj.evaluate(D, tmpl, obj.toPlan(X, tmpl, D.t));
            lb = zeros(1, size(X, 2)); ub = [span, span, ones(1, nS - 1)];
            if tmpl.fixLeader, ub(1) = 0; end
            f = @(x) obj.evaluate(D, tmpl, obj.toPlan(x, tmpl, D.t));
            % refine the best few grid points that are apart from each other
            [~, order] = sort(J);
            step = [span, span, ones(1, nS - 1)] / max(2 * g - 1, 1);
            if nS > 1, step = [span, span, ones(1, nS - 1)] / 5; end
            starts = order(1);
            for k = order(2:end)'
                if numel(starts) >= 3, break; end
                if all(any(abs(X(starts, :) - X(k, :)) > 2 * step, 2)), starts(end+1) = k; end %#ok<AGROW>
            end
            cost = inf; xb = X(order(1), :);
            for k = starts
                [x, fx] = obj.coordRefine(f, X(k, :), lb, ub, span, step);
                if fx < cost, cost = fx; xb = x; end
            end
            % fmincon polish (bounds and x1 + x2 <= span)
            Ain = [1 1 zeros(1, nS - 1)]; bin = span;
            try
                [x, fx] = fmincon(@(x) f(x(:)'), xb', Ain, bin, [], [], lb', ub', [], obj.options);
                if fx < cost && all(x' >= lb - obj.tol) && all(x' <= ub + obj.tol) && Ain * x <= bin + obj.tol
                    xb = min(max(x', lb), ub); cost = fx;
                end
            catch
            end
            z = obj.toPlan(xb, tmpl, D.t);
        end

        function [x, fx] = coordRefine(obj, f, x, lb, ub, span, step)
            % Coordinate-wise line searches (fminbnd) around x, within +-step, two rounds with the
            % step halved; x(1) + x(2) <= span. Robust at the kinks of the piecewise cost.
            fx = f(x);
            opts = optimset('TolX', 1e-7, 'Display', 'off');
            for round = 1:3
                for v = 1:numel(x)
                    hi = ub(v);
                    if v <= 2, hi = min(hi, span - x(3 - v)); end
                    a = max(lb(v), x(v) - step(v)); b = min(hi, x(v) + step(v));
                    if b - a <= obj.tol, continue; end
                    y = x;
                    [s, fs] = fminbnd(@(s) f(setv(y, v, s)), a, b, opts);
                    if fs < fx, x(v) = s; fx = fs; end
                    % the ends of the bracket (fminbnd never evaluates them)
                    for e = [a, b]
                        fe = f(setv(x, v, e));
                        if fe < fx, x(v) = e; fx = fe; end
                    end
                end
                step = step / 2;
            end
            function y = setv(y, v, s), y(v) = s; end
        end

        function P = toPlan(~, X, tmpl, t)
            % Parameters -> plan [d_S, tf, d_M] (members at their current departure times)
            nS = tmpl.nS;
            da = t + X(:, 1);
            tf = da + tmpl.rhoS(1) + X(:, 2);
            P = [da, t + X(:, 3:2 + nS - 1) .* (tf - t), tf, repmat(tmpl.dCur, size(X, 1), 1)];
        end

        function [cost, z, ok] = solveShifts(obj, D, tmpl, z0, score0)
            % Step 2 (Pi*): the members' departure times under the IR constraints
            % score_m(Pi) <= score_m(Pi0) (score0, from the best response Pi0), starting from the
            % best response z0 of this assignment of next targets; the group's decisions are
            % refined at the same time. ok = false: no IR-feasible plan found
            nS = tmpl.nS; nM = tmpl.nM; t = D.t;
            J0 = obj.evaluate(D, tmpl, z0);
            w0 = z0(nS + 1) - t;
            % candidate starts: one member at a time leaves now, leaves when the leader arrives,
            % or stays until tf
            arriveLeader = z0(1) + tmpl.rhoS(1);
            Z = z0;
            for m = 1:nM
                for d = [t, arriveLeader, z0(nS + 1), t + 0.5 * w0]
                    zz = z0; zz(nS + 1 + m) = d; Z(end+1, :) = zz; %#ok<AGROW>
                end
            end
            [J, num] = obj.evaluate(D, tmpl, Z);
            w = Z(:, nS + 1) - t;
            feas = true(size(J));
            for m = 1:nM
                feas = feas & sum(num(:, D.memberSets{m}), 2) ./ w <= score0(m) + 1e-9;
            end
            J(~feas) = Inf;
            [Jb, k] = min(J);
            zs = Z(k, :);
            % fmincon over [d_S, tf, d_M] with IR as nonlinear constraints
            lb = [t * ones(1, nS), t + tmpl.rhoS(1), t * ones(1, nM)];
            ub = [(t + tmpl.Hm) * ones(1, nS), t + tmpl.Hm, max(t + tmpl.Hm, tmpl.dCur)];
            if tmpl.fixLeader, ub(1) = t; end
            % d_a + rho_a <= tf;  d_k <= tf for the other group agents
            Ain = zeros(nS, nS + 1 + nM);
            Ain(1, 1) = 1; Ain(1, nS + 1) = -1;
            for k = 2:nS, Ain(k, k) = 1; Ain(k, nS + 1) = -1; end
            bin = [-tmpl.rhoS(1); zeros(nS - 1, 1)];
            f = @(z) obj.evaluate(D, tmpl, z(:)');
            nonl = @(z) obj.irConstraint(D, tmpl, z(:)', score0);
            ok = isfinite(Jb);   % some candidate start satisfies IR
            try
                [x, fx, flag] = fmincon(f, zs', Ain, bin, [], [], lb', ub', nonl, obj.optionsIR);
                c = nonl(x);
                if flag > 0 && fx < Jb && all(c <= 1e-9) && all(Ain * x <= bin + 1e-9)
                    zs = x'; Jb = fx; ok = true;
                end
            catch
            end
            cost = Jb; z = zs;
            if ~ok, cost = J0; z = z0; end
        end

        function [c, ceq] = irConstraint(obj, D, tmpl, z, score0)
            [~, num] = obj.evaluate(D, tmpl, z);
            w = max(z(tmpl.nS + 1) - D.t, obj.tol);
            c = cellfun(@(s) sum(num(s)) / w, D.memberSets) - score0;
            ceq = [];
        end
    end
end
