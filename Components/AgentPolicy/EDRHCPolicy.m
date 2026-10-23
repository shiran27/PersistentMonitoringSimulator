classdef EDRHCPolicy < Policy
    % Event-driven receding horizon control (ED-RHC) for persistent monitoring on networks,
    % with the weighted RHCP3 objective of RHC-alpha: S. Welikala and C. G. Cassandras,
    % "Event-driven receding horizon control for distributed persistent monitoring in network
    % systems", Automatica 127 (2021) 109519, Secs. II-IV. With an energy-aware transit model
    % (below) it is the energy-aware variant of S. Welikala and C. G. Cassandras, "Event-driven
    % receding horizon control of energy-aware dynamic agents for distributed persistent
    % monitoring", arXiv 2102.12963 (IEEE CDC 2021), Secs. III-IV and Apps. D-E (registered as
    % ED-ORHC-SO and ED-ORHC-FO-3).
    %
    % Each agent decides for itself, at its events, using only local information: its target
    % i and the 1-hop neighbors of i (their R, A, B, the lengths of the edges (i, j), and
    % whether each neighbor is covered). An agent at target i solves a receding horizon control
    % problem (RHCP) at these events:
    %   RHCP1 on arrival:                 active time u_i (R_i decreasing)
    %   RHCP2 when R_i reaches 0:         inactive time v_i (R_i = 0)
    %   RHCP3 when u_i or v_i is over:    next target j (and transit time rho_ij), then it leaves
    % and re-solves RHCP1 (active) or RHCP2 (inactive) when a neighbor becomes covered or
    % uncovered. A target is covered while another agent is at it or on its way to it; covered
    % neighbors are left out of the RHCPs, so agents never share a target. Each RHCP plans
    % [u_i, v_i] at i, the transit rho_ij, and [u_j, v_j] at a neighbor j, over the variable
    % horizon w = u_i + v_i + rho_ij + u_j + v_j <= H, and minimizes the neighborhood cost
    %   J_sH = (1/w) * sum over the neighborhood of int_t^{t+w} R_m(s) ds.
    % In RHCP3 the cost of j is weighted by alpha and the rest by 1 - alpha (RHC-alpha, Sec.
    % IV-A). The default is the paper's nominal alpha = 1/|N_i|^2 (N_i includes i), as in the JS
    % simulator; alpha = 0.5 gives the unweighted RHC method of Sec. III.
    %
    % Transit models (property transit):
    %   "FO-0"            rho_ij = y_ij / maxSpeed, fixed (first-order agents moving at constant
    %                     speed), or the agent's shortestTransit if that is longer (a
    %                     PhysicalAgent's own pace)
    %   "SO"              rho_ij is a decision: the RHCP minimizes energyWeight * E(rho) + J_sH
    %                     with the energy E(rho) of the minimum-energy speed profile, 12 y^2/rho^3
    %                     without limits (Theorem 3)
    %   "FO-3"            as "SO" with trapezoid profiles, 27 y^2/(2 rho^3) without limits
    %                     (Theorem 5)
    % (The older names "constant speed" and "FO" still work; see SpeedProfile.canonical.)
    % speedLimit and accelLimit (Inf by default) bound the energy-aware profiles. E(rho) is then
    % that of the bounded minimum-energy profile (Apps. D-E, generalized to both limits; see
    % SpeedProfile), and rho is no shorter than the limits allow, nor than the agent's
    % shortestTransit (the shortest transit it can keep; 0 for an AbstractAgent).
    % The energy E(rho) is the agent's own (Agent.transitEnergyModel): the closed form above
    % for an AbstractAgent, which follows the profile exactly; for a PhysicalAgent, that of the
    % reference it will follow (the planned profile where it fits, else its fastest one retimed),
    % including the turns' acceleration v^2 kappa on curved paths.
    % When an agent leaves, the policy hands it the planned transit time rho_ij; the energy-
    % aware models also hand it their profile and limits. A AbstractAgent then follows the
    % plan exactly (with profile "FO-0" it is the first-order agent of the Automatica paper;
    % with profile "SO" under transit "FO-0" it is the JS simulator's SO#); a
    % PhysicalAgent tracks it with feedback.
    % All parameters (alpha, transit, limits, energyWeight, H, ...) are properties, set in
    % runTrial with 'Policy_<name>'.
    %
    % The RHCPs are solved numerically: each smooth case of the paper's case split (v > 0
    % only after R has been driven to 0) is searched on a grid and refined with fmincon,
    % instead of with the closed forms of the paper's Appendix A. With an energy-aware transit
    % model rho_ij is a third variable of each case, searched over [0.1 y/maxSpeed, H], or from
    % the shortest transit the limits allow.
    properties
        alpha = "nominal"    % RHCP3 weight of the next target: "nominal" = 1/|N_i|^2 (N_i
                             % includes i), or a number in [0, 1]; 0.5 = unweighted ED-RHC
        transit = "FO-0"             % "FO-0", "SO" or "FO-3" (see above)
        speedLimit = Inf             % speed limit of the SO / FO-3 profiles
        accelLimit = Inf             % |acceleration| limit of the SO / FO-3 profiles
        avoidStations = false        % true: never go to or charge at a charging station (stations are
                                     % left out of the neighborhood unless every neighbor is one)
        energyWeight = 7.68e-3       % weight of the energy objective int u^2 dt (alpha in eqs. 3
                                     % and 20 of the energy-aware paper): its 213.3e-6 for a
                                     % 600 x 600 space, times 36 for our 100 x 100 canvas;
                                     % weightFromBeta gives the paper's App. A rule
        clearFirst = false   % true: every visit clears the target. On arrival the agent stays
                             % until R_i = 0 (no RHCP1), and the RHCPs only plan visits to j
                             % that clear it (the JS simulator's energy-aware methods)
        H = 250              % upper bound on the planning horizon w [s]
        recheckTime = 0      % wait before RHCP3 is tried again when every neighbor is covered
                             % (0 = every step, as in the JS simulator)
        gridSize = 15        % grid points per decision variable before the fmincon refinement
        traceDecisions = false   % record every RHCP decision in decisions
        decisions = zeros(0, 6)  % [time, agent, RHCP kind, target i, decision (u_i, v_i or j), rho_ij]
    end

    properties (Access = private)
        states               % agent index -> struct(stage, target, covered, hold)
        options              % fmincon options
        energyModels         % cache: edge, agent and plan -> E(rho) (see energyOf)
    end

    properties (Constant)
        agentAssumption = "plans transit"   % what it assumes of the agents' motion (see ComponentRegistry.assumptionOf)
    end

    properties (Constant, Access = private)
        tol = 1e-6
        minTransit = 0.1     % shortest rho_ij searched, as a fraction of y_ij / maxSpeed
    end

    methods
        function obj = EDRHCPolicy(opts)
            arguments
                opts.Alpha = "nominal"
                opts.Transit string = "FO-0"
            end
            obj.alpha = opts.Alpha;
            obj.transit = opts.Transit;
            obj.states = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
            obj.energyModels = containers.Map('KeyType', 'char', 'ValueType', 'any');
        end

        function resetReservations(obj)
            % Called by ScenarioModel on every reset: forget the agents' stages and decisions
            obj.states = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.decisions = zeros(0, 6);
        end

        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            % Called when the agent has no path and no dwell left: on arrival (or at t = 0), and
            % when an active or inactive time is over
            cmd = struct();
            if obj.avoidStations, agent.chargeAllowed = false; end
            s = obj.stateOf(agent);
            if s.stage == "travel"
                u = obj.solve(1, agent, model, adj, curIdx, simTime);
                if u > obj.tol
                    obj.dwell(agent, model, adj, curIdx, "active", u);
                    return;
                end
            end
            if s.stage ~= "inactive" && model.targets(curIdx).R <= obj.tol
                v = obj.solve(2, agent, model, adj, curIdx, simTime);
                if v > obj.tol
                    obj.dwell(agent, model, adj, curIdx, "inactive", v);
                    return;
                end
            end
            cmd = obj.depart(agent, model, adj, curIdx, simTime);
        end

        function onStep(obj, agent, model, simTime)
            % Covering and uncovering events: re-solve RHCP1 or RHCP2 for a dwell in progress
            if ~isKey(obj.states, agent.index) || agent.dwellRemaining <= 0, return; end
            s = obj.states(agent.index);
            if s.stage == "travel" || s.hold, return; end
            adj = model.buildAdjacency();
            covered = obj.coveredNeighbors(agent, model, adj, s.target);
            if isequal(covered, s.covered), return; end
            if s.stage == "active"
                agent.dwellRemaining = obj.solve(1, agent, model, adj, s.target, simTime);
            else
                agent.dwellRemaining = obj.solve(2, agent, model, adj, s.target, simTime);
            end
            s.covered = covered;
            obj.states(agent.index) = s;
        end

        function best = rhcp(obj, kind, agent, model, i, simTime, adj)
            % Solves RHCP<kind> (1, 2 or 3) for the agent at target i and returns the best plan:
            % best.j (next target; [] if every neighbor is covered), best.x = [u_i v_i u_j v_j],
            % best.rho (transit time to j), best.cost, and best.uBi (active time that drives
            % R_i to 0)
            if nargin < 7, adj = model.buildAdjacency(); end
            nbrs = find(adj(i, :));
            avail = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            if obj.avoidStations   % stations only when there is no other way on
                st = arrayfun(@(k) isa(model.targets(k), 'ChargingStation'), nbrs);
                if ~all(st), avail = avail(~ismember(avail, nbrs(st))); end
            end
            hood = [i, avail];                    % the neighborhood without covered neighbors
            tg = model.targets(hood);
            R = [tg.R];
            A = arrayfun(@(t) t.growthRateAt(simTime, model.uncertaintyMode), tg);
            B = [tg.B];
            R(R < obj.tol) = 0;
            uBi = obj.clearTime(R(1), A(1), B(1));

            if kind == 3
                weights = obj.weights(numel(nbrs) + 1);
            else
                weights = [1 1];
            end
            best = struct('cost', inf, 'j', [], 'x', zeros(1, 4), 'rho', NaN, 'uBi', uBi);
            for k = 2:numel(hood)
                [y, pts] = obj.pathLength(model, i, hood(k));
                rho0 = y / agent.state.maxSpeed;
                rhoAgent = agent.shortestTransit(pts);   % what the agent can keep (0: any)
                if obj.transit == "FO-0"
                    rho0 = max(rho0, rhoAgent);
                    if rho0 > obj.H, continue; end
                    rhoRange = [rho0 rho0];
                else
                    rhoRange = [max([obj.minTransit * rho0, ...
                        SpeedProfile.minTransit(y, obj.speedLimit, obj.accelLimit), rhoAgent]), obj.H];
                end
                d = struct('R', R, 'A', A, 'B', B, 'j', k, 'rho', rho0, 'w', weights, ...
                    'y', y, 'vb', obj.speedLimit, 'ub', obj.accelLimit);
                rhoRange = obj.transitRange(agent, model, i, hood(k), pts, rhoRange);
                if isempty(rhoRange), continue; end   % no affordable transit to this neighbor
                if rhoRange(2) <= rhoRange(1), d.rho = rhoRange(1); end   % one transit time left
                if obj.transit ~= "FO-0"   % the energy the agent draws, E(rho)
                    d.E = obj.energyOf(agent, model, i, hood(k), pts, y, rhoRange);
                end
                [ac, bc] = obj.planConstraint(agent, model, i, hood(k), d, rhoRange);
                for c = obj.cases(kind, d, uBi, rhoRange(1), ac, bc)
                    [cost, x, rho] = obj.solveCase(d, c, rhoRange);
                    if cost < best.cost
                        best = struct('cost', cost, 'j', hood(k), 'x', x, 'rho', rho, 'uBi', uBi);
                    end
                end
            end
        end
    end

    methods (Static)
        function J = planCost(d, P)
            % Weighted neighborhood cost (1/w) * int R over [t, t + w] for plans
            % P = [ui vi uj vj] or [ui vi uj vj rho] (one row each; rho defaults to d.rho), by
            % Theorem 1 of the paper on the piecewise linear R trajectories
            ui = P(:, 1); vi = P(:, 2); uj = P(:, 3); vj = P(:, 4);
            if size(P, 2) > 4, rho = P(:, 5); else, rho = d.rho; end
            j = d.j;
            others = true(size(d.R)); others([1 j]) = false;
            w = ui + vi + rho + uj + vj;
            % target i: active (R falls), inactive (R = 0), then grows after the agent leaves
            Ri1 = max(d.R(1) - (d.B(1) - d.A(1)) * ui, 0);
            after = rho + uj + vj;
            Ji = ui .* (d.R(1) + Ri1) / 2 + after .* (2 * Ri1 + d.A(1) * after) / 2;
            % target j: grows until the agent arrives, falls while it is active, then 0
            arrive = ui + vi + rho;
            Rj1 = d.R(j) + d.A(j) * arrive;
            Jj = arrive .* (d.R(j) + Rj1) / 2 + uj .* (2 * Rj1 - (d.B(j) - d.A(j)) * uj) / 2;
            % every other target grows over the whole horizon
            Jm = w .* (2 * sum(d.R(others)) + sum(d.A(others)) * w) / 2;
            J = (d.w(1) * Jj + d.w(2) * (Ji + Jm)) ./ w;
        end

        function w = weightFromBeta(beta, model, bound)
            % Energy weight by App. A, eq. (77), of the energy-aware paper: beta in [0, 1)
            % balances energy against sensing (0.5 = equal weight), with the preferred speed or
            % acceleration bound of the agents:
            %   bound = "speed":  w = beta/(1 - beta) * ybar^2 / vmax^4
            %   bound = "accel":  w = beta/(1 - beta) / umax^2
            % ybar is the mean edge length of the model's network and vmax, umax the agents'
            % mean maxSpeed and maxAccel.
            arguments
                beta (1,1) double {mustBeInRange(beta, 0, 1, 'exclude-upper')}
                model
                bound string {mustBeMember(bound, ["speed", "accel"])} = "speed"
            end
            states = [model.agents.state];
            if bound == "speed"
                ybar = mean(arrayfun(@(e) sum(vecnorm(diff(e.curvePoints, 1, 1), 2, 2)), model.edges));
                w = beta / (1 - beta) * ybar^2 / mean([states.maxSpeed])^4;
            else
                w = beta / (1 - beta) / mean([states.maxAccel])^2;
            end
        end
    end

    methods (Access = protected)
        function covered = coveredNeighbors(~, agent, model, adj, i)
            % For each neighbor of i (in index order): is another agent at it or heading to it?
            nbrs = find(adj(i, :));
            covered = false(size(nbrs));
            for b = 1:numel(model.agents)
                other = model.agents(b);
                if other ~= agent
                    covered = covered | nbrs == other.current_target_idx;
                end
            end
        end

        function [a, b] = planConstraint(~, ~, ~, ~, ~, ~, ~)
            % Extra constraints a * [u_i v_i u_j v_j rho]' <= b (one row each) on every plan of
            % an RHCP at i that visits neighbor j (args: agent, model, i, j, d, rhoRange). None by
            % default (a = []); BatteryAwareRHCPolicy bounds the plan's energy
            a = []; b = [];
        end

        function rhoRange = transitRange(~, ~, ~, ~, ~, ~, rhoRange)
            % The transit times [lo hi] searched for neighbor j (args: agent, model, i, j, pts,
            % rhoRange); [] makes j unaffordable. Unchanged by default
        end

        function J = planObjective(obj, d, P)
            % The cost of plans P (rows [ui vi uj vj], or [ui vi uj vj rho] when rho is a
            % decision) that the RHCP minimizes: planCost plus the energy term energyWeight *
            % E(rho), scaled like planCost. With the weights [1 1] of RHCP1 and RHCP2 planCost is
            % J_sH; with the RHCP3 weights [alpha, 1 - alpha] it is J_sH / 2 at alpha = 0.5, so the
            % energy term is halved there and alpha = 0.5 gives the paper's RHCP3 objective
            % J_sH + energyWeight * E
            if size(P, 2) < 5
                J = obj.planCost(d, P);
            else
                J = obj.planCost(d, P) + sum(d.w) / 2 * obj.energyWeight * d.E(P(:, 5));
            end
        end

        function cmd = depart(obj, agent, model, adj, curIdx, simTime)
            % RHCP3: the next target j and the transit to it, handed over to the agent
            cmd = struct();
            s = obj.stateOf(agent);
            [j, rho] = obj.solve(3, agent, model, adj, curIdx, simTime);
            if isempty(j)
                % Every neighbor is covered: stay at i (still servicing it) and try RHCP3 again;
                % the agent leaves as soon as a neighbor is uncovered, or solves RHCP2 first if
                % R_i reaches 0 meanwhile (as the JS simulator does)
                stage = s.stage;
                if stage == "travel", stage = "active"; end
                obj.dwell(agent, model, adj, curIdx, stage, obj.recheckTime, true);
                return;
            end
            obj.states(agent.index) = struct('stage', "travel", 'target', j, 'covered', [], 'hold', false);
            if isprop(agent, 'transitTime')   % hand the planned transit over to the agent
                agent.transitTime = rho;
                if obj.transit == "FO-0"   % the agent's own profile over rho
                    agent.transitProfile = ""; agent.transitLimits = [Inf Inf];
                else
                    agent.transitProfile = obj.transit; agent.transitLimits = [obj.speedLimit, obj.accelLimit];
                end
            end
            cmd.kind = "move";
            cmd.targetIdx = j;
        end

        function forgetAgent(obj, index)
            % Drop the agent's stage: its next plan starts as on arrival (RHCP1)
            if isKey(obj.states, index), remove(obj.states, index); end
        end
    end

    methods
        function set.transit(obj, t)
            % "FO-0", "SO" or "FO-3"; the older names "constant speed" and "FO" still work
            t = SpeedProfile.canonical(t);
            assert(any(t == SpeedProfile.names), 'EDRHCPolicy:Transit', ...
                'Unknown transit model "%s" (use %s)', t, strjoin(SpeedProfile.names, ", "));
            obj.transit = t;
        end
    end

    methods (Access = protected)
        function s = stateOf(obj, agent)
            if isKey(obj.states, agent.index)
                s = obj.states(agent.index);
            else
                s = struct('stage', "travel", 'target', agent.current_target_idx, 'covered', [], 'hold', false);
            end
        end

        function dwell(obj, agent, model, adj, i, stage, duration, hold)
            % hold = true: waiting for a neighbor to be uncovered (no re-solve on events)
            if nargin < 8, hold = false; end
            agent.dwellRemaining = duration;
            obj.states(agent.index) = struct('stage', stage, 'target', i, ...
                'covered', obj.coveredNeighbors(agent, model, adj, i), 'hold', hold);
        end

        function [out, rho] = solve(obj, kind, agent, model, adj, i, simTime)
            % The decision of RHCP<kind> at target i: u_i (kind 1), v_i (kind 2), or the next
            % target j and the transit time rho_ij (kind 3; j = [] when every neighbor is covered)
            if kind == 1 && obj.clearFirst
                % JS simulator's energy-aware methods: no RHCP1, stay until R_i = 0
                tg = model.targets(i);
                out = obj.clearTime(tg.R, tg.growthRateAt(simTime, model.uncertaintyMode), tg.B);
                best = struct('rho', NaN);
            else
                best = obj.rhcp(kind, agent, model, i, simTime, adj);
                switch kind   % with no neighbor to plan for, u_i = v_i = 0: try RHCP3 (as the JS)
                    case 1
                        if isempty(best.j), out = 0; else, out = best.x(1); end
                    case 2
                        if isempty(best.j), out = 0; else, out = best.x(2); end
                    case 3
                        out = best.j;
                end
            end
            rho = best.rho;
            if obj.traceDecisions
                value = out;
                if isempty(value), value = NaN; end
                obj.decisions(end+1, :) = [simTime, agent.index, kind, i, value, rho];
            end
        end

        function w = weights(obj, hoodSize)
            % [weight of the planned next target j, weight of every other target]
            a = obj.alpha;
            if isstring(a) || ischar(a)
                assert(a == "nominal", 'EDRHCPolicy:Alpha', 'alpha must be "nominal" or a number');
                a = 1 / hoodSize^2;
            end
            w = [a, 1 - a];
        end

        function cs = cases(obj, kind, d, uBi, rhoLo, ac, bc)
            % The smooth cases of RHCP<kind>, each as [ui vi uj vj] = M*[x; y; rho] + c over
            % x, y >= 0 with Aineq*[x; y; rho] <= bineq (paper Sec. III, eqs. 20, 36, 46), and
            % the extra constraint ac * [ui vi uj vj rho]' <= bc if ac is not empty
            Rj = d.R(d.j); Aj = d.A(d.j); Bj = d.B(d.j);
            H = obj.H;
            if Bj > Aj
                % u_j that drives R_j to 0 if j is reached at t + rho is c00 + c1*rho; c1 is
                % also its growth per unit of waiting before leaving i
                c00 = Rj / (Bj - Aj);
                c1 = Aj / (Bj - Aj);
            else
                c00 = inf; c1 = 0;                    % R_j cannot be driven to 0
            end
            cs = struct('M', {}, 'c', {}, 'Aineq', {}, 'bineq', {}, 'clearsJ', {});
            switch kind
                case 3   % u_i = v_i = 0; x = u_j or v_j
                    cs(end+1) = mk([0 0 0; 0 0 0; 1 0 0; 0 0 0], [0 0 0 0], ...
                        [1 0 -c1; 1 0 1; 0 1 0], [c00; H; 0], false);
                    if isfinite(c00)
                        cs(end+1) = mk([0 0 0; 0 0 0; 0 0 c1; 1 0 0], [0 0 c00 0], ...
                            [1 0 1 + c1; 0 1 0], [H - c00; 0], true);
                    end
                case 2   % u_i = 0; x = v_i
                    cs(end+1) = mk([0 0 0; 1 0 0; 0 1 0; 0 0 0], [0 0 0 0], ...
                        [-c1 1 -c1; 1 1 1], [c00; H], false);
                    if isfinite(c00)
                        cs(end+1) = mk([0 0 0; 1 0 0; c1 0 c1; 0 1 0], [0 0 c00 0], ...
                            [1 + c1 1 1 + c1], H - c00, true);
                    end
                case 1
                    % u_i <= uBi, v_i = 0
                    cs(end+1) = mk([1 0 0; 0 0 0; 0 1 0; 0 0 0], [0 0 0 0], ...
                        [1 0 0; -c1 1 -c1; 1 1 1], [uBi; c00; H], false);
                    if isfinite(c00)
                        cs(end+1) = mk([1 0 0; 0 0 0; c1 0 c1; 0 1 0], [0 0 c00 0], ...
                            [1 0 0; 1 + c1 1 1 + c1], [uBi; H - c00], true);
                    end
                    % u_i = uBi (R_i driven to 0), v_i >= 0
                    if isfinite(uBi)
                        cs(end+1) = mk([0 0 0; 1 0 0; 0 1 0; 0 0 0], [uBi 0 0 0], ...
                            [-c1 1 -c1; 1 1 1], [c00 + c1 * uBi; H - uBi], false);
                        if isfinite(c00)
                            cs(end+1) = mk([0 0 0; 1 0 0; c1 0 c1; 0 1 0], [uBi 0 c00 + c1 * uBi 0], ...
                                [1 + c1 1 1 + c1], H - uBi - c00 - c1 * uBi, true);
                        end
                    end
            end
            if nargin > 5 && ~isempty(ac)
                for k = 1:numel(cs)
                    cs(k).Aineq = [cs(k).Aineq; ac(:, 1:4) * cs(k).M + [zeros(size(ac, 1), 2), ac(:, 5)]];
                    cs(k).bineq = [cs(k).bineq; bc(:) - ac(:, 1:4) * cs(k).c'];
                end
            end
            % drop empty cases: x = y = 0 with the shortest transit is in every nonempty case
            cs = cs(arrayfun(@(c) all(c.bineq - c.Aineq(:, 3) * rhoLo >= -obj.tol), cs));
            if obj.clearFirst   % plan to clear j too: keep the cases with u_j = its clear time
                cs = cs([cs.clearsJ]);
            end

            function c = mk(M, c, Aineq, bineq, clearsJ)
                c = struct('M', M, 'c', c, 'Aineq', Aineq, 'bineq', bineq, 'clearsJ', clearsJ);
            end
        end

        function [cost, x, rho] = solveCase(obj, d, c, rhoRange)
            % Grid search over the case's feasible set, then fmincon from the best point. With a
            % fixed transit time (rhoRange(1) = rhoRange(2)) rho is substituted and the search is
            % over [x y]; otherwise over [x y rho], rho on a geometric grid.
            keep = isfinite(c.bineq);             % a bound of inf (R_j cannot be cleared) is no bound
            Ain = c.Aineq(keep, :);
            bin = c.bineq(keep);
            fixed = rhoRange(2) <= rhoRange(1);
            if fixed
                M = c.M(:, 1:2); c0 = c.c + rhoRange(1) * c.M(:, 3)';
                bin = bin - Ain(:, 3) * rhoRange(1); Ain = Ain(:, 1:2);
                lb = [0 0]; ub = obj.H * [1 1];
            else
                M = c.M; c0 = c.c;
                lb = [0 0 rhoRange(1)]; ub = [obj.H obj.H rhoRange(2)];
            end
            n = numel(lb);
            for v = n:-1:1   % bounds implied by rows whose other x, y coefficients are >= 0 (rho first)
                o = [1:v-1, v+1:n];
                xy = o(o <= 2);
                rows = Ain(:, v) > 0 & all(Ain(:, xy) >= 0, 2);
                if any(rows)
                    least = max(Ain(rows, o), 0) * lb(o)' + min(Ain(rows, o), 0) * ub(o)';
                    ub(v) = min([ub(v); (bin(rows) - least) ./ Ain(rows, v)]);
                end
            end
            ub = max(ub, lb);
            g = obj.gridSize;
            if fixed
                [X, Y] = meshgrid(linspace(0, ub(1), g), linspace(0, ub(2), g));
                Z = [X(:), Y(:)];
            else
                [X, Y, P] = meshgrid(linspace(0, ub(1), g), linspace(0, ub(2), g), ...
                    lb(3) * (ub(3) / lb(3)).^linspace(0, 1, g));
                Z = [X(:), Y(:), P(:)];
            end
            Z = Z(all(Z * Ain' <= bin' + obj.tol, 2), :);
            if isempty(Z), Z = lb; end
            costs = cost3(Z);
            [~, k] = min(costs);
            if ~isfinite(costs(k))   % no plan of this case has a defined cost: infeasible
                cost = Inf; x = Z(k, :) * M' + c0;
                if fixed, rho = rhoRange(1); else, rho = Z(k, 3); end
                return;
            end
            [z, fz] = fmincon(@(z) cost3(z(:)'), Z(k, :)', Ain, bin, [], [], lb', ub', [], obj.options);
            if fz <= costs(k) && all(z(:)' >= lb - obj.tol) && all(Ain * z <= bin + obj.tol)
                cost = fz; z = max(z(:)', lb);
            else
                cost = costs(k); z = Z(k, :);
            end
            x = z * M' + c0;
            if fixed, rho = rhoRange(1); else, rho = z(3); end

            function J = cost3(Z)
                if fixed
                    J = obj.planObjective(d, Z * M' + c0);
                else
                    J = obj.planObjective(d, [Z * M' + c0, Z(:, 3)]);
                end
            end
        end

        function f = energyOf(obj, agent, model, i, j, pts, y, rhoRange)
            % The agent's energy objective int |a|^2 dt for the transit i -> j as a function of
            % rho (Agent.transitEnergyModel: the closed form E(y, rho) for an AbstractAgent; a
            % PhysicalAgent's own reference, turns included), cached per edge, agent kind and plan
            if norm(pts(1, :) - model.targets(i).position) > norm(pts(end, :) - model.targets(i).position)
                pts = flipud(pts);                   % from i, as the agent will cross it
            end
            key = sprintf('%s|%d>%d|%d|%.10g|%.10g,%.10g|%s|%.10g|%.10g|%.10g|%.10g', ...
                agent.energyModelKey(), i, j, size(pts, 1), y, pts(end, 1), pts(end, 2), ...
                obj.transit, obj.speedLimit, obj.accelLimit, rhoRange(1), rhoRange(2));
            if isKey(obj.energyModels, key)
                f = obj.energyModels(key);
            else
                f = agent.transitEnergyModel(pts, y, obj.transit, obj.speedLimit, obj.accelLimit, rhoRange);
                obj.energyModels(key) = f;
            end
        end

        function [y, pts] = pathLength(~, model, i, j)
            % Length y of the path of edge (i, j), and its points
            pts = model.findEdge(i, j).curvePoints;
            y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
        end

        function u = clearTime(~, R, A, B)
            % Active time that drives R to 0 (inf if the agent cannot)
            if B > A, u = R / (B - A); else, u = inf; end
        end
    end
end
