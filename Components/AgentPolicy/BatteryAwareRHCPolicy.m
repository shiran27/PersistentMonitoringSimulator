classdef BatteryAwareRHCPolicy < EDRHCPolicy
    % Battery-aware ED-RHC: ED-RHC (EDRHCPolicy) whose plans keep enough charge to reach a
    % charging station (ChargingStation targets in the model). Each agent knows its battery's
    % energy left E and its power model (Battery), and plans with the energy its plans draw:
    %   idle at a target    Battery.idlePower per second
    %   transit i -> j      Agent.transitEnergy(path, rho_ij): moving power over rho_ij, plus
    %                       alphaVel * y_ij and gammaAcc * int |a|^2 dt (divided by eta)
    % and with its reserve E_c(j): the energy of the cheapest route from target j to a station
    % (graph shortest path over the transit energies of the edges), plus the idle energy of
    % waiting there while every other agent that uses that station charges fully once (the
    % worst case: all of them arrive together, as agents that start full tend to).
    %
    % Terminal SOC constraint: every RHCP plan [u_i v_i rho_ij u_j v_j] at target i must leave,
    % when it ends at j, enough energy to reach a station plus a reserve socMin of the capacity:
    %   idlePower * (u_i + v_i + u_j + v_j) + E_ij(rho_ij) <= E - socMin * capacity - E_c(j).
    % The constraint is linear in the plan, so it joins each smooth case of the RHCPs (an extra
    % row of their constraints). It caps the dwells of RHCP1 and RHCP2 and drops the neighbors j
    % that cannot be afforded from RHCP3. Stations are never planned as monitoring visits.
    %
    % Recharging: when no uncovered neighbor can be afforded even without dwelling (or, with
    % every neighbor covered, when it cannot afford to wait), the agent heads to the nearest
    % station along its cheapest route, one edge per decision. Before its last edge it joins the
    % station's queue (first come, first served) and waits (queueWait) until a dock is free for
    % every agent ahead of it and for itself; it then rests there until the battery is full (the
    % station charges at its chargePower) and resumes ED-RHC with RHCP3. An agent that is
    % already full when it arrives leaves at once and starts no session. Agents on their way to
    % a station do not block the targets they pass, so an agent leaving a station can always
    % get out.
    %
    % Without stations in the model, or for an agent from whose target no station can be
    % reached, it is ED-RHC (with one warning). The transit model is ED-RHC's "FO-0"
    % by default; "SO" and "FO-3" also work (the constraint then takes the acceleration energy
    % at the shortest transit searched, an upper bound).
    properties
        socMin = 0.05     % charge always kept in reserve, as a fraction of the capacity
        queueWait = 1     % wait [s] before trying again while the station is taken
        dockAware = false % true: a station with m docks is taken only when m other agents charge at
                          % or head to it, and the reserve counts ceil((n - 1)/m) full charges (the
                          % energy-aware paper's stations); false: one agent at a time per station
        charges = 0       % charging sessions started so far (all agents)
    end

    properties (Access = private)
        modes             % agent index -> "monitor", "toStation" or "charging"
        routes            % agent index -> cheapest routes to the stations (see route)
        waiting           % station index -> agents waiting to go there, first come, first served
        warnedNoStation = false
    end

    methods
        function obj = BatteryAwareRHCPolicy(opts)
            arguments
                opts.Alpha = "nominal"
                opts.Transit string = "FO-0"
            end
            obj@EDRHCPolicy(Alpha=opts.Alpha, Transit=opts.Transit);
            obj.resetModes();
        end

        function resetReservations(obj)
            resetReservations@EDRHCPolicy(obj);
            obj.resetModes();
            obj.charges = 0;
        end

        function cmd = plan(obj, agent, model, adj, i, simTime)
            cmd = struct();
            if ~any(obj.stations(model))
                cmd = plan@EDRHCPolicy(obj, agent, model, adj, i, simTime);
                return;
            end
            switch obj.modeOf(agent)
                case "charging"
                    if obj.isFull(agent)   % charged: leave by RHCP3
                        obj.modes(agent.index) = "monitor";
                        obj.forgetAgent(agent.index);
                        cmd = obj.depart(agent, model, adj, i, simTime);
                    else
                        agent.dwellRemaining = obj.timeToFull(agent, model.targets(i));
                    end
                case "toStation"
                    if isa(model.targets(i), 'ChargingStation')
                        if obj.isFull(agent)   % nothing to charge: no session
                            obj.modes(agent.index) = "monitor";
                            obj.forgetAgent(agent.index);
                            cmd = obj.depart(agent, model, adj, i, simTime);
                        else
                            obj.modes(agent.index) = "charging";
                            obj.charges = obj.charges + 1;
                            agent.dwellRemaining = obj.timeToFull(agent, model.targets(i));
                        end
                    elseif obj.noStation(agent, model, i)
                        obj.modes(agent.index) = "monitor";
                        cmd = plan@EDRHCPolicy(obj, agent, model, adj, i, simTime);
                    else
                        cmd = obj.hop(agent, model, i);
                    end
                otherwise
                    if obj.canAffordNeighbor(agent, model, adj, i) || obj.noStation(agent, model, i)
                        cmd = plan@EDRHCPolicy(obj, agent, model, adj, i, simTime);
                    else
                        obj.modes(agent.index) = "toStation";
                        cmd = obj.hop(agent, model, i);
                    end
            end
        end

        function onStep(obj, agent, model, simTime)
            % Covering events re-solve dwells only while monitoring
            if obj.modeOf(agent) == "monitor"
                onStep@EDRHCPolicy(obj, agent, model, simTime);
            end
        end

        function m = modeOf(obj, agent)
            % "monitor", "toStation" or "charging"
            m = "monitor";
            if isKey(obj.modes, agent.index), m = obj.modes(agent.index); end
        end
    end

    methods (Access = protected)
        function [a, b] = planConstraint(obj, agent, model, i, j, d, rhoRange)
            % The terminal SOC constraint on plans from i that visit j (see the class help)
            a = []; b = [];
            if ~any(obj.stations(model)), return; end
            if isa(model.targets(j), 'ChargingStation')    % not a monitoring visit
                a = zeros(1, 5); b = -1;
                return;
            end
            r = obj.route(agent, model);
            if ~isfinite(r.E(i)), return; end   % no station reachable: plain ED-RHC (noStation)
            bat = agent.battery;
            budget = bat.remaining - obj.socMin * bat.capacity - obj.reserve(agent, model, j);
            Pi = bat.idlePower();
            if obj.transit == "FO-0"
                a = [Pi Pi Pi Pi 0];
                b = budget - obj.edgeEnergy(agent, model, i, j, d.rho);
            else
                Ja = d.E(rhoRange(1));                   % the agent's own E(rho), at its largest
                a = [Pi Pi Pi Pi bat.movePower()];
                b = budget - (bat.alphaVel * d.y + bat.gammaAcc * Ja) / bat.eta;
            end
        end

        function covered = coveredNeighbors(obj, agent, model, adj, i)
            % As in ED-RHC, but an agent on its way to a station covers only the station
            nbrs = find(adj(i, :));
            covered = false(size(nbrs));
            for k = 1:numel(model.agents)
                other = model.agents(k);
                if other == agent, continue; end
                t = other.current_target_idx;
                if obj.modeOf(other) == "toStation" && ~isa(model.targets(t), 'ChargingStation')
                    continue;
                end
                covered = covered | nbrs == t;
            end
        end
    end

    methods (Access = private)
        function resetModes(obj)
            obj.modes = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.routes = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.waiting = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.warnedNoStation = false;
        end

        function s = stations(~, model)
            s = arrayfun(@(t) isa(t, 'ChargingStation'), model.targets);
        end

        function ok = canAffordNeighbor(obj, agent, model, adj, i)
            % Can the agent go to some uncovered neighbor (not a station) and still reach a
            % station from there, without dwelling? With every neighbor covered: can it afford
            % to wait here (ED-RHC waits for a neighbor to be uncovered)?
            bat = agent.battery;
            budget = bat.remaining - obj.socMin * bat.capacity;
            nbrs = find(adj(i, :));
            free = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            free = free(~arrayfun(@(j) isa(model.targets(j), 'ChargingStation'), free));
            if isempty(free)
                ok = obj.reserve(agent, model, i) <= budget;
                return;
            end
            ok = false;
            for j = free
                if obj.edgeEnergy(agent, model, i, j) + obj.reserve(agent, model, j) <= budget
                    ok = true; return;
                end
            end
        end

        function E = reserve(obj, agent, model, j)
            % Energy needed from target j: the route to the nearest station and the wait there
            r = obj.route(agent, model);
            E = r.E(j);
            if isfinite(E)
                E = E + agent.battery.idlePower() * obj.queueTime(agent, model, r.station(j));
            end
        end

        function W = queueTime(obj, agent, model, st)
            % Longest wait at station st: every other agent that uses it (its nearest station)
            % charging fully once (dockAware: ceil(that number / m) full charges)
            W = 0;
            P = model.targets(st).chargePower;
            c = zeros(1, 0);
            for o = model.agents(:)'
                if o == agent || o.batteryEmpty(), continue; end
                ro = obj.route(o, model);
                if ro.station(o.current_target_idx) ~= st, continue; end
                c(end+1) = o.battery.capacity / (P - o.battery.idlePower()); %#ok<AGROW>
                W = W + c(end);
            end
            m = model.targets(st).docks;
            if obj.dockAware && isfinite(m) && ~isempty(c)
                W = ceil(numel(c) / m) * max(c);
            end
        end

        function cmd = hop(obj, agent, model, i)
            % The next edge toward the nearest station (wait while that station is taken)
            cmd = struct();
            r = obj.route(agent, model);
            k = r.next(i);
            if isnan(k)   % no station reachable: wait
                agent.dwellRemaining = obj.queueWait;
                return;
            end
            if isa(model.targets(k), 'ChargingStation')
                % first come, first served: join the station's queue, and go only when a dock
                % is free for every agent ahead in the queue and for this one
                q = obj.queueOf(model, k);
                if ~ismember(agent.index, q), q(end+1) = agent.index; end
                [n, m] = obj.occupancy(agent, model, k);
                if find(q == agent.index, 1) > m - n
                    obj.waiting(k) = q;
                    agent.dwellRemaining = obj.queueWait;   % taken: queue here
                    return;
                end
                obj.waiting(k) = q(q ~= agent.index);
            end
            agent.transitTime = r.rho(i, k);
            agent.transitProfile = ""; agent.transitLimits = [Inf Inf];
            cmd.kind = "move";
            cmd.targetIdx = k;
        end

        function tf = noStation(obj, agent, model, i)
            % No station reachable from target i: fall back to ED-RHC (and warn once)
            r = obj.route(agent, model);
            tf = ~isfinite(r.E(i));
            if tf && ~obj.warnedNoStation
                warning('BatteryAwareRHCPolicy:NoStation', ...
                    'No charging station is reachable from target %d; the agent continues with ED-RHC.', i);
                obj.warnedNoStation = true;
            end
        end

        function q = queueOf(obj, model, k)
            % The agents waiting for station k, without those that no longer head there
            q = zeros(1, 0);
            if isKey(obj.waiting, k), q = obj.waiting(k); end
            keep = false(size(q));
            for c = 1:numel(q)
                o = model.agents([model.agents.index] == q(c));
                if isempty(o), continue; end
                keep(c) = ~o.batteryEmpty() && isKey(obj.modes, q(c)) && obj.modes(q(c)) == "toStation";
            end
            q = q(keep);
        end

        function [n, m] = occupancy(obj, agent, model, k)
            % Agents charging at station k or heading to it, and the docks it offers this policy
            st = model.targets(k);
            m = 1;
            if obj.dockAware, m = st.docks; end
            n = 0;
            for o = model.agents(:)'
                if o == agent || o.current_target_idx ~= k, continue; end
                if ~o.batteryEmpty() || norm(o.state.pos - st.position) < model.detectionRadius
                    n = n + 1;
                end
            end
        end

        function E = edgeEnergy(obj, agent, model, i, j, rho)
            % Predicted energy of the transit i -> j (at the route's transit time by default)
            r = obj.route(agent, model);
            if nargin < 6 || abs(rho - r.rho(i, j)) < 1e-9
                E = r.Eedge(i, j);
            else
                pts = model.findEdge(i, j).curvePoints;
                E = agent.transitEnergy(pts, rho);
            end
        end

        function r = route(obj, agent, model)
            % Cheapest routes to the stations for this agent: r.E(j) energy from j to the
            % nearest station r.station(j), r.next(j) the first target on that route, and per edge the
            % transit time r.rho(i, j) (ED-RHC's FO-0 transit) and energy r.Eedge(i, j).
            % Computed once per run and agent (Dijkstra from the stations)
            n = numel(model.targets);
            key = agent.index;
            if isKey(obj.routes, key)
                r = obj.routes(key);
                if r.n == n && r.nEdges == numel(model.edges), return; end
            end
            rho = inf(n); Eedge = inf(n);
            for e = model.edges
                a = e.targets(1).index; b = e.targets(2).index;
                pts = e.curvePoints;
                y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
                t = max(y / agent.state.maxSpeed, agent.shortestTransit(pts));
                rho(a, b) = t; rho(b, a) = t;
                Eedge(a, b) = agent.transitEnergy(pts, t);
                Eedge(b, a) = agent.transitEnergy(flipud(pts), t);
            end
            E = inf(n, 1); next = nan(n, 1); station = nan(n, 1);
            E(obj.stations(model)) = 0;
            station(obj.stations(model)) = find(obj.stations(model));
            todo = true(n, 1);
            while any(todo)
                cand = find(todo);
                [~, k] = min(E(cand));
                u = cand(k);
                if isinf(E(u)), break; end
                todo(u) = false;
                for v = find(isfinite(Eedge(:, u)))'   % v reaches the station through u
                    if todo(v) && Eedge(v, u) + E(u) < E(v)
                        E(v) = Eedge(v, u) + E(u);
                        next(v) = u;
                        station(v) = station(u);
                    end
                end
            end
            r = struct('n', n, 'nEdges', numel(model.edges), 'E', E, 'next', next, ...
                'station', station, 'rho', rho, 'Eedge', Eedge);
            obj.routes(key) = r;
        end

        function ok = isFull(~, agent)
            ok = agent.battery.energyUsed <= 1e-3 * agent.battery.capacity;
        end

        function t = timeToFull(~, agent, station)
            net = station.chargePower - agent.battery.idlePower();
            assert(net > 0, 'BatteryAwareRHCPolicy:Charger', ...
                'The station''s chargePower (%g W) must exceed the idle power (%.1f W)', ...
                station.chargePower, agent.battery.idlePower());
            t = max(agent.battery.energyUsed / net, 0.05);
        end
    end
end
