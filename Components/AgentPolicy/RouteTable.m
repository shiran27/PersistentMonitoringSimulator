classdef RouteTable < handle
    % Route tables to the charging stations (energy-aware paper, Sec. 6.1, Lemma 2). For every
    % node k (target or station) and station sigma: the least excess route energy D(k, sigma),
    % the route time T(k, sigma) and the next node next(k, sigma). An edge's weight is its
    % least excess energy min over rho >= rho_min of E^ex_kl(rho) (Agent.transitPlanEnergy),
    % attained at the energy-optimal transit time rho^E_kl, for the given agent's power model
    % and limits (a homogeneous team). The tables come from a synchronous distributed
    % Bellman-Ford: in each round, every node whose table changed sends it to its neighbors;
    % rounds and messages count them (the paper's offline cost).
    %   R = RouteTable(model, agent)        build for the model's graph and stations
    %   R.route(k, sigma)                   the nodes from k to station sigma
    %   [home, util, ok] = R.homeStations(starts, agent, Pbar)   home station of each agent
    properties
        stations        % station node indices (1 x S)
        D               % n x S least excess route energy [J]
        T               % n x S route time along that route [s]
        next            % n x S next node (0 at the station itself or when unreachable)
        edgeEnergy      % n x n least excess energy of each edge [J] (Inf: no edge)
        edgeTime        % n x n its energy-optimal transit time rho^E [s]
        rounds = 0      % synchronous Bellman-Ford rounds with messages
        messages = 0    % tables sent to neighbors over all rounds
    end

    methods
        function obj = RouteTable(model, agent)
            n = numel(model.targets);
            obj.stations = find(arrayfun(@(t) isa(t, 'ChargingStation'), model.targets(:)'));
            obj.edgeEnergy = inf(n); obj.edgeTime = nan(n);
            for e = model.edges(:)'
                i = e.targets(1).index; j = e.targets(2).index;
                pts = e.curvePoints;
                if isempty(pts), pts = [e.targets(1).position; e.targets(2).position]; end
                [E, rho] = RouteTable.edgeOptimum(agent, pts);
                if E < obj.edgeEnergy(i, j)                % undirected; the cheaper of parallel edges
                    obj.edgeEnergy(i, j) = E; obj.edgeEnergy(j, i) = E;
                    obj.edgeTime(i, j) = rho; obj.edgeTime(j, i) = rho;
                end
            end
            obj.bellmanFord();
        end

        function nodes = route(obj, k, sigma)
            % The nodes from k to station sigma (a node index) along the least-energy route
            s = find(obj.stations == sigma, 1);
            nodes = k;
            while nodes(end) ~= sigma && obj.next(nodes(end), s) > 0 && numel(nodes) <= size(obj.D, 1)
                nodes(end+1) = obj.next(nodes(end), s); %#ok<AGROW>
            end
        end

        function [home, util, ok] = homeStations(obj, model, starts, agent, Pbar)
            % Home station of each agent (node indices), offline: the numbers n_sigma of agents
            % per station minimize the largest utilization n_sigma Pbar/(m_sigma (P_c - P_I))
            % (water-filling), and the agents are then matched to stations for the least total
            % route energy from their start nodes (matchpairs). util: each station's
            % utilization; ok: Lemma 4's condition P_c >= ceil(n_sigma/m_sigma) P_I. Pbar is
            % the agents' mean power away from a station (default P_I; it scales util only)
            N = numel(starts); S = numel(obj.stations);
            PI = agent.battery.idleDraw();   % idle power drawn from the battery, losses included
            if nargin < 5, Pbar = PI; end
            m = arrayfun(@(s) model.targets(s).docks, obj.stations);
            m(~isfinite(m)) = N;
            Pc = arrayfun(@(s) model.targets(s).chargePower, obj.stations);
            w = 1 ./ (m .* (Pc - PI));
            n = zeros(1, S);
            for a = 1:N
                [~, s] = min((n + 1) .* w);
                n(s) = n(s) + 1;
            end
            slots = repelem(1:S, n);
            cost = obj.D(starts(:), slots);
            cost(~isfinite(cost)) = 1e12;
            M = matchpairs(cost, 1e15);
            home = zeros(1, N);
            home(M(:, 1)) = obj.stations(slots(M(:, 2)));
            util = n .* Pbar .* w;
            ok = Pc >= ceil(n ./ m) * PI;
        end
    end

    methods (Static)
        function [E, rho] = edgeOptimum(agent, pts)
            % Least excess energy of a transit along pts and its transit time. E^ex(rho) is
            % convex (Proposition 1): double b from 2 rho_min while it still falls, so the
            % minimum lies in [max(rho_min, b/4), b], and refine there (fminbnd)
            lo = agent.limitTransit(pts);
            f = @(r) RouteTable.excess(agent, pts, r);
            a = lo; b = 2 * lo; fa = f(a); fb = f(b);
            while fb < fa && b < 1e4 * lo
                a = b; fa = fb; b = 2 * b; fb = f(b);
            end
            [rho, E] = fminbnd(f, max(lo, a / 2), b, optimset('TolX', 1e-7 * b));
            fLo = f(lo);
            if fLo <= E, rho = lo; E = fLo; end   % the minimum at rho_min
        end

        function E = excess(agent, pts, rho)
            [~, E] = agent.transitPlanEnergy(pts, rho);
        end
    end

    methods (Access = private)
        function bellmanFord(obj)
            n = size(obj.edgeEnergy, 1); S = numel(obj.stations);
            obj.D = inf(n, S); obj.T = inf(n, S); obj.next = zeros(n, S);
            for s = 1:S, obj.D(obj.stations(s), s) = 0; obj.T(obj.stations(s), s) = 0; end
            nbrs = arrayfun(@(k) find(isfinite(obj.edgeEnergy(k, :))), 1:n, 'UniformOutput', false);
            active = false(1, n); active(obj.stations) = true;
            obj.rounds = 0; obj.messages = 0;
            while any(active)
                D0 = obj.D; T0 = obj.T;                   % tables sent this round
                obj.rounds = obj.rounds + 1;
                changed = false(1, n);
                for l = find(active)
                    obj.messages = obj.messages + numel(nbrs{l});
                    for k = nbrs{l}
                        cand = obj.edgeEnergy(k, l) + D0(l, :);
                        old = obj.D(k, :);
                        better = cand < old & (isinf(old) | cand < old - 1e-9 * max(1, abs(old)));
                        if any(better)
                            obj.D(k, better) = cand(better);
                            obj.T(k, better) = obj.edgeTime(k, l) + T0(l, better);
                            obj.next(k, better) = l;
                            changed(k) = true;
                        end
                    end
                end
                active = changed;
            end
        end
    end
end
