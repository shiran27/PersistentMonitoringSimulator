classdef EnergyAwareEDRHCPolicy < EDRHCPolicy
    % Energy-aware event-driven RHC with charging coordination: the method of the energy-aware
    % paper (its blueprint, Claude_Notes/For_Shirantha/REPORT_EnergyAwareAgents.md, Secs. 5-6).
    % It keeps ED-RHC's events and ordering cases (EDRHCPolicy) and changes three things:
    %
    % 1. Objective (Sec. 5.2). For a plan U = (u_i, v_i, j, rho_ij, u_j, v_j) of length w,
    %      N(U) = J(t, t_f) + omega E^ex_ij(rho) + dhhat(U),
    %    with J the neighborhood's uncertainty integral over the horizon, E^ex the transit's
    %    excess energy over idling (Agent.transitPlanEnergy, the full power model with battery
    %    losses), and dhhat the terminal correction (with terminal = "hhat"): the targets' relative
    %    values hhat_k(X(t_f)) - hhat_k(X(t)) (CollabTargetCost.terminal, as in Collab-ED-RHC)
    %    and the energy value hhat_E, which adds mu_a (E^ex + D_j - D_i)/(P_c - P_I) with mu_a >= 0
    %    the targets' marginal cost of delay at the event (Proposition 3). objectiveForm:
    %      "dinkelbach"  minimize N - lambda w, with Dinkelbach's iteration on lambda (default)
    %      "ratio"       minimize N / w directly (the same minimizers at lambda*)
    %      "alphaH"      minimize (J + dhhat_targets)/w + alphaH E^ex (ED-ORHC's constant weight)
    %    terminal = "alpha" (default) uses ED-RHC-alpha's weights in J instead of hhat_k, and
    %    "none" drops hhat_k; all three keep hhat_E (the energy treatment is the same), and mu_a
    %    always comes from the hhat_k derivatives at the event.
    % 2. Booking rows (Sec. 6.3). Every agent holds a booking (station sigma, dock, slot start
    %    t_a). A plan to j is admissible only if t_f + T(j, sigma) + delta <= t_a (a row on w)
    %    and E^ex_ij(rho) + D(j, sigma) + P_I (t_a - t) <= E - E_min (an interval of rho),
    %    with D and T the route energy and time to the station (RouteTable). With a finite
    %    mission end T, t_f <= T.
    % 3. Charging (Secs. 6.2, 6.8). An agent leaves for its station when no uncovered neighbor
    %    is affordable or at its latest departure, follows the least-energy route without
    %    covering targets, waits there until t_a (not charging; with earlyStart it starts as soon
    %    as its dock is idle and no other booking on it comes first, and holds the dock through
    %    its slot), charges until full or the end of its slot, and books its next slot. With
    %    pullForward, a station offers time freed on a dock (a session that ended full before its
    %    slot's end) to the first booked agent in line (one that asked for an earlier slot, then
    %    the earliest booking) that is at most half full, can reach the dock in that time and use
    %    it until its slot; it accepts at its next event, claims the dock and comes now. Once its
    %    energy covers its mean power until T it releases its booking. Slots of length
    %    t_s = (Q - E_min)/(P_c - P_I) start at k t_s on every dock; bookingStrategy "a" (default)
    %    takes the free slot nearest the agent's predicted need (its energy at its mean power since
    %    its last charge), "b" the latest free slot in its window [e_a, l_a], and after each
    %    commit the latest free slot that keeps the committed plan feasible. An agent that heads to
    %    its station early moves its booking to the earliest free slot it can reach, which only
    %    raises its energy on arrival. Home stations balance the stations' utilization
    %    (RouteTable.homeStations).
    % The rest of the paper's station scheduler (reported costs, reassignments, offers to other
    % stations) is not part of this class yet. Stations charge only the agents in their slots
    % (Agent.chargeAllowed is false otherwise, also for an agent dwelling at a station).
    % Parameters are properties ('Policy_<name>' in runTrial); runTrial sets missionEnd.
    properties
        objectiveForm = "dinkelbach"   % "dinkelbach", "ratio" or "alphaH"
        terminal = "alpha"             % "alpha" (default), "hhat" or "none"
        omega = 0                      % energy weight of the global objective [1/s]
        alphaH = 0                     % energy weight of the "alphaH" form
        socMin = 0.1                   % reserve E_min = socMin Q
        missionEnd = Inf               % mission end T [s]
        bookingStrategy = "a"          % "a" predicted need (default), "b" latest safe slot (also after
                                       % each commit)
        messageDelay = 0               % delta [s], added to the time row
        marginTime = 0                 % margins of physical agents (time row [s], energy row [J])
        marginEnergy = 0
        stepSize = 0                   % simulation step [s] (runTrial sets it): each transit may end up
                                       % to a step late and draw stepEnergy * stepSize more than its
                                       % plan, so (1 + route hops) times both join the margins
        stepEnergy = 5                 % per transit and second of step [J/s]: twice the 2.3 J/s
                                       % measured on the energy setups
        priceScale = 1                 % multiplies hhat_E's price mu_a (0: energy unpriced by hhat_E)
        priceForm = "renewal"          % hhat_E's slope mu_a/(P_c - P_I + Pbar_a) ("renewal", default:
                                       % the slope of the renewal model's value of stored energy;
                                       % Pbar_a the agent's mean power since its last charge) or
                                       % mu_a/(P_c - P_I) ("charging")
        priceAveraging = false         % true: mu_a is the running mean of the agent's values at its
                                       % events, instead of the value at this event
        pullForward = true             % station pull-forward: time freed on a dock (a session ended before
                                       % its slot's end, or a released booking of the current slot) is
                                       % offered to the first booked agent in line (those that asked for
                                       % an earlier slot first, then by booking start) whose battery is at
                                       % most 1 - pullMinCharge full; it accepts at its next event if it
                                       % can still reach the dock in the freed time
        pullMinCharge = 0.5            % least share of the capacity an offered session must refill
        otherStations = true           % the D2 request also asks the other stations for an earlier slot
                                       % (the home booking is released after the grant)
        earlyStart = true              % work-conserving docks: a booked agent at its station starts
                                       % charging as soon as its dock is idle and no other booking on
                                       % it comes first (its session still ends by its own slot's end)
        dinkelbachTol = 1e-6           % relative tolerance of Dinkelbach's iteration
        dinkelbachIterations = 20
        charges = 0                    % charging sessions so far
        sessions = struct('agent', {}, 'station', {}, 'dock', {}, 'slot', {}, 'slotEnd', {}, 'arrive', {}, ...
            'start', {}, 'stop', {}, 'Ein', {}, 'Eout', {})   % one row per session
        windowMisses = 0               % bookings placed outside the agent's window (none expected)
        modeTime = zeros(0, 4)         % per agent: seconds monitoring, to a station, waiting, charging
        stats = struct()               % diagnostics: RHCPs solved, those whose energy or time row
                                       % binds at the optimum, "no affordable neighbor" events, and
                                       % the planned transits [rho, rho_min]
        traceEvents = false            % record events in trace
        trace = zeros(0, 6)            % [time, agent, event, node, energy left, booked slot start];
                                       % events: 1 plan, 2 go to station, 3 dwell to latest departure,
                                       % 4 wait, 5 charge, 6 charged, 7 release, 8 depart
    end

    properties (Access = private)
        modes          % agent index -> "monitor", "toStation", "waiting" or "charging"
        bookings       % agent index -> struct(station, dock, slot, start, length, arrive)
        released       % agent index -> true once it no longer needs a station
        lambdas        % agent index -> lambda* of its last Dinkelbach solve
        lastNode       % agent index -> node of its last decision
        lastCharge     % agent index -> [time, energy left] after its last charge (or start)
        lastStep       % agent index -> time of its last onStep (for modeTime)
        lambdaNow = 0  % lambda of the solve in progress
        ctx            % data of the neighbor j being solved (transitRange -> planObjective)
        hood           % data of the neighborhood being solved (rhcp -> planObjective)
        routes         % RouteTable of the run
        homes          % agent index -> home station
        grids          % station index -> docks x slots occupancy (agent index, 0 = free)
        tables         % cache: key -> E^ex table of an edge
        gapSum, gapCount, lastLeft, gapInit   % per target: unattended-time history
        initialized = false
        marginT = 0, marginE = 0       % margins in use: the given ones plus the step's
        rows                           % neighbor j -> its row limits in the solve in progress
        muMean                         % agent index -> [running mean of mu_a, number of events]
        offers                         % agent index -> struct(station, dock, until): freed time offered to it
        wantsEarlier                   % agent index -> true while it asked for an earlier slot
        freed                          % 1000 station + dock -> end of the time freed on that dock
        lastOfferCheck = -Inf          % time of the stations' last check for freed time
        slotLens                       % station index -> slot length t_s
        hopsTo                         % node x station column: route hops (for the relay count)
    end

    methods
        function obj = EnergyAwareEDRHCPolicy(opts)
            arguments
                opts.Alpha = "nominal"
                opts.Transit string = "SO"
            end
            obj@EDRHCPolicy(Alpha=opts.Alpha, Transit=opts.Transit);
            obj.clearState();
        end

        function resetReservations(obj)
            resetReservations@EDRHCPolicy(obj);
            obj.clearState();
        end

        function cmd = plan(obj, agent, model, adj, i, simTime)
            obj.ensureInit(model, simTime);
            obj.noteArrival(agent, model, i, simTime);
            obj.note(1, agent, i, simTime);
            cmd = struct();
            switch obj.modeOf(agent)
                case "toStation"
                    cmd = obj.goStation(agent, model, i, simTime);
                case "waiting"
                    obj.startCharging(agent, model, i, simTime);
                case "charging"
                    obj.finishCharging(agent, model, i, simTime);
                    obj.forgetAgent(agent.index);
                    cmd = obj.depart(agent, model, adj, i, simTime);
                otherwise
                    obj.maybeRelease(agent, simTime);
                    if obj.acceptOffer(agent, model, i, simTime)
                        cmd = obj.goStation(agent, model, i, simTime);
                    elseif obj.mustLeave(agent, i, simTime)
                        cmd = obj.goStation(agent, model, i, simTime);
                    else
                        cmd = plan@EDRHCPolicy(obj, agent, model, adj, i, simTime);
                    end
            end
        end

        function onStep(obj, agent, model, simTime)
            % Time in each mode (modeTime); covering events re-solve dwells only while monitoring
            if isKey(obj.lastStep, agent.index)
                k = find(obj.modeOf(agent) == ["monitor", "toStation", "waiting", "charging"]);
                if size(obj.modeTime, 1) < agent.index, obj.modeTime(agent.index, 4) = 0; end
                obj.modeTime(agent.index, k) = obj.modeTime(agent.index, k) + simTime - obj.lastStep(agent.index);
            end
            obj.lastStep(agent.index) = simTime;
            if obj.pullForward && simTime > obj.lastOfferCheck && obj.freed.Count > 0
                obj.lastOfferCheck = simTime;
                obj.offerFreedTime(model, simTime);
            end
            if obj.modeOf(agent) == "waiting" && obj.earlyStart
                obj.tryEarlyStart(agent, model, simTime);
            end
            if obj.modeOf(agent) == "monitor"
                onStep@EDRHCPolicy(obj, agent, model, simTime);
            end
        end

        function best = rhcp(obj, kind, agent, model, i, simTime, adj)
            % RHCP<kind> at target i with the energy-aware objective; "dinkelbach" iterates
            % lambda <- N/w at the minimizer until it settles (lambda* of the corrected ratio)
            if nargin < 7, adj = model.buildAdjacency(); end
            obj.ensureInit(model, simTime);
            obj.prepareHood(agent, model, adj, i, simTime);
            obj.rows = containers.Map('KeyType', 'double', 'ValueType', 'any');
            if obj.objectiveForm ~= "dinkelbach"
                best = rhcp@EDRHCPolicy(obj, kind, agent, model, i, simTime, adj);
                obj.noteBinding(best);
                obj.noteNone(best, kind, agent, model, adj, i);
                obj.moveLater(agent, model, i, best);
                return;
            end
            lam = 0;
            if isKey(obj.lambdas, agent.index), lam = obj.lambdas(agent.index); end
            for it = 1:obj.dinkelbachIterations
                obj.lambdaNow = lam;
                best = rhcp@EDRHCPolicy(obj, kind, agent, model, i, simTime, adj);
                if isempty(best.j), break; end
                w = sum(best.x) + best.rho;
                ratio = best.cost / w + lam;           % N / w at the minimizer
                done = abs(ratio - lam) <= obj.dinkelbachTol * max(1, abs(lam));
                lam = ratio;
                if done, break; end
            end
            obj.lambdas(agent.index) = lam;
            obj.noteBinding(best);
            obj.noteNone(best, kind, agent, model, adj, i);
            obj.moveLater(agent, model, i, best);
        end

        function m = modeOf(obj, agent)
            % "monitor", "toStation", "waiting" or "charging"
            m = "monitor";
            if isKey(obj.modes, agent.index), m = obj.modes(agent.index); end
        end

        function b = bookingOf(obj, agent)
            % The agent's booking (struct with station, dock, slot, start, length), [] if none
            b = [];
            if isKey(obj.bookings, agent.index), b = obj.bookings(agent.index); end
        end

        function R = routeTable(obj)
            R = obj.routes;
        end
    end

    methods (Access = protected)
        function rr = transitRange(obj, agent, model, i, j, pts, rr)
            % The transit times to j that keep the energy row (an interval, E^ex being convex),
            % within the agent's own limits (Lemma 1); also prepares j's data for planObjective
            obj.ctx = struct('j', j, 'E', @(r) zeros(size(r)), 'D', 0);
            if isa(model.targets(j), 'ChargingStation'), return; end   % excluded by planConstraint
            tab = obj.energyTable(agent, model, i, j, pts);
            obj.ctx.E = tab.E;
            [budget, D] = obj.energyBudget(agent, j);
            obj.ctx.D = D;
            if obj.transit == "FO-0"   % fixed transit time
                if tab.E(rr(1)) > budget, rr = []; end
                return;
            end
            lo = max(rr(1), tab.rhoMin); hi = min(rr(2), obj.H);
            if lo > hi, rr = []; return; end
            r = lo * (hi / lo).^linspace(0, 1, 400);
            ok = tab.E(r) <= budget;
            if ~any(ok), rr = []; return; end
            k = find(ok);
            rr = [r(k(1)), r(k(end))];
            obj.rows(j) = struct('lo', lo, 'hi', hi, 'rho', rr, 'limJ', Inf, 'limI', Inf);
        end

        function [a, b] = planConstraint(obj, agent, model, i, j, d, rhoRange) %#ok<INUSD>
            % The booking's time rows t + w + T(j, sigma) + delta <= t_a and t + u_i + v_i +
            % T(i, sigma) + delta <= t_a, and t + w <= T
            a = []; b = [];
            if isa(model.targets(j), 'ChargingStation')   % stations are not monitoring visits
                a = zeros(1, 5); b = -1;
                return;
            end
            t = obj.hood.t;
            lim = obj.missionEnd - t;
            bk = obj.activeBooking(agent);
            if ~isempty(bk)
                col = obj.stationColumn(bk.station);
                slack = bk.start - t - obj.messageDelay - obj.marginT;
                lim = min(lim, slack - obj.routes.T(j, col));
                % and the station stays reachable from i after the dwell there (route times are
                % along least-energy routes, so T(i) <= rho_ij + T(j) need not hold)
                a = [1 1 0 0 0]; b = slack - obj.routes.T(i, col);
                if isKey(obj.rows, j)
                    rw = obj.rows(j); rw.limJ = lim; rw.limI = b; obj.rows(j) = rw;
                end
            end
            if isfinite(lim), a = [a; ones(1, 5)]; b = [b; lim]; end
        end

        function J = planObjective(obj, d, P)
            % The energy-aware objective of plans P (see the class help)
            n = size(P, 1);
            if size(P, 2) < 5, rho = d.rho * ones(n, 1); else, rho = P(:, 5); end
            w = sum(P(:, 1:4), 2) + rho;
            dd = d;
            if obj.terminal ~= "alpha", dd.w = [1 1]; end
            Jint = EDRHCPolicy.planCost(dd, [P(:, 1:4), rho]) .* w;
            Eex = obj.ctx.E(rho);
            Eex = Eex(:);
            dh = 0;
            if obj.terminal == "hhat"
                dh = obj.terminalTargets(d, P(:, 1:4), rho, w);
            end
            switch obj.objectiveForm
                case "alphaH"
                    J = (Jint + dh) ./ w + obj.alphaH * Eex;
                otherwise
                    N = Jint + obj.omega * Eex + dh + obj.hood.muE * (Eex + obj.ctx.D - obj.hood.Di);   % hhat_E
                    if obj.objectiveForm == "ratio"
                        J = N ./ w;
                    else
                        J = N - obj.lambdaNow * w;
                    end
            end
        end

        function cmd = depart(obj, agent, model, adj, i, simTime)
            % RHCP3, or the way to the station when no uncovered neighbor is affordable; the
            % agent follows the planned profile within its own limits
            [affordable, uncovered] = obj.canAffordNeighbor(agent, model, adj, i, simTime);
            if ~isempty(obj.activeBooking(agent)) && uncovered && ~affordable
                % Uncovered neighbors, none affordable: go now if an earlier free slot can be
                % reached, else keep servicing i until the latest departure (idling here costs what
                % waiting there does). With every neighbor covered, ED-RHC's hold and retry applies
                obj.stats.noAffordable = obj.stats.noAffordable + 1;
                b = obj.bookings(agent.index);
                latest = b.start - obj.routes.T(i, obj.stationColumn(b.station)) - obj.messageDelay - obj.marginT;
                full = agent.battery.soc >= 0.99;   % nothing to charge: no earlier slot to ask for
                if (~full && obj.requestEarlier(agent, model, i, simTime)) || simTime >= latest - 1e-9
                    cmd = obj.goStation(agent, model, i, simTime);
                else
                    cmd = struct();
                    obj.wantsEarlier(agent.index) = true;   % first in line for freed time
                    obj.note(3, agent, i, simTime);
                    obj.dwell(agent, model, adj, i, "inactive", latest - simTime, true);
                end
                return;
            end
            cmd = depart@EDRHCPolicy(obj, agent, model, adj, i, simTime);
            if isfield(cmd, 'targetIdx') && obj.transit ~= "FO-0" && isprop(agent, 'transitLimits')
                pts = model.findEdge(i, cmd.targetIdx).curvePoints;
                agent.transitLimits = [agent.segmentSpeedLimit(pts), agent.state.maxAccel];
                obj.stats.transits(end+1, :) = [agent.transitTime, agent.limitTransit(pts)];
            end
            if isfield(cmd, 'targetIdx')
                obj.noteDeparture(agent, model, i, simTime);
                obj.note(8, agent, i, simTime);
            end
        end

        function covered = coveredNeighbors(obj, agent, model, adj, i)
            % As in ED-RHC, but agents on their way to, waiting at or charging at a station cover
            % only that station
            nbrs = find(adj(i, :));
            covered = false(size(nbrs));
            for b = 1:numel(model.agents)
                other = model.agents(b);
                if other == agent, continue; end
                k = other.current_target_idx;
                if obj.modeOf(other) ~= "monitor" && ~isa(model.targets(k), 'ChargingStation'), continue; end
                covered = covered | nbrs == k;
            end
        end
    end

    methods (Access = private)
        function clearState(obj)
            obj.modes = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.bookings = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.released = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.lambdas = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.lastNode = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.lastCharge = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.lastStep = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.muMean = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.offers = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.wantsEarlier = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.freed = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.lastOfferCheck = -Inf;
            obj.slotLens = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.modeTime = zeros(0, 4);
            obj.homes = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.grids = containers.Map('KeyType', 'double', 'ValueType', 'any');
            if isempty(obj.tables), obj.tables = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
            obj.routes = [];
            obj.marginT = obj.marginTime; obj.marginE = obj.marginEnergy;
            obj.charges = 0; obj.windowMisses = 0;
            obj.stats = struct('rhcp', 0, 'energyBinds', 0, 'timeBinds', 0, 'noAffordable', 0, ...
                'transits', zeros(0, 2), 'none', zeros(1, 5), ...  % none: covered, energy, time, cost, kind 1-2
                'offers', 0, 'accepted', 0, ...                     % pull-forward offers sent and accepted
                'movedLater', 0, 'foreign', 0, ...                  % "b" moves after commits; grants elsewhere
                'bookingChanges', 0, ...                            % bookings made, moved and released
                'messages', 0, 'relays', 0);                        % end-to-end messages; their relay hops
            obj.rows = containers.Map('KeyType', 'double', 'ValueType', 'any');
            obj.sessions = obj.sessions([]);
            obj.initialized = false;
        end

        function ensureInit(obj, model, simTime)
            % At the first decision of a run: route tables, home stations, slot grids, the
            % targets' unattended-time history, and every agent's first booking
            if obj.initialized, return; end
            obj.initialized = true;
            nT = numel(model.targets);
            obj.gapSum = zeros(1, nT); obj.gapCount = zeros(1, nT); obj.lastLeft = nan(1, nT);
            starts = [model.agents.current_target_idx];
            obj.lastLeft(~ismember(1:nT, starts)) = simTime;
            adj = model.buildAdjacency();
            speed = mean(arrayfun(@(a) a.state.maxSpeed, model.agents));
            obj.gapInit = zeros(1, nT);
            for k = 1:nT
                nb = find(adj(:, k) | adj(k, :)');
                y = arrayfun(@(j) sum(vecnorm(diff(model.findEdge(j, k).curvePoints, 1, 1), 2, 2)), nb);
                if isempty(y), y = 0; end
                obj.gapInit(k) = max(y) / speed;
            end
            if ~any(arrayfun(@(t) isa(t, 'ChargingStation'), model.targets)), return; end
            ref = model.agents(1);
            obj.routes = RouteTable(model, ref);
            obj.hopsTo = zeros(nT, numel(obj.routes.stations));
            for k = 1:nT
                for c = 1:numel(obj.routes.stations)
                    obj.hopsTo(k, c) = numel(obj.routes.route(k, obj.routes.stations(c))) - 1;
                end
            end
            hops = max(obj.hopsTo(:));
            obj.marginT = obj.marginTime + (1 + hops) * obj.stepSize;
            obj.marginE = obj.marginEnergy + (1 + hops) * obj.stepEnergy * obj.stepSize;
            [home, ~, ~] = obj.routes.homeStations(model, starts, ref);
            for k = 1:numel(model.agents)
                obj.homes(model.agents(k).index) = home(k);
            end
            for s = obj.routes.stations
                st = model.targets(s);
                m = st.docks; if ~isfinite(m), m = numel(model.agents); end
                K = 4;
                if isfinite(obj.missionEnd), K = ceil(obj.missionEnd / st.slotLength(ref, obj.socMin)) + 2; end
                obj.grids(s) = zeros(m, K);
                obj.slotLens(s) = st.slotLength(ref, obj.socMin);
            end
            for k = 1:numel(model.agents)
                a = model.agents(k);
                a.chargeAllowed = false;   % agents charge only in their slots (startCharging)
                obj.lastCharge(a.index) = [simTime, a.battery.remaining];
                obj.book(a, model, a.current_target_idx, simTime, simTime, true);
            end
        end

        function book(obj, agent, model, i, t, tFree, initial)
            % A booking at the agent's home station in its window [e_a, l_a] seen from node i at
            % time t, with the agent free to leave at tFree: strategy "b" the latest free
            % dock-slot with start <= l_a, "a" the free one nearest the predicted need. Lemma 4
            % guarantees one after a charge; at the start (initial) none is an error
            if nargin < 7, initial = false; end
            s = obj.homes(agent.index);
            col = obj.stationColumn(s);
            st = model.targets(s);
            ts = st.slotLength(agent, obj.socMin);
            bat = agent.battery;
            e = tFree + obj.routes.T(i, col) + obj.messageDelay + obj.marginT;
            l = t + (bat.remaining - obj.socMin * bat.capacity - obj.routes.D(i, col) - obj.marginE) / bat.idleDraw();
            G = obj.grids(s);
            kLo = max(ceil(e / ts - 1e-9), 0); kHi = floor(l / ts + 1e-9);
            if kHi + 1 > size(G, 2), G(:, end+1:kHi + 1) = 0; end
            ks = kHi:-1:kLo;                    % "b": the latest first
            if obj.bookingStrategy == "a"       % "a": nearest the need at the mean power since the last charge
                need = t + (bat.remaining - obj.socMin * bat.capacity - obj.routes.D(i, col)) / obj.meanPower(agent, t);
                [~, order] = sort(abs(ks * ts - need));
                ks = ks(order);
            end
            pick = [];
            for k = ks
                d = find(G(:, k + 1) == 0, 1);
                if ~isempty(d), pick = [d, k]; break; end
            end
            if isempty(pick) && initial
                error('EnergyAwareEDRHCPolicy:NoSlot', ['No free slot at station %d in agent %d''s ' ...
                    'initial window [%.0f, %.0f] s: check Lemma 4 (P_c >= ceil(n/m) P_I) and the batteries'], ...
                    s, agent.index, e, l);
            end
            if isempty(pick)                    % none in the window: the earliest free one after e
                obj.windowMisses = obj.windowMisses + 1;
                k = max(kLo, 0);
                while true
                    if k + 1 > size(G, 2), G(:, end+1) = 0; end %#ok<AGROW>
                    d = find(G(:, k + 1) == 0, 1);
                    if ~isempty(d), pick = [d, k]; break; end
                    k = k + 1;
                end
            end
            G(pick(1), pick(2) + 1) = agent.index;
            obj.grids(s) = G;
            obj.bookings(agent.index) = struct('station', s, 'dock', pick(1), 'slot', pick(2), ...
                'start', pick(2) * ts, 'length', ts, 'arrive', NaN);
            obj.released(agent.index) = false;
            obj.message(2, i, s);   % request and grant
            obj.stats.bookingChanges = obj.stats.bookingChanges + 1;
        end

        function moved = requestEarlier(obj, agent, model, i, t)
            % D2, step 1: asks the home station (and with otherStations the others) for the
            % earliest free slot the agent can reach from i, and takes the earliest one that beats
            % its booking, the lower route energy on a tie (true if it moved). Another station
            % grants only if the agent's energy covers the route and idling until the slot, and if
            % its own agents keep Lemma 4's condition with the visitor; the home booking is then
            % released
            moved = false;
            b = obj.bookings(agent.index);
            bat = agent.battery;
            best = [b.start, -Inf, NaN, NaN, NaN];  % start, D, station, dock, slot (no tie with the booking)
            for s = obj.routes.stations
                if s ~= b.station && ~obj.otherStations, continue; end
                obj.message(2, i, s);                % request and reply
                if s ~= b.station && ~obj.admits(model, s, agent), continue; end
                col = obj.stationColumn(s);
                G = obj.grids(s); len = obj.slotLens(s);
                D = obj.routes.D(i, col);
                e = t + obj.routes.T(i, col) + obj.messageDelay + obj.marginT;
                for k = max(ceil(e / len - 1e-9), 0):size(G, 2) - 1
                    g = k * len;
                    if g > best(1) + 1e-9, break; end
                    d = find(G(:, k + 1) == 0, 1);
                    if isempty(d), continue; end
                    if s ~= b.station && bat.remaining - obj.socMin * bat.capacity - obj.marginE - D - ...
                            bat.idleDraw() * (g - t) < 0
                        break;                       % later slots need even more energy
                    end
                    if g < best(1) - 1e-9 || (abs(g - best(1)) <= 1e-9 && D < best(2))
                        best = [g, D, s, d, k];
                    end
                    break;
                end
            end
            if isnan(best(3)), return; end
            s = best(3);
            if s ~= b.station
                obj.unbook(agent);
                obj.message(1, i, b.station);        % releases the home booking
                obj.stats.foreign = obj.stats.foreign + 1;
                b.station = s; b.length = obj.slotLens(s);
            end
            G = obj.grids(s);
            G(G == agent.index) = 0; G(best(4), best(5) + 1) = agent.index;
            obj.grids(s) = G;
            b.dock = best(4); b.slot = best(5); b.start = best(1);
            obj.bookings(agent.index) = b;
            obj.stats.bookingChanges = obj.stats.bookingChanges + 1;
            moved = true;
        end

        function ok = admits(obj, model, s, agent)
            % Station s takes one more agent if Lemma 4's condition P_c >= ceil(n/m) P_I still
            % holds for its own agents and its visitors
            st = model.targets(s);
            n = sum(cell2mat(values(obj.homes)) == s) + 1;
            for a = model.agents(:)'
                b = obj.activeBooking(a);
                if a.index ~= agent.index && ~isempty(b) && b.station == s && obj.homes(a.index) ~= s
                    n = n + 1;
                end
            end
            ok = ~isfinite(st.docks) || st.chargePower >= ceil(n / st.docks) * agent.battery.idleDraw();
        end

        function message(obj, n, node, s)
            % n messages between an agent at node and station s, relayed along the route table
            obj.stats.messages = obj.stats.messages + n;
            obj.stats.relays = obj.stats.relays + n * obj.hopsTo(node, obj.stationColumn(s));
        end

        function moveLater(obj, agent, model, i, best)
            % Strategy "b" after a commit: the booking moves to the latest free slot that keeps the
            % committed plan to best.j feasible, i.e. whose start is at most where the energy row
            % for the plan's E^ex binds (the time rows only bound the start from below)
            if obj.bookingStrategy ~= "b" || isempty(best.j), return; end
            b = obj.activeBooking(agent);
            if isempty(b), return; end
            obj.message(2, i, b.station);   % request and reply
            tab = obj.energyTable(agent, model, i, best.j, model.findEdge(i, best.j).curvePoints);
            budget = obj.energyBudget(agent, best.j);
            latest = b.start + (budget - tab.E(best.rho)) / agent.battery.idleDraw();
            G = obj.grids(b.station);
            kHi = floor(latest / b.length + 1e-9);
            if kHi + 1 > size(G, 2), G(:, end+1:kHi + 1) = 0; end
            for k = kHi:-1:b.slot + 1
                d = find(G(:, k + 1) == 0, 1);
                if ~isempty(d)
                    G(G == agent.index) = 0; G(d, k + 1) = agent.index;
                    obj.grids(b.station) = G;
                    b.dock = d; b.slot = k; b.start = k * b.length;
                    obj.bookings(agent.index) = b;
                    obj.stats.movedLater = obj.stats.movedLater + 1;
                    obj.stats.bookingChanges = obj.stats.bookingChanges + 1;
                    return;
                end
            end
        end

        function unbook(obj, agent)
            % Frees the agent's slot, and the earlier ones an early session held (startCharging)
            if ~isKey(obj.bookings, agent.index), return; end
            b = obj.bookings(agent.index);
            G = obj.grids(b.station);
            G(G == agent.index) = 0;
            obj.grids(b.station) = G;
            remove(obj.bookings, agent.index);
        end

        function b = activeBooking(obj, agent)
            b = [];
            if isKey(obj.bookings, agent.index) && ~(isKey(obj.released, agent.index) && obj.released(agent.index))
                b = obj.bookings(agent.index);
            end
        end

        function c = stationColumn(obj, s)
            c = find(obj.routes.stations == s, 1);
        end

        function [budget, D] = energyBudget(obj, agent, j)
            % The energy row's budget for E^ex_ij: E - E_min - D(j, sigma) - P_I (t_a - t), or
            % with no booking E - E_min - P_I (T - t) (none without a mission end or stations)
            bat = agent.battery;
            t = obj.hood.t;
            bk = obj.activeBooking(agent);
            D = 0;
            if ~isempty(bk)
                D = obj.routes.D(j, obj.stationColumn(bk.station));
                budget = bat.remaining - obj.socMin * bat.capacity - obj.marginE - D - bat.idleDraw() * (bk.start - t);
            elseif ~isempty(obj.routes) && isfinite(obj.missionEnd)
                budget = bat.remaining - obj.socMin * bat.capacity - obj.marginE - bat.idleDraw() * (obj.missionEnd - t);
                budget = max(budget, 0);   % released: keep idling possible, never strand the agent
            else
                budget = Inf;
            end
        end

        function tab = energyTable(obj, agent, model, i, j, pts)
            % E^ex of the transit i -> j against rho, tabulated from the agent's least-loss
            % profile (Agent.transitPlanEnergy) and interpolated (pchip in log rho); Inf below
            % the shortest transit. Cached per edge, agent model and transit profile
            if norm(pts(1, :) - model.targets(i).position) > norm(pts(end, :) - model.targets(i).position)
                pts = flipud(pts);
            end
            b = agent.battery;
            key = sprintf('%d>%d|%d|%.6g,%.6g|%s|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g|%.10g', ...
                i, j, size(pts, 1), pts(end, 1), pts(end, 2), obj.transit, agent.state.maxSpeed, agent.state.maxAccel, ...
                agent.maxNormalAccel, b.c1, b.c2, b.c3, b.movePower(), b.idlePower(), b.beta, obj.H);
            if isKey(obj.tables, key), tab = obj.tables(key); return; end
            lo = agent.limitTransit(pts);
            if obj.transit == "FO-0", lo = sum(vecnorm(diff(pts, 1, 1), 2, 2)) / agent.state.maxSpeed; end
            hi = max(obj.H, 2 * lo);
            r = lo * (hi / lo).^linspace(0, 1, 48);
            E = zeros(size(r));
            for k = 1:numel(r), [~, E(k)] = agent.transitPlanEnergy(pts, r(k), obj.transit); end
            F = griddedInterpolant(log(r), E, 'pchip', 'nearest');
            tab = struct('rhoMin', lo, 'E', @(q) EnergyAwareEDRHCPolicy.lookup(F, lo, q));
            obj.tables(key) = tab;
        end

        function prepareHood(obj, agent, model, adj, i, simTime)
            % The neighborhood's data for the objective (as EDRHCPolicy.rhcp builds it): states,
            % unattended times, clearing rates, hhat at X(t), and mu_a for hhat_E
            nbrs = find(adj(i, :));
            avail = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            hd = [i, avail];
            tg = model.targets(hd);
            R = [tg.R]; R(R < 1e-6) = 0;
            A = arrayfun(@(t) t.growthRateAt(simTime, model.uncertaintyMode), tg);
            B = [tg.B];
            gap = arrayfun(@(k) obj.gapOf(k), hd);
            nLow = max(1, floor(A ./ max(B, eps)) + 1);
            c = nLow .* B - A;
            lam = A .* gap / 2;
            s = obj.missionEnd - simTime;
            h0 = CollabTargetCost.terminal(R, A, c, gap, s, lam);
            dl = 1e-4 * max(1, gap);
            mu = max(sum((CollabTargetCost.terminal(R, A, c, gap + dl, s, lam) - h0) ./ dl, 'omitnan'), 0);
            h0 = sum(h0, 'omitnan');
            bk = obj.activeBooking(agent);
            muE = 0; Di = 0;
            if ~isempty(bk)
                col = obj.stationColumn(bk.station);
                Di = obj.routes.D(i, col);
                rate = model.targets(bk.station).chargePower - agent.battery.idleDraw();
                if obj.priceForm == "renewal", rate = rate + obj.meanPower(agent, simTime); end
                if obj.priceAveraging   % running mean of mu_a over the agent's events
                    m = [0 0];
                    if isKey(obj.muMean, agent.index), m = obj.muMean(agent.index); end
                    m = [(m(1) * m(2) + mu) / (m(2) + 1), m(2) + 1];
                    obj.muMean(agent.index) = m;
                    mu = m(1);
                end
                muE = obj.priceScale * mu / rate;
            end
            obj.hood = struct('t', simTime, 'nodes', hd, 'gap', gap, 'c', c, 'lam', lam, ...
                'h0', h0, 'mu', mu, 'muE', muE, 'Di', Di);
        end

        function dh = terminalTargets(obj, d, P, rho, w)
            % sum over the neighborhood of hhat_k(X(t_f)) - hhat_k(X(t)) for plans P (rows)
            h = obj.hood;
            ui = P(:, 1); vi = P(:, 2); uj = P(:, 3);
            j = d.j;
            K = numel(d.R);
            Rf = d.R + d.A .* w;                                   % others grow over the horizon
            Ri1 = max(d.R(1) - (d.B(1) - d.A(1)) * ui, 0);
            Rf(:, 1) = Ri1 + d.A(1) * (rho + uj + P(:, 4));        % i after the agent left
            Rj1 = d.R(j) + d.A(j) * (ui + vi + rho);
            Rf(:, j) = max(Rj1 - (d.B(j) - d.A(j)) * uj, 0);       % j after its dwell
            s = obj.missionEnd - (h.t + w);
            hf = CollabTargetCost.terminal(Rf, repmat(d.A, size(Rf, 1), 1), repmat(h.c, size(Rf, 1), 1), ...
                repmat(h.gap, size(Rf, 1), 1), repmat(s, 1, K), repmat(h.lam, size(Rf, 1), 1));
            dh = sum(hf, 2, 'omitnan') - h.h0;
        end

        function offerFreedTime(obj, model, t)
            % Station pull-forward: time freed on a dock goes to the first booked agent in line that
            % can use it (one that asked for an earlier slot first, then the earliest booking): at
            % most 1 - pullMinCharge full, monitoring, able to reach the dock before the freed time
            % ends, and with the dock free from now through its own slot. One offer per dock at a
            % time; the freed time lapses at its end or once an agent takes it
            for key = cell2mat(keys(obj.freed))
                until = obj.freed(key);
                s = floor(key / 1000); d = key - 1000 * s;
                if t >= until, remove(obj.freed, key); continue; end
                if obj.dockBusy(s, d, t), continue; end
                if any(cellfun(@(f) f.station == s && f.dock == d, values(obj.offers))), continue; end
                G = obj.grids(s);
                col = obj.stationColumn(s);
                cands = []; order = [];
                for a = model.agents(:)'
                    b = obj.activeBooking(a);
                    if isempty(b) || b.station ~= s || obj.modeOf(a) ~= "monitor", continue; end
                    if isKey(obj.offers, a.index), continue; end
                    if a.battery.soc > 1 - obj.pullMinCharge, continue; end   % not worth a session yet
                    arrive = t + obj.routes.T(a.current_target_idx, col) + obj.messageDelay + obj.marginT;
                    if arrive >= min(until, b.start), continue; end
                    if ~obj.dockFreeUntil(G, d, t, b, a.index), continue; end
                    cands(end+1) = a.index; %#ok<AGROW>
                    order(end+1) = b.start - 1e6 * isKey(obj.wantsEarlier, a.index); %#ok<AGROW>
                end
                if isempty(cands), continue; end
                [~, o] = min(order);
                obj.offers(cands(o)) = struct('station', s, 'dock', d, 'until', until);
                obj.stats.offers = obj.stats.offers + 1;
                obj.message(1, model.agents([model.agents.index] == cands(o)).current_target_idx, s);
            end
        end

        function freeTime(obj, s, d, t, until)
            % A dock's time from t to until is freed: offer it (pullForward)
            if obj.pullForward && until > t + 1e-6
                obj.freed(1000 * s + d) = until;
            end
        end

        function busy = dockBusy(obj, s, d, t)
            % Is dock d of station s taken now: its current slot booked, held by a session or
            % claimed by an agent on its way (startCharging, acceptOffer)?
            G = obj.grids(s);
            k = max(floor(t / obj.slotLens(s) + 1e-9), 0) + 1;
            busy = k <= size(G, 2) && G(d, k) ~= 0;
        end

        function ok = dockFreeUntil(obj, G, d, t, b, self)
            % Could the agent with booking b use dock d from now until its own slot: no other
            % booking on d from now through b's slot (b's own slot on d free or its own)?
            k = max(floor(t / b.length + 1e-9), 0) + 1;
            if b.slot + 1 > size(G, 2), ok = false; return; end
            cells = G(d, k:b.slot + 1);
            ok = all(cells == 0 | cells == self);
        end

        function ok = acceptOffer(obj, agent, ~, i, t)
            % At an event: take offered freed time if the agent can still reach the dock in it and
            % the dock is still free until its booking; the booking moves to that dock (same slot)
            % and claims it from now on, so no later booking lands between
            ok = false;
            if ~obj.pullForward || ~isKey(obj.offers, agent.index), return; end
            f = obj.offers(agent.index); remove(obj.offers, agent.index);
            obj.message(1, i, f.station);   % accepts or declines
            b = obj.activeBooking(agent);
            if isempty(b) || b.station ~= f.station, return; end
            col = obj.stationColumn(b.station);
            if t + obj.routes.T(i, col) + obj.messageDelay + obj.marginT >= min(f.until, b.start), return; end
            G = obj.grids(b.station);
            if ~obj.dockFreeUntil(G, f.dock, t, b, agent.index), return; end
            G(G == agent.index) = 0;
            kNow = max(floor(t / b.length + 1e-9), 0);
            G(f.dock, kNow + 1:b.slot + 1) = agent.index;
            obj.grids(b.station) = G;
            b.dock = f.dock; obj.bookings(agent.index) = b;
            obj.stats.bookingChanges = obj.stats.bookingChanges + 1;
            key = 1000 * b.station + f.dock;
            if isKey(obj.freed, key), remove(obj.freed, key); end
            obj.stats.accepted = obj.stats.accepted + 1;
            ok = true;
        end

        function tryEarlyStart(obj, agent, model, t)
            % Work-conserving docks: start the booked session now if nobody charges on the
            % agent's dock and no other booking on it lies between now and the agent's slot (the
            % session ends by the agent's own slot's end, so it overlaps no other)
            b = obj.bookings(agent.index);
            G = obj.grids(b.station);
            kNow = max(floor(t / b.length + 1e-9), 0);
            cells = G(b.dock, kNow + 1:b.slot + 1);
            if any(cells ~= 0 & cells ~= agent.index), return; end
            for o = model.agents(:)'
                if o.index ~= agent.index && obj.modeOf(o) == "charging" && isKey(obj.bookings, o.index)
                    ob = obj.bookings(o.index);
                    if ob.station == b.station && ob.dock == b.dock, return; end
                end
            end
            obj.startCharging(agent, model, b.station, t);
        end

        function noteBinding(obj, best)
            % Diagnostics: does the optimum sit on the energy row (rho at an end of its interval
            % that the row, not Lemma 1 or H, sets) or on a time row (w or u_i + v_i at its limit)?
            obj.stats.rhcp = obj.stats.rhcp + 1;
            if isempty(best.j) || ~isKey(obj.rows, best.j), return; end
            rw = obj.rows(best.j); tol = 1e-3;
            w = sum(best.x) + best.rho;
            if (rw.rho(1) > rw.lo * (1 + tol) && best.rho <= rw.rho(1) * (1 + tol)) || ...
                    (rw.rho(2) < rw.hi * (1 - tol) && best.rho >= rw.rho(2) * (1 - tol))
                obj.stats.energyBinds = obj.stats.energyBinds + 1;
            end
            if w >= rw.limJ - tol * max(1, rw.limJ) || sum(best.x(1:2)) >= rw.limI - tol * max(1, rw.limI)
                obj.stats.timeBinds = obj.stats.timeBinds + 1;
            end
        end

        function noteNone(obj, best, kind, agent, model, adj, i)
            % Diagnostics: why an RHCP found no next target (stats.none: every neighbor covered;
            % an uncovered one excluded by the energy row; by a time row; by its cost (none finite);
            % RHCP1 or RHCP2)
            if ~isempty(best.j), return; end
            if kind < 3, obj.stats.none(5) = obj.stats.none(5) + 1; return; end
            nbrs = find(adj(i, :));
            nbrs = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            nbrs = nbrs(~arrayfun(@(j) isa(model.targets(j), 'ChargingStation'), nbrs));
            if isempty(nbrs), c = 1;
            elseif ~any(arrayfun(@(j) isKey(obj.rows, j), nbrs)), c = 2;
            elseif all(arrayfun(@(j) ~isKey(obj.rows, j) || obj.rows(j).limJ < obj.rows(j).rho(1) || obj.rows(j).limI < 0, nbrs)), c = 3;
            else, c = 4;
            end
            obj.stats.none(c) = obj.stats.none(c) + 1;
        end

        function P = meanPower(obj, agent, t)
            % The agent's mean power since its last charge (an estimate before 60 s of data)
            bat = agent.battery;
            P = bat.idleDraw() + 0.5 * (bat.movePower() - bat.idlePower() + bat.c3 * agent.state.maxSpeed);
            if isKey(obj.lastCharge, agent.index)
                lc = obj.lastCharge(agent.index);
                if t - lc(1) > 60, P = max(bat.idleDraw(), (lc(2) - bat.remaining) / (t - lc(1))); end
            end
        end

        function note(obj, event, agent, i, t)
            if ~obj.traceEvents, return; end
            b = obj.activeBooking(agent); start = NaN;
            if ~isempty(b), start = b.start; end
            obj.trace(end+1, :) = [t, agent.index, event, i, agent.battery.remaining, start];
        end

        function g = gapOf(obj, k)
            if obj.gapCount(k) > 0, g = obj.gapSum(k) / obj.gapCount(k); else, g = obj.gapInit(k); end
        end

        function noteArrival(obj, agent, model, i, t)
            % Unattended-time history: a target attended again after being left alone
            if isKey(obj.lastNode, agent.index) && obj.lastNode(agent.index) == i, return; end
            obj.lastNode(agent.index) = i;
            others = model.agents([model.agents.index] ~= agent.index);   % (no ~= on a mixed-type array)
            alone = ~any(arrayfun(@(o) isempty(o.path) && o.current_target_idx == i, others));
            if alone && ~isnan(obj.lastLeft(i))
                obj.gapSum(i) = obj.gapSum(i) + (t - obj.lastLeft(i));
                obj.gapCount(i) = obj.gapCount(i) + 1;
            end
        end

        function noteDeparture(obj, agent, model, i, t)
            others = model.agents([model.agents.index] ~= agent.index);   % (no ~= on a mixed-type array)
            if ~any(arrayfun(@(o) isempty(o.path) && o.current_target_idx == i, others))
                obj.lastLeft(i) = t;
            end
        end

        function maybeRelease(obj, agent, t)
            % End of the mission: release the booking once the energy left covers idling until T
            % plus the excess still to come (Sec. 6.8), estimated at the agent's mean power since
            % its last charge; covering idling alone would leave it no energy to move
            bk = obj.activeBooking(agent);
            if isempty(bk) || ~isfinite(obj.missionEnd), return; end
            bat = agent.battery;
            if bat.remaining - obj.socMin * bat.capacity - obj.marginE >= ...
                    max(obj.meanPower(agent, t), bat.idleDraw()) * (obj.missionEnd - t)
                if bk.start <= t, obj.freeTime(bk.station, bk.dock, t, bk.start + bk.length); end
                obj.unbook(agent);
                obj.message(1, agent.current_target_idx, bk.station);
                obj.stats.bookingChanges = obj.stats.bookingChanges + 1;
                obj.released(agent.index) = true;
                obj.note(7, agent, NaN, t);
            end
        end

        function tf = mustLeave(obj, agent, i, t)
            % The latest departure for the booked slot (or no energy for any plan left)
            bk = obj.activeBooking(agent);
            tf = false;
            if isempty(bk), return; end
            col = obj.stationColumn(bk.station);
            tf = t >= bk.start - obj.routes.T(i, col) - obj.messageDelay - obj.marginT - 1e-9;
        end

        function [ok, uncovered] = canAffordNeighbor(obj, agent, model, adj, i, t)
            % Is any uncovered monitoring neighbor reachable by the shortest plan (no dwells)
            % within both booking rows (ok)? Is there any uncovered monitoring neighbor at all?
            ok = false;
            nbrs = find(adj(i, :));
            nbrs = nbrs(~obj.coveredNeighbors(agent, model, adj, i));
            nbrs = nbrs(~arrayfun(@(j) isa(model.targets(j), 'ChargingStation'), nbrs));
            uncovered = ~isempty(nbrs);
            bk = obj.activeBooking(agent);
            if isempty(bk), ok = uncovered; return; end
            col = obj.stationColumn(bk.station);
            obj.hood.t = t;
            for j = nbrs
                pts = model.findEdge(i, j).curvePoints;
                rr = obj.transitRange(agent, model, i, j, pts, [agent.limitTransit(pts), obj.H]);
                if isempty(rr), continue; end
                if t + rr(1) + obj.routes.T(j, col) + obj.messageDelay + obj.marginT <= bk.start
                    ok = true; return;
                end
            end
        end

        function cmd = goStation(obj, agent, model, i, t)
            % Toward the booked station (on leaving the targets, first an earlier free slot);
            % there, wait (not charging) until the slot, or charge
            cmd = struct();
            if obj.modeOf(agent) == "monitor", obj.note(2, agent, i, t); end
            obj.modes(agent.index) = "toStation";
            if isKey(obj.offers, agent.index), remove(obj.offers, agent.index); end
            if isKey(obj.wantsEarlier, agent.index), remove(obj.wantsEarlier, agent.index); end
            obj.forgetAgent(agent.index);
            b = obj.bookings(agent.index);
            if i ~= b.station
                cmd = obj.hop(agent, model, i, t);
                return;
            end
            if isnan(b.arrive), b.arrive = t; obj.bookings(agent.index) = b; end
            if t < b.start - 1e-9
                obj.note(4, agent, i, t);
                obj.modes(agent.index) = "waiting";
                agent.chargeAllowed = false;
                agent.dwellRemaining = b.start - t;
            else
                obj.startCharging(agent, model, i, t);
            end
        end

        function cmd = hop(obj, agent, model, i, t)
            % One edge of the least-energy route to the booked station, at its energy-optimal
            % transit time, covering nothing on the way
            b = obj.bookings(agent.index);
            col = obj.stationColumn(b.station);
            nxt = obj.routes.next(i, col);
            cmd = struct('kind', "move", 'targetIdx', nxt);
            pts = model.findEdge(i, nxt).curvePoints;
            agent.transitTime = obj.routes.edgeTime(i, nxt);
            agent.transitProfile = obj.transit;
            agent.transitLimits = [agent.segmentSpeedLimit(pts), agent.state.maxAccel];
            obj.noteDeparture(agent, model, i, t);
        end

        function startCharging(obj, agent, model, i, t)
            % The session holds its dock from now through its slot, so no later booking lands
            % between (an early start ends by its own slot's end)
            b = obj.bookings(agent.index);
            G = obj.grids(b.station);
            kNow = min(max(floor(t / b.length + 1e-9), 0), b.slot);
            G(b.dock, kNow + 1:b.slot + 1) = agent.index;
            obj.grids(b.station) = G;
            obj.note(5, agent, i, t);
            obj.modes(agent.index) = "charging";
            agent.chargeAllowed = true;
            obj.charges = obj.charges + 1;
            st = model.targets(i);
            toFull = agent.battery.energyUsed / max(st.chargePower - agent.battery.idleDraw(), eps);
            agent.dwellRemaining = max(min(b.start + b.length - t, toFull), 1e-3);
            obj.sessions(end+1) = struct('agent', agent.index, 'station', i, 'dock', b.dock, ...
                'slot', b.start, 'slotEnd', b.start + b.length, 'arrive', b.arrive, 'start', t, 'stop', NaN, ...
                'Ein', agent.battery.remaining, 'Eout', NaN);
        end

        function finishCharging(obj, agent, model, i, t)
            k = find([obj.sessions.agent] == agent.index & isnan([obj.sessions.stop]), 1, 'last');
            if ~isempty(k)
                obj.sessions(k).stop = t; obj.sessions(k).Eout = agent.battery.remaining;
                obj.freeTime(i, obj.sessions(k).dock, t, obj.sessions(k).slotEnd);   % full before the slot's end
            end
            obj.unbook(agent);
            obj.modes(agent.index) = "monitor";
            agent.chargeAllowed = false;
            obj.lastCharge(agent.index) = [t, agent.battery.remaining];
            obj.book(agent, model, i, t, t);
            obj.note(6, agent, i, t);
        end
    end

    methods (Static, Access = private)
        function E = lookup(F, lo, q)
            E = F(log(max(q, lo)));
            E(q < lo * (1 - 1e-9)) = Inf;
        end
    end
end
