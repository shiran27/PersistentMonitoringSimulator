classdef ScenarioModel < handle
    properties
        agents = []
        targets = Target.empty
        edges = Edge.empty
        walls = []
        cumUncertaintyIntegral = 0
        lastLogTime = 0
        lastUncertainty = 0
        hasLastSample = false
        policyMap               % name -> persistent policy instance (experiments read stats off these)
        activePolicy = 'Random Walk'
        uncertaintyMode = 'Static Uniform'      % constant A_i, as in the paper; see UncertaintyModes
        pathPlanner
        agentFactory
        targetFactory           % @(idx, pos) target: the registered type targetType
        targetType = "Uncertainty Target"   % target type (ComponentRegistry name) of the targets
        metrics = {}            % Metric objects updated every step; the first is the objective
                                % plotted in the GUI (default: AverageUncertainty, J(t))
        bounds = [0 100 0 100]  % [xMin xMax yMin yMax]
        collisions = []         % per agent: steps in which its motion crossed a wall (it was then
                                % stopped at the wall); planned paths avoid walls, so this stays 0
                                % unless an agent leaves its path
        detectionRadius = 1.2   % an agent at rest (not in transit) this close to a target services it
        agentSpec = AgentSpec() % spec (settings) of the agents added next; the type is agentFactory
        background = ""         % background image of the layout (file name; see backgroundFile)
        registry                % ComponentRegistry (policy assumptions)
    end

    properties (Constant)
        UncertaintyModes = {'Static Uniform', 'Static Priority Target', 'Dynamic Uncertainty'}
    end

    methods (Static)
        function f = backgroundFile(name)
            % Full path of a layout's background image: a file in Configurations/backgrounds/,
            % in the project root, or a full path ("" when there is none)
            f = "";
            if strlength(string(name)) == 0, return; end
            root = fileparts(fileparts(mfilename('fullpath')));
            for c = [fullfile(root, 'Configurations', 'backgrounds', name), fullfile(root, name), string(name)]
                if isfile(c), f = string(c); return; end
            end
        end
    end

    properties (Dependent)
        policy
    end

    methods
        function obj = ScenarioModel(pathPlanner, agentFactory)
            if nargin < 1 || isempty(pathPlanner), pathPlanner = DijkstraCornerPlanner(); end
            if nargin < 2 || isempty(agentFactory)
                agentFactory = @(idx, pos, sp) AbstractAgent(idx, pos, sp);
            end
            obj.pathPlanner  = pathPlanner;
            obj.agentFactory = agentFactory;
            obj.targetFactory = @(idx, pos) Target(idx, pos);
            obj.metrics = {AverageUncertainty()};

            % One instance per registered policy, kept for the model's lifetime
            reg = ComponentRegistry();
            obj.registry = reg;
            obj.policyMap = containers.Map('KeyType','char','ValueType','any');
            names = reg.keysFor('policy');
            for k = 1:numel(names)
                f = reg.factoryFor('policy', names{k});
                obj.policyMap(names{k}) = f();
            end
        end

        function pol = get.policy(obj)
            pol = obj.policyMap(obj.activePolicy);
        end

        function setPathPlanner(obj, planner)
            obj.pathPlanner = planner;
            obj.replanAllEdges();
        end

        function setAgentFactory(obj, factory)
            obj.agentFactory = factory;
        end

        function setAgentType(obj, name)
            % The registered agent type of the agents added next
            obj.agentFactory = obj.registry.factoryFor('agenttype', char(name));
            obj.agentSpec.type = string(name);
        end

        function setTargetType(obj, name)
            % The registered target type of every target: the scenario is rebuilt with targets
            % of that type (same positions, rates, edges, walls and agents)
            obj.targetFactory = obj.registry.factoryFor('targettype', char(name));
            obj.targetType = string(name);
            if ~isempty(obj.targets)
                s = obj.exportLayout();
                obj.importLayout(s);
            end
        end

        function setMetrics(obj, names)
            % The registered metrics to compute, the first one being the objective
            names = string(names);
            obj.metrics = arrayfun(@(n) feval(obj.registry.factoryFor('metric', char(n))), ...
                names, 'UniformOutput', false);
            for k = 1:numel(obj.metrics), obj.metrics{k}.reset(obj); end
        end

        function v = metricValues(obj, simTime)
            % Current value of every metric (struct; field names from the metrics' class names)
            v = struct();
            for k = 1:numel(obj.metrics)
                v.(class(obj.metrics{k})) = obj.metrics{k}.value(obj, simTime);
            end
        end

        function msg = checkAgentFit(obj)
            % "" when the agents suit the active policy, else a note on what to expect
            msg = "";
            if isempty(obj.agents), return; end
            name = string(obj.activePolicy);
            assumption = obj.registry.assumptionOf(name, obj.policy);
            physical = arrayfun(@(a) isa(a, 'PhysicalAgent'), obj.agents);
            shaped = arrayfun(@(a) ~isa(a, 'PhysicalAgent') && a.profile ~= "FO-0", obj.agents);
            switch assumption
                case "FO-0"
                    if any(shaped)
                        msg = name + " plans transits as path length / speed; agents with SO or FO-3 profiles take 1.5x longer than planned.";
                    end
                    if any(physical)
                        msg = strtrim(msg + " " + name + " assumes FO-0 (constant-speed) agents; physical agents lag behind its plans.");
                    end
                case "plans transit"
                    if any(physical)
                        msg = name + " hands each transit's plan to the agents; physical agents track it with some lag.";
                    end
            end
            n = obj.registry.requiredAgents(name);
            if n > 0 && numel(obj.agents) ~= n
                msg = strtrim(msg + " " + sprintf("%s is built for exactly %d agents (this scenario has %d).", name, n, numel(obj.agents)));
            end
        end

        function setUncertaintyMode(obj, mode)
            if ischar(mode) || isstring(mode)
                % Target treats any unknown name as a spiking mode, so reject typos here
                assert(any(strcmp(char(mode), obj.UncertaintyModes)), ...
                    'ScenarioModel:UnknownMode', 'Unknown uncertainty mode "%s"', char(mode));
                obj.uncertaintyMode = char(mode);
            end
        end

        function setActivePolicy(obj, policyName)
            if ischar(policyName) || isstring(policyName)
                obj.activePolicy = char(obj.registry.resolve(policyName));   % older names still work
            end
        end

        function addWall(obj, p1, p2)
            obj.walls = [obj.walls; p1(1), p1(2), p2(1), p2(2)];
        end

        function replanAllEdges(obj)
            for k = 1:numel(obj.edges)
                e = obj.edges(k);
                pStart = e.targets(1).position;
                pEnd = e.targets(2).position;
                e.curvePoints = obj.planShortestPath(pStart, pEnd);
                if ~isempty(e.lineHandle) && isgraphics(e.lineHandle)
                    delete(e.lineHandle);
                    e.lineHandle = [];
                end
            end
        end

        function step(obj, dtSim, simTime)
            if isempty(obj.agents), return; end

            % 1. UPDATE RESIDING AGENTS AT TARGETS FIRST
            % Only agents at rest service a target: arrived (no path) or waiting to leave (a
            % dwell before a new path). Agents in transit service nothing, even when they pass
            % close to a target; detectionRadius is the tolerance for "at the target".
            if ~isempty(obj.targets)
                for t = 1:numel(obj.targets)
                    nearby = [];
                    for a_idx = 1:numel(obj.agents)
                        a = obj.agents(a_idx);
                        if ~isempty(a.path) && a.dwellRemaining <= 0, continue; end   % in transit
                        distToTarget = norm(a.state.pos - obj.targets(t).position);
                        if distToTarget < obj.detectionRadius
                            if isempty(nearby), nearby = a; else, nearby(end+1) = a; end
                        end
                    end
                    obj.targets(t).updateResidingAgents(nearby, simTime);
                end
            end

            % 2. UPDATE TARGET UNCERTAINTIES & REFRESH SPIKE STATES
            % Both modes integrate with dtSim; mode string dictates if A(t) spikes or stays A_base
            if ~isempty(obj.targets)
                for tIdx = 1:numel(obj.targets)
                    obj.targets(tIdx).updateUncertainty(dtSim, simTime, obj.uncertaintyMode);
                end
            end

            % 3. UPDATE AGENT STATES AND EXECUTE POLICIES
            % Event-driven policies may define onStep(agent, model, simTime) to watch for
            % events (e.g. a neighbor becoming covered) and revise a dwell in progress
            watch = ismethod(obj.policy, 'onStep');
            for k = 1:numel(obj.agents)
                a = obj.agents(k);
                if watch, obj.policy.onStep(a, obj, simTime); end
                if a.dwellRemaining <= 0 && isempty(a.path)
                    adj = obj.buildAdjacency();
                    cmd = obj.policy.plan(a, obj, adj, a.current_target_idx, simTime);
                    % Policies return struct() (no 'kind') when the agent has nowhere to go
                    if isfield(cmd, 'kind') && strcmp(cmd.kind, "move")
                        edge = obj.findEdge(a.current_target_idx, cmd.targetIdx);
                        if ~isempty(edge)
                            a.path = edge.curvePoints;
                            if norm(a.path(1,:) - a.state.pos) > norm(a.path(end,:) - a.state.pos)
                                a.path = flipud(a.path);
                            end
                            a.pathIndex = 1;
                            a.current_target_idx = cmd.targetIdx;
                            % A dwell requested in the command (the RHC policies set
                            % a.dwellRemaining themselves) is spent at the current target
                            % before leaving, like the RHC dwells
                            if isfield(cmd, 'dwellSeconds'), a.dwellRemaining = cmd.dwellSeconds; end
                        end
                    end
                end
                p0 = a.state.pos;
                if a.dwellRemaining > 0
                    % Dwelling: hold at the current target before following the new path
                    a.dwellRemaining = max(0, a.dwellRemaining - dtSim);
                    obj.holdAgent(a, dtSim);
                else
                    a.update(dtSim);
                end
                obj.stopAtWalls(k, a, p0);
            end
        end

        function stopAtWalls(obj, k, a, p0)
            % If agent k's motion in this step (p0 to its new position) crossed a wall, it stops
            % just before the wall and the collision is counted
            p1 = a.state.pos;
            if isempty(obj.walls) || isequal(p0, p1), return; end
            W = obj.walls; c = W(:, 1:2); d = W(:, 3:4); v = p1 - p0;
            den = (d(:, 2) - c(:, 2)) * v(1) - (d(:, 1) - c(:, 1)) * v(2);
            ua = ((d(:, 1) - c(:, 1)) .* (p0(2) - c(:, 2)) - (d(:, 2) - c(:, 2)) .* (p0(1) - c(:, 1))) ./ den;
            ub = (v(1) * (p0(2) - c(:, 2)) - v(2) * (p0(1) - c(:, 1))) ./ den;
            hit = abs(den) > 1e-12 & ua > 1e-9 & ua <= 1 & ub >= 0 & ub <= 1;
            if ~any(hit), return; end
            t = min(ua(hit));
            a.state.pos = p0 + max(0, t - 1e-3) * v;
            a.state.vel = [0 0];
            if numel(obj.collisions) < k, obj.collisions(k) = 0; end
            obj.collisions(k) = obj.collisions(k) + 1;
        end

        function holdAgent(~, a, dtSim)
            % An idle update with the path set aside: the agent stays at the target and keeps
            % drawing its idle battery power
            path = a.path;
            a.path = [];
            a.update(dtSim);
            a.path = path;
        end

        function [uNow, JNow] = updateTargetsAndLogObjective(obj, simTime, dtSim)
            % 1. AGGREGATE CURRENT TOTAL UNCERTAINTY ACROSS TARGETS
            uNow = 0;
            for t = 1:numel(obj.targets)
                uNow = uNow + obj.targets(t).R;
            end

            % 2. INTEGRATE CUMULATIVE OBJECTIVE (TRAPEZOIDAL RULE)
            if ~obj.hasLastSample
                obj.lastLogTime = simTime;
                obj.lastUncertainty = uNow;
                obj.hasLastSample = true;
            else
                dt = simTime - obj.lastLogTime;
                if dt > 0
                    obj.cumUncertaintyIntegral = obj.cumUncertaintyIntegral + 0.5 * (obj.lastUncertainty + uNow) * dt;
                    obj.lastLogTime = simTime;
                    obj.lastUncertainty = uNow;
                end
            end

            % 3. UPDATE THE METRICS; THE FIRST ONE (DEFAULT: J(t), THE RUNNING TIME-AVERAGED
            % UNCERTAINTY) IS THE OBJECTIVE
            for k = 1:numel(obj.metrics), obj.metrics{k}.update(obj, simTime, dtSim); end
            JNow = obj.metrics{1}.value(obj, simTime);
        end
        function [e, ok, msg] = addEdgeByTargets(obj, t1Idx, t2Idx)
            e = Edge.empty; ok = false; msg = "";
            pStart = obj.targets(t1Idx).position;
            pEnd = obj.targets(t2Idx).position;
            pathPoints = obj.planShortestPath(pStart, pEnd);
            idx = numel(obj.edges) + 1;
            e = Edge(idx, [obj.targets(t1Idx), obj.targets(t2Idx)], pathPoints);
            obj.edges(end+1) = e;
            ok = true;
            if isprop(obj.pathPlanner, 'noRoute') && obj.pathPlanner.noRoute
                msg = sprintf("No wall-free route from T%d to T%d: the edge is a straight line through the walls.", t1Idx, t2Idx);
            end
        end

        function path = planShortestPath(obj, pStart, pEnd)
            path = obj.pathPlanner.plan(pStart, pEnd, obj.walls, obj.bounds);
        end


        function s = exportLayout(obj)
            s.targets = reshape([obj.targets.position], 2, []).';
            s.edges = [];
            for k = 1:numel(obj.edges)
                ids = [obj.edges(k).targets(1).index, obj.edges(k).targets(2).index];
                s.edges = [s.edges; ids];
            end
            s.walls = obj.walls;
            % Scenario fields below are optional on import, so older layout files still load
            s.agentStarts = arrayfun(@(a) a.initialTargetIdx, obj.agents);   % target index per agent
            s.agentSpeed = arrayfun(@(a) a.state.maxSpeed, obj.agents);      % max speed per agent
            s.agentAccel = arrayfun(@(a) a.state.maxAccel, obj.agents);      % max acceleration per agent
            s.agentProfile = arrayfun(@(a) a.profile, obj.agents);           % speed profile per agent
            s.agentBatteryWh = arrayfun(@(a) a.battery.capacityWh, obj.agents);   % battery capacity per agent
            s.agentStopWhenEmpty = arrayfun(@(a) double(a.stopWhenEmpty), obj.agents);  % stops when empty
            % Full power model and turn limit (energy-aware paper), stored only when set
            c2 = arrayfun(@(a) a.battery.c2, obj.agents);
            an = arrayfun(@(a) a.maxNormalAccel, obj.agents);
            r0 = arrayfun(@(a) a.battery.R0, obj.agents);
            if any(c2 ~= 0), s.agentMotorC2 = c2; end                        % Battery c2 per agent
            if any(isfinite(an)), s.agentMaxNormalAccel = an; end            % a_n per agent
            if any(r0 ~= 0)                                                   % battery losses
                s.agentBatteryR0 = r0; s.agentBatteryVoc = arrayfun(@(a) a.battery.Voc, obj.agents);
            end
            st = find(arrayfun(@(t) isa(t, 'ChargingStation'), obj.targets));
            s.chargers = st;                                                  % charging stations (targets)
            s.chargePower = arrayfun(@(k) obj.targets(k).chargePower, st);    % their power [W]
            s.background = obj.background;                                    % background image file
            s.targetType = obj.targetType;                                    % target type (registry name)
            s.uncertaintyMode = obj.uncertaintyMode;
            s.A = [obj.targets.A_uniformBase].';   % base growth rate A_i per target
            s.B = [obj.targets.B].';               % decay rate B_i per agent, per target
            s.R0 = [obj.targets.initialR].';       % initial uncertainty R_i(0) per target
        end

        function importLayout(obj, s)
            % Fields missing from older files keep their defaults; A, B, R0 and agentSpeed may
            % also be scalars that apply to every target / agent
            obj.clearAll();
            if isfield(s, 'targetType') && ismember(string(s.targetType), obj.registry.keysFor('targettype'))
                obj.targetFactory = obj.registry.factoryFor('targettype', char(s.targetType));
                obj.targetType = string(s.targetType);
            end
            stations = [];
            if isfield(s, 'chargers'), stations = s.chargers(:)'; end
            for k = 1:size(s.targets, 1)
                if ismember(k, stations)                % a charging station
                    t = ChargingStation(k, s.targets(k,:));
                    p = layoutValue(s, 'chargePower', find(stations == k, 1));
                    if ~isempty(p), t.chargePower = p; end
                    obj.targets(end+1) = t;
                else
                    t = obj.addTarget(s.targets(k,:));
                end
                t.setRates(layoutValue(s, 'A', k), layoutValue(s, 'B', k), layoutValue(s, 'R0', k));
            end
            if isfield(s, 'walls'), obj.walls = s.walls; end
            for k = 1:size(s.edges, 1)
                obj.addEdgeByTargets(double(s.edges(k,1)), double(s.edges(k,2)));
            end
            if isfield(s, 'uncertaintyMode'), obj.setUncertaintyMode(s.uncertaintyMode); end
            obj.background = "";
            if isfield(s, 'background'), obj.background = string(s.background); end
            if isfield(s, 'agentStarts')
                for k = 1:numel(s.agentStarts)
                    % each agent: the current type and spec, with the layout's stored settings
                    spec = obj.agentSpec;
                    speed = layoutValue(s, 'agentSpeed', k);
                    if ~isempty(speed), spec.maxSpeed = speed; end
                    accel = layoutValue(s, 'agentAccel', k);
                    if ~isempty(accel), spec.maxAccel = accel; end
                    prof = layoutValue(s, 'agentProfile', k);
                    if ~isempty(prof), spec.profile = prof; end
                    wh = layoutValue(s, 'agentBatteryWh', k);
                    if ~isempty(wh), spec.batteryWh = wh; end
                    stop = layoutValue(s, 'agentStopWhenEmpty', k);
                    if ~isempty(stop), spec.stopWhenEmpty = stop; end
                    for f = ["MotorC2", "MaxNormalAccel", "BatteryR0", "BatteryVoc"]   % full power model
                        val = layoutValue(s, char("agent" + f), k);
                        if ~isempty(val), spec.(lower(extractBefore(f, 2)) + extractAfter(f, 1)) = val; end
                    end
                    obj.addAgentOnTarget(obj.targets(s.agentStarts(k)).position, spec.maxSpeed, 1e-6, spec);
                end
            end
        end

        function t = addStation(obj, pos, chargePower)
            % A charging station (ChargingStation) at pos, numbered like a target; connect it
            % with edges as any target. chargePower [W] defaults to the station's own default
            t = ChargingStation(numel(obj.targets) + 1, pos);
            if nargin > 2, t.chargePower = chargePower; end
            obj.targets(end+1) = t;
        end

        function t = addTarget(obj, pos)
            t = obj.targetFactory(numel(obj.targets) + 1, pos);
            obj.targets(end+1) = t;
        end

        function [a, ok, msg] = addAgentOnTarget(obj, clickPos, speed, tol, spec)
            % A new agent on the target nearest clickPos, of the current type, with spec
            % (default: agentSpec) and maximum speed speed
            a = []; ok = false; msg = "";
            [tIdx, dist] = obj.findNearestTarget(clickPos);
            if isempty(tIdx) || dist > tol, msg = "Click near target"; return; end
            if nargin < 5, spec = obj.agentSpec; end
            spec.maxSpeed = speed;
            a = spec.spawn(numel(obj.agents)+1, obj.targets(tIdx).position, obj.agentFactory);
            a.current_target_idx = tIdx;
            a.initialTargetIdx = tIdx;
            a.initialPosition = a.state.pos;
            if isempty(obj.agents), obj.agents = a; else, obj.agents(end+1) = a; end
            ok = true;
        end

        function clearAll(obj)
            obj.agents = []; obj.targets = Target.empty; obj.edges = Edge.empty;
            obj.walls = []; obj.cumUncertaintyIntegral = 0; obj.hasLastSample = false;
            obj.lastLogTime = 0; obj.lastUncertainty = 0;
            obj.resetPolicyReservations();
            for k = 1:numel(obj.metrics), obj.metrics{k}.reset(obj); end
        end

        function resetPolicyReservations(obj)
            % RHCPolicy keeps a persistent reservation table; stale locks survive resets otherwise
            pols = obj.policyMap.values;
            for k = 1:numel(pols)
                if ismethod(pols{k}, 'resetReservations'), pols{k}.resetReservations(); end
            end
        end

        function resetSimulationState(obj)
            obj.cumUncertaintyIntegral = 0;
            obj.hasLastSample = false;
            obj.lastLogTime = 0;
            obj.lastUncertainty = 0;
            for k = 1:numel(obj.agents), obj.agents(k).resetToInitial(); end
            for t = 1:numel(obj.targets), obj.targets(t).reset(); end
            obj.resetPolicyReservations();
            for k = 1:numel(obj.metrics), obj.metrics{k}.reset(obj); end
            obj.collisions = zeros(1, numel(obj.agents));
        end

        function [idx, dist] = findNearestTarget(obj, pos)
            idx = []; dist = inf;
            if isempty(obj.targets), return; end
            P = reshape([obj.targets.position], 2, []).';
            d = hypot(P(:,1)-pos(1), P(:,2)-pos(2));
            [dist, idx] = min(d);
        end

        function adj = buildAdjacency(obj)
            n = numel(obj.targets); adj = false(n,n);
            for k = 1:numel(obj.edges)
                e = obj.edges(k); i = e.targets(1).index; j = e.targets(2).index;
                if i>=1 && i<=n && j>=1 && j<=n, adj(i,j)=true; adj(j,i)=true; end
            end
        end

        function e = findEdge(obj, t1, t2)
            e = [];
            for k = 1:numel(obj.edges)
                ids = [obj.edges(k).targets.index];
                % Exact match; ismember-based check made findEdge(i,i) return any edge touching i,
                % so "hold at current node" commands sent agents down a random edge
                if isequal(ids, [t1 t2]) || isequal(ids, [t2 t1]), e = obj.edges(k); return; end
            end
        end

    end
end

function v = layoutValue(s, name, k)
% Element k of an optional layout field: [] when absent, the value itself when scalar
v = [];
if ~isfield(s, name) || isempty(s.(name)), return; end
v = s.(name);
if ~isscalar(v), v = v(k); end
end
