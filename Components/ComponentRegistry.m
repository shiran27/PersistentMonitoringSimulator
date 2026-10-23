classdef ComponentRegistry < handle
    % Central dictionary of modular components.
    % To register a new algorithm, add one line to the constructor.
    properties
        pathPlanners   % name -> @() planner
        policies       % name -> @() policy
        agentTypes     % name -> @(idx, pos, speed) agent
        targetTypes    % name -> @(idx, pos) target
        metrics        % name -> @() metric
        aliases        % older agent type names -> @(idx, pos, speed) agent; resolved by factoryFor
        retired        % retired component names -> what replaces them (factoryFor errors with it)
        renamed        % older component names -> current names, any kind (resolve, factoryFor)
                       % but not listed by keysFor (so not offered in the GUI)
        assumptions    % policy name -> agent assumption, for policies that do not declare one
        agentCounts    % policy name -> number of agents it is built for
    end

    methods
        function obj = ComponentRegistry()
            obj.pathPlanners = containers.Map('KeyType','char','ValueType','any');
            obj.pathPlanners('Dijkstra Corners') = @() DijkstraCornerPlanner();
            obj.pathPlanners('Energy Efficient') = @() EnergyEfficientPlanner();
            obj.pathPlanners('Grid A*')          = @() GridAStarPlanner();

            obj.policies = containers.Map('KeyType','char','ValueType','any');
            obj.policies('Random Walk')       = @() RandomWalkPolicy();
            obj.policies('Classical RHC [1]') = @() RHCPolicy();
            obj.policies('Divergent RHC')     = @() DivergentRHCPolicy();
            obj.policies('Joint RHC')         = @() JointRHCPolicy();
            obj.policies('Joint RHC N')       = @() JointRHCPolicyN();
            obj.policies('ED-RHC')            = @() EDRHCPolicy();
            obj.policies('ED-ORHC-SO')        = @() EDRHCPolicy(Transit="SO");
            obj.policies('ED-ORHC-FO-3')      = @() EDRHCPolicy(Transit="FO-3");
            obj.policies('Battery-Aware ED-RHC') = @() BatteryAwareRHCPolicy();
            obj.policies('Energy-Aware ED-RHC') = @() EnergyAwareEDRHCPolicy();
            obj.policies('Collab-ED-RHC')     = @() CollabEDRHCPolicy();
            obj.policies('Simple Policy (example)') = @() SimplePolicy();

            % What each policy assumes of the agents' motion (a policy may also declare it as a
            % constant property agentAssumption; see assumptionOf)
            obj.assumptions = containers.Map('KeyType','char','ValueType','any');
            for name = ["Classical RHC [1]", "Divergent RHC", "Joint RHC", "Joint RHC N"]
                obj.assumptions(char(name)) = "FO-0";   % transit = path length / maxSpeed
            end
            obj.agentCounts = containers.Map('KeyType','char','ValueType','double');
            obj.agentCounts('Joint RHC') = 2;

            obj.agentTypes = containers.Map('KeyType','char','ValueType','any');
            obj.agentTypes('Abstract Agent')   = @(idx, pos, sp) AbstractAgent(idx, pos, sp);
            obj.agentTypes('Physical Agent') = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);
            obj.agentTypes('Simple Agent (example)') = @(idx, pos, sp) SimpleAgent(idx, pos, sp);

            obj.targetTypes = containers.Map('KeyType','char','ValueType','any');
            obj.targetTypes('Uncertainty Target')     = @(idx, pos) Target(idx, pos);
            obj.targetTypes('Simple Target (example)') = @(idx, pos) SimpleTarget(idx, pos);

            obj.metrics = containers.Map('KeyType','char','ValueType','any');
            obj.metrics('Average Uncertainty J(t)') = @() AverageUncertainty();
            obj.metrics('Simple Metric (example)')  = @() SimpleMetric();
            obj.metrics('Minimum SOC')              = @() MinimumSOC();

            % Agent types before the consolidation of 2026-10-20, kept so older scripts and
            % layouts run unchanged (e.g. William's Joint RHC studies use 'Linear Agent')
            obj.aliases = containers.Map('KeyType','char','ValueType','any');
            obj.aliases('Linear Agent')       = @(idx, pos, sp) AbstractAgent(idx, pos, sp);
            obj.aliases('Second-Order Agent') = @(idx, pos, sp) AbstractAgent(idx, pos, sp, Profile="SO");
            obj.aliases('Default Agent')      = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);
            obj.aliases('Energy Agent')       = @(idx, pos, sp) PhysicalAgent(idx, pos, sp);

            % Components no longer offered, with what replaces them (factoryFor says so)
            % Components renamed, with their current names: the profiles follow the energy-aware
            % paper (FO-0 constant speed, SO, FO-3 trapezoid) since 2026-10-22
            obj.renamed = containers.Map('KeyType','char','ValueType','any');
            obj.renamed('ED-ORHC-FO') = 'ED-ORHC-FO-3';

            obj.retired = containers.Map('KeyType','char','ValueType','any');
            obj.retired('Battery Efficient') = "retired on 2026-10-21 (Old_Design/" + ...
                "BatteryEfficientPolicy.m): use 'Battery-Aware ED-RHC' with a charging station, or 'ED-RHC'";
        end

        function a = assumptionOf(obj, name, policy)
            % What policy <name> assumes of the agents' motion:
            %   "any"             nothing (e.g. Random Walk: dwells from R at arrival)
            %   "plans transit"   it hands each transit's time (and profile) to the agent
            %   "FO-0"            transit time = path length / maxSpeed (constant speed), not
            %                     handed over
            %   "unknown"         not declared
            if nargin > 2 && isprop(policy, 'agentAssumption')
                a = string(policy.agentAssumption);
            elseif isKey(obj.assumptions, char(name))
                a = obj.assumptions(char(name));
            else
                a = "unknown";
            end
        end

        function n = requiredAgents(obj, name)
            % Number of agents policy <name> is built for (0 = any)
            n = 0;
            if isKey(obj.agentCounts, char(name)), n = obj.agentCounts(char(name)); end
        end

        function names = keysFor(obj, kind)
            names = obj.mapFor(kind).keys;
        end

        function name = resolve(obj, name)
            % The current name of a component (older names in renamed map to their new names)
            if isKey(obj.renamed, char(name)), name = obj.renamed(char(name)); end
        end

        function h = factoryFor(obj, kind, name)
            name = obj.resolve(name);
            m = obj.mapFor(kind);
            if ~isKey(m, name) && strcmpi(kind, 'agenttype') && isKey(obj.aliases, name)
                h = obj.aliases(name);
            elseif ~isKey(m, name) && isKey(obj.retired, char(name))
                error('ComponentRegistry:Retired', '"%s" was %s', name, obj.retired(char(name)));
            else
                h = m(name);
            end
        end
    end

    methods (Access=private)
        function m = mapFor(obj, kind)
            switch lower(kind)
                case 'pathplanner', m = obj.pathPlanners;
                case 'policy',      m = obj.policies;
                case 'agenttype',   m = obj.agentTypes;
                case 'targettype',  m = obj.targetTypes;
                case 'metric',      m = obj.metrics;
                otherwise
                    error('ComponentRegistry:UnknownKind', 'Unknown kind: %s', kind);
            end
        end
    end
end
