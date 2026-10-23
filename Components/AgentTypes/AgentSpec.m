classdef AgentSpec
    % How to spawn an agent: its type (a ComponentRegistry name) and settings. One spec serves
    % every place that creates agents:
    %   GUI         Agent Type / Agent Speed / Agent Accel / Speed Profile set ScenarioModel.agentSpec,
    %               the spec of the next agents placed
    %   layouts     agentSpeed, agentAccel and agentProfile per agent (the type comes from the
    %               current selection, so a layout runs with either agent type)
    %   runTrial    AgentType, AgentSpeed and Agent_<setting> parameters (e.g. Agent_profile)
    %   code        spec = AgentSpec(Type="Physical Agent", MaxSpeed=10, Profile="SO");
    %               agent = spec.spawn(index, position)
    % Unset settings ("" or NaN) keep the agent type's defaults.
    properties
        type string = "Abstract Agent"   % registered agent type (older aliases also work)
        maxSpeed double = 15             % top speed (and nominal transit speed)
        maxAccel double = 15             % largest |acceleration|
        profile string = ""              % speed profile: "FO-0", "SO", "FO-3" ("" = type default)
        kp double = NaN                  % PhysicalAgent tracking gains (NaN = TrackingController defaults)
        kd double = NaN
        batteryWh double = NaN           % battery capacity [Wh] (NaN = Battery default)
        stopWhenEmpty double = NaN       % 1: the agent stops when its battery is empty (NaN = no)
        motorC2 double = NaN             % Battery c2, power per unit squared speed (NaN = 0)
        maxNormalAccel double = NaN      % normal acceleration in turns, a_n (NaN = Inf: no limit)
        batteryR0 double = NaN           % Battery R0 and Voc: losses P + (R0/Voc^2) P^2 (NaN = none)
        batteryVoc double = NaN
    end

    properties (Constant)
        settings = ["maxSpeed", "maxAccel", "profile", "kp", "kd", "batteryWh", "stopWhenEmpty", ...
            "motorC2", "maxNormalAccel", "batteryR0", "batteryVoc"]
    end

    methods
        function obj = AgentSpec(opts)
            arguments
                opts.Type string = "Abstract Agent"
                opts.MaxSpeed double = 15
                opts.MaxAccel double = 15
                opts.Profile string = ""
                opts.Kp double = NaN
                opts.Kd double = NaN
                opts.BatteryWh double = NaN
                opts.StopWhenEmpty double = NaN
                opts.MotorC2 double = NaN
                opts.MaxNormalAccel double = NaN
                opts.BatteryR0 double = NaN
                opts.BatteryVoc double = NaN
            end
            obj.type = opts.Type; obj.maxSpeed = opts.MaxSpeed; obj.maxAccel = opts.MaxAccel;
            obj.profile = opts.Profile; obj.kp = opts.Kp; obj.kd = opts.Kd;
            obj.batteryWh = opts.BatteryWh;
            obj.stopWhenEmpty = opts.StopWhenEmpty;
            obj.motorC2 = opts.MotorC2; obj.maxNormalAccel = opts.MaxNormalAccel;
            obj.batteryR0 = opts.BatteryR0; obj.batteryVoc = opts.BatteryVoc;
        end

        function a = spawn(obj, index, position, factory)
            % A new agent of this spec. factory: the type's @(idx, pos, speed) constructor
            % (default: looked up in a ComponentRegistry)
            if nargin < 4, factory = ComponentRegistry().factoryFor('agenttype', char(obj.type)); end
            a = factory(index, position, obj.maxSpeed);
            obj.applyTo(a);
        end

        function applyTo(obj, a)
            % Set this spec's settings on an existing agent (unset ones are left alone)
            a.state.maxSpeed = obj.maxSpeed;
            a.state.maxAccel = obj.maxAccel;
            if obj.profile ~= "", a.profile = obj.profile; end
            if ~isnan(obj.kp), a.controller.kp = obj.kp; end
            if ~isnan(obj.kd), a.controller.kd = obj.kd; end
            if ~isnan(obj.batteryWh), a.battery.capacityWh = obj.batteryWh; end
            if ~isnan(obj.stopWhenEmpty), a.stopWhenEmpty = logical(obj.stopWhenEmpty); end
            if ~isnan(obj.motorC2), a.battery.c2 = obj.motorC2; end
            if ~isnan(obj.maxNormalAccel), a.maxNormalAccel = obj.maxNormalAccel; end
            if ~isnan(obj.batteryR0), a.battery.R0 = obj.batteryR0; end
            if ~isnan(obj.batteryVoc), a.battery.Voc = obj.batteryVoc; end
        end
    end

    methods (Static)
        function spec = of(a, type)
            % The spec of an existing agent (type: its registered name, if known)
            if nargin < 2, type = ""; end
            spec = AgentSpec(Type=type, MaxSpeed=a.state.maxSpeed, MaxAccel=a.state.maxAccel, ...
                Profile=a.profile, Kp=a.controller.kp, Kd=a.controller.kd, BatteryWh=a.battery.capacityWh, ...
                StopWhenEmpty=double(a.stopWhenEmpty), MotorC2=a.battery.c2, ...
                MaxNormalAccel=a.maxNormalAccel, BatteryR0=a.battery.R0, BatteryVoc=a.battery.Voc);
        end
    end
end
