classdef Charges < Metric
    % Charging sessions so far, of all agents: a session starts when a station begins to
    % recharge an agent's battery (its energy drawn falls over a step). Any policy
    properties (Access = private)
        last = []       % energy drawn by each agent at the last step
        charging = []   % recharged at the last step
        count = 0
    end

    methods
        function obj = Charges()
            obj.label = "charging sessions";
            obj.plotMax = 10;
        end

        function reset(obj, ~)
            obj.last = []; obj.charging = []; obj.count = 0;
        end

        function update(obj, model, ~, ~)
            if isempty(model.agents), return; end
            used = arrayfun(@(a) a.battery.energyUsed, model.agents(:)');
            if numel(obj.last) ~= numel(used)
                obj.last = used; obj.charging = false(size(used)); return;
            end
            now = used < obj.last - 1e-9;
            obj.count = obj.count + sum(now & ~obj.charging);
            obj.charging = now; obj.last = used;
        end

        function v = value(obj, ~, ~)
            v = obj.count;
        end
    end
end
