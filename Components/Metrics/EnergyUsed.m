classdef EnergyUsed < Metric
    % Energy drawn from the batteries of all agents so far [kJ], charging not subtracted (each
    % step's increase of an agent's energy drawn; a step in which a station recharges it counts 0)
    properties (Access = private)
        last = []
        total = 0
    end

    methods
        function obj = EnergyUsed()
            obj.label = "energy used [kJ]";
            obj.plotMax = 100;
        end

        function reset(obj, ~)
            obj.last = []; obj.total = 0;
        end

        function update(obj, model, ~, ~)
            if isempty(model.agents), return; end
            used = arrayfun(@(a) a.battery.energyUsed, model.agents(:)');
            if numel(obj.last) == numel(used)
                obj.total = obj.total + sum(max(used - obj.last, 0));
            end
            obj.last = used;
        end

        function v = value(obj, ~, ~)
            v = obj.total / 1000;
        end
    end
end
