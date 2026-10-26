classdef MonitoringShare < Metric
    % Share of the agents' time spent monitoring [%]: not on the way to, waiting at or charging
    % at a station. From the policy's own record when it keeps one (Energy-Aware ED-RHC's
    % modeTime); otherwise an agent counts as away while a station is its destination or the
    % node it rests at
    properties (Access = private)
        away = 0
        total = 0
    end

    methods
        function obj = MonitoringShare()
            obj.label = "monitoring share [%]";
            obj.plotMax = 100;
        end

        function reset(obj, ~)
            obj.away = 0; obj.total = 0;
        end

        function update(obj, model, ~, dt)
            if isempty(model.agents), return; end
            st = arrayfun(@(a) ~isempty(a.current_target_idx) && ...
                isa(model.targets(a.current_target_idx), 'ChargingStation'), model.agents(:)');
            obj.away = obj.away + sum(st) * dt;
            obj.total = obj.total + numel(model.agents) * dt;
        end

        function v = value(obj, model, ~)
            pol = model.policy;
            if isprop(pol, 'modeTime') && ~isempty(pol.modeTime) && sum(pol.modeTime(:)) > 0
                v = 100 * sum(pol.modeTime(:, 1)) / sum(pol.modeTime(:));
            else
                v = 100 * (1 - obj.away / max(obj.total, eps));
            end
        end
    end
end
