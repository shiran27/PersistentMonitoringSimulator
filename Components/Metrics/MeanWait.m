classdef MeanWait < Metric
    % Mean time an agent spends at a station without charging, per charging session [s]: the
    % queueing and the wait for a booked slot. Any policy
    properties (Access = private)
        last = []
        charging = []
        sessions = 0
        waited = 0
    end

    methods
        function obj = MeanWait()
            obj.label = "mean wait at stations [s]";
            obj.plotMax = 60;
        end

        function reset(obj, ~)
            obj.last = []; obj.charging = []; obj.sessions = 0; obj.waited = 0;
        end

        function update(obj, model, ~, dt)
            if isempty(model.agents), return; end
            used = arrayfun(@(a) a.battery.energyUsed, model.agents(:)');
            if numel(obj.last) ~= numel(used)
                obj.last = used; obj.charging = false(size(used)); return;
            end
            now = used < obj.last - 1e-9;
            obj.sessions = obj.sessions + sum(now & ~obj.charging);
            atStation = arrayfun(@(a) isempty(a.path) && ~isempty(a.current_target_idx) && ...
                isa(model.targets(a.current_target_idx), 'ChargingStation'), model.agents(:)');
            obj.waited = obj.waited + sum(atStation & ~now) * dt;
            obj.charging = now; obj.last = used;
        end

        function v = value(obj, ~, ~)
            v = obj.waited / max(obj.sessions, 1);
        end
    end
end
