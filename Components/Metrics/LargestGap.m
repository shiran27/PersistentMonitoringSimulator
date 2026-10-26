classdef LargestGap < Metric
    % The longest time any target has gone unattended so far [s]: no agent at rest there.
    % Stations are not counted
    properties (Access = private)
        since = []      % time each target has been unattended
        longest = 0
    end

    methods
        function obj = LargestGap()
            obj.label = "largest unattended time [s]";
            obj.plotMax = 100;
        end

        function reset(obj, ~)
            obj.since = []; obj.longest = 0;
        end

        function update(obj, model, ~, dt)
            n = numel(model.targets);
            if numel(obj.since) ~= n, obj.since = zeros(1, n); end
            for k = 1:n
                tg = model.targets(k);
                if isa(tg, 'ChargingStation'), continue; end
                if isempty(tg.residingAgents)
                    obj.since(k) = obj.since(k) + dt;
                    obj.longest = max(obj.longest, obj.since(k));
                else
                    obj.since(k) = 0;
                end
            end
        end

        function v = value(obj, ~, ~)
            v = obj.longest;
        end
    end
end
