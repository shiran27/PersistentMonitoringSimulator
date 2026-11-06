classdef SimulationClock < handle
    properties
        currentTime double = 0
        endTime double = 60
        dt double = 0.01  % Fixed stable delta (e.g., 10ms)
        timeScale double = 1.0
    end

    methods
        function obj = SimulationClock(endTime, dt)
            if nargin >= 1 && ~isempty(endTime), obj.endTime = endTime; end
            if nargin >= 2 && ~isempty(dt), obj.dt = dt; end
        end

        function reset(obj)
            obj.currentTime = 0;
        end

        function dtSim = tick(obj)
            % One step of dt (timeScale only sets how many steps the GUI runs per frame). The
            % time stays on the grid k dt, so no round-off accumulates, and the last step ends
            % exactly at endTime (shortened if endTime is not a multiple of dt)
            remaining = obj.endTime - obj.currentTime;
            if remaining <= 0
                dtSim = 0;
                obj.currentTime = obj.endTime;
                return;
            end
            next = (floor(obj.currentTime / obj.dt + 1e-6) + 1) * obj.dt;
            if next >= obj.endTime - 1e-9 * obj.dt
                dtSim = remaining;
                obj.currentTime = obj.endTime;
            else
                dtSim = obj.dt;
                obj.currentTime = next;
            end
        end

        function tf = isFinished(obj)
            tf = obj.currentTime >= (obj.endTime - 1e-12);
        end
    end
end