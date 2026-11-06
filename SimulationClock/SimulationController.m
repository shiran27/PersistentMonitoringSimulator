classdef SimulationController < handle
    properties
        model; renderer; clock; timerObj; isRunning = false;
        timeCallback; statusCallback; objectiveCallback; stateCallback;
        activePolicy = 'Divergent RHC';
    end
    
    methods
        function obj = SimulationController(model, renderer, clock, timeCb, statusCb, objCb, stateCb)
            obj.model = model; obj.renderer = renderer; obj.clock = clock;
            obj.timeCallback = timeCb; obj.statusCallback = statusCb; obj.objectiveCallback = objCb;
            if nargin >= 7, obj.stateCallback = stateCb; end
            obj.timerObj = timer('ExecutionMode','fixedSpacing','Period',0.03,'TimerFcn',@(~,~)obj.onTimerTick());
        end
        
        function setActivePolicy(obj, policyName)
            obj.activePolicy = policyName;
            if ~isempty(obj.model) && (isprop(obj.model, 'setActivePolicy') || ismethod(obj.model, 'setActivePolicy'))
                obj.model.setActivePolicy(policyName);
            end
        end
        
        function play(obj)
            if obj.clock.isFinished(), obj.reset(); end
            if obj.isRunning, return; end
            obj.tellMissionEnd();
            obj.isRunning = true; start(obj.timerObj);
            obj.notifyState();
        end
        
        function pause(obj)
            obj.isRunning = false;
            if isvalid(obj.timerObj), stop(obj.timerObj); end
            obj.notifyState();
        end
        
        function setEndTime(obj, tEnd)
            obj.clock.endTime = max(0, tEnd);
            obj.tellMissionEnd();
            if obj.clock.currentTime > obj.clock.endTime, obj.clock.currentTime = obj.clock.endTime; end
            if ~isempty(obj.timeCallback), obj.timeCallback(obj.clock.currentTime, obj.clock.endTime); end
        end
        
        function setDt(obj, dt)
            obj.clock.dt = max(1e-4, dt);
        end
        
        function setTimeScale(obj, s)
            obj.clock.timeScale = max(0, s);
        end
        
        function reset(obj)
            obj.pause();
            obj.clock.reset();
            if ismethod(obj.model, 'resetSimulationState')
                obj.model.resetSimulationState();
            end
            obj.renderer.renderAll(obj.model);
            if ~isempty(obj.timeCallback), obj.timeCallback(0, obj.clock.endTime); end
        end
        
        function runToEnd(obj)
            obj.pause();
            obj.tellMissionEnd();
            obj.isRunning = true; obj.notifyState();
            while ~obj.clock.isFinished()
                dtSim = obj.clock.tick();
                obj.model.step(dtSim, obj.clock.currentTime);
                [~, JNow] = obj.model.updateTargetsAndLogObjective(obj.clock.currentTime, dtSim);
                if mod(round(obj.clock.currentTime/obj.clock.dt), 100) == 0
                    if ~isempty(obj.objectiveCallback), obj.objectiveCallback(obj.clock.currentTime, JNow); end
                    if ~isempty(obj.timeCallback), obj.timeCallback(obj.clock.currentTime, obj.clock.endTime); end
                    drawnow limitrate;
                end
            end
            obj.renderer.renderAgents(obj.model);
            obj.isRunning = false; obj.notifyState();
        end

        function tellMissionEnd(obj)
            % Policies that plan up to the mission end are told the end time, as runTrial does
            if isempty(obj.model), return; end
            pol = obj.model.policy;
            if ~isempty(pol) && isprop(pol, 'missionEnd'), pol.missionEnd = obj.clock.endTime; end
        end

        function notifyState(obj)
            if ~isempty(obj.stateCallback), obj.stateCallback(obj.isRunning); end
        end
        
        function onTimerTick(obj)
            if obj.clock.isFinished(), obj.pause(); return; end
          
            stepsToRun = max(1, round(obj.clock.timeScale));
            lastJ = 0;
            for i = 1:stepsToRun
                if obj.clock.isFinished(), break; end
                dtSim = obj.clock.tick();
                obj.model.step(dtSim, obj.clock.currentTime);
                [~, lastJ] = obj.model.updateTargetsAndLogObjective(obj.clock.currentTime, dtSim);
            end
            if ~isempty(obj.objectiveCallback), obj.objectiveCallback(obj.clock.currentTime, lastJ); end
            obj.renderer.renderAgents(obj.model);
            if ~isempty(obj.timeCallback), obj.timeCallback(obj.clock.currentTime, obj.clock.endTime); end
        end
    end
end