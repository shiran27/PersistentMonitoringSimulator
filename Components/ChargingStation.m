classdef ChargingStation < Target
    % A charging station (dock): a node of the network with nothing to monitor (R = 0, A = 0)
    % where every agent at rest recharges its battery at chargePower until full (the agent
    % still draws its idle power, so the net rate is chargePower - Battery.idlePower). Layouts
    % place stations by target index (fields chargers and chargePower). BatteryAwareRHCPolicy
    % sends agents here before their batteries run out; other policies treat a station as an
    % ordinary node. It is drawn as a yellow square without an uncertainty bar, and it records
    % the energy it delivered and the time it was in use.
    % docks (m_sigma, energy-aware paper Sec. 6.2) limits how many agents charge at once: an
    % agent at rest takes a free dock first come, first served, and holds it until it leaves;
    % others wait without charging. Inf (the default) lets every agent at rest charge. Agents
    % whose chargeAllowed is false (a policy's choice) are not charged.
    properties
        chargePower = 100      % charging power [W]
        docks = Inf            % number of docks (agents charging at once)
        energyDelivered = 0    % energy delivered so far [J]
        busyTime = 0           % time with an agent charging so far [s]
        dockTime = 0           % dock-seconds of charging so far (busyTime counts one per step)
        docked = []            % indices of the agents holding a dock
    end

    methods
        function obj = ChargingStation(index, position, chargePower)
            obj@Target(index, position);
            if nargin > 2, obj.chargePower = chargePower; end
            obj.color = [1 0.82 0.1];
            obj.markerSize = 12;
            obj.noUncertainty();
        end

        function updateUncertainty(obj, dt, ~, ~)
            % Recharge the agents at rest here (the model's per-step target update), up to docks
            % of them; R stays 0
            ag = obj.residingAgents;
            if iscell(ag), ag = [ag{:}]; end
            if ~isempty(ag), ag = ag([ag.chargeAllowed]); end   % e.g. not before a booked slot
            if isfinite(obj.docks) && ~isempty(ag)
                ids = [ag.index];
                obj.docked = obj.docked(ismember(obj.docked, ids));        % those still here
                waiting = ids(~ismember(ids, obj.docked));
                obj.docked = [obj.docked, waiting(1:min(end, obj.docks - numel(obj.docked)))];
                ag = ag(ismember(ids, obj.docked));
            elseif isempty(ag)
                obj.docked = [];
            end
            obj.dockTime = obj.dockTime + numel(ag) * dt;
            for a = ag(:)'
                before = a.battery.energyUsed;
                a.battery.recharge(obj.chargePower, dt);
                obj.energyDelivered = obj.energyDelivered + before - a.battery.energyUsed;
            end
            if ~isempty(ag), obj.busyTime = obj.busyTime + dt; end
            obj.R = 0;
        end

        function ts = slotLength(obj, agent, socMin)
            % Slot length t_s = (Q - E_min)/(P_c - P_I): a full charge from the reserve
            % E_min = socMin Q under constant power, for agent's battery (Sec. 6.2; P_I with the
            % battery's losses)
            b = agent.battery;
            ts = (1 - socMin) * b.capacity / (obj.chargePower - b.idleDraw());
        end

        function configure(obj, p)
            configure@Target(obj, p);
            obj.noUncertainty();
        end

        function setRates(obj, ~, B, ~)
            % Layout rates: only B is kept (a station has no uncertainty)
            setRates@Target(obj, [], B, []);
        end

        function reset(obj)
            reset@Target(obj);
            obj.energyDelivered = 0;
            obj.busyTime = 0;
            obj.dockTime = 0;
            obj.docked = [];
            obj.color = [1 0.82 0.1];
            obj.markerSize = 12;
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle)
                set(obj.graphicHandle, 'MarkerSize', obj.markerSize, 'MarkerFaceColor', obj.color);
            end
            if ~isempty(obj.labelHandle) && isvalid(obj.labelHandle)
                set(obj.labelHandle, 'Color', 'k');
            end
        end

        function draw(obj, ax)
            obj.graphicHandle = plot(ax, obj.position(1), obj.position(2), 's', ...
                'MarkerSize', obj.markerSize, 'MarkerFaceColor', obj.color, ...
                'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'PickableParts', 'none');
            obj.labelHandle = text(ax, obj.position(1), obj.position(2), num2str(obj.index), ...
                'Color', 'k', 'FontSize', 8, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'PickableParts', 'none');
        end
    end

    methods (Access = private)
        function noUncertainty(obj)
            obj.A_uniformBase = 0; obj.A_priorityBase = 0; obj.A_base = 0;
            obj.A_spike = 0; obj.A = 0; obj.R = 0;
        end
    end
end
