classdef MainUI < handle
    properties
        fig
        ax
        speedField
        accelField          % max acceleration of agents added next
        profileDropdown     % speed profile of agents added next
        scenarioDropdown    % ready-made scenarios (scenarioCatalog)
        paperProfileDropdown % paper profile: the components shown (paperProfiles)
        modeLabel
        statusLabel
        timeLabel
        endTimeField
        dtField
        timeScaleSlider
        plannerDropdown
        policyDropdown
        agentTypeDropdown
        targetTypeDropdown  % target type of every target (ScenarioModel.setTargetType)
        objectiveDropdown   % metric plotted in the objective window (ScenarioModel.metrics{1})
        stationPowerField   % charging power of the stations added next (Add Station)
        registry
        model
        renderer
        controller
        clock
        sim
        objectiveWin
        gridStep = 5;
        snapTargets = true;
        frameBuffer = {}       % Accumulated frames for video export
        uncertaintyDropdown % New Dropdown Property
        recordCheckbox      % Frame capture is slow; only record when asked
    end
    
    methods
        function obj = MainUI(opts)
            % MainUI('Monitor', k) opens the windows on monitor k (a row of
            % get(groot, 'MonitorPositions')); by default they open on the primary monitor.
            % MainUI('PaperProfile', name) starts with a paper profile (paperProfiles)
            arguments
                opts.Monitor double = []
                opts.PaperProfile (1,1) string = "Full"
            end
            obj.registry = ComponentRegistry();
            [mainPos, objectivePos] = MainUI.windowPositions(opts.Monitor);
            obj.buildUI(mainPos);

            plannerFactory = obj.registry.factoryFor('pathplanner', obj.plannerDropdown.Value);
            agentFactory   = obj.registry.factoryFor('agenttype',   obj.agentTypeDropdown.Value);
            obj.model = ScenarioModel(plannerFactory(), agentFactory);
            obj.model.agentSpec = AgentSpec(Type=string(obj.agentTypeDropdown.Value), ...
                MaxSpeed=obj.speedField.Value, MaxAccel=obj.accelField.Value, ...
                Profile=string(obj.profileDropdown.Value));
            obj.renderer = ScenarioRenderer(obj.ax);
            obj.controller = GraphEditController(obj.model, obj.renderer, @(m)obj.setStatus(m));
            obj.objectiveWin = ObjectivePlotWindow(objectivePos);
            obj.clock = SimulationClock(obj.endTimeField.Value, obj.dtField.Value);
            
            obj.sim = SimulationController(obj.model, obj.renderer, obj.clock, ...
                @(t,tEnd)obj.setTime(t,tEnd), ...
                @(m)obj.setStatus(m), ...
                @(t,J)obj.objectiveWin.addPoint(t,J), ...
                @(running)obj.onRunStateChanged(running));

            % Bind event listeners and initial UI state
            obj.fig.WindowButtonMotionFcn = @(~,~)obj.safeOnMouseMove();
            obj.controller.setMode("idle");
            
            % Initialize default selected policy, uncertainty and objective states
            obj.onPolicyChanged(obj.policyDropdown.Value);
            obj.onUncertaintyModeChanged(obj.uncertaintyDropdown.Value);
            obj.onObjectiveChanged(obj.objectiveDropdown.Value);
            
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
            if opts.PaperProfile ~= "Full", obj.setPaperProfile(opts.PaperProfile); end
        end

        function setPaperProfile(obj, name)
            % Shows only what paper profile <name> lists (Configurations/paperProfiles) in the
            % component, speed profile and scenario dropdowns; "Full" shows everything. A
            % selection the profile hides moves to the first shown item, as if chosen by hand.
            P = paperProfiles();
            k = find([P.Name] == string(name), 1);
            assert(~isempty(k), 'MainUI:PaperProfile', 'Unknown paper profile "%s"', name);
            P = P(k);
            obj.paperProfileDropdown.Value = char(P.Name);
            menus = {'pathplanner', obj.plannerDropdown,    @obj.onPlannerChanged
                     'policy',      obj.policyDropdown,     @obj.onPolicyChanged
                     'agenttype',   obj.agentTypeDropdown,  @obj.onAgentTypeChanged
                     'targettype',  obj.targetTypeDropdown, @obj.onTargetTypeChanged
                     'metric',      obj.objectiveDropdown,  @obj.onObjectiveChanged};
            for r = 1:size(menus, 1)
                names = string(obj.registry.keysFor(menus{r, 1}));
                MainUI.showOnly(menus{r, 2}, names, names, P.(menus{r, 1}), menus{r, 3});
            end
            MainUI.showOnly(obj.profileDropdown, MainUI.profileLabels, SpeedProfile.names, ...
                P.speedprofile, @obj.onAgentProfileChanged);
            cat = scenarioCatalog();
            names = ["(build your own)"; cat.Name];
            shown = MainUI.listed(names, P.scenario); shown(1) = true;
            MainUI.showOnly(obj.scenarioDropdown, names, names, names(shown), @(~) []);
            obj.setStatus("Paper profile: " + P.Name + " (" + P.Description + ")");
        end

        % File I/O behind the Save/Load Layout buttons, callable without the file dialogs
        function saveLayout(obj, file)
            s = obj.model.exportLayout();
            save(file, 's');
        end

        function loadScenario(obj, name)
            % Loads the ready-made scenario <name> (scenarioCatalog), as the Scenario dropdown does
            cat = scenarioCatalog();
            k = find(cat.Name == string(name), 1);
            assert(~isempty(k), 'MainUI:Scenario', 'Unknown scenario "%s"', name);
            obj.loadLayout(cat.File(k));
            MainUI.showValue(obj.scenarioDropdown, name);
            obj.setStatus(cat.Name(k) + ": " + cat.Description(k));
            obj.noteAgentFit();
        end

        function loadLayout(obj, file)
            tmp = load(file, 's');
            obj.model.importLayout(tmp.s);
            % The layout may carry its target type, uncertainty mode and agents; show them
            MainUI.showValue(obj.targetTypeDropdown, obj.model.targetType);
            obj.uncertaintyDropdown.Value = obj.model.uncertaintyMode;
            if ~isempty(obj.model.agents)
                a = obj.model.agents(1);
                obj.speedField.Value = a.state.maxSpeed;
                obj.onAgentAccelChanged(a.state.maxAccel);
                obj.onAgentProfileChanged(a.profile);
            end
            st = obj.model.targets(arrayfun(@(t) isa(t, 'ChargingStation'), obj.model.targets));
            if ~isempty(st)   % show the stations' power (editing it sets every station's)
                obj.stationPowerField.Value = st(1).chargePower;
                obj.controller.stationPower = st(1).chargePower;
            end
            % The layout's background map, if it names one
            bg = ScenarioModel.backgroundFile(obj.model.background);
            if bg ~= "", obj.renderer.loadBackgroundImage(bg); else, obj.renderer.clearBackground(); end
            obj.renderer.clearAxes();
            obj.renderer.renderAll(obj.model);
            obj.onResetSimulation();
            obj.noteAgentFit();
        end
    end

    properties (Constant)
        profileLabels = ["FO-0 (constant)", "SO (smooth)", "FO-3 (trapezoid)"]   % of SpeedProfile.names
    end

    methods (Static)
        function tf = listed(names, patterns)
            % Which names the patterns list (all when patterns is empty); a pattern ending in *
            % matches every name that starts with the rest (paperProfiles' scenario entries)
            names = string(names); tf = true(size(names));
            if isempty(patterns), return; end
            tf = false(size(names));
            for q = string(patterns(:))'
                if endsWith(q, "*"), tf = tf | startsWith(names, extractBefore(q, strlength(q)));
                else, tf = tf | names == q;
                end
            end
        end

        function k = secondaryMonitor()
            % A monitor other than the primary one (whose origin is [1 1]), for windows opened by
            % scripts and tests; 1 when there is only one monitor
            mons = get(groot, 'MonitorPositions');
            k = find(mons(:, 1) ~= 1 | mons(:, 2) ~= 1, 1);
            if isempty(k), k = 1; end
        end
    end

    methods (Static, Access = private)
        function showOnly(dd, labels, data, allowed, onChange)
            % Shows the items of dropdown dd whose data the profile lists (allowed; all when
            % empty). When the selection is hidden, selects the first shown item and calls
            % onChange with it, as a click would.
            keep = MainUI.listed(data, allowed);
            current = string(dd.Value);
            if isempty(dd.ItemsData)
                dd.Items = cellstr(data(keep));
            else
                dd.Items = cellstr(labels(keep)); dd.ItemsData = cellstr(data(keep));
            end
            if any(data(keep) == current)
                dd.Value = char(current);   % new Items may have moved the selection
            else
                first = data(find(keep, 1));
                dd.Value = char(first);
                onChange(char(first));
            end
        end

        function showValue(dd, value)
            % Selects value in dd, adding it when the paper profile hides it (a loaded layout or
            % scenario may use it)
            value = string(value);
            if isempty(dd.ItemsData)
                if ~any(string(dd.Items) == value), dd.Items = [dd.Items, {char(value)}]; end
            elseif ~any(string(dd.ItemsData) == value)
                label = value;
                k = find(SpeedProfile.names == value, 1);   % the speed profile dropdown
                if ~isempty(k), label = MainUI.profileLabels(k); end
                dd.Items = [dd.Items, {char(label)}]; dd.ItemsData = [dd.ItemsData, {char(value)}];
            end
            dd.Value = char(value);
        end

        function [mainPos, objPos] = windowPositions(monitor)
            % Default spots on the primary monitor, or fitted onto the given monitor: main window
            % at its top left, objective window to the right or below; if neither fits, at the
            % monitor's bottom right corner, over the main window
            mainPos = [100 100 1200 780];
            objPos = [1350 200 700 450];
            if isempty(monitor), return; end
            mons = get(groot, 'MonitorPositions');
            m = mons(min(monitor, size(mons, 1)), :);   % [left bottom width height]
            gap = 20;
            top = m(2) + m(4) - 1 - 60;                 % clear of the menu bar and title bar
            w = min(1200, m(3) - 2*gap);
            h = min(780, m(4) - 120);
            mainPos = [m(1) + gap, top - h, w, h];
            ow = min(700, m(3) - 2*gap);
            below = mainPos(2) - gap - 30 - 450;          % 30: title bar
            if mainPos(1) + w + gap + ow <= m(1) + m(3) - gap
                objPos = [mainPos(1) + w + gap, top - 450, ow, 450];
            elseif below >= m(2) + gap
                objPos = [m(1) + gap, below, ow, 450];
            else
                objPos = [m(1) + m(3) - gap - ow, m(2) + gap, ow, 450];
            end
        end
    end

    methods (Access = private)
        function buildUI(obj, position)
            obj.fig = uifigure('Name','Multi-Agent Persistent Monitoring Simulator', ...
                'Position', position);
            obj.fig.CloseRequestFcn = @(~,~) obj.forceCloseAll();

            % Grid layouts size everything automatically; the left column scrolls if the window is short
            main = uigridlayout(obj.fig, [1 2], 'ColumnWidth', {280, '1x'});
            left = uigridlayout(main, [2 1], 'RowHeight', {'fit', 'fit'}, ...
                'Padding', [0 0 0 0], 'Scrollable', 'on');
            p  = uigridlayout(uipanel(left, 'Title','Tools'), [22 2], ...
                'RowHeight', [repmat({'fit'}, 1, 21) {36}], 'ColumnWidth', {'1x', '1x'}, ...
                'RowSpacing', 5, 'Padding', [8 6 8 6]);
            tp = uigridlayout(uipanel(left, 'Title','Time Control & Analysis'), [7 4], ...
                'RowHeight', repmat({'fit'}, 1, 7), 'ColumnWidth', {'fit', '1x', 'fit', '1x'}, ...
                'RowSpacing', 5, 'Padding', [8 6 8 6]);
            obj.ax = uiaxes(main);

            try
                disableDefaultInteractivity(obj.ax);
            catch
            end

            grid(obj.ax,'on'); axis(obj.ax,'equal');
            xlim(obj.ax,[0 100]); ylim(obj.ax,[0 100]);

            % ---- Tools panel ----
            % Paper profile (Configurations/paperProfiles): shows only what one paper uses
            place(uilabel(p,'Text','Paper profile:', 'FontWeight','bold'), 1, 1);
            profiles = paperProfiles();
            obj.paperProfileDropdown = place(uidropdown(p, ...
                'Items', cellstr([profiles.Name]), 'Value', 'Full', ...
                'ValueChangedFcn', @(s,~)obj.setPaperProfile(s.Value)), 1, 2);
            % Ready-made scenarios (Configurations/scenarioCatalog): one click loads the map,
            % targets, edges, walls and agents (of the selected Agent Type)
            place(uilabel(p,'Text','Scenario:', 'FontWeight','bold'), 2, [1 2]);
            cat = scenarioCatalog();
            obj.scenarioDropdown = place(uidropdown(p, ...
                'Items', [{'(build your own)'}, cellstr(cat.Name')], 'Value', '(build your own)', ...
                'ValueChangedFcn', @(s,~)obj.onScenarioChanged(s.Value)), 3, [1 2]);
            place(uibutton(p,'Text','Add Target', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addTarget")), 4, 1);
            place(uibutton(p,'Text','Add Agent', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addAgent")), 5, 1);
            obj.agentTypeDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('agenttype'), 'Value', 'Abstract Agent', ...
                'ValueChangedFcn', @(s,~)obj.onAgentTypeChanged(s.Value)), 5, 2);
            place(uibutton(p,'Text','Add Edge', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addEdge")), 4, 2);
            place(uibutton(p,'Text','Add Wall', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addWall")), 6, 1);
            place(uibutton(p,'Text','Idle', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("idle")), 6, 2);
            place(uibutton(p,'Text','Import Map...', ...
                'ButtonPushedFcn', @(~,~)obj.controller.importBackground()), 7, 1);
            place(uibutton(p,'Text','Clear All', ...
                'ButtonPushedFcn', @(~,~)obj.onClearAll()), 7, 2);
            place(uibutton(p,'Text','Save Layout...', ...
                'ButtonPushedFcn', @(~,~)obj.onSaveLayout()), 8, 1);
            place(uibutton(p,'Text','Load Layout...', ...
                'ButtonPushedFcn', @(~,~)obj.onLoadLayout()), 8, 2);

            % Charging stations (ChargingStation): click to place one with the power on the right
            place(uibutton(p,'Text','Add Station', ...
                'ButtonPushedFcn', @(~,~)obj.setMode("addStation")), 9, 1);
            obj.stationPowerField = place(uieditfield(p,'numeric','Value',100,'Limits',[0 Inf], ...
                'ValueDisplayFormat', '%g W', 'Tooltip', 'Charging power of the stations added next and of every station in the layout', ...
                'ValueChangedFcn', @(s,~)obj.onStationPowerChanged(s.Value)), 9, 2);

            % Settings of agents added next (agent type above, speed, acceleration, profile)
            place(uilabel(p,'Text','Agent Speed:'), 10, 1);
            obj.speedField = place(uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf]), 10, 2);
            place(uilabel(p,'Text','Agent Accel:'), 11, 1);
            obj.accelField = place(uieditfield(p,'numeric','Value',15,'Limits',[0.01 Inf], ...
                'ValueChangedFcn', @(s,~)obj.onAgentAccelChanged(s.Value)), 11, 2);
            place(uilabel(p,'Text','Speed Profile:'), 12, 1);
            obj.profileDropdown = place(uidropdown(p, ...
                'Items', cellstr(MainUI.profileLabels), 'ItemsData', cellstr(SpeedProfile.names), ...
                'Value', 'FO-0', ...
                'ValueChangedFcn', @(s,~)obj.onAgentProfileChanged(s.Value)), 12, 2);

            place(uilabel(p,'Text','Path Planner:'), 13, 1);
            obj.plannerDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('pathplanner'), 'Value', 'Dijkstra Corners', ...
                'ValueChangedFcn', @(s,~)obj.onPlannerChanged(s.Value)), 13, 2);

            % Active Policy Selector (full width so long policy names fit)
            place(uilabel(p,'Text','Active Policy:', 'FontWeight','bold'), 14, [1 2]);
            obj.policyDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('policy'), ...
                'Value', 'Random Walk', ...
                'ValueChangedFcn', @(s,~)obj.onPolicyChanged(s.Value)), 15, [1 2]);

            % Target type (its dynamics); the uncertainty mode of the default Uncertainty Target:
            % constant A_i by default (as in the paper), spikes are opt-in
            place(uilabel(p,'Text','Targets:', 'FontWeight','bold'), 16, [1 2]);
            obj.targetTypeDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('targettype'), 'Value', 'Uncertainty Target', ...
                'ValueChangedFcn', @(s,~)obj.onTargetTypeChanged(s.Value)), 17, [1 2]);
            obj.uncertaintyDropdown = place(uidropdown(p, ...
                'Items', ScenarioModel.UncertaintyModes, ...
                'Value', 'Static Uniform', ...
                'ValueChangedFcn', @(s,~)obj.onUncertaintyModeChanged(s.Value)), 18, [1 2]);

            % Objective: the metric plotted over time in the objective window
            place(uilabel(p,'Text','Objective:', 'FontWeight','bold'), 19, [1 2]);
            obj.objectiveDropdown = place(uidropdown(p, ...
                'Items', obj.registry.keysFor('metric'), 'Value', 'Average Uncertainty J(t)', ...
                'ValueChangedFcn', @(s,~)obj.onObjectiveChanged(s.Value)), 20, [1 2]);

            obj.modeLabel = place(uilabel(p,'Text','Mode: idle', 'FontWeight','bold'), 21, [1 2]);
            obj.statusLabel = place(uilabel(p,'Text','', 'WordWrap','on', ...
                'VerticalAlignment','top'), 22, [1 2]);

            obj.ax.PickableParts = 'all';
            obj.ax.HitTest = 'on';
            obj.ax.ButtonDownFcn = @(ax,~)obj.onAxesClick(ax);

            % ---- Time Control panel ----
            obj.timeLabel = place(uilabel(tp,'Text','t = 0.00 / 60.00', 'FontWeight','bold'), 1, [1 4]);

            place(uibutton(tp,'Text','Play', ...
                'ButtonPushedFcn', @(~,~)obj.onPlaySimulation()), 2, [1 2]);
            place(uibutton(tp,'Text','Pause', ...
                'ButtonPushedFcn', @(~,~)obj.sim.pause()), 2, [3 4]);
            place(uibutton(tp,'Text','Run to End', ...
                'ButtonPushedFcn', @(~,~)obj.onRunToEndSimulation()), 3, [1 4]);
            place(uibutton(tp,'Text','Reset Time', ...
                'ButtonPushedFcn', @(~,~)obj.onResetSimulation()), 4, [1 2]);
            place(uibutton(tp,'Text','Clear Plots', ...
                'ButtonPushedFcn', @(~,~)obj.onClearPlots()), 4, [3 4]);

            place(uilabel(tp,'Text','End time:'), 5, 1);
            obj.endTimeField = place(uieditfield(tp,'numeric','Value',60,'Limits',[0 Inf], ...
                'ValueChangedFcn', @(s,~)obj.sim.setEndTime(s.Value)), 5, 2);
            place(uilabel(tp,'Text','dt:'), 5, 3);
            obj.dtField = place(uieditfield(tp,'numeric','Value',0.05,'Limits',[1e-4 Inf], ...
                'ValueChangedFcn', @(s,~)obj.sim.setDt(s.Value)), 5, 4);

            place(uilabel(tp,'Text','Speed:'), 6, 1);
            obj.timeScaleSlider = place(uislider(tp, 'Limits',[0 10], 'Value',1, ...
                'ValueChangedFcn', @(s,~)obj.sim.setTimeScale(s.Value)), 6, [2 4]);

            obj.recordCheckbox = place(uicheckbox(tp,'Text','Record', 'Value', false), 7, [1 2]);
            place(uibutton(tp,'Text','Save Video...', ...
                'ButtonPushedFcn', @(~,~)obj.onSaveVideo()), 7, [3 4]);
        end

        function runLabel = getCombinedRunLabel(obj)
            % Formats a clear plot legend string combining Policy & Uncertainty mode
            pol = string(obj.policyDropdown.Value);
            unc = string(obj.uncertaintyDropdown.Value);
            switch unc
                case "Dynamic Uncertainty",    runLabel = pol + " (Dynamic)";
                case "Static Priority Target", runLabel = pol + " (Static Priority)";
                otherwise,                     runLabel = pol + " (Static Uniform)";
            end
        end

        function onPlaySimulation(obj)
            % Tag a new run label with the policy and uncertainty mode if starting from t = 0
            % Restarting a finished run: reset properly so the new plot line gets its label
            if obj.clock.isFinished(), obj.onResetSimulation(); end
            if obj.clock.currentTime == 0 && ~isempty(obj.objectiveWin)
                obj.objectiveWin.startNewRunLine(obj.getCombinedRunLabel());
            end
            if obj.clock.currentTime == 0
                obj.frameBuffer = {};  % Fresh buffer for each new run
            end
            obj.sim.play();
        end
        
        function onRunToEndSimulation(obj)
            % Restarting a finished run: reset properly so the new plot line gets its label
            if obj.clock.isFinished(), obj.onResetSimulation(); end
            if obj.clock.currentTime == 0 && ~isempty(obj.objectiveWin)
                obj.objectiveWin.startNewRunLine(obj.getCombinedRunLabel());
            end
            if obj.clock.currentTime == 0
                obj.frameBuffer = {};  % Fresh buffer for each new run
            end
            obj.sim.runToEnd();
        end
        
        function onPolicyChanged(obj, newPolicy)
            % Sync active policy across model and simulation controller
            if ~isempty(obj.model)
                if isprop(obj.model, 'setActivePolicy') || ismethod(obj.model, 'setActivePolicy')
                    obj.model.setActivePolicy(newPolicy);
                end
            end
            if ~isempty(obj.sim)
                if isprop(obj.sim, 'setActivePolicy') || ismethod(obj.sim, 'setActivePolicy')
                    obj.sim.setActivePolicy(newPolicy);
                end
            end
            obj.setStatus("Switched active policy to: " + string(newPolicy));
            if ~isempty(obj.model), obj.noteAgentFit(); end
        end

        function onUncertaintyModeChanged(obj, newMode)
            % Sync active uncertainty mode across model and simulation controller
            if ~isempty(obj.model)
                if isprop(obj.model, 'setUncertaintyMode') || ismethod(obj.model, 'setUncertaintyMode')
                    obj.model.setUncertaintyMode(newMode);
                end
            end
            if ~isempty(obj.sim)
                if isprop(obj.sim, 'setUncertaintyMode') || ismethod(obj.sim, 'setUncertaintyMode')
                    obj.sim.setUncertaintyMode(newMode);
                end
            end
            obj.setStatus("Switched uncertainty mode to: " + string(newMode));
        end
        
        function onResetSimulation(obj)
            % 1. Pause active timers
            if ~isempty(obj.sim), obj.sim.pause(); end
            
            % 2. Enforce active policy and uncertainty states before resetting
            currentPolicy = obj.policyDropdown.Value;
            currentUncertainty = obj.uncertaintyDropdown.Value;
            
            obj.onPolicyChanged(currentPolicy);
            obj.onUncertaintyModeChanged(currentUncertainty);
            
            % 3. Reset internal buffers while keeping historical trial lines
            if ~isempty(obj.objectiveWin), obj.objectiveWin.reset(); end
            if ~isempty(obj.sim), obj.sim.reset(); end
            
            % 4. Redraw axes and reset time readout
            obj.renderer.renderAll(obj.model);
            obj.setTime(0, obj.clock.endTime);
            obj.setStatus("Reset sim | Policy: " + string(currentPolicy) + " | Mode: " + string(currentUncertainty));
        end
        
        function onClearPlots(obj)
            if ~isempty(obj.objectiveWin)
                obj.objectiveWin.clearAllTrials();
                obj.setStatus("Cleared all trial comparison plots.");
            end
        end
        
        function onPlannerChanged(obj, name)
            factory = obj.registry.factoryFor('pathplanner', name);
            obj.model.setPathPlanner(factory());
            obj.renderer.renderAll(obj.model);
            obj.setStatus(sprintf('Path planner: %s', name));
        end

        function onScenarioChanged(obj, name)
            if name == "(build your own)", return; end
            obj.loadScenario(name);
        end

        function onAgentTypeChanged(obj, name)
            obj.model.setAgentType(name);
            obj.setStatus(sprintf('Agent type: %s (applies to new agents)', name));
        end

        function onTargetTypeChanged(obj, name)
            % Rebuilds the scenario with targets of the new type (positions, rates, edges and
            % agents are kept)
            obj.model.setTargetType(name);
            obj.renderer.clearAxes();
            obj.renderer.renderAll(obj.model);
            obj.onResetSimulation();
            obj.setStatus(sprintf('Target type: %s', name));
        end

        function onObjectiveChanged(obj, name)
            % The metric plotted over time (the model's first metric)
            obj.model.setMetrics(string(name));
            m = obj.model.metrics{1};
            obj.objectiveWin.setObjective(m.label, m.plotMax);
            obj.setStatus(sprintf('Objective: %s', name));
        end

        function onStationPowerChanged(obj, value)
            % The power of the stations added next and of every station already placed (so a
            % point of the energy paper's station sweep can be set up here)
            obj.controller.stationPower = value;
            st = obj.model.targets(arrayfun(@(t) isa(t, 'ChargingStation'), obj.model.targets));
            for t = st(:)', t.chargePower = value; end
            if ~isempty(st)
                obj.setStatus(sprintf('Station power: %g W (also the %d station(s) in the layout)', value, numel(st)));
            end
        end

        function onAgentAccelChanged(obj, value)
            obj.model.agentSpec.maxAccel = value;
            obj.accelField.Value = value;
        end

        function onAgentProfileChanged(obj, name)
            obj.model.agentSpec.profile = string(name);
            MainUI.showValue(obj.profileDropdown, SpeedProfile.canonical(name));
            obj.setStatus(sprintf('Speed profile: %s (applies to new agents)', name));
        end

        function noteAgentFit(obj)
            % Shows what to expect when the agents do not suit the active policy
            msg = obj.model.checkAgentFit();
            if msg ~= "", obj.setStatus("Note: " + msg); end
        end

        function onRunStateChanged(obj, running)
            % Lock component choices while a run is in progress
            state = 'on';
            if running, state = 'off'; end
            dds = {obj.plannerDropdown, obj.policyDropdown, obj.agentTypeDropdown, obj.uncertaintyDropdown, ...
                obj.profileDropdown, obj.targetTypeDropdown, obj.objectiveDropdown, obj.paperProfileDropdown};
            for k = 1:numel(dds)
                if ~isempty(dds{k}) && isvalid(dds{k}), dds{k}.Enable = state; end
            end
        end

        function forceCloseAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            delete(timerfindall); 
            if ~isempty(obj.objectiveWin) && isprop(obj.objectiveWin, 'fig') && isgraphics(obj.objectiveWin.fig)
                delete(obj.objectiveWin.fig);
            end
            delete(obj.fig);
        end
        
        function onSaveLayout(obj)
            [f,d] = uiputfile('*.mat','Save layout');
            if isequal(f,0), return; end
            obj.saveLayout(fullfile(d,f));
        end

        function onLoadLayout(obj)
            [f,d] = uigetfile('*.mat','Load layout');
            if isequal(f,0), return; end
            obj.loadLayout(fullfile(d,f));
        end

        function onClearAll(obj)
            if ~isempty(obj.sim), obj.sim.pause(); end
            obj.controller.clearAll();
            obj.clock.reset();
            if ~isempty(obj.objectiveWin), obj.objectiveWin.clearAllTrials(); end
            obj.setTime(0, obj.clock.endTime);
            obj.setStatus("Cleared environment layout and trials.");
        end
        
        function setMode(obj, m)
            obj.modeLabel.Text = "Mode: " + string(m);
            obj.controller.setMode(m);
        end
        
        function setStatus(obj, msg)
            if isvalid(obj.statusLabel)
                obj.statusLabel.Text = msg;
            end
        end
        
        function setTime(obj, t, tEnd)
            if isempty(obj.timeLabel) || ~isvalid(obj.timeLabel), return; end
            obj.timeLabel.Text = sprintf('t = %.2f / %.2f', t, tEnd);
            if t > 0 && obj.recordCheckbox.Value
                try
                    drawnow limitrate;
                    obj.frameBuffer{end+1} = getframe(obj.fig);
                catch
                end
            end
        end
        
        function onAxesClick(obj, ax)
            cp = ax.CurrentPoint;
            obj.controller.onCanvasClick([cp(1,1) cp(1,2)], obj.speedField.Value);
        end
        
        function safeOnMouseMove(obj)
            if isempty(obj.ax) || ~isvalid(obj.ax), return; end
            cp = obj.ax.CurrentPoint;
            obj.controller.onCanvasMove([cp(1,1) cp(1,2)]);
        end

        function onSaveVideo(obj)
            if isempty(obj.frameBuffer)
                uialert(obj.fig, ...
                    'No frames recorded yet. Run the simulation first, then save.', ...
                    'No Recording');
                return;
            end
            [f, d] = uiputfile( ...
                {'*.mp4','MPEG-4 Video (*.mp4)'; '*.avi','AVI Video (*.avi)'}, ...
                'Save Simulation Video');
            if isequal(f, 0), return; end
            [~,~,ext] = fileparts(f);
            profile = 'MPEG-4';
            if strcmpi(ext, '.avi'), profile = 'Motion JPEG AVI'; end
            obj.setStatus('Saving video...');
            try
                vw = VideoWriter(fullfile(d, f), profile);
                vw.FrameRate = 20;
                open(vw);
                for i = 1:numel(obj.frameBuffer)
                    writeVideo(vw, obj.frameBuffer{i});
                end
                close(vw);
                obj.setStatus('Video saved: ' + string(f));
            catch err
                obj.setStatus('Video save failed: ' + string(err.message));
            end
        end
    end
end

function c = place(c, row, col)
% Assigns a component to a uigridlayout cell and returns it
c.Layout.Row = row;
c.Layout.Column = col;
end
