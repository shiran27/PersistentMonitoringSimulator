classdef guiTest < matlab.uitest.TestCase
    % Drives the real MainUI with the App Testing Framework (needs a display; windows open
    % and close on a secondary monitor if there is one). Run from the repo root:
    %   matlab -batch "addpath(genpath(pwd)); assertSuccess(runtests('guiTest'))"
    % Screenshots go to results/gui/ (git-ignored). File dialogs are bypassed by calling
    % MainUI.loadLayout/saveLayout and ScenarioRenderer.loadBackgroundImage with a path.

    properties
        ui
        root
        outDir
    end

    methods (TestMethodSetup)
        function openUI(tc)
            tc.root = fileparts(fileparts(mfilename('fullpath')));
            tc.outDir = fullfile(tc.root, 'results', 'gui');
            if ~isfolder(tc.outDir), mkdir(tc.outDir); end
            rng(0);
            tc.ui = MainUI('Monitor', MainUI.secondaryMonitor());
            tc.addTeardown(@() closeUI(tc.ui));
            waitForLayout(tc.ui);
        end
    end

    methods (Test)
        function showsPaperDefaults(tc)
            tc.verifyEqual(tc.ui.uncertaintyDropdown.Value, 'Static Uniform');
            tc.verifyEqual(tc.ui.model.uncertaintyMode, 'Static Uniform');
            tc.verifyNotEmpty(findall(tc.ui.fig, 'Type', 'uilabel', 'Text', 'Agent Speed:'));
        end

        function opensOnRequestedMonitor(tc)
            mons = get(groot, 'MonitorPositions');
            m = mons(MainUI.secondaryMonitor(), :);
            drawnow;
            for f = [tc.ui.fig, tc.ui.objectiveWin.fig]
                p = f.Position;
                tc.verifyTrue(p(1) >= m(1) && p(1) + p(3) <= m(1) + m(3) && p(2) >= m(2) ...
                    && p(2) + p(4) <= m(2) + m(4), ...
                    sprintf('"%s" at %s lies on monitor %s', f.Name, mat2str(p), mat2str(m)));
            end
        end

        function loadsReportLayoutAndRunsToEnd(tc)
            tc.ui.renderer.loadBackgroundImage(fullfile(tc.root, 'LabLayout.png'));
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            m = tc.ui.model;
            tc.verifyEqual([m.agents.initialTargetIdx], [1 6 2 3 5]);
            tc.verifyTrue(all(arrayfun(@(a) isgraphics(a.graphicHandle), m.agents)), 'agents drawn');
            tc.verifyEqual(tc.ui.speedField.Value, 15);
            tc.verifyEqual(tc.ui.uncertaintyDropdown.Value, 'Static Uniform');
            snap(tc, 'report_layout_loaded');

            tc.choose(tc.ui.policyDropdown, 'Classical RHC [1]');
            tc.type(tc.ui.endTimeField, 60);
            tc.press(button(tc, 'Run to End'));
            tc.verifyEqual(tc.ui.clock.currentTime, 60, 'AbsTol', 1e-9);
            J = tc.ui.objectiveWin.currentLine.YData;
            tc.verifyTrue(any(isfinite(J)) && all(J(isfinite(J)) >= 0), 'objective plotted');
            tc.verifyEqual(char(tc.ui.policyDropdown.Enable), 'on', 'dropdowns unlocked after the run');
            heights = arrayfun(@(t) t.barHandle.Position(4), m.targets);
            fullHeights = arrayfun(@(t) t.barFrameHandle.Position(4), m.targets);
            tc.verifyLessThanOrEqual(nnz(heights >= fullHeights - 1e-9), 1, 'at most the most uncertain bar is full');
            tops = arrayfun(@(t) sum(t.barFrameHandle.Position([2 4])), m.targets);
            tc.verifyLessThanOrEqual(max(tops), 100, 'bars stay inside the canvas');
            snap(tc, 'report_layout_after_60s');
        end

        function guiAndHeadlessRunsAgree(tc)
            % The same scenario run through the GUI and through runTrial gives the same J(T).
            % ED-RHC is deterministic.
            T = 60;
            p = paperDefaults();
            tc.choose(tc.ui.agentTypeDropdown, char(p.AgentType));   % same agents as runTrial
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            tc.choose(tc.ui.policyDropdown, 'ED-RHC');
            tc.type(tc.ui.endTimeField, T);
            tc.press(button(tc, 'Run to End'));
            Jgui = tc.ui.model.cumUncertaintyIntegral / tc.ui.clock.currentTime;
            p.EndTime = T;
            p.Log = false;
            out = runTrial(p);
            tc.verifyEqual(Jgui, out.finalJ, 'RelTol', 1e-12);
        end

        function playsPausesAndResets(tc)
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            tc.press(button(tc, 'Play'));
            tc.verifyEqual(char(tc.ui.policyDropdown.Enable), 'off', 'dropdowns locked while running');
            pause(2);
            tc.press(button(tc, 'Pause'));
            t = tc.ui.clock.currentTime;
            tc.verifyGreaterThan(t, 0);
            pause(0.5);
            tc.verifyEqual(tc.ui.clock.currentTime, t, 'clock stopped after Pause');
            tc.press(button(tc, 'Reset Time'));
            tc.verifyEqual(tc.ui.clock.currentTime, 0);
            m = tc.ui.model;
            pos = cell2mat(arrayfun(@(a) a.state.pos, m.agents(:), 'UniformOutput', false));
            starts = reshape([m.targets([1 6 2 3 5]).position], 2, []).';
            tc.verifyEqual(pos, starts, 'agents back on their start targets');
        end

        function savesClearsAndReloads(tc)
            tc.choose(tc.ui.agentTypeDropdown, 'Physical Agent');
            tc.choose(tc.ui.profileDropdown, 'SO (smooth)');
            tc.type(tc.ui.accelField, 10);
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            a = tc.ui.model.agents;
            tc.verifyClass(a, 'PhysicalAgent');   % loaded agents use the selected settings
            tc.verifyEqual([a.profile], repmat("SO", 1, numel(a)));
            tc.verifyEqual(arrayfun(@(x) x.state.maxAccel, a), repmat(10, 1, numel(a)));
            saved = tc.ui.model.exportLayout();
            f = [tempname '.mat'];
            tc.addTeardown(@() delete(f));
            tc.ui.saveLayout(f);
            tc.press(button(tc, 'Clear All'));
            tc.verifyEmpty(tc.ui.model.targets);
            tc.verifyEmpty(tc.ui.model.agents);
            tc.ui.loadLayout(f);
            tc.verifyEqual(tc.ui.model.exportLayout(), saved);
        end

        function buildsScenarioByClicking(tc)
            % Click positions follow how a user works: on empty canvas, then on the target dots
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [20 80]);
            tc.press(tc.ui.ax, [70 40]);
            tc.verifyEqual(numel(tc.ui.model.targets), 2, 'targets added on empty canvas');

            tc.press(button(tc, 'Add Edge'));
            tc.press(tc.ui.ax, [20 80]);
            tc.press(tc.ui.ax, [70 40]);
            tc.verifyEqual(numel(tc.ui.model.edges), 1, 'edge added by clicking both target dots');

            tc.press(button(tc, 'Add Agent'));
            tc.press(tc.ui.ax, [20 80]);
            tc.verifyEqual(numel(tc.ui.model.agents), 1, 'agent added by clicking a target dot');
            tc.choose(tc.ui.agentTypeDropdown, 'Physical Agent');
            tc.press(tc.ui.ax, [20 80]);
            tc.verifyEqual(numel(tc.ui.model.agents), 2, 'second agent added by clicking the agent on that target');
            tc.verifyEqual(arrayfun(@(a) string(class(a)), tc.ui.model.agents), ["AbstractAgent", "PhysicalAgent"], ...
                'agent types can be mixed');

            tc.press(button(tc, 'Add Wall'));
            tc.press(tc.ui.ax, [45 45]);
            tc.press(tc.ui.ax, [45 75]);
            tc.verifyEqual(size(tc.ui.model.walls, 1), 1, 'wall added');
            if ~isempty(tc.ui.model.edges)
                tc.verifyGreaterThan(size(tc.ui.model.edges(1).curvePoints, 1), 2, 'edge replanned around the wall');
            end
            tc.press(tc.ui.ax, [45 60]);
            tc.press(tc.ui.ax, [60 60]);
            tc.verifyEqual(size(tc.ui.model.walls, 1), 2, 'wall started on top of an existing wall');
            snap(tc, 'built_by_clicking');
        end

        function runsAgentOnTargetWithoutEdges(tc)
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [50 50]);
            tc.press(button(tc, 'Add Agent'));
            tc.press(tc.ui.ax, [50 50]);
            tc.type(tc.ui.endTimeField, 5);
            tc.press(button(tc, 'Run to End'));
            tc.verifyEqual(tc.ui.clock.currentTime, 5, 'AbsTol', 1e-9, 'run finished without error');
        end

        function loadsEveryScenario(tc)
            % One click on the Scenario dropdown loads the map, targets, edges, walls and agents
            cat = scenarioCatalog();
            for name = cat.Name(~startsWith(cat.Name, "JS:"))'
                tc.choose(tc.ui.scenarioDropdown, char(name));
                tmp = load(cat.File(cat.Name == name), 's');
                m = tc.ui.model;
                tc.verifyEqual(numel(m.targets), size(tmp.s.targets, 1), name + ": targets");
                tc.verifyEqual(numel(m.agents), numel(tmp.s.agentStarts), name + ": agents");
                if isfield(tmp.s, 'background') && strlength(string(tmp.s.background)) > 0
                    tc.verifyTrue(isgraphics(tc.ui.renderer.bgImageHandle), name + ": background shown");
                end
                tc.type(tc.ui.endTimeField, 3);
                tc.press(button(tc, 'Run to End'));
                tc.verifyTrue(isfinite(m.cumUncertaintyIntegral), name + ": runs");
                snap(tc, char("scenario_" + lower(regexprep(name, '[^A-Za-z]', ''))));
            end
        end

        function addsStationByClicking(tc)
            % Add Station places a charging station with the power typed next to the button
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [20 50]);
            tc.type(tc.ui.stationPowerField, 250);
            tc.press(button(tc, 'Add Station'));
            drawnow;   % let the mode switch land before the click (after a long test it can lag)
            tc.assertEqual(tc.ui.controller.mode, "addStation", 'Add Station switches the mode');
            tc.press(tc.ui.ax, [70 50]);
            drawnow;
            m = tc.ui.model;
            tc.verifyEqual(numel(m.targets), 2, 'station added by clicking');
            tc.verifyClass(m.targets(2), 'ChargingStation');
            tc.verifyEqual(m.targets(2).chargePower, 250, 'station power from the field');
            tc.press(button(tc, 'Add Edge'));
            tc.press(tc.ui.ax, [20 50]);
            tc.press(tc.ui.ax, [70 50]);
            tc.verifyEqual(numel(m.edges), 1, 'station connected like a target');
            snap(tc, 'station_added');
        end

        function paperProfileFiltersMenus(tc)
            % A paper profile shows only what its paper uses; the model follows the shown
            % selections; a layout may still use a hidden option; Full shows everything again
            tc.verifyEqual(tc.ui.paperProfileDropdown.Value, 'Full');
            tc.choose(tc.ui.paperProfileDropdown, 'Energy paper');
            tc.verifyEqual(sort(string(tc.ui.policyDropdown.Items)), ...
                sort(["ED-RHC", "ED-ORHC-SO", "ED-ORHC-FO-3", "Battery-Aware ED-RHC", "Energy-Aware ED-RHC"]));
            tc.verifyEqual(string(tc.ui.profileDropdown.ItemsData), ["SO", "FO-3"]);
            tc.verifyEqual(tc.ui.model.activePolicy, tc.ui.policyDropdown.Value, 'model follows the shown policy');
            tc.verifyEqual(tc.ui.model.agentSpec.profile, string(tc.ui.profileDropdown.Value), ...
                'new agents take the shown speed profile');
            snap(tc, 'paper_profile_energy');
            tc.ui.loadLayout(fullfile(tc.root, 'Configurations', 'ReportLayout.mat'));
            tc.verifyEqual(string(tc.ui.profileDropdown.Value), tc.ui.model.agents(1).profile, ...
                'a loaded layout shows its agents'' profile even if the paper profile hides it');
            tc.choose(tc.ui.paperProfileDropdown, 'Collaboration paper');
            tc.verifyEqual(sort(string(tc.ui.policyDropdown.Items)), sort(["ED-RHC", "Collab-ED-RHC"]));
            names = scenarioCatalog().Name;
            maps = ["Lab floor (Stevens)", "Office floor", "City patrol", "Harbor patrol", "Campus"];
            shown = names(startsWith(names, ["JS: ", "Collab: "]) | ismember(names, maps));
            tc.verifyEqual(sort(string(tc.ui.scenarioDropdown.Items(2:end))), sort(shown(:)'), ...
                'JS presets, Collab setups and the five maps');
            tc.choose(tc.ui.scenarioDropdown, 'JS: 3 agents, general');
            tc.verifyEqual(numel(tc.ui.model.agents), 3);
            tc.choose(tc.ui.paperProfileDropdown, 'Full');
            tc.verifyEqual(numel(tc.ui.policyDropdown.Items), numel(tc.ui.registry.keysFor('policy')));
            tc.verifyEqual(numel(tc.ui.scenarioDropdown.Items), height(scenarioCatalog()) + 1);
        end

        function addsTargetsOverBackgroundImage(tc)
            tc.ui.renderer.loadBackgroundImage(fullfile(tc.root, 'LabLayout.png'));
            tc.press(button(tc, 'Add Target'));
            tc.press(tc.ui.ax, [30 30]);
            tc.verifyEqual(numel(tc.ui.model.targets), 1, 'target added by clicking on the background image');
        end
    end

    methods (Access = private)
        function b = button(tc, text)
            b = findall(tc.ui.fig, 'Type', 'uibutton', 'Text', text);
            tc.assertNumElements(b, 1, "button " + text);
        end

        function snap(tc, name)
            drawnow;
            exportapp(tc.ui.fig, fullfile(tc.outDir, [name '.png']));
            exportapp(tc.ui.objectiveWin.fig, fullfile(tc.outDir, [name '_objective.png']));
        end
    end
end

function waitForLayout(ui)
% Clicks on the canvas land where its laid-out position says. A new window lays out its grid
% asynchronously (slowly after a long test), and until then the axes keep a default 400 x 300
% px size, so wait until their size has settled
drawnow;
last = [];
t0 = tic;
while toc(t0) < 10
    p = getpixelposition(ui.ax, true);
    if isequal(p, last) && p(3) > 400, break; end
    last = p;
    pause(0.1); drawnow;
end
end

function closeUI(ui)
% CloseRequestFcn stops the timer and closes both windows
if isvalid(ui.fig), close(ui.fig); end
end
