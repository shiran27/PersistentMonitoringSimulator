function files = makeReadmeScreenshots(opts)
% The README's four screenshots (screenshots/*.png), captured from MainUI through the same
% callbacks as the GUI controls: the main window with the reference scenario after 60 s, a
% scenario as loaded, a run in progress, and the objective window with two runs overlaid. The
% windows open on the secondary monitor when there is one and close when done.
arguments
    opts.Monitor (1,1) double = MainUI.secondaryMonitor()
end
root = fileparts(which('MainUI'));
out = fullfile(root, 'screenshots');
files = fullfile(out, ["overview.png", "scenario.png", "running.png", "objective.png"]);
ui = MainUI('Monitor', opts.Monitor);
cleanup = onCleanup(@() close(ui.fig));
% overview: the reference scenario (Physical Agents, ED-RHC) after 60 s
choose(ui.agentTypeDropdown, 'Physical Agent');
choose(ui.scenarioDropdown, 'Lab floor (Stevens)');
choose(ui.policyDropdown, 'ED-RHC');
setEndTime(ui, 60); rng(1); press(ui, 'Run to End');
exportapp(ui.fig, files(1));
% the objective window: the same scenario under Random Walk, overlaid on the ED-RHC run
press(ui, 'Reset Time'); choose(ui.policyDropdown, 'Random Walk'); rng(1); press(ui, 'Run to End');
exportapp(ui.objectiveWin.fig, files(4));
% a scenario as loaded: targets, edges, walls and agents
choose(ui.scenarioDropdown, 'Office floor with charging dock');
exportapp(ui.fig, files(2));
% a run in progress: agents on their paths
choose(ui.scenarioDropdown, 'Campus'); choose(ui.policyDropdown, 'ED-RHC');
setEndTime(ui, 25); rng(1); press(ui, 'Run to End');
exportapp(ui.fig, files(3));
end

function choose(dropdown, value)
dropdown.Value = value;
dropdown.ValueChangedFcn(dropdown, []);
end

function setEndTime(ui, T)
ui.endTimeField.Value = T;
ui.endTimeField.ValueChangedFcn(ui.endTimeField, []);
end

function press(ui, text)
b = findall(ui.fig, 'Type', 'uibutton', 'Text', text);
b.ButtonPushedFcn(b, []);
end
