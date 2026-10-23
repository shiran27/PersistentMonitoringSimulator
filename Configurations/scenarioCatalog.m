function T = scenarioCatalog()
% Ready-made scenarios for the GUI's Scenario dropdown (and MainUI.loadScenario): name, layout
% file and a one-line description. Each layout holds the targets, edges, walls and agent starts;
% the maps also name a background image (Configurations/backgrounds/, drawn by
% makeScenarioLayouts). Agents take the selected Agent Type, so every scenario runs with either
% agent type and any policy.
here = fileparts(mfilename('fullpath'));
rows = {
    "Lab floor (Stevens)", "ReportLayout.mat",  "Stevens lab floor plan: 11 targets, 5 agents (the simulator paper's reference scenario)."
    "Office floor",        "OfficeFloor.mat",   "Office and lab building floor: 8 rooms with doors, halls and a lobby: 14 targets, 3 robots."
    "Office floor with charging dock", "OfficeDock.mat", "The Office floor with a charging dock by the lobby and small batteries (2.5 Wh): for Battery-Aware ED-RHC."
    "City patrol",         "CityPatrol.mat",    "Police patrol on a street grid: 16 intersections and 5 sites, 4 patrol cars on the roads."
    "Harbor patrol",       "HarborPatrol.mat",  "Harbor, piers, breakwater and two islands: 12 buoys and berths, 3 boats."
    "Campus",              "Campus.mat",        "Building complex with a pond, quad and parking: 11 sites, 5 agents."
    "Museum (event detection example)", "Museum.mat", "Beginner's example (Examples/EventDetection): four galleries and a hall with event targets, 3 robots at speed 4."
    "JS: 1 agent, 3 targets",  "Legacy/JS_1Agent_3T.mat",      "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 1 agent, 4 targets",  "Legacy/JS_1Agent_4T.mat",      "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 1 agent, 5 targets",  "Legacy/JS_1Agent_5T.mat",      "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 2 agents, 5 targets", "Legacy/JS_2Agent_5T.mat",      "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 3 agents, 9 targets", "Legacy/JS_3Agent_9T.mat",      "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 3 agents, maze",      "Legacy/JS_3Agent_Maze.mat",    "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 3 agents, general",   "Legacy/JS_3Agent_General.mat", "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "JS: 3 agents, room",      "Legacy/JS_3Agent_Room.mat",    "Preset of the legacy JS simulator (RHC papers), scaled to the canvas."
    "Collab: 2 agents, 5 targets, group target", "Collab/Collab_2Agent_5T_Group.mat", "JS 2 agents, 5 targets with A = 12 > B = 10 at target 3, which only two agents can clear (collaboration paper; Collab-ED-RHC, T = 500 s, dt = 0.02 s)."
    "Collab: 3 agents, general, group targets", "Collab/Collab_3Agent_General_Group.mat", "JS 3 agents, general with A = 12 > B = 10 at targets 3 and 8 (collaboration paper; Collab-ED-RHC, T = 500 s, dt = 0.02 s)."
    "Collab: general, 2 agents", "Collab/Collab_General_N2.mat", "JS general layout with 2 agents spread over the targets (collaboration paper's team-size study)."
    "Collab: general, 3 agents", "Collab/Collab_General_N3.mat", "JS general layout with 3 agents spread over the targets (collaboration paper's team-size study)."
    "Collab: general, 4 agents", "Collab/Collab_General_N4.mat", "JS general layout with 4 agents spread over the targets (collaboration paper's team-size study)."
    "Collab: general, 5 agents", "Collab/Collab_General_N5.mat", "JS general layout with 5 agents spread over the targets (collaboration paper's team-size study)."
    "Collab: general, 6 agents", "Collab/Collab_General_N6.mat", "JS general layout with 6 agents spread over the targets (collaboration paper's team-size study)."
    "Energy: office, 1 dock",   "Energy/Energy_Office_1Dock.mat",   "Office floor with a 120 W charging dock (1 dock) and 3 robots of the energy-aware paper: 1 m/s, DC-motor power model, 6 Wh batteries that stop when empty."
    "Energy: office, 2 docks",  "Energy/Energy_Office_2Docks.mat",  "The office floor of the energy-aware paper with a 2-dock charging station (120 W) and 3 robots."
    "Energy: campus, 2 stations", "Energy/Energy_Campus_2Stations.mat", "Campus with two 120 W charging stations of 2 docks each and 6 robots of the energy-aware paper."
    };
T = table([rows{:, 1}]', fullfile(here, [rows{:, 2}]'), [rows{:, 3}]', ...
    'VariableNames', {'Name', 'File', 'Description'});
end
