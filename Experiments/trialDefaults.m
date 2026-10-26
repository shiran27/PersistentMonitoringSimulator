function p = trialDefaults()
% Default trial parameters. Shared by runTrial (fills unset parameters) and
% analyzeResults (knows which table columns are parameters, fills missing ones).
% These are the settings of the Joint RHC study (2 agents, spikes, B = 20), not the
% simulator paper's (Static Uniform, A = 1, B = 10, ReportLayout.mat with 5 agents; see
% README). The ablation and diagnostic scripts in Experiments/ rely on them; the paper's
% settings are in Experiments/paper/paperDefaults.m.
p = struct( ...
    'Policy', "Joint RHC", ...                    % any ComponentRegistry policy name
    'Planner', "Dijkstra Corners", ...            % any ComponentRegistry path planner name
    'AgentType', "Linear Agent", ...              % any ComponentRegistry agent type name
    'UncertaintyMode', "Dynamic Uncertainty", ... % | "Static Priority Target" | "Static Uniform"
    'Layout', "ExampleLayout1.mat", ...           % file in Configurations/
    'NumAgents', 2, ...                           % ignored when AgentStarts is given
    'AgentStarts', "auto", ...                    % "auto" (spread evenly) | "layout" (stored starts) | e.g. [1 5]
    'B', 20, ...                                  % decay rate per agent
    'A_base', 1, ...                              % baseline growth (all targets but target 1)
    'A_priority', 5, ...                          % target 1 baseline (priority + dynamic modes)
    'A_spike', 28, ...                            % growth during a spike (dynamic mode)
    'SpikeDuration', 10, ...
    'SpikeInterval', 250, ...
    'SpikeStagger', 50, ...                       % spike offset between consecutive targets
    'AgentSpeed', 15, ...
    'DetectionRadius', 1.2, ...                   % an agent this close to a target services it
    'EndTime', 500, ...
    'Dt', 0.05, ...
    'TargetRates', "params", ...                  % "params": A_base and B for every target | "layout": the layout's own
    'AgentSettings', "params", ...                % "params": agents from AgentType, AgentSpeed, Agent_<setting> | "layout": the layout's agents and settings
    'Seed', -1);                                  % rng seed set before simulating; -1 = leave rng as is
end
