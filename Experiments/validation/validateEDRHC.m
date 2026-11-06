function T = validateEDRHC(opts)
% Compares ED-RHC with alpha = 0.5 (the unweighted RHC method) and with the nominal alpha =
% 1/|N_i|^2 (the RHC-alpha method) with the legacy JavaScript simulator
% (Simulator_PerMon_JS, github shiran27/PersistentMonitoring), which produced the results of
% Welikala and Cassandras, Automatica 2021. Each JS preset is in Configurations/Legacy/ (scaled
% by 1/6 to the 100 x 100 canvas, with speed 50/6 so the transit times are the JS ones) and is
% run under the JS settings: first-order agents (Linear Agent), A = 1, B = 10, R(0) = 0.5,
% T = 500 s, dt = 0.01 s, H = 250, and service only while an agent is at the target.
%   T = validateEDRHC()              all presets, both alphas (~10 min)
%   T = validateEDRHC(Presets=[1 4]) a subset
%   T = validateEDRHC(CapHorizon=true) with the horizon capped at T - t near the mission end
% JsJ holds the JS J(T), read from simulateHybridSystemFast on 2026-10-20; three of them are in
% the paper's Table I (1-Agent 5T = SASE2, 2-Agent 5T = MASE2, 3-Agent General = MASE1).
arguments
    opts.Presets (1,:) double = 1:8
    opts.Dt (1,1) double = 0.01
    opts.CapHorizon (1,1) logical = false   % EDRHCPolicy.capHorizon (the JS simulator caps it)
end
presets = {
    % layout file            JS RHC     JS RHC-alpha (methods 3 and 4)
    "JS_1Agent_3T.mat",       22.359,    22.905
    "JS_1Agent_4T.mat",       47.113,    47.097
    "JS_1Agent_5T.mat",      141.425,   121.425
    "JS_2Agent_5T.mat",       63.692,    40.359
    "JS_3Agent_9T.mat",       99.369,    76.299
    "JS_3Agent_Maze.mat",    140.919,   138.300
    "JS_3Agent_General.mat", 105.630,    96.631
    "JS_3Agent_Room.mat",    118.175,   103.748};
alphas = {0.5, "nominal"};
labels = ["alpha 0.5", "alpha nominal"];
rows = {};
for k = opts.Presets
    for c = 1:2
        p = struct('Layout', fullfile("Legacy", presets{k, 1}), 'AgentStarts', "layout", ...
            'Policy', "ED-RHC", 'Policy_alpha', alphas{c}, 'Planner', "Dijkstra Corners", 'AgentType', "Linear Agent", ...
            'UncertaintyMode', "Static Uniform", 'A_base', 1, 'B', 10, 'EndTime', 500, ...
            'Dt', opts.Dt, 'DetectionRadius', 1e-6, 'Policy_H', 250, 'Policy_capHorizon', opts.CapHorizon);
        s = load(fullfile(fileparts(which('MainUI')), 'Configurations', 'Legacy', presets{k, 1}), 's');
        p.AgentSpeed = s.s.agentSpeed(1);
        tic;
        out = runTrial(p);
        jsJ = presets{k, c + 1};
        rows(end+1, :) = {erase(presets{k, 1}, ".mat"), labels(c), jsJ, out.finalJ, ...
            100 * (out.finalJ - jsJ) / jsJ, toc}; %#ok<AGROW>
        fprintf('%-18s %-13s JS %8.3f  MATLAB %8.3f  (%+6.2f%%)  %.0f s\n', rows{end, :});
    end
end
T = cell2table(rows, 'VariableNames', {'Preset', 'Alpha', 'JsJ', 'MatlabJ', 'DiffPercent', 'Seconds'});
end
