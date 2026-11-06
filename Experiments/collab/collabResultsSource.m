%% Collaboration Paper Results
% This live script regenerates every figure and result of the collaboration paper (W. Getts
% and S. Welikala, "Collaborative event-driven receding horizon control for persistent
% monitoring in network systems", |Paper_ActiveCollaboration/main.tex|): the illustrative
% figures of Secs. III and VI, the benchmark configurations and maps, and the tables and plots
% of Sec. VIII. Run all sections (Live Editor: *Run All*). To get the same report without the Live
% Editor, run from the repository root:
%
%   matlab -batch "addpath(genpath(pwd)); exportCollabResults"
%
% The controller is |Collab-ED-RHC| (|Components/AgentPolicy/CollabEDRHCPolicy.m|), compared
% with |ED-RHC| (alpha = 0.5) and |ED-RHC-alpha| (nominal alpha). Every setup is a layout file
% that the GUI also lists (Scenario dropdown): the JS presets in |Configurations/Legacy/|, the
% collaboration setups in |Configurations/Collab/|, and the simulator's maps (Lab floor, Office
% floor, City patrol, Harbor patrol, Campus) in |Configurations/|. Results go to |results/collab/|; when
% the paper's folder sits next to this repository (|../Paper_ActiveCollaboration|), its
% |Figures/Collab/| and |generated/| folders receive the figures and tables. Simulations already
% saved in |results/collab/runs/| are not run again (delete them to rerun). This file is built
% from |Experiments/collab/collabResultsSource.m| by |buildCollabResults|.

%% Settings
% |QuickMode| shortens every run to 60 s and keeps two configurations and one map, to check the
% pipeline in a few minutes. A full run (120 simulations) took about 3.5 hours on a 10-core
% computer that also ran other jobs, with one parallel worker per core (Parallel Computing
% Toolbox, used when available); it takes less on an idle computer and longer on one core.
% Every run is deterministic, so serial and parallel runs give the same results.
QuickMode = false;
quick = getenv("COLLAB_RESULTS_QUICK");   % set by exportCollabResults(Quick=true)
if quick ~= "", QuickMode = quick == "1"; end
root = fileparts(which('MainUI'));
assert(root ~= "", 'Make the repository root (the folder with MainUI.m) the current folder.');
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
warning('off', 'runTrial:AgentPolicyFit');
outDir = fullfile(root, 'results', 'collab');
if QuickMode, outDir = fullfile(root, 'results', 'collab_quick'); end
fprintf('%s mode, output in %s\n', string(QuickMode), outDir);

%% Layouts of the experiments
% Writes |Configurations/Collab/|: the two configurations with targets that only two agents can
% clear (A_i = 12 > B_i = 10) and the General configuration with N = 2..6 agents.
layouts = makeCollabLayouts()

%% Figs. 1-4: uncertainty dynamics of the four target classes
% Exact trajectories from the recursion of Lemma 1 (|CollabTargetCost.lindley|), with A_i = 1
% and B_i = 1.5 for every agent: an avoided target, a target visited by three agents, a target
% abandoned by three agents, and a target traded from two leaving to two arriving agents.
% Fig. 5: agent neighborhoods and cost sets under ED-RHC and Collab-ED-RHC.
% Figs. 6-7: the benchmark configurations with the targets of the group study, and the
% team-size layouts. The maps of the map study, drawn as in the GUI (background, walls, paths).
collabFigures(outDir);
showImages(fullfile(outDir, ["fig_class_avoided", "fig_class_visited", "fig_class_abandoned", ...
    "fig_class_traded", "fig_neighborhoods", "fig_setups", "fig_setups_team", "fig_maps"] + ".png"));

%% Simulations
% Benchmark (8 configurations x 4 methods and 3 ablations), group study (2 configurations x 4
% methods), team size (N = 2..6, 3 methods), maps (5 maps x 4 methods, and Collab (B) without
% the terminal correction) and the group study with seeds 2-5 of the stream that draws the
% candidate plans of a group (Collab (A) and (B); seed 1 is the default). Each run is saved in
% |runs/<id>.mat|.
runs = collabExperiments("list", Quick=QuickMode);
fprintf('%d runs\n', height(runs));
collabExperiments("run", Quick=QuickMode, ResultsDir=outDir);

%% Tables, plots and the numbers quoted in Sec. VIII
% J_T of every run, the LaTeX tables of Sec. VIII, the J(t) and R_i(t) plots, and
% |collab_macros.tex|, which defines every number the text quotes (full mode only).
T = collabExperiments("collect", Quick=QuickMode, ResultsDir=outDir);
disp(T(:, {'Study', 'Scenario', 'Method', 'J', 'Solves', 'SolveCPU', 'ShiftEvents', 'Events'}))
showImages(fullfile(outDir, ["fig_J_general", "fig_R_group"] + ".png"));

%% Output files
% The figures go to the paper's |Figures/Collab/|, and the tables, the macros and |runs.csv| to
% its |generated/| folder.
paperDir = fullfile(fileparts(root), 'Paper_ActiveCollaboration');
if ~QuickMode && isfolder(paperDir)
    figOut = fullfile(paperDir, 'Figures', 'Collab');
    gen = fullfile(paperDir, 'generated');
    if ~isfolder(figOut), mkdir(figOut); end
    if ~isfolder(gen), mkdir(gen); end
    copyfile(fullfile(outDir, 'fig_*.pdf'), figOut);
    copyfile(fullfile(outDir, 'tab_*.tex'), gen);
    copyfile(fullfile(outDir, 'runs.csv'), gen);
    copyfile(fullfile(outDir, 'collab_macros.tex'), gen);
    fprintf('Copied the figures to %s and the tables to %s\n', figOut, gen);
end
writelines("Finished " + string(datetime("now")), fullfile(outDir, "finished.txt"));   % read by exportCollabResults
