%% Energy-Aware Paper Results
% This live script regenerates every figure and result of the energy-aware paper (S. Welikala
% et al., "Energy-aware persistent monitoring with battery-powered agents",
% |Paper_EnergyAwarePerMon|): the transit profiles of Sec. III, the setups, and the studies of
% Sec. VII (S1-S9 of its simulator specification). Run all sections (Live Editor: *Run All*).
% To get the same report without the Live Editor, run from the repository root:
%
%   matlab -batch "addpath(genpath(pwd)); exportEnergyResults"
%
% The method is |Energy-Aware ED-RHC| (|Components/AgentPolicy/EnergyAwareEDRHCPolicy.m|),
% compared with |Battery-Aware ED-RHC| (revision 1), |ED-RHC|, |ED-RHC-alpha| and |ED-ORHC-SO/-FO-3|.
% Every setup is a layout file that the GUI also lists (Scenario dropdown, "Energy: ..."), in
% |Configurations/Energy/| (built by |makeEnergyLayouts|). Results go to |results/energy/|; when
% the paper's folder sits next to this repository (|../Paper_EnergyAwarePerMon|), its
% |Figures/Energy/| and |generated/| folders receive the figures and tables. Simulations already
% saved in |results/energy/runs/| are not run again (delete them to rerun). This file is built
% from |Experiments/energy/energyResultsSource.m| by |buildEnergyResults|.

%% Settings
% |QuickMode| shortens every run to 300 s, keeps one seed, two setups and no ablations, to check
% the pipeline. A full run has about 720 simulations of one hour each; with one parallel worker
% per core (Parallel Computing Toolbox, used when available) it takes several hours. Every run is
% deterministic (seeded streams), so serial and parallel runs give the same results.
% |Strategy| is Energy-Aware ED-RHC's booking strategy in every study but S5: the winner of the
% 5-seed comparison of the strategies (lowest mean J over the three setups; R on a tie).
QuickMode = false;
quick = getenv("ENERGY_RESULTS_QUICK");   % set by exportEnergyResults(Quick=true)
if quick ~= "", QuickMode = quick == "1"; end
Strategy = "a";
root = fileparts(which('MainUI'));
assert(root ~= "", 'Make the repository root (the folder with MainUI.m) the current folder.');
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
warning('off', 'runTrial:AgentPolicyFit');
warning('off', 'optim:fmincon:SwitchingToMediumScale');
outDir = fullfile(root, 'results', 'energy');
if QuickMode, outDir = fullfile(root, 'results', 'energy_quick'); end
fprintf('%s mode, output in %s\n', string(QuickMode), outDir);

%% Layouts of the experiments
% Writes |Configurations/Energy/|: the office with its station (1 and 2 docks), the campus with
% two stations, the station sweep (1 and 2 docks, 2 to 6 robots) and the office without a
% station. Files whose content did not change are kept.
layouts = makeEnergyLayouts()

%% S1: transit profiles; the setups
% SO speed profiles with and without the speed term c2, the motion loss against the transit
% time (Theorem 1, the c2 = 0 optimum, FO-3), and FO-3's acceleration time (Theorem 3). The
% setups as the GUI draws them, with their stations.
energyFigures(outDir);
showImages(fullfile(outDir, ["fig_profiles", "fig_setups"] + ".png"));

%% Simulations
% S2 (main comparison), S3 (objective forms), S4 (terminal values), S5 (booking strategies and
% message delays), S6 (station sweep), S7 (charging methods), S8 (differential-drive robots,
% without and with margins) and S9 (stopping criteria). Each run is saved in |runs/<id>.mat|; the
% runs that need a pilot (S3's alpha_H, S8's margins, S9's K) follow their pilots.
runs = energyExperiments("list", Quick=QuickMode, Strategy=Strategy);
disp(groupcounts(runs, 'Study'))
energyExperiments("run", Quick=QuickMode, ResultsDir=outDir, Strategy=Strategy);

%% Tables, figures and the numbers quoted in Sec. VII
% runs.csv, the LaTeX tables (S2 main, S3 forms, S4 terminal, S5 booking, S7 charging, S8
% physical, S9 stopping), the figures (S2 J(t) and SOC, S6 stations, S8 errors, S9 stopping) and
% |energy_macros.tex| with every number the text quotes (full mode only).
T = energyExperiments("collect", Quick=QuickMode, ResultsDir=outDir, Strategy=Strategy);
disp(groupsummary(T(T.Study == "S2", :), {'Scenario', 'Method'}, 'mean', {'J', 'Energy', 'MinSoc', 'Share'}))
showImages(fullfile(outDir, ["fig_main", "fig_stations", "fig_errors", "fig_stopping"] + ".png"));

%% Output files
% The figures go to the paper's |Figures/Energy/|, and the tables, the macros and |runs.csv| to
% its |generated/| folder.
paperDir = fullfile(fileparts(root), 'Paper_EnergyAwarePerMon');
if ~QuickMode && isfolder(paperDir)
    figOut = fullfile(paperDir, 'Figures', 'Energy');
    gen = fullfile(paperDir, 'generated');
    if ~isfolder(figOut), mkdir(figOut); end
    if ~isfolder(gen), mkdir(gen); end
    copyfile(fullfile(outDir, 'fig_*.pdf'), figOut);
    copyfile(fullfile(outDir, 'tab_*.tex'), gen);
    copyfile(fullfile(outDir, 'runs.csv'), gen);
    copyfile(fullfile(outDir, 'energy_macros.tex'), gen);
    fprintf('Copied the figures to %s and the tables to %s\n', figOut, gen);
end
writelines("Finished " + string(datetime("now")), fullfile(outDir, "finished.txt"));   % read by exportEnergyResults

function showImages(files)
% Shows the PNGs that exist (the live script displays images, not PDFs)
for f = files(:)'
    if isfile(f)
        figure('Visible', 'on'); imshow(imread(f)); title(erase(string(f), fileparts(f) + filesep), 'Interpreter', 'none');
    end
end
end
