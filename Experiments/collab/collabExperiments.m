function out = collabExperiments(mode, opts)
% Experiments of the collaboration paper (W. Getts and S. Welikala, "Collaborative event-driven
% receding horizon control for persistent monitoring in network systems", in preparation):
% Collab-ED-RHC (CollabEDRHCPolicy) against ED-RHC (alpha = 0.5) and ED-RHC-alpha (nominal alpha;
% EDRHCPolicy) on layout files that the GUI also lists (Configurations/scenarioCatalog):
%   bench  the eight JS presets of Welikala and Cassandras, Automatica 2021
%          (Configurations/Legacy), with 4 methods and 3 ablations of Collab-ED-RHC
%   group  Configurations/Collab/Collab_*_Group.mat: targets that only two agents can clear
%   scale  Configurations/Collab/Collab_General_N2..N6.mat: 2 to 6 agents
%   maps   the simulator's maps (collabMaps: Lab floor, Office floor, City patrol, Harbor patrol,
%          Campus) with their walls and per-target growth rates, with the 4 methods and Collab (B)
%          without the terminal correction
%   seeds  the group study's Collab (A) and (B) with seeds 2-5 of the group candidates' stream
%          (seed 1 is the group study)
% Run from CollabResults.mlx (Experiments/collab/collabResultsSource.m), or directly:
%   collabExperiments("list")      the runs (one row each)
%   collabExperiments("run")       runs every run not saved yet (in parallel when the Parallel
%                                  Computing Toolbox is available) into <ResultsDir>/runs/<id>.mat
%   collabExperiments("collect")   runs.csv, the LaTeX tables, the numbers quoted in the paper
%                                  (collab_macros.tex) and the plots
% Quick = true: 60 s runs on two configurations and one map (pipeline check). Settings of every
% run: Abstract Agents with the FO-0 profile at the layout's speed, A_i = 1, B_i = 10 and R_i(0) of
% the layout (0.5 in the JS presets) unless the layout says otherwise (group study and maps:
% TargetRates = "layout"), T = 500 s, dt = 0.02 s, H = 250, service only at the target.
arguments
    mode string {mustBeMember(mode, ["list", "run", "collect"])} = "list"
    opts.Quick (1,1) logical = false
    opts.ResultsDir (1,:) char = ''
    opts.Parallel (1,1) logical = true
    opts.Dt (1,1) double = 0.02
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
resDir = opts.ResultsDir;
if isempty(resDir)
    resDir = fullfile(root, 'results', 'collab');
    if opts.Quick, resDir = fullfile(root, 'results', 'collab_quick'); end
end
runDir = fullfile(resDir, 'runs');
L = runList(opts);
switch mode
    case "list"
        out = L;
    case "run"
        if ~isfolder(runDir), mkdir(runDir); end
        todo = find(arrayfun(@(id) ~isfile(runFile(runDir, id)), 1:height(L)));
        fprintf('%d of %d runs to do\n', numel(todo), height(L));
        params = L.Params(todo); info = L(todo, 1:3);
        if opts.Parallel && canUseParallel()
            parfor q = 1:numel(todo)
                oneRun(todo(q), params{q}, info(q, :), runDir);
            end
        else
            for q = 1:numel(todo)
                oneRun(todo(q), params{q}, info(q, :), runDir);
            end
        end
        out = [];
    case "collect"
        out = collect(L, runDir, resDir, opts.Quick);
end
end

function ok = canUseParallel()
ok = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));
end

function f = runFile(runDir, id)
f = fullfile(runDir, sprintf('%03d.mat', id));
end

function oneRun(id, p, info, runDir)
tic; o = runTrial(p); sec = toc;
r = struct('id', id, 'study', info.Study, 'scenario', info.Scenario, 'method', info.Method, ...
    'J', o.finalJ, 'seconds', sec, 'peakR', o.peakR, 'coLocation', o.coLocationTime, ...
    't', o.log.t(1:50:end), 'Jt', o.log.J(1:50:end), 'R', o.log.R(1:50:end, :));
r.stats = struct();
if isprop(o.policy, 'stats') && isstruct(o.policy.stats), r.stats = o.policy.stats; end
save(runFile(runDir, id), 'r');
fprintf('%3d %-8s %-22s %-22s J = %9.3f  %6.0f s\n', id, info.Study, info.Scenario, info.Method, o.finalJ, sec);
end

function L = runList(opts)
presets = ["JS_1Agent_3T", "JS_1Agent_4T", "JS_1Agent_5T", "JS_2Agent_5T", ...
    "JS_3Agent_9T", "JS_3Agent_Maze", "JS_3Agent_General", "JS_3Agent_Room"];
groups = ["JS_2Agent_5T", "Collab_2Agent_5T_Group"; "JS_3Agent_General", "Collab_3Agent_General_Group"];
sizes = 2:6;
mainMethods = ["ED-RHC", "ED-RHC-alpha", "Collab-A", "Collab-B"];
ablations = ["Collab-B-noterminal", "Collab-B-history", "Collab-A-noshare"];
maps = collabMaps();
seeds = 2:5;
T = 500;
if opts.Quick
    presets = ["JS_1Agent_3T", "JS_2Agent_5T"]; groups = groups(1, :); sizes = 2:3; ablations = strings(1, 0); T = 60;
    maps = maps(maps.Layout == "HarborPatrol", :);
    seeds = 2;
end
rows = {};
for k = 1:numel(presets)
    for m = methods([mainMethods, ablations])
        rows(end+1, :) = {"bench", presets(k), m.label, params(fullfile("Legacy", presets(k) + ".mat"), m.params, T, opts)}; %#ok<AGROW>
    end
end
for g = 1:size(groups, 1)
    for m = methods(mainMethods)
        p = params(fullfile("Collab", groups(g, 2) + ".mat"), m.params, T, opts);
        p.TargetRates = "layout";
        rows(end+1, :) = {"group", groups(g, 1), m.label, p}; %#ok<AGROW>
    end
end
for n = sizes
    for m = methods(["ED-RHC", "ED-RHC-alpha", "Collab-B"])
        p = params(fullfile("Collab", "Collab_General_N" + n + ".mat"), m.params, T, opts);
        rows(end+1, :) = {"scale", "General_N" + n, m.label, p}; %#ok<AGROW>
    end
end
% the maps come last, so that adding them kept the ids (file names) of the runs above
for k = 1:height(maps)
    for m = methods(mainMethods)
        p = params(maps.Layout(k) + ".mat", m.params, T, opts);
        p.TargetRates = "layout";
        rows(end+1, :) = {"maps", maps.Layout(k), m.label, p}; %#ok<AGROW>
    end
end
% Collab (B) without the terminal correction on the maps (added after the map runs, also last)
for k = 1:height(maps)
    p = params(maps.Layout(k) + ".mat", methods("Collab-B-noterminal").params, T, opts);
    p.TargetRates = "layout";
    rows(end+1, :) = {"maps", maps.Layout(k), "Collab-B-noterminal", p}; %#ok<AGROW>
end
% the group study with other seeds of the group candidates' stream (CollabEDRHCPolicy.sampleSeed)
for g = 1:size(groups, 1)
    for m = methods(["Collab-A", "Collab-B"])
        for sd = seeds
            p = params(fullfile("Collab", groups(g, 2) + ".mat"), m.params, T, opts);
            p.TargetRates = "layout"; p.Policy_sampleSeed = sd;
            rows(end+1, :) = {"seeds", groups(g, 1) + "/seed" + sd, m.label, p}; %#ok<AGROW>
        end
    end
end
L = cell2table(rows, 'VariableNames', {'Study', 'Scenario', 'Method', 'Params'});
end

function M = methods(labels)
all = struct( ...
    'ED_RHC', struct('Policy', "ED-RHC", 'Policy_alpha', 0.5), ...
    'ED_RHC_alpha', struct('Policy', "ED-RHC", 'Policy_alpha', "nominal"), ...
    'Collab_A', struct('Policy', "Collab-ED-RHC", 'Policy_coordination', "A"), ...
    'Collab_B', struct('Policy', "Collab-ED-RHC", 'Policy_coordination', "B"), ...
    'Collab_B_noterminal', struct('Policy', "Collab-ED-RHC", 'Policy_coordination', "B", 'Policy_terminal', "none"), ...
    'Collab_B_history', struct('Policy', "Collab-ED-RHC", 'Policy_coordination', "B", 'Policy_deltaRule', "history"), ...
    'Collab_A_noshare', struct('Policy', "Collab-ED-RHC", 'Policy_coordination', "A", 'Policy_sharing', false));
M = struct('label', {}, 'params', {});
for l = labels
    M(end+1) = struct('label', l, 'params', all.(strrep(l, '-', '_'))); %#ok<AGROW>
end
end

function p = params(layout, extra, T, opts)
s = load(fullfile(fileparts(which('MainUI')), 'Configurations', layout), 's');
p = struct('Layout', layout, 'AgentStarts', "layout", ...
    'Planner', "Dijkstra Corners", 'AgentType', "Abstract Agent", 'Agent_profile', "FO-0", ...
    'UncertaintyMode', "Static Uniform", 'A_base', 1, 'B', 10, 'EndTime', T, ...
    'Dt', opts.Dt, 'DetectionRadius', 1e-6, 'Policy_H', 250, 'AgentSpeed', s.s.agentSpeed(1), ...
    'Log', true);
f = fieldnames(extra);
for q = 1:numel(f), p.(f{q}) = extra.(f{q}); end
end

% ------------------------------------------------------------------------------------------
function T = collect(L, runDir, resDir, quick)
rows = {};
for id = 1:height(L)
    file = runFile(runDir, id);
    if ~isfile(file), continue; end
    S = load(file); r = S.r; st = r.stats;
    rows(end+1, :) = {id, L.Study(id), L.Scenario(id), L.Method(id), r.J, r.seconds, r.peakR, ...
        r.coLocation, field(st, 'solves'), field(st, 'cpu'), field(st, 'shiftEvents'), ...
        field(st, 'shifts'), sum(field(st, 'events'))}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'Id', 'Study', 'Scenario', 'Method', 'J', 'Seconds', 'PeakR', ...
    'CoLocation', 'Solves', 'SolveCPU', 'ShiftEvents', 'Shifts', 'Events'});
writetable(T, fullfile(resDir, 'runs.csv'));
writeTables(T, resDir);
if ~quick, writeMacros(T, resDir); end
makePlots(L, runDir, resDir);
end

function v = field(st, f)
if isfield(st, f), v = st.(f); else, v = NaN; end
end

function writeTables(T, resDir)
% LaTeX tables (complete tabular environments, \input outside a tabular): J_T of the benchmark
% configurations (main methods; ablations), the group study, and the team-size study
presets = ["JS_1Agent_3T", "JS_1Agent_4T", "JS_1Agent_5T", "JS_2Agent_5T", "JS_3Agent_9T", ...
    "JS_3Agent_Maze", "JS_3Agent_General", "JS_3Agent_Room"];
names = ["1A-3T", "1A-4T", "1A-5T", "2A-5T", "3A-9T", "3A-Maze", "3A-General", "3A-Room"];
main = ["ED-RHC", "ED-RHC-alpha", "Collab-A", "Collab-B"];
heads = ["ED-RHC", "ED-RHC-$\alpha$", "Collab (A)", "Collab (B)"];
tab(fullfile(resDir, 'tab_bench.tex'), T(T.Study == "bench", :), presets, names, main, heads);
tab(fullfile(resDir, 'tab_ablation.tex'), T(T.Study == "bench", :), presets, names, ...
    ["Collab-B", "Collab-B-noterminal", "Collab-B-history", "Collab-A-noshare"], ...
    ["(B)", "(B), $\hat h\equiv 0$", "(B), $\delta_k=\bar\delta_k$", "(A), no sharing"]);
tab(fullfile(resDir, 'tab_group.tex'), T(T.Study == "group", :), ["JS_2Agent_5T", "JS_3Agent_General"], ...
    ["2A-5T", "3A-General"], main, heads);
maps = collabMaps();
tab(fullfile(resDir, 'tab_maps.tex'), T(T.Study == "maps", :), maps.Layout', maps.Name', ...
    [main, "Collab-B-noterminal"], [heads, "(B), $\hat h\equiv 0$"], "Map", numel(main));
S = T(T.Study == "scale", :);
fid = fopen(fullfile(resDir, 'tab_scale.tex'), 'w');
fprintf(fid, '%% generated by collabExperiments("collect")\n');
fprintf(fid, '\\begin{tabular}{|c|c|c|c|c|c|c|}\n\\hline\n');
fprintf(fid, '$N$ & ED-RHC & ED-RHC-$\\alpha$ & (B) & Ch. (\\%%) & Time (ms) & Shifts \\\\ \\hline\n');
for n = 2:6
    r = @(m) S(S.Scenario == "General_N" + n & S.Method == m, :);
    e = r("ED-RHC"); a = r("ED-RHC-alpha"); c = r("Collab-B");
    if isempty(e) || isempty(a) || isempty(c), continue; end
    fprintf(fid, '%d & %.1f & %.1f & %.1f & %.1f & %.0f & %.0f \\\\ \\hline\n', n, e.J, a.J, c.J, ...
        100 * (c.J - e.J) / e.J, 1000 * c.SolveCPU / c.Solves, c.ShiftEvents);
end
fprintf(fid, '\\end{tabular}\n');
fclose(fid);
end

function tab(file, T, presets, names, methods, heads, first, nBold)
% nBold: the lowest value among the first nBold columns is bold (default: all columns)
if nargin < 7, first = "Config."; end
if nargin < 8, nBold = numel(methods); end
fid = fopen(file, 'w');
fprintf(fid, '%% generated by collabExperiments("collect")\n');
fprintf(fid, '\\begin{tabular}{|l|%s}\n\\hline\n', repmat('c|', 1, numel(methods)));
fprintf(fid, '%s & %s \\\\ \\hline\n', first, strjoin(heads, ' & '));
for k = 1:numel(presets)
    v = nan(1, numel(methods));
    for m = 1:numel(methods)
        r = T(T.Scenario == presets(k) & T.Method == methods(m), :);
        if ~isempty(r), v(m) = r.J(1); end
    end
    if all(isnan(v)), continue; end
    vr = round(v, 1);   % bold every value that ties with the lowest one as printed
    cells = arrayfun(@(x) sprintf('%.1f', x), v, 'UniformOutput', false);
    for b = find(vr(1:nBold) == min(vr(1:nBold)))
        cells{b} = ['\textbf{' cells{b} '}'];
    end
    fprintf(fid, '%s & %s \\\\ \\hline\n', names(k), strjoin(cells, ' & '));
end
fprintf(fid, '\\end{tabular}\n');
fclose(fid);
end

function writeMacros(T, resDir)
% Every number quoted in Sec. VIII of the paper, as LaTeX macros (\cr...), so the text follows
% the results. The qualitative statements around them (which method wins where) were written
% for these results; check them when the numbers change.
J = @(study, sc, m) T.J(T.Study == study & T.Scenario == sc & T.Method == m);
row = @(study, sc, m) T(T.Study == study & T.Scenario == sc & T.Method == m, :);
pc = @(a, b) 100 * (a - b) / b;
ma = ["JS_2Agent_5T", "JS_3Agent_9T", "JS_3Agent_Maze", "JS_3Agent_General", "JS_3Agent_Room"];
key = ["TwoAFiveT", "ThreeANineT", "Maze", "General", "Room"];
mac = strings(0, 1);
% agreement with Tab. I of the Automatica paper (RHC and RHC-alpha on SASE2, MASE2, MASE1)
ref = [141.4 63.7 105.5; 121.3 40.4 96.6];
sc = ["JS_1Agent_5T", "JS_2Agent_5T", "JS_3Agent_General"];
dev = [arrayfun(@(k) abs(pc(J("bench", sc(k), "ED-RHC"), ref(1, k))), 1:3), ...
    arrayfun(@(k) abs(pc(J("bench", sc(k), "ED-RHC-alpha"), ref(2, k))), 1:3)];
add("TabIDev", ceil(max(dev)), '%d');
% single agent: reduction (no terminal correction) and the effect of the terminal correction
sa = ["JS_1Agent_3T", "JS_1Agent_4T", "JS_1Agent_5T"];
red = arrayfun(@(s) abs(pc(J("bench", s, "Collab-B-noterminal"), J("bench", s, "ED-RHC"))), sa);
add("ReductionDev", max(0.01, ceil(100 * max(red)) / 100), '%.2f');
sk = ["SAThree", "SAFour", "SAFive"];
for k = 1:3, add(sk(k), abs(pc(J("bench", sa(k), "Collab-A"), J("bench", sa(k), "ED-RHC"))), '%.1f'); end
% multi-agent: Collab against ED-RHC and ED-RHC-alpha
dA = arrayfun(@(s) -pc(J("bench", s, "Collab-A"), J("bench", s, "ED-RHC")), ma);
dB = arrayfun(@(s) -pc(J("bench", s, "Collab-B"), J("bench", s, "ED-RHC")), ma);
aA = arrayfun(@(s) pc(J("bench", s, "Collab-A"), J("bench", s, "ED-RHC-alpha")), ma);
for k = 1:5
    add("AvsED" + key(k), dA(k), '%.1f');
    add("AvsAlpha" + key(k), abs(aA(k)), '%.1f');
end
add("AvsEDMean", mean(dA), '%.1f');
add("BvsEDMean", mean(dB), '%.1f');
% coordination modes
bA = arrayfun(@(s) pc(J("bench", s, "Collab-B"), J("bench", s, "Collab-A")), ma);
for k = 1:5, add("BvsA" + key(k), abs(bA(k)), '%.1f'); end
add("BvsAMaxWorse", max(bA), '%.1f');
share = arrayfun(@(s) 100 * row("bench", s, "Collab-B").ShiftEvents / row("bench", s, "Collab-B").Events, ma);
add("ShiftShareMin", floor(min(share)), '%d');
add("ShiftShareMax", ceil(max(share)), '%d');
% ablations
nt = arrayfun(@(s) pc(J("bench", s, "Collab-B-noterminal"), J("bench", s, "Collab-B")), ma);
add("NoTermMin", min(nt), '%.1f'); add("NoTermMax", max(nt), '%.1f');
hi = arrayfun(@(s) pc(J("bench", s, "Collab-B-history"), J("bench", s, "Collab-B")), ma);
add("HistMax", max(hi), '%.1f'); add("HistMin", abs(min(hi)), '%.1f');
ns = arrayfun(@(s) pc(J("bench", s, "Collab-A-noshare"), J("bench", s, "Collab-A")), ma);
add("NoShareMaze", ns(3), '%.1f');
add("NoShareOthersMax", ceil(max(abs(ns([1 2 4 5])))), '%d');
% group study
g = ["JS_2Agent_5T", "JS_3Agent_General"]; gk = ["TwoAFiveT", "General"];
for k = 1:2
    add("GroupED" + gk(k), J("group", g(k), "ED-RHC") / J("group", g(k), "Collab-A"), '%.1f');
    add("GroupAlpha" + gk(k), J("group", g(k), "ED-RHC-alpha") / J("group", g(k), "Collab-A"), '%.1f');
    add("GroupShare" + gk(k), row("group", g(k), "Collab-A").CoLocation, '%.0f');
    add("GroupPeak" + gk(k), row("group", g(k), "ED-RHC").PeakR / row("group", g(k), "Collab-A").PeakR, '%.1f');
end
% maps: changes of Collab (A) and (B) with respect to ED-RHC (absolute values; the text says
% which are reductions), Collab (B) against ED-RHC-alpha and the peak uncertainties on Lab floor
M = collabMaps(); mk = ["Lab", "Office", "City", "Harbor", "Campus"];
for k = 1:height(M)
    e = J("maps", M.Layout(k), "ED-RHC");
    add("MapsBvsED" + mk(k), abs(pc(J("maps", M.Layout(k), "Collab-B"), e)), '%.1f');
    add("MapsAvsED" + mk(k), abs(pc(J("maps", M.Layout(k), "Collab-A"), e)), '%.1f');
end
add("MapsBvsAlphaLab", abs(pc(J("maps", "ReportLayout", "Collab-B"), J("maps", "ReportLayout", "ED-RHC-alpha"))), '%.1f');
pk = ["ED", "Alpha", "A", "B"]; mm = ["ED-RHC", "ED-RHC-alpha", "Collab-A", "Collab-B"];
for q = 1:4, add("MapsPeakLab" + pk(q), row("maps", "ReportLayout", mm(q)).PeakR, '%.0f'); end
add("MapsCoLocMax", max(T.CoLocation(T.Study == "maps")), '%.1f');
% maps without the terminal correction: change of Collab (B) when hhat = 0, and against ED-RHC
for k = 1:height(M)
    nt = J("maps", M.Layout(k), "Collab-B-noterminal");
    if isempty(nt), continue; end
    add("MapsNoTermVsB" + mk(k), abs(pc(nt, J("maps", M.Layout(k), "Collab-B"))), '%.1f');
    add("MapsNoTermVsED" + mk(k), abs(pc(nt, J("maps", M.Layout(k), "ED-RHC"))), '%.1f');
end
% group study over seeds 1-5 of the group candidates' stream (seed 1 is Tab. IV): range of J_T
% of Collab (A) and (B), and the smallest ratios of ED-RHC and ED-RHC-alpha to Collab (A)
for k = 1:2
    for mm = ["Collab-A", "Collab-B"]
        Js = [J("group", g(k), mm); arrayfun(@(sd) J("seeds", g(k) + "/seed" + sd, mm), 2:5)'];
        add("SeedMin" + gk(k) + extractAfter(mm, "-"), min(Js), '%.1f');
        add("SeedMax" + gk(k) + extractAfter(mm, "-"), max(Js), '%.1f');
        if mm == "Collab-A"
            add("SeedEDRatio" + gk(k), J("group", g(k), "ED-RHC") / max(Js), '%.1f');
            add("SeedAlphaRatio" + gk(k), J("group", g(k), "ED-RHC-alpha") / max(Js), '%.1f');
        end
    end
end
% team size
S = T(T.Study == "scale", :);
Jn = @(n, m) S.J(S.Scenario == "General_N" + n & S.Method == m);
gainN = arrayfun(@(n) -pc(Jn(n, "Collab-B"), Jn(n, "ED-RHC")), 3:6);
add("ScaleGainMin", min(gainN), '%.1f'); add("ScaleGainMax", max(gainN), '%.1f');
add("ScaleAlphaMax", max(arrayfun(@(n) pc(Jn(n, "Collab-B"), Jn(n, "ED-RHC-alpha")), 3:6)), '%.1f');
c = arrayfun(@(n) S(S.Scenario == "General_N" + n & S.Method == "Collab-B", :), 2:6, 'UniformOutput', false);
shareN = cellfun(@(x) 100 * x.ShiftEvents / x.Events, c);
timeN = cellfun(@(x) x.SolveCPU / x.Solves, c);
add("ScaleShareMin", min(shareN), '%.1f'); add("ScaleShareMax", max(shareN), '%.1f');
add("ScaleTimeMin", min(timeN), '%.2f'); add("ScaleTimeMax", max(timeN), '%.2f');
writelines(["% generated by collabExperiments(""collect""): numbers quoted in Sec. VIII"; mac(:)], ...
    fullfile(resDir, 'collab_macros.tex'));

    function add(name, val, fmt)
        mac(end+1) = "\newcommand{\cr" + name + "}{" + sprintf(fmt, val) + "}";
    end
end

function makePlots(L, runDir, resDir)
% J(t) on 2A-5T and 3A-General under the four main methods; R_i(t) of the group targets on
% 3A-General under ED-RHC and Collab (A). PDF (vector) and PNG (300 dpi).
methods = ["ED-RHC", "ED-RHC-alpha", "Collab-A", "Collab-B"];
labels = ["ED-RHC", "ED-RHC-$\alpha$", "Collab (A)", "Collab (B)"];
colors = [0.0 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19; 0.49 0.18 0.56];
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 3.5 2.5]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for sc = ["JS_2Agent_5T", "JS_3Agent_General"]
    nexttile; hold on; grid on; box on;
    for m = 1:numel(methods)
        r = loadRun(L, runDir, "bench", sc, methods(m));
        if isempty(r), continue; end
        plot(r.t, r.Jt, 'LineWidth', 1.1, 'Color', colors(m, :), 'DisplayName', labels(m));
    end
    title(replace(sc, ["JS_2Agent_5T", "JS_3Agent_General"], ["2A-5T", "3A-General"]), ...
        'Interpreter', 'latex', 'FontWeight', 'normal');
    xlabel('Time: $t$ [s]', 'Interpreter', 'latex');
    set(gca, 'FontSize', 8, 'TickLabelInterpreter', 'latex', 'XLim', [0 500], 'XTick', 0:250:500);
    if sc == "JS_3Agent_General"
        legend('Interpreter', 'latex', 'Location', 'southeast', 'Box', 'on', 'FontSize', 7);
    end
end
ylabel(tl, 'Objective: $J(t)$', 'Interpreter', 'latex', 'FontSize', 9);
saveFig(f, fullfile(resDir, 'fig_J_general'));

f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 3.5 2.4]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for m = ["ED-RHC", "Collab-A"]
    nexttile; hold on; grid on; box on;
    r = loadRun(L, runDir, "group", "JS_3Agent_General", m);
    if isempty(r), continue; end
    plot(r.t, r.R(:, 3), 'LineWidth', 1.1, 'Color', colors(1, :), 'DisplayName', '$i = 3$');
    plot(r.t, r.R(:, 8), 'LineWidth', 1.1, 'Color', colors(2, :), 'DisplayName', '$i = 8$');
    title(strrep(m, "Collab-A", "Collab (A)"), 'Interpreter', 'latex', 'FontWeight', 'normal');
    xlabel('Time: $t$ [s]', 'Interpreter', 'latex');
    set(gca, 'FontSize', 8, 'TickLabelInterpreter', 'latex', 'XLim', [0 500], 'XTick', 0:250:500);
    if m == "ED-RHC"
        legend('Interpreter', 'latex', 'Location', 'northwest', 'Box', 'on', 'FontSize', 7);
    end
end
ylabel(tl, 'Uncertainty: $R_i(t)$, $i\in\{3,8\}$', 'Interpreter', 'latex', 'FontSize', 9);
saveFig(f, fullfile(resDir, 'fig_R_group'));
end

function saveFig(f, base)
exportgraphics(f, base + ".pdf", 'ContentType', 'vector');
exportgraphics(f, base + ".png", 'Resolution', 300);
close(f);
end

function r = loadRun(L, runDir, study, scenario, method)
r = [];
id = find(L.Study == study & L.Scenario == scenario & L.Method == method, 1);
if ~isempty(id) && isfile(runFile(runDir, id)), S = load(runFile(runDir, id)); r = S.r; end
end
