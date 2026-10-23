function out = energyExperiments(mode, opts)
% Experiments of the energy-aware paper (Sec. VII; studies S2-S9 of its simulator
% specification, Claude_Notes/Project_Notes/SPEC_EnergyPaper_Simulator.md) on layout files that
% the GUI also lists (Configurations/Energy, "Energy: ..." scenarios):
%   S2 main      Office 1 dock, Office 2 docks, Campus: Energy-Aware ED-RHC (EA) with SO, FO-3 and
%                FO-0 against Battery-Aware ED-RHC (BA, FO-0), ED-RHC and ED-RHC-alpha (FO-0,
%                unlimited batteries) and ED-ORHC-SO/-FO-3 (unlimited, and with batteries)
%   S3 forms     EA-SO: ratio and alphaH (at alpha_H = omega~/w_bar of the dinkelbach pilot, x0.5, x2)
%   S4 terminal  EA-SO: terminal "hhat" and "none" (S2 has "alpha")
%   S5 booking   EA-SO: strategies R (without and with the one-slot report deadband), a, b, e;
%                R with the deadband at delta = 1 s and 5 s
%   S6 stations  Energy_Sweep_m<1|2>_n<2..6> with P_c at 0.8 to 2 times ceil(n/m) P_I
%   S7 charging  Battery1RC: constant power, CC-CV, CC-CV with a tail slot, swapping (60 s)
%   S8 physical  DiffDrive agents without margins, then with margins from their largest errors
%   S9 stopping  Office without a station: ED-ORHC-SO and the K-weighted method (EA with
%                hhat_E = 0, omega = kappa J_R/(N P)), stopped at T, at an energy or at a visit count
% Run from EnergyResults.mlx (Experiments/energy/energyResultsSource.m), or directly:
%   energyExperiments("list")      the runs (one row each)
%   energyExperiments("run")       runs every run not saved yet into <ResultsDir>/runs/<id>.mat,
%                                  first those without a pilot, then those that need one (S3's
%                                  alphaH, S8 with margins, S9's K-weighted runs); in parallel when
%                                  the Parallel Computing Toolbox is available
%   energyExperiments("collect")   runs.csv, the LaTeX tables, energy_macros.tex and the figures
% Settings (Q19): T = 3600 s, dt = 0.1 s, R_i(0) ~ U[0, 100] per seed, the layouts' robots and rates;
% seeds 10 (S2), 5 (S3-S5, S7, S8, S9), 3 (S6). Quick = true: 300 s, one seed, fewer setups and no
% ablations (a check of the pipeline).
arguments
    mode string {mustBeMember(mode, ["list", "run", "collect"])} = "list"
    opts.Quick (1,1) logical = false
    opts.ResultsDir (1,:) char = ''
    opts.Parallel (1,1) logical = true
    opts.Strategy (1,1) string = "a"     % EA's booking strategy (the 5-seed comparison's winner)
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
resDir = opts.ResultsDir;
if isempty(resDir)
    resDir = fullfile(root, 'results', 'energy');
    if opts.Quick, resDir = fullfile(root, 'results', 'energy_quick'); end
end
runDir = fullfile(resDir, 'runs');
L = runList(opts);
switch mode
    case "list"
        out = L;
    case "run"
        if ~isfolder(runDir), mkdir(runDir); end
        for phase = ["", "pilot"]   % runs without a pilot first
            todo = find(arrayfun(@(id) ~isfile(runFile(runDir, id)), 1:height(L)) & ...
                (L.Needs ~= "")' == (phase == "pilot"));
            fprintf('%d runs to do (%s)\n', numel(todo), ternary(phase == "", "independent", "after their pilots"));
            params = cell(1, numel(todo));
            for q = 1:numel(todo), params{q} = resolve(L, todo(q), runDir); end
            info = L(todo, 1:5);
            if opts.Parallel && canUseParallel()
                parfor q = 1:numel(todo)
                    oneRun(todo(q), params{q}, info(q, :), runDir);
                end
            else
                for q = 1:numel(todo)
                    oneRun(todo(q), params{q}, info(q, :), runDir);
                end
            end
        end
        out = [];
    case "collect"
        out = energyCollect(L, runDir, resDir, opts.Quick);
end
end

function ok = canUseParallel()
ok = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));
end

function f = runFile(runDir, id)
f = fullfile(runDir, sprintf('%04d.mat', id));
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

% ------------------------------------------------------------------------------------------
function L = runList(opts)
% One row per run: Study, Scenario, Method, Seed, Variant (free text), Params, Needs (the id of
% the run, or the "study|scenario|method" group, whose results complete Params)
T = 3600; seeds = 1:10; seeds5 = 1:5; seeds3 = 1:3;
setups = ["Energy_Office_1Dock", "Energy_Office_2Docks", "Energy_Campus_2Stations"];
if opts.Quick
    T = 300; seeds = 1; seeds5 = 1; seeds3 = 1; setups = ["Energy_Office_1Dock", "Energy_Campus_2Stations"];
end
st = opts.Strategy;
rows = {};
    function add(study, scenario, method, seed, variant, p, needs)
        if nargin < 7, needs = ""; end
        rows(end+1, :) = {string(study), string(scenario), string(method), seed, string(variant), p, string(needs)};
    end
% S2: main comparison
main = ["EA-SO", "EA-FO-3", "EA-FO-0", "BA", "ED-RHC", "ED-RHC-alpha", "ED-ORHC-SO", "ED-ORHC-SO-bat", ...
    "ED-ORHC-FO-3", "ED-ORHC-FO-3-bat"];
if opts.Quick, main = ["EA-SO", "EA-FO-0", "BA", "ED-RHC", "ED-ORHC-SO-bat"]; end
for sc = setups, for m = main, for s = seeds
    add("S2", sc, m, s, "", method(m, base(sc, T, s), st));
end, end, end
% S3: objective forms (dinkelbach is S2's EA-SO); alphaH from the seed-1 pilot of each setup
for sc = setups, for s = seeds5
    p = method("EA-SO", base(sc, T, s), st); p.Policy_objectiveForm = "ratio";
    add("S3", sc, "ratio", s, "", p);
    if ~opts.Quick
        for f = [0.5 1 2]
            p = method("EA-SO", base(sc, T, s), st); p.Policy_objectiveForm = "alphaH";
            add("S3", sc, "alphaH", s, sprintf("x%g", f), p, "S2|" + sc + "|EA-SO|1");
        end
    end
end, end
% S4: terminal values (alpha is S2's EA-SO)
if ~opts.Quick
    for sc = setups, for term = ["hhat", "none"], for s = seeds5
        p = method("EA-SO", base(sc, T, s), st); p.Policy_terminal = term;
        add("S4", sc, term, s, "", p);
    end, end, end
end
% S5: booking strategies (the S2 strategy's runs are reused) and message delays
strategies = setdiff(["R", "a", "b", "e"], st, 'stable');
if opts.Quick, strategies = strategies(1); end
for sc = setups, for s = seeds5
    for k = strategies
        add("S5", sc, k, s, "", method("EA-SO", base(sc, T, s), k));
    end
    p = method("EA-SO", base(sc, T, s), "R"); p.Policy_reportDeadband = 1;
    add("S5", sc, "R", s, "deadband", p);
    for d = [1 5]
        if opts.Quick && d == 5, continue; end
        p = method("EA-SO", base(sc, T, s), "R"); p.Policy_reportDeadband = 1; p.Policy_messageDelay = d;
        add("S5", sc, "R", s, sprintf("deadband, delta %g", d), p);
    end
end, end
% S6: stations (P_c in units of Lemma 4's bound ceil(n/m) P_I, at least P_I + 10 W)
ms = 1:2; ns = 2:6; facs = [0.8 1 1.25 1.6 2];
if opts.Quick, ms = 1; ns = [2 4]; facs = [1 2]; end
PI = idleDraw();
for m = ms, for n = ns, for f = facs, for s = seeds3
    sc = sprintf("Energy_Sweep_m%d_n%d", m, n);
    p = method("EA-SO", base(sc, T, s), st);
    p.StationPower = max(f * ceil(n / m) * PI, PI + 10);
    add("S6", sc, "EA-SO", s, sprintf("x%g", f), p);
end, end, end, end
% S7: charging methods with the circuit battery
variants = ["constant power", "CC-CV", "CC-CV tail", "swap"];
s7 = ["Energy_Office_1Dock", "Energy_Campus_2Stations"];
for sc = s7, for v = variants, for s = seeds5
    p = method("EA-SO", base(sc, T, s), st);
    p.BatteryModel = "1RC";
    switch v
        case "constant power", p.Battery_chargeMethod = "constant power";
        case "CC-CV",          p.Battery_chargeMethod = "CC-CV";
        case "CC-CV tail",     p.Battery_chargeMethod = "CC-CV"; p.Policy_tailSlot = true;
        case "swap",           p.Battery_chargeMethod = "CC-CV"; p.StationSwapTime = 60;
    end
    add("S7", sc, v, s, "", p);
end, end, end
% S8: DiffDrive agents without margins, then with margins from their largest errors
for sc = setups, for s = seeds5
    p = method("EA-SO", base(sc, T, s), st); p.AgentType = "DiffDrive Agent";
    add("S8", sc, "no margins", s, "", p);
end, end
for sc = setups, for s = seeds5
    p = method("EA-SO", base(sc, T, s), st); p.AgentType = "DiffDrive Agent";
    add("S8", sc, "margins", s, "", p, "S8|" + sc + "|no margins");
end, end
% S9: stopping criteria on the office without a station; the stops are read from each run's log
kappas = [0 0.25 0.5 1 2 4];
if opts.Quick, kappas = [0 1]; end
sc = "Energy_Office_NoStation";
for s = seeds5
    p = base(sc, T, s); p.Policy = "ED-ORHC-SO"; p.Agent_profile = "SO";
    add("S9", sc, "ED-ORHC-SO", s, "", p);
    for k = kappas
        p = base(sc, T, s); p.Policy = "Energy-Aware ED-RHC"; p.Policy_priceScale = 0;
        if k == 0
            add("S9", sc, "K-weighted", s, "kappa 0", p);
        else
            add("S9", sc, "K-weighted", s, sprintf("kappa %g", k), p, sprintf("S9|%s|K-weighted|%d", sc, s));
        end
    end
end
L = cell2table(rows, 'VariableNames', {'Study', 'Scenario', 'Method', 'Seed', 'Variant', 'Params', 'Needs'});
end

function p = base(layout, T, seed)
p = struct('Layout', "Energy/" + layout + ".mat", 'AgentStarts', "layout", 'AgentSettings', "layout", ...
    'TargetRates', "layout", 'UncertaintyMode', "Static Uniform", 'Planner', "Dijkstra Corners", ...
    'AgentType', "Abstract Agent", 'EndTime', T, 'Dt', 0.1, 'Seed', seed, 'InitialR', 100, 'Log', true, ...
    'Metrics', ["Average Uncertainty J(t)", "Minimum SOC", "Energy used", "Charges", ...
                "Mean wait at stations", "Monitoring share", "Largest gap"]);
end

function p = method(m, p, strategy)
% The S2 methods: EA (strategy, default settings otherwise) with SO, FO-3 or FO-0 transits; BA with
% FO-0 robots and a 10% reserve; ED-RHC and ED-RHC-alpha (FO-0) and ED-ORHC with unlimited
% batteries, and ED-ORHC with the layout's batteries ("-bat": they run out)
switch m
    case "EA-SO",   p.Policy = "Energy-Aware ED-RHC"; p.Policy_bookingStrategy = strategy;
    case "EA-FO-3", p.Policy = "Energy-Aware ED-RHC"; p.Policy_bookingStrategy = strategy;
                    p.Policy_transit = "FO-3"; p.Agent_profile = "FO-3";
    case "EA-FO-0", p.Policy = "Energy-Aware ED-RHC"; p.Policy_bookingStrategy = strategy;
                    p.Policy_transit = "FO-0"; p.Agent_profile = "FO-0";
    case "BA",      p.Policy = "Battery-Aware ED-RHC"; p.Agent_profile = "FO-0"; p.Policy_socMin = 0.1;
    case "ED-RHC",  p.Policy = "ED-RHC"; p.Policy_alpha = 0.5; p.Agent_profile = "FO-0"; p.Agent_stopWhenEmpty = false;
    case "ED-RHC-alpha", p.Policy = "ED-RHC"; p.Policy_alpha = "nominal"; p.Agent_profile = "FO-0"; p.Agent_stopWhenEmpty = false;
    case "ED-ORHC-SO", p.Policy = "ED-ORHC-SO"; p.Agent_profile = "SO"; p.Agent_stopWhenEmpty = false;
    case "ED-ORHC-SO-bat", p.Policy = "ED-ORHC-SO"; p.Agent_profile = "SO";
    case "ED-ORHC-FO-3", p.Policy = "ED-ORHC-FO-3"; p.Agent_profile = "FO-3"; p.Agent_stopWhenEmpty = false;
    case "ED-ORHC-FO-3-bat", p.Policy = "ED-ORHC-FO-3"; p.Agent_profile = "FO-3";
end
end

function P = idleDraw()
% P_I of the energy robots (with their battery losses)
b = Battery(CapacityWh=9); b.R0 = 0.1; b.Voc = 14.4;
P = b.idleDraw();
end

% ------------------------------------------------------------------------------------------
function p = resolve(L, id, runDir)
% A run's parameters, completed from its pilot's saved results
p = L.Params{id};
needs = L.Needs(id);
if needs == "", return; end
parts = split(needs, "|");
group = L.Study == parts(1) & L.Scenario == parts(2) & L.Method == parts(3);
if numel(parts) > 3, group = group & L.Seed == str2double(parts(4)); end
if parts(1) == "S9", group = group & L.Variant == "kappa 0"; end
R = arrayfun(@(k) loadRun(runDir, k), find(group), 'UniformOutput', false);
R = [R{:}];
assert(~isempty(R), 'energyExperiments:Pilot', 'Run %d needs %s, which is not saved yet', id, needs);
switch L.Study(id)
    case "S3"   % alpha_H = omega~/w_bar of the dinkelbach pilot, scaled
        f = str2double(extractAfter(L.Variant(id), "x"));
        p.Policy_alphaH = f * R(1).stats.muEMean / R(1).stats.planMean;
    case "S8"   % the largest error per transit, and of the SOC estimate, over the runs without margins
        tr = [R.transits];
        p.Policy_transitMargins = [max(max([tr.actual] - [tr.planned]), 0), max(max([tr.cellE] - [tr.planE]), 0)];
        p.Policy_marginEnergy = max([R.socErrMax]) * R(1).capacity;
    case "S9"   % omega = kappa J_R/(N P) of the kappa = 0 run of the same seed
        k = str2double(extractAfter(L.Variant(id), "kappa "));
        p.Policy_omega = k * R(1).J / (R(1).nAgents * R(1).meanPower);
end
end

function r = loadRun(runDir, id)
S = load(runFile(runDir, id), 'r'); r = S.r;
end

function oneRun(id, p, info, runDir)
% One run, saved as runs/<id>.mat: J and its time course, SOC, energy and visits over time, the
% metrics, the policy's statistics, and DiffDrive agents' transit and SOC-estimate records
tic; o = runTrial(p); sec = toc;
L = o.log; every = max(1, round(10 / L.dt));   % every 10 s
pol = o.policy; ags = o.model.agents;
r = struct('id', id, 'study', info.Study, 'scenario', info.Scenario, 'method', info.Method, ...
    'seed', info.Seed, 'variant', info.Variant, 'J', o.finalJ, 'seconds', sec, ...
    't', L.t(1:every:end), 'Jt', L.J(1:every:end), 'soc', L.soc(1:every:end, :), ...
    'energy', sum(L.energy(1:every:end, :), 2), 'visits', L.visits(1:every:end), ...
    'metrics', o.metricValues, 'nAgents', numel(ags), 'capacity', ags(1).battery.capacity, ...
    'meanPower', sum(L.energy(end, :)) / (numel(ags) * L.t(end)), 'minSoc', min(L.soc(:)), ...
    'stats', struct(), 'charges', NaN, 'modeTime', [], 'windowMisses', NaN, 'transits', [], ...
    'socErrMax', NaN, 'trueMinSoc', NaN, 'violations', NaN, 'delays', NaN, 'params', p);
st = o.model.targets(arrayfun(@(t) isa(t, 'ChargingStation'), o.model.targets));
r.busy = NaN;   % share of the dock time in use
if ~isempty(st), r.busy = sum([st.dockTime]) / (sum([st.docks]) * L.t(end)); end
if isprop(pol, 'stats') && isstruct(pol.stats), r.stats = pol.stats; r.stats.transits = []; end
if isprop(pol, 'sessions'), r.charges = numel(pol.sessions); r.modeTime = pol.modeTime; r.windowMisses = pol.windowMisses; end
if all(arrayfun(@(a) isa(a, 'DiffDriveAgent'), ags))
    r.transits = [ags.transits];
    logs = arrayfun(@(a) a.socLog, ags, 'UniformOutput', false);
    allLog = vertcat(logs{:});
    r.socErrMax = max(abs(allLog(:, 3) - allLog(:, 2)));
    r.trueMinSoc = min(allLog(:, 2));
    r.violations = sum(cellfun(@(g) any(g(:, 2) < pol.socMin - 1e-9), logs));
    r.delays = sum([ags.delayEvents]);
end
save(runFile(runDir, id), 'r');
fprintf('%4d %-3s %-24s %-16s %2d %-10s J = %9.2f  %6.0f s\n', id, info.Study, info.Scenario, info.Method, ...
    info.Seed, info.Variant, o.finalJ, sec);
end
