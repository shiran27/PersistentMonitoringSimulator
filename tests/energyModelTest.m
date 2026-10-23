function energyModelTest()
% Headless test of the full power model of the energy-aware paper (Batch 1 of its simulator
% specification): P = P_m + c1 u^2 + c2 v^2 + c3 v, the least-loss SO and FO-3 profiles, the
% curvature speed limit and battery losses. Run from the repo root:
%   matlab -batch "addpath(genpath(pwd)); energyModelTest"
%
% 1. The defaults change nothing: c2 = 0 gives the earlier profiles bit for bit, and the
%    quadrature of a profile matches its closed-form energy.
% 2. SO without limits: Theorem 1's closed form, the profile's integrals and a QP solve agree.
% 3. SO with a speed limit (Theorem 2), an acceleration limit, and both: profile against QP.
% 4. FO-3: Theorem 3's root against a dense search over trapezoids, with and without limits.
% 5. Every profile starts and ends at rest, covers y, keeps both limits, has u = dv/dt and
%    v = dl/dt, and (SO) a continuous u.
% 6. An Abstract Agent with c2 > 0 and battery losses draws the energy predicted for its plan.
% 7. The curvature speed limit (Assumption A1) and the shortest transit (Lemma 1).
% 8. AgentSpec, runTrial (Agent_motorC2, ...) and layouts carry the new settings.
% The QP checks need quadprog (Optimization Toolbox); an expired MOSEK toolbox that shadows it
% is taken off the path for this test. Prints one line per check and errors at the end
% (non-zero exit under -batch) if any failed.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
q = which('quadprog');
if contains(q, 'mosek', 'IgnoreCase', true), rmpath(fileparts(q)); end
haveQP = ~isempty(which('quadprog')) && license('test', 'Optimization_Toolbox');
failures = strings(0); nChecks = 0;
sets = struct('name', {"corridor (Tokekar)", "simulator-like"}, 'c1', {17.75, 0.556}, 'c2', {1.16, 0.05});
y = 10;

% ---- 1. Defaults ----
same = true;
for name = ["FO-0", "SO", "FO-3"]
    for lim = [Inf Inf; 3 Inf; Inf 2; 3 2]'
        for rho = [2 5 20]
            if rho < SpeedProfile.minTransit(y, lim(1), lim(2)), continue; end
            same = same && isequal(SpeedProfile.shape(name, y, rho, lim(1), lim(2)), ...
                SpeedProfile.shape(name, y, rho, lim(1), lim(2), 0));
        end
    end
end
check(same, 'c2 = 0 (nu = 0) gives the earlier profiles bit for bit');
err = 0;
for name = ["SO", "FO-3"]
    for lim = [Inf Inf; 3 Inf; Inf 2; 3 2]'
        for rho = [5 20]
            Eu = SpeedProfile.integrals(SpeedProfile.shape(name, y, rho, lim(1), lim(2)));
            err = max(err, abs(Eu / SpeedProfile.energy(name, y, rho, lim(1), lim(2)) - 1));
        end
    end
end
check(err < 1e-12, sprintf('quadrature of int u^2 matches the closed forms (largest error %.1e)', err));
b = Battery(); b.drain(1.3, 0.7, true, 0.05);
Eold = ((b.cpuMove + b.commActive) + (b.P0 + b.alphaVel * 1.3 + b.gammaAcc * 0.7^2)) / b.eta * 0.05;   % as drain groups it
check(b.energyUsed == Eold, 'default battery drains exactly as before (c2 = 0, R0 = 0)');

% ---- 2-5. Profiles of the full power model ----
for k = 1:numel(sets)
    c1 = sets(k).c1; c2 = sets(k).c2; nu = sqrt(c2 / c1);
    % 2. SO without limits (Theorem 1)
    e1 = 0; e2 = 0;
    for rho = [6 10 20 40]
        th = nu * rho / 2;
        Lcf = c1 * nu^3 * y^2 / (2 * (th - tanh(th)));
        P = SpeedProfile.shape("SO", y, rho, Inf, Inf, nu);
        Lp = SpeedProfile.loss("SO", y, rho, Inf, Inf, c1, c2);
        e1 = max(e1, abs(Lp / Lcf - 1));
        if haveQP, e2 = max(e2, abs(Lcf / qpMin(c1, c2, y, rho, Inf, Inf) - 1)); end
        sane(P, Inf, Inf, true, sprintf('%s: SO free, rho %g', sets(k).name, rho));
    end
    check(e1 < 1e-9, sprintf('%s: SO free, Theorem 1 = profile integrals (largest error %.1e)', sets(k).name, e1));
    if haveQP, check(e2 < 1e-4, sprintf('%s: SO free, Theorem 1 = QP (largest error %.1e)', sets(k).name, e2)); end
    % 3. SO with limits: speed (Theorem 2), acceleration, both
    cases = {1.3, Inf, [8 10 14], "hypV"; 1.6, Inf, [8 10 14], "hypV"; ...
             Inf, 0.6, [8.5 10 14], "hypA"; 1.0, 0.6, [12 14 20], "hypVA"};
    for c = 1:size(cases, 1)
        [vb, ub, rhos, kind] = cases{c, :};
        e1 = 0; e2 = 0; kinds = strings(0);
        for rho = rhos
            if rho < SpeedProfile.minTransit(y, vb, ub), continue; end
            P = SpeedProfile.shape("SO", y, rho, vb, ub, nu);
            kinds(end+1) = P.kind; %#ok<AGROW>
            Lp = SpeedProfile.loss("SO", y, rho, vb, ub, c1, c2);
            if P.kind == "hypV", e1 = max(e1, abs(Lp / soSpeedClosedForm(c1, c2, y, rho, vb) - 1)); end
            if haveQP, e2 = max(e2, abs(Lp / qpMin(c1, c2, y, rho, vb, ub) - 1)); end
            sane(P, vb, ub, true, sprintf('%s: SO vb %g ub %g rho %g (%s)', sets(k).name, vb, ub, rho, P.kind));
        end
        label = sprintf('%s: SO, speed limit %g, acceleration limit %g (%s)', sets(k).name, vb, ub, strjoin(unique(kinds), ", "));
        check(any(kinds == kind), label + ": regime " + kind + " reached");
        if kind == "hypV", check(e1 < 1e-9, sprintf('%s: Theorem 2 = profile integrals (largest error %.1e)', label, e1)); end
        if haveQP, check(e2 < 5e-4, sprintf('%s: profile = QP (largest error %.1e)', label, e2)); end
    end
    % 4. FO-3 (Theorem 3), without and with limits
    e1 = 0; e2 = 0;
    for lim = [Inf Inf; 1.0 Inf; Inf 0.6; 1.0 0.6]'
        for rho = [10 14 20 40]
            if rho < SpeedProfile.minTransit(y, lim(1), lim(2)), continue; end
            bounds = [rho / 2, rho - y / lim(1), (rho - sqrt(max(rho^2 - 4 * y / lim(2), 0))) / 2];
            taus = unique([linspace(1e-4, rho / 2, 400001), bounds(bounds > 0 & bounds <= rho / 2)]);
            vc = y ./ (rho - taus);
            ok = vc <= lim(1) & vc ./ taus <= lim(2);
            Lt = y^2 * (2 * c1 ./ taus + c2 * (rho - 4 * taus / 3)) ./ (rho - taus).^2;
            Lp = SpeedProfile.loss("FO-3", y, rho, lim(1), lim(2), c1, c2);
            e1 = max(e1, abs(Lp / min(Lt(ok)) - 1));
            if all(isinf(lim))
                P = SpeedProfile.shape("FO-3", y, rho, Inf, Inf, nu);
                e2 = max(e2, abs(c2 * P.s^2 * (2 * P.s - rho) - 3 * c1 * (3 * P.s - rho)) / (c1 * rho));
            end
            sane(SpeedProfile.shape("FO-3", y, rho, lim(1), lim(2), nu), lim(1), lim(2), false, ...
                sprintf('%s: FO-3 rho %g', sets(k).name, rho));
        end
    end
    check(e1 < 1e-6, sprintf('%s: FO-3 loss = best trapezoid of a dense search, with and without limits (largest error %.1e)', sets(k).name, e1));
    check(e2 < 1e-12, sprintf('%s: FO-3 acceleration time solves Theorem 3''s cubic', sets(k).name));
end

% ---- 6. Drawn = predicted energy, with c2 > 0 and battery losses ----
for R0 = [0 0.15]
    a = AbstractAgent(1, [0 0], 2, Profile="SO");
    a.battery.c2 = 0.05; a.battery.R0 = R0; a.state.maxAccel = 100;
    a.path = [10 0]; a.transitTime = 12; a.transitProfile = "SO";
    P = SpeedProfile.shape("SO", 10, 12, Inf, Inf, a.motionNu());
    Epred = a.battery.planEnergy(P);
    dt = 1e-3; n = 0;
    while ~isempty(a.path) && n < 1e5, a.update(dt); n = n + 1; end
    check(P.kind == "hyp" && abs(a.battery.energyUsed / Epred - 1) < 2e-4, sprintf(['Abstract Agent, c2 0.05, ' ...
        'R0 %g: drawn %.2f J, predicted %.2f J (%+.3f%%)'], R0, a.battery.energyUsed, Epred, 100 * (a.battery.energyUsed / Epred - 1)));
end
b = Battery(); b.c2 = 0.05;
[Eu, Ev] = SpeedProfile.integrals(P);
check(abs(b.planEnergy(P) - (b.movePower() * 12 + b.c3 * 10 + b.c1 * Eu + b.c2 * Ev)) < 1e-9 && ...
    abs(b.excessEnergy(P) - (b.planEnergy(P) - b.idlePower() * 12)) < 1e-9, ...
    'without losses, E = P_M rho + c3 y + L and E^ex = E - P_I rho');

% ---- 7. Curvature speed limit and shortest transit ----
r = 4; ang = (0:0.1:pi/2)';
pts = [-5 0; 0 0; r * sin(ang), r * (1 - cos(ang)); r, r + 5];
a = AbstractAgent(1, [0 0], 5);
check(a.segmentSpeedLimit(pts) == 5, 'no normal acceleration limit: the speed limit is maxSpeed');
a.maxNormalAccel = 2; a.state.maxAccel = 1.5;
vb = a.segmentSpeedLimit(pts);
check(abs(vb - sqrt(2 * r)) < 1e-9, sprintf('arc of radius %g, a_n 2: speed limit %.4f = sqrt(a_n r)', r, vb));
yy = sum(vecnorm(diff(pts, 1, 1), 2, 2));
check(abs(a.limitTransit(pts) - (yy / vb + vb / 1.5)) < 1e-12, 'shortest transit y/v + v/u (Lemma 1)');
P = a.planProfile(pts, 1, "SO");
check(abs(P.rho - a.limitTransit(pts)) < 1e-12, 'a plan shorter than Lemma 1''s transit is stretched to it');

% ---- 8. Settings in AgentSpec, runTrial and layouts ----
spec = AgentSpec(MotorC2=0.05, MaxNormalAccel=2, BatteryR0=0.1, BatteryVoc=12);
a = spec.spawn(1, [0 0]);
back = AgentSpec.of(a);
check(a.battery.c2 == 0.05 && a.maxNormalAccel == 2 && a.battery.R0 == 0.1 && a.battery.Voc == 12 && ...
    back.motorC2 == 0.05 && back.maxNormalAccel == 2, 'AgentSpec sets and reads c2, a_n, R0 and Voc');
lay = load(fullfile(root, 'Configurations', 'ReportLayout.mat'), 's');
m = ScenarioModel(); m.importLayout(lay.s);
s0 = m.exportLayout();
m.agents(2).battery.c2 = 0.05; m.agents(2).maxNormalAccel = 2; m.agents(2).battery.R0 = 0.1;
s1 = m.exportLayout();
m2 = ScenarioModel(); m2.importLayout(s1);
check(~isfield(s0, 'agentMotorC2') && ~isfield(s0, 'agentBatteryR0') && m2.agents(2).battery.c2 == 0.05 && ...
    m2.agents(1).battery.c2 == 0 && m2.agents(2).maxNormalAccel == 2 && m2.agents(2).battery.R0 == 0.1, ...
    'layouts store c2, a_n and R0 only when set, and load them per agent');
warning('off', 'runTrial:AgentPolicyFit');
t = struct('Layout', "ReportLayout.mat", 'AgentStarts', "layout", 'Policy', "ED-RHC", 'EndTime', 20, 'Seed', 1, ...
    'Agent_motorC2', 0.05, 'Agent_batteryR0', 0.1, 'Log', true);
out = runTrial(t);
t = rmfield(t, {'Agent_motorC2', 'Agent_batteryR0'});
base = runTrial(t);
check(sum(out.metrics.Energy) > sum(base.metrics.Energy) && out.finalJ == base.finalJ, sprintf(['runTrial ' ...
    'Agent_motorC2 / Agent_batteryR0: energy %.1f kJ against %.1f kJ, same J'], sum(out.metrics.Energy) / 1000, ...
    sum(base.metrics.Energy) / 1000));

if isempty(failures)
    fprintf('\nenergyModelTest PASSED: %d checks\n', nChecks);
else
    error('energyModelTest:Failed', 'energyModelTest FAILED: %d of %d checks\n  %s', ...
        numel(failures), nChecks, strjoin(failures, newline + "  "));
end

    function check(ok, label)
        nChecks = nChecks + 1;
        if ok
            fprintf('PASS  %s\n', label);
        else
            fprintf('FAIL  %s\n', label);
            failures(end+1) = string(label);
        end
    end

    function sane(P, vb, ub, smooth, label)
        % Rest to rest over y, within the limits, u = dv/dt and v = dl/dt, u continuous (SO)
        tt = linspace(0, P.rho, 4001); h = tt(2) - tt(1);
        l = zeros(size(tt)); v = l; u = l;
        for i = 1:numel(tt), [l(i), v(i), u(i)] = SpeedProfile.at(P, tt(i)); end
        vs = max(v); us = max(abs(u));
        dv = (v(3:end) - v(1:end-2)) / (2 * h); dl = (l(3:end) - l(1:end-2)) / (2 * h);
        mid = 2:numel(tt) - 1;
        jumps = max(abs(diff(u)));
        ok = abs(l(end) - P.y) < 1e-9 * P.y && abs(v(1)) < 1e-9 * vs && abs(v(end)) < 1e-9 * vs && ...
            all(v >= -1e-9 * vs) && vs <= vb * (1 + 1e-9) && us <= ub * (1 + 1e-9) && ...
            max(abs(dl - v(mid))) < 1e-3 * vs && (~smooth || (max(abs(dv - u(mid))) < 1e-2 * us && jumps < 0.05 * us));
        if ~ok, check(false, label + ": profile sanity"); end
    end
end

function L = soSpeedClosedForm(c1, c2, y, rho, vb)
% Theorem 2 of the energy-aware paper (speed limit active), as in its check script
nu = sqrt(c2 / c1);
x = fzero(@(x) (sinh(x) - x) ./ (cosh(x) - 1) - nu * (vb * rho - y) / (2 * vb), [1e-3, 40]);
t1 = x / nu;
B = -vb / (cosh(x) - 1); K = vb * cosh(x) / (cosh(x) - 1);
Iu = B^2 * nu^2 * (sinh(2 * x) / (4 * nu) - t1 / 2);
Iv = K^2 * t1 + 2 * K * B * sinh(x) / nu + B^2 * (sinh(2 * x) / (4 * nu) + t1 / 2);
L = 2 * (c1 * Iu + c2 * Iv) + c2 * vb^2 * (rho - 2 * t1);
end

function J = qpMin(c1, c2, y, rho, vb, ub)
% min int (c1 u^2 + c2 v^2) dt, rest to rest over y in rho, 0 <= v <= vb, |u| <= ub: a QP on
% N steps (the energy-aware paper's check script, Claude_Notes/Tools/scripts/energy)
N = 600; h = rho / N; n = N + 1;
D = ([-eye(N) zeros(N, 1)] + [zeros(N, 1) eye(N)]) / h;
Wv = h * ones(n, 1); Wv([1 end]) = h / 2;
H = 2 * (c1 * h * (D' * D) + c2 * diag(Wv));
Aeq = [Wv'; [1 zeros(1, N)]; [zeros(1, N) 1]]; beq = [y; 0; 0];
if isfinite(ub), A = [D; -D]; b = ub * ones(2 * N, 1); else, A = []; b = []; end
opts = optimoptions('quadprog', 'Display', 'off', 'OptimalityTolerance', 1e-12, 'ConstraintTolerance', 1e-12);
[~, fval, flag] = quadprog(H, zeros(n, 1), A, b, Aeq, beq, zeros(n, 1), vb * ones(n, 1), [], opts);
if flag <= 0, J = NaN; else, J = fval; end
end
