function diffDriveTest()
% Headless test of DiffDriveAgent and Battery1RC (Batch 4 of the energy-aware paper's simulator
% specification). Run from the repo root:
%   matlab -batch "addpath(genpath(pwd)); diffDriveTest"
%
% 1. Fit: on a straight transit the cells deliver what the plan model predicts, within 3% (the
%    rest is the RC pair and V(SOC), which the plan leaves out).
% 2. Tracking: straight and L-shaped transits end within 0.2 s of the plan and at the target.
% 3. Regulator: with motors too weak for the plan, the robot re-times and counts a delay event.
% 4. Battery: the SOC estimate drifts by less than 1% over 10 minutes and resets at full;
%    CC-CV takes longer than constant power, whose time is (Q - E)/(P_c - P_I) plus the
%    circuit's resistive losses (within 10%).
% 5. Energy-Aware ED-RHC runs with DiffDrive agents on the office and records their transits.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root)); rmpath(genpath(fullfile(root, 'Old_Design')));
warning('off', 'optim:fmincon:SwitchingToMediumScale');
failures = strings(0); nChecks = 0;
spec = AgentSpec(Type="DiffDrive Agent", MaxSpeed=2, MaxAccel=1, Profile="SO", BatteryWh=9, ...
    MotorC1=17.75, MotorC2=1.16, MotorC3=10.46, MaxNormalAccel=1, BatteryR0=0.1, BatteryVoc=14.4);

% ---- 1-2. Fit and tracking ----
for c = 1:2
    if c == 1, pts = [0 0; 20 0]; name = "straight"; else, pts = [0 0; 10 0; 10 10]; name = "L-shaped"; end
    [a, tr] = transit(spec, pts, 1.3);
    if c == 1
        check(abs(tr.cellE / tr.planE - 1) < 0.03, sprintf(['%s transit: the cells delivered %.1f J, the plan ' ...
            'predicted %.1f J (%+.1f%%)'], name, tr.cellE, tr.planE, 100 * (tr.cellE / tr.planE - 1)));
    end
    check(abs(tr.actual - tr.planned) < 0.2 && norm(a.state.pos - pts(end, :)) < a.arriveTol && ~tr.delayed, ...
        sprintf('%s transit: %.2f s against %.2f s planned, %.3f m from the target', name, tr.actual, ...
        tr.planned, norm(a.state.pos - pts(end, :))));
end

% ---- 3. Regulator ----
a = spec.spawn(1, [0 0]); a.maxCurrent = 2.5; a.resetToInitial();
[a, tr] = transit(spec, [0 0; 30 0], 1.0, a);
check(a.delayEvents > 0 && tr.delayed, sprintf(['weak motors (2.5 A): %d delay event(s), %.2f s against %.2f s ' ...
    'planned'], a.delayEvents, tr.actual, tr.planned));

% ---- 4. Battery ----
b = Battery1RC(CapacityWh=9);
for k = 1:600, b.drainPower(60, 1); end
drift = abs(b.socEstimate - b.socTrue);
Ein = b.trueRemaining;
t = 0; while ~b.full && t < 5000, b.drainPower(23.1, 1); b.recharge(120, 1); t = t + 1; end
check(drift < 0.01 && b.socEstimate == 1 && b.socTrue > 0.98, sprintf(['SOC estimate: off by %.4f after ' ...
    '600 s at 60 W; reset to 1 at full (true SOC %.4f)'], drift, b.socTrue));
cp = Battery1RC(CapacityWh=9); cp.chargeMethod = "constant power";
for k = 1:300, cp.drainPower(60, 1); end
E0 = cp.trueRemaining; tc = 0;
while ~cp.full && tc < 5000, cp.drainPower(23.1, 1); cp.recharge(120, 1); tc = tc + 1; end
predicted = (cp.storedEnergy(1) - E0) / (120 - 23.1);
check(tc >= predicted && tc / predicted - 1 < 0.10 && t > (b.storedEnergy(1) - Ein) / (120 - 23.1), sprintf(['charging ' ...
    'at 120 W: constant power %d s (%.0f s without the circuit''s losses); CC-CV %d s (longer: the CV tail)'], ...
    tc, predicted, t));

% ---- 5. With the policy ----
q = struct('Layout', "Energy/Energy_Office_1Dock.mat", 'AgentStarts', "layout", 'AgentSettings', "layout", ...
    'TargetRates', "layout", 'UncertaintyMode', "Static Uniform", 'AgentType', "DiffDrive Agent", ...
    'Policy', "Energy-Aware ED-RHC", 'EndTime', 600, 'Dt', 0.1, 'Seed', 1, 'Log', true);
out = runTrial(q);
ags = out.model.agents;
tr = [ags.transits];
err = sort(abs([tr.actual] - [tr.planned]));
p95 = err(max(1, ceil(0.95 * numel(err))));
check(all(arrayfun(@(x) isa(x, 'DiffDriveAgent'), ags)) && numel(tr) > 10 && p95 < 1, ...
    sprintf(['Energy-Aware ED-RHC with DiffDrive agents (600 s): %d transits, arrival error 95th percentile ' ...
    '%.2f s, %d delay event(s), energy error median %+.1f%%'], numel(tr), p95, ...
    sum([ags.delayEvents]), 100 * median([tr.cellE] ./ [tr.planE] - 1)));

if isempty(failures)
    fprintf('diffDriveTest PASSED: %d checks\n', nChecks);
else
    error('diffDriveTest FAILED: %d of %d checks', numel(failures), nChecks);
end

    function check(ok, msg)
        nChecks = nChecks + 1;
        if ok, fprintf('PASS  %s\n', msg); else, fprintf('FAIL  %s\n', msg); failures(end+1) = msg; end
    end
end

function [a, tr] = transit(spec, pts, slack, a)
% One transit along pts, planned at slack times the agent's shortest transit (SO profile)
if nargin < 4, a = spec.spawn(1, pts(1, :)); a.resetToInitial(); end
a.transitTime = slack * a.limitTransit(pts);
a.transitProfile = "SO";
a.transitLimits = [a.segmentSpeedLimit(pts), a.state.maxAccel];
a.path = pts(2:end, :);
t = 0;
while ~isempty(a.path) && t < 300, a.update(0.1); t = t + 0.1; end
tr = a.transits(end);
end
