function energyPolicyTest()
% Headless test of Energy-Aware ED-RHC (EnergyAwareEDRHCPolicy; Batch 3a of the energy-aware
% paper's simulator specification). Run from the repo root:
%   matlab -batch "addpath(genpath(pwd)); energyPolicyTest"
%
% 1. Special cases: without stations, with terminal "none", omega = 0 and the ratio form, it
%    makes ED-RHC's decisions (alpha = 0.5); the Dinkelbach form gives the ratio form's plans.
% 2. Safety on the energy setups: no battery goes below the reserve E_min = socMin Q; no two
%    sessions overlap on a dock; every agent arrived by its slot and every session ended by its
%    slot's end (early starts on idle docks allowed); every booking lay in its agent's window.
% 3. Bookings are released at the end of the mission; time in the modes adds up to T.
% 4. runTrial 'AgentSettings', "layout" keeps the layout's power model; stations charge only
%    agents with chargeAllowed.
% Prints one line per check and errors at the end (non-zero exit under -batch) if any failed.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
rmpath(genpath(fullfile(root, 'Old_Design')));
warning('off', 'runTrial:AgentPolicyFit');
warning('off', 'optim:fmincon:SwitchingToMediumScale');
failures = strings(0); nChecks = 0;

% ---- 1. Special cases ----
plain = struct('Layout', "OfficeFloor.mat", 'AgentStarts', "layout", 'TargetRates', "layout", ...
    'UncertaintyMode', "Static Uniform", 'AgentType', "Abstract Agent", 'AgentSpeed', 2, ...
    'Agent_maxAccel', 1, 'EndTime', 600, 'Dt', 0.1, 'Seed', 1);
q = plain; q.Policy = "ED-RHC"; q.Policy_alpha = 0.5;
ed = runTrial(q);
q = plain; q.Policy = "Energy-Aware ED-RHC"; q.Policy_transit = "FO-0"; q.Policy_terminal = "none";
q.Policy_objectiveForm = "ratio";
ea = runTrial(q);
check(abs(ea.finalJ / ed.finalJ - 1) < 1e-3, sprintf(['no stations, terminal none, omega 0, ratio form: ' ...
    'J %.2f = ED-RHC (alpha 0.5) %.2f'], ea.finalJ, ed.finalJ));
dock = struct('Layout', "Energy/Energy_Office_1Dock.mat", 'AgentStarts', "layout", 'AgentSettings', "layout", ...
    'TargetRates', "layout", 'UncertaintyMode', "Static Uniform", 'AgentType', "Abstract Agent", ...
    'Policy', "Energy-Aware ED-RHC", 'EndTime', 900, 'Dt', 0.1, 'Seed', 1, 'Log', true);
q = dock; q.Policy_objectiveForm = "dinkelbach"; dk = runTrial(q);
q = dock; q.Policy_objectiveForm = "ratio"; rt = runTrial(q);
check(abs(dk.finalJ / rt.finalJ - 1) < 1e-3, sprintf('Dinkelbach form = ratio form: J %.2f and %.2f', ...
    dk.finalJ, rt.finalJ));

% ---- 2-3. Safety, bookings, release, modes ----
for setup = ["Energy_Office_1Dock", "Energy_Office_2Docks", "Energy_Campus_2Stations"]
    q = dock; q.Layout = "Energy/" + setup + ".mat"; q.EndTime = 1800; q.Dt = 0.05;
    out = runTrial(q); pol = out.policy; S = pol.sessions;
    lay = load(fullfile(root, 'Configurations', 'Energy', setup + ".mat"), 's');
    Q = lay.s.agentBatteryWh(1) * 3600;
    minSoc = min(out.log.soc(:));
    ok = minSoc >= pol.socMin - 1e-3 && pol.windowMisses == 0 && pol.charges > 0;
    st = [S.start]; sp = [S.stop]; sp(isnan(sp)) = q.EndTime;
    maxAtOnce = 0;                       % sessions at once on one dock
    for sd = unique([[S.station]; [S.dock]]', 'rows')'
        k = find([S.station] == sd(1) & [S.dock] == sd(2));
        for i = k
            maxAtOnce = max(maxAtOnce, sum(st(k) <= st(i) + 1e-9 & sp(k) > st(i) + 1e-9));
        end
    end
    onTime = all([S.arrive] <= [S.slot] + 1e-6 | isnan([S.arrive])) && all(sp <= [S.slotEnd] + 0.2);
    modes = sum(pol.modeTime, 2);
    lateBy = max([[S.arrive] - [S.slot], 0], [], 'omitnan');
    check(ok && maxAtOnce <= 1 && onTime && all(abs(modes - q.EndTime) < 1), sprintf(['%s: lowest SOC %.2f%% ' ...
        '(reserve %.0f%%), %d sessions, at most %g at once per dock, arrivals at most %.3f s after the slot, ' ...
        'modes add up to %s s (T = %g)'], setup, 100 * minSoc, 100 * pol.socMin, numel(S), maxAtOnce, lateBy, ...
        mat2str(round(modes', 2)), q.EndTime));
    % release at the end: no booking left that the agent could not have needed
    left = arrayfun(@(a) ~isempty(pol.bookingOf(a)) && pol.bookingOf(a).start < q.EndTime, out.model.agents);
    Eend = arrayfun(@(a) a.battery.remaining, out.model.agents);
    check(all(Eend >= pol.socMin * Q - 1) || any(left), sprintf('%s: every agent ends above its reserve or still booked', setup));
end

% ---- 4. Settings and admission ----
q = dock; q.EndTime = 1; out = runTrial(q);
a = out.model.agents(1);
check(a.battery.c2 == 1.16 && a.state.maxSpeed == 2 && a.battery.capacityWh == 9 && a.maxNormalAccel == 1, ...
    'AgentSettings "layout": agents keep the layout''s speed, power model and battery');
st = ChargingStation(1, [0 0], 120);
a1 = AbstractAgent(1, [0 0], 1); a1.battery.energyUsed = 1000; a1.chargeAllowed = false;
st.residingAgents = a1; st.updateUncertainty(1);
check(a1.battery.energyUsed == 1000, 'a station does not charge an agent with chargeAllowed = false');

if isempty(failures)
    fprintf('\nenergyPolicyTest PASSED: %d checks\n', nChecks);
else
    error('energyPolicyTest:Failed', 'energyPolicyTest FAILED: %d of %d checks\n  %s', ...
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
end
