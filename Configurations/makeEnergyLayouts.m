function files = makeEnergyLayouts()
% Builds the layout files of the energy-aware paper's experiments in Configurations/Energy/,
% listed in scenarioCatalog ("Energy: ..." rows) so the GUI loads the same setups as runTrial:
%   Energy_Office_1Dock.mat      the office floor with its charging dock (1 dock) and 3 robots
%   Energy_Office_2Docks.mat     the same with 2 docks
%   Energy_Campus_2Stations.mat  the campus with two stations of 2 docks each and 6 robots
%   Energy_Sweep_m<m>_n<n>.mat   the office's station with m = 1, 2 docks and n = 2..6 robots
%                                (Sec. VII's station study, S6; runTrial 'StationPower' sweeps P_c)
%   Energy_Office_NoStation.mat  the office floor without a station, robots that do not stop when
%                                empty (energy metered; Ting-You Chung's stopping criteria, S9)
% Maps are in meters. Every robot has the paper's parameters (its simulator specification):
% 2 m/s and 1 m/s^2, SO profiles, the DC-motor model of Tokekar et al.'s corridor calibration
% (c1, c2, c3) = (17.75, 1.16, 10.46), with P_I and P_M from the Battery defaults (so the
% energy-optimal speed sqrt(dP/c2) = 1.45 m/s lies within the limits), a 1 m/s^2 turn limit,
% 9 Wh batteries (at the 43.7 W an agent draws while monitoring, a full battery lasts 741 s,
% about 20% of a one-hour mission) that
% stop the robot when empty, and an internal resistance of 0.1 Ohm at 14.4 V. Stations charge
% at 120 W, which meets Lemma 4 (P_c >= ceil(n/m) P_I = 69.4 W for 3 robots on 1 dock) with
% slack. Targets have A = 1, B = 10, R(0) = 0. Each file is a base layout loaded with
% ScenarioModel.importLayout, changed, and saved with exportLayout (rewritten only if its
% content changed). Run it again after editing:
%   matlab -batch "addpath(genpath(pwd)); makeEnergyLayouts"
here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, 'Energy');
if ~isfolder(outDir), mkdir(outDir); end
files = strings(0, 1);
files(end+1) = make1("OfficeDock", "Energy_Office_1Dock", @(m) setDocks(m, 1));
files(end+1) = make1("OfficeDock", "Energy_Office_2Docks", @(m) setDocks(m, 2));
files(end+1) = make1("Campus", "Energy_Campus_2Stations", @campusStations);
for m = 1:2
    for n = 2:6
        files(end+1) = make1("OfficeDock", sprintf("Energy_Sweep_m%d_n%d", m, n), @(md) sweep(md, m, n)); %#ok<AGROW>
    end
end
files(end+1) = make1("OfficeFloor", "Energy_Office_NoStation", @(md) [], 0);
fprintf('%d layouts in %s (changed ones rewritten)\n', numel(files), outDir);

    function file = make1(base, name, change, stopWhenEmpty)
        if nargin < 4, stopWhenEmpty = 1; end
        tmp = load(fullfile(here, base + ".mat"), 's');
        model = ScenarioModel(DijkstraCornerPlanner());
        model.importLayout(tmp.s);
        change(model);
        for t = model.targets(:)'
            if isa(t, 'ChargingStation')
                t.chargePower = 120;
            else
                t.setRates(1, 10, 0);
            end
        end
        spec = AgentSpec(Type="Abstract Agent", MaxSpeed=2, MaxAccel=1, Profile="SO", BatteryWh=9, ...
            StopWhenEmpty=stopWhenEmpty, MotorC1=17.75, MotorC2=1.16, MotorC3=10.46, MaxNormalAccel=1, ...
            BatteryR0=0.1, BatteryVoc=14.4);
        for a = model.agents(:)'
            spec.applyTo(a);
        end
        model.setUncertaintyMode('Static Uniform');
        s = model.exportLayout();
        file = fullfile(outDir, name + ".mat");
        if isfile(file)
            old = load(file, 's');
            if isequaln(old.s, s), return; end   % unchanged: keep the file (MAT headers hold a date)
        end
        save(file, 's');
    end
end

function setDocks(model, m)
for t = model.targets(:)'
    if isa(t, 'ChargingStation'), t.docks = m; end
end
end

function campusStations(model)
% Two stations of 2 docks on the open lawns west and east, each linked to two sites, and a
% sixth robot (3 per station)
west = model.addStation([15 52], 120, 2);
east = model.addStation([88 50], 120, 2);
model.addEdgeByTargets(west.index, 1); model.addEdgeByTargets(west.index, 6);
model.addEdgeByTargets(east.index, 3); model.addEdgeByTargets(east.index, 5);
model.addAgentOnTarget(model.targets(11).position, 1, 1e-6);
end

function sweep(model, m, n)
% The office's station with m docks and n robots on targets spread over the floor (the first n
% of lobby, west hall, east hall, office 2, lab B, lounge)
setDocks(model, m);
starts = [14 9 13 2 6 4];
model.agents = [];
for k = 1:n
    model.addAgentOnTarget(model.targets(starts(k)).position, 2, 1e-6);
end
end
