function tests = collabPolicyTest
% Checks of Collab-ED-RHC (CollabEDRHCPolicy). Run: runtests('collabPolicyTest') (about 2 min).
%   - reduction: with one agent and no terminal correction it reproduces ED-RHC with alpha = 0.5
%     (Th. 2 of the collaboration paper), up to the numerical RHCP solutions
%   - sharing: on targets that one agent cannot clear (A > B) the agents share them, and the
%     result is better than ED-RHC, which cannot share
%   - groups of three or more agents at one target are planned jointly
%   - bookkeeping: every solve is counted, shifts only happen in coordination mode B
tests = functiontests(localfunctions);
end

function testReductionToEDRHC(tc)
p = base("JS_1Agent_3T", 120);
p.Policy = "ED-RHC"; p.Policy_alpha = 0.5;
ed = runTrial(p);
p = base("JS_1Agent_3T", 120);
p.Policy = "Collab-ED-RHC"; p.Policy_coordination = "A"; p.Policy_terminal = "none";
co = runTrial(p);
verifyEqual(tc, co.finalJ, ed.finalJ, 'RelTol', 0.01);
end

function testSharingWhereNeeded(tc)
p = base("JS_2Agent_5T", 80);
p.Layout = fullfile("Collab", "Collab_2Agent_5T_Group.mat");   % A_3 = 12: needs two agents (B = 10)
p.TargetRates = "layout";
p.Policy = "ED-RHC"; p.Policy_alpha = 0.5;
ed = runTrial(p);
p.Policy = "Collab-ED-RHC"; p = rmfield(p, 'Policy_alpha');
co = runTrial(p);
verifyGreaterThan(tc, co.coLocationTime, 0);
verifyLessThan(tc, co.finalJ, ed.finalJ);
end

function testGroupOfThree(tc)
% three and four agents residing at one target plan jointly (sampled candidates, nS - 1 >= 2)
for starts = {[1 1 1], [1 1 1 1]}
    p = base("JS_3Agent_General", 10);
    p.AgentStarts = starts{1}; p.Policy = "Collab-ED-RHC";
    o = runTrial(p);
    verifyGreaterThan(tc, o.policy.stats.solves, 0);
    verifyTrue(tc, isfinite(o.finalJ));
end
end

function testStats(tc)
p = base("JS_3Agent_General", 40);
p.Policy = "Collab-ED-RHC"; p.Policy_coordination = "A";
a = runTrial(p);
verifyEqual(tc, a.policy.stats.shiftEvents, 0);
verifyEqual(tc, sum(a.policy.stats.events), a.policy.stats.solves);
verifyGreaterThan(tc, a.policy.stats.solves, 0);
end

function p = base(preset, T)
s = load(fullfile(fileparts(which('MainUI')), 'Configurations', 'Legacy', preset + ".mat"), 's');
p = struct('Layout', fullfile("Legacy", preset + ".mat"), 'AgentStarts', "layout", ...
    'Planner', "Dijkstra Corners", 'AgentType', "Abstract Agent", 'Agent_profile', "FO-0", ...
    'UncertaintyMode', "Static Uniform", ...
    'A_base', 1, 'B', 10, 'EndTime', T, 'Dt', 0.05, 'DetectionRadius', 1e-6, 'Policy_H', 250, ...
    'AgentSpeed', s.s.agentSpeed(1));
end
