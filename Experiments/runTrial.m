function out = runTrial(params)
% One headless simulation run. Returns metrics plus the policy object (for its stats).
% Called by runSweep (grids of trials), the diagnostic scripts, and the paper experiments
% (Experiments/paper). Parameters: see trialDefaults. With params.Log = true the output also
% holds out.log (per-step time, J(t), target uncertainties R, agent states, SOC and energy) and out.metrics
% (trialMetrics: path length, heading change, acceleration cost, final SOC per agent). out also
% holds the energy-aware RHC metrics Je, JT, vmax and umax. Agents are spawned from one
% AgentSpec: AgentType, AgentSpeed, and Agent_<setting> for the spec's settings (e.g.
% 'Agent_profile', "SO", 'Agent_maxAccel', 10, 'Agent_kp', 16, 'Agent_batteryWh', 50); any other
% Agent_<property> is set on every agent (or its state or controller). A warning says when the
% agents do not suit the policy (ScenarioModel.checkAgentFit).
% Optional: TargetType (a registered target type; default: the layout's, else "Uncertainty
% Target") and Metrics (registered metric names; the first is the objective whose time course
% finalJ, steadyJ and slopeJ describe; default: J(t)). out.metricValues holds every metric's
% final value (fields named by metric class).

p = withDefaults(params);
root = fileparts(fileparts(mfilename('fullpath')));
if exist('ScenarioModel', 'class') ~= 8
    addpath(genpath(root));  % fallback when the project path is not loaded
end

% ---- Build scenario ----
layoutFile = char(p.Layout);                 % file name in Configurations/, or any full path
if ~isfile(layoutFile), layoutFile = fullfile(root, 'Configurations', layoutFile); end
tmp = load(layoutFile, 's');
if isfield(p, 'TargetType'), tmp.s.targetType = string(p.TargetType); end
reg = ComponentRegistry();
p.Policy = string(reg.resolve(char(p.Policy)));   % older names (e.g. 'ED-ORHC-FO') still work
plannerFactory = reg.factoryFor('pathplanner', char(p.Planner));
model = ScenarioModel(plannerFactory(), reg.factoryFor('agenttype', char(p.AgentType)));
model.importLayout(tmp.s);
if isfield(p, 'Metrics') && ~isempty(p.Metrics), model.setMetrics(p.Metrics); end
model.detectionRadius = p.DetectionRadius;
stored = arrayfun(@(a) a.initialTargetIdx, model.agents);   % start targets saved in the layout
% AgentSettings: "params" (default) places new agents from the trial parameters ("layout"
% starts reuse the stored start targets); "layout" keeps the layout's agents with their stored
% settings (speed, limits, profile, battery and power model), and only Agent_<setting>
% parameters given explicitly change them
keepAgents = string(p.AgentSettings) == "layout";
assert(keepAgents || string(p.AgentSettings) == "params", 'runTrial:AgentSettings', ...
    'AgentSettings must be "params" or "layout"');
if ~keepAgents, model.agents = []; end
% TargetRates: "params" (default) gives every target A_base and B; "layout" keeps the layout file's
% per-target A, B and R(0) (e.g. targets that only a group of agents can clear, A_i > B_i)
layoutRates = [arrayfun(@(t) t.A_uniformBase, model.targets); [model.targets.B]; [model.targets.initialR]];
for t = model.targets
    t.configure(p);
end
if string(p.TargetRates) == "layout"
    for k = 1:numel(model.targets)
        model.targets(k).setRates(layoutRates(1, k), layoutRates(2, k), layoutRates(3, k));
    end
else
    assert(string(p.TargetRates) == "params", 'runTrial:TargetRates', 'TargetRates must be "params" or "layout"');
end

starts = parseStarts(p.AgentStarts, stored);
if keepAgents
    assert(~isempty(model.agents), 'runTrial:AgentSettings', 'AgentSettings "layout": %s stores no agents', char(p.Layout));
    starts = stored;
end
if ~isempty(starts), p.NumAgents = numel(starts); end
% ponytail: Joint RHC's pairwise planner is only designed for two agents; lift this when it scales
assert(p.Policy ~= "Joint RHC" || p.NumAgents == 2, ...
    'Joint RHC is only built for 2 agents (got NumAgents = %d)', p.NumAgents);

% Agents start on AgentStarts, or spread evenly over target indices ("auto")
nT = numel(model.targets);
if isempty(starts)
    starts = mod(round((0:p.NumAgents-1) * nT / p.NumAgents), nT) + 1;   % "auto": spread evenly
end
assert(all(starts >= 1 & starts <= nT & starts == round(starts)), ...
    'AgentStarts %s must be target indices 1..%d of %s', mat2str(starts), nT, char(p.Layout));
% One spec for every agent: the type, AgentSpeed, and the Agent_<setting> parameters
spec = AgentSpec(Type=string(p.AgentType), MaxSpeed=p.AgentSpeed);
f = fieldnames(p);
for i = 1:numel(f)
    if startsWith(f{i}, 'Agent_') && ismember(extractAfter(f{i}, 'Agent_'), AgentSpec.settings)
        spec.(extractAfter(f{i}, 'Agent_')) = p.(f{i});
    end
end
model.agentSpec = spec;
if keepAgents
    for a = model.agents   % the layout's settings, with the explicit Agent_<setting> on top
        own = AgentSpec.of(a, string(p.AgentType));
        for i = 1:numel(f)
            if startsWith(f{i}, 'Agent_') && ismember(extractAfter(f{i}, 'Agent_'), AgentSpec.settings)
                own.(extractAfter(f{i}, 'Agent_')) = p.(f{i});
            end
        end
        own.applyTo(a);
    end
else
    for k = 1:p.NumAgents
        model.addAgentOnTarget(model.targets(starts(k)).position, spec.maxSpeed, 1e-6);
    end
end

model.setActivePolicy(char(p.Policy));

% Other Agent_<property> parameters set that property on every agent, its state or controller
for i = 1:numel(f)
    if startsWith(f{i}, 'Agent_') && ~ismember(extractAfter(f{i}, 'Agent_'), AgentSpec.settings)
        name = extractAfter(f{i}, 'Agent_');
        for a = model.agents
            if isprop(a, name), a.(name) = p.(f{i});
            elseif isprop(a.state, name), a.state.(name) = p.(f{i});
            elseif isprop(a.controller, name), a.controller.(name) = p.(f{i});
            else, error('runTrial:AgentProperty', 'Agents have no property "%s"', name);
            end
        end
    end
end

% Parameters named Policy_<property> set that property on the active policy (ablations).
% A switch only some policies have (e.g. Joint-only) is skipped for the others, so one sweep
% can mix policies; a name no policy has is a typo and errors.
pol = model.policyMap(char(p.Policy));
allPols = model.policyMap.values;
f = fieldnames(p);
for i = 1:numel(f)
    if startsWith(f{i}, 'Policy_')
        name = extractAfter(f{i}, 'Policy_');
        assert(any(cellfun(@(q) isprop(q, name), allPols)), 'No policy has property "%s"', name);
        if isprop(pol, name)
            pol.(name) = p.(f{i});
        end
    end
end
% A policy that plans up to the mission end (missionEnd) is told EndTime unless set explicitly,
% and one that allows for the simulation step (stepSize) is told Dt
if isprop(pol, 'missionEnd') && ~isfield(p, 'Policy_missionEnd')
    pol.missionEnd = p.EndTime;
end
if isprop(pol, 'stepSize') && ~isfield(p, 'Policy_stepSize')
    pol.stepSize = p.Dt;
end
% EnergyBeta (optional, in [0, 1)) sets an energy-aware policy's energyWeight by the paper's
% App. A rule (EDRHCPolicy.weightFromBeta; EnergyBound "speed" (default) or "accel")
if isfield(p, 'EnergyBeta') && isprop(pol, 'energyWeight')
    bound = "speed";
    if isfield(p, 'EnergyBound'), bound = p.EnergyBound; end
    pol.energyWeight = EDRHCPolicy.weightFromBeta(p.EnergyBeta, model, bound);
end
model.setUncertaintyMode(char(p.UncertaintyMode));
fit = model.checkAgentFit();
if fit ~= "", warning('runTrial:AgentPolicyFit', '%s', fit); end
model.resetSimulationState();  % also clears RHC reservation tables from earlier trials
% -1 (the default) keeps the caller's rng state. The generator is named because parfor workers
% default to another one, so a seeded run gives the same result in serial and in parallel
if p.Seed >= 0, rng(p.Seed, 'twister'); end

% ---- Simulate (same stepping as SimulationController.runToEnd) ----
clock = SimulationClock(p.EndTime, p.Dt);
n = ceil(p.EndTime / p.Dt) + 1;
tLog = zeros(n, 1); jLog = zeros(n, 1);
logOn = isfield(p, 'Log') && p.Log;
if logOn, L = emptyLog(n, numel(model.agents), nT); end
peakR = 0; coLocationTime = 0; k = 0;
while ~clock.isFinished()
    dt = clock.tick();
    model.step(dt, clock.currentTime);
    [~, J] = model.updateTargetsAndLogObjective(clock.currentTime, dt);
    k = k + 1; tLog(k) = clock.currentTime; jLog(k) = J;
    if logOn
        L = logAgents(L, k, model.agents);
        L.R(k, :) = [model.targets.R];
    end
    peakR = max([peakR, model.targets.R]);
    if any(arrayfun(@(tg) numel(tg.residingAgents) > 1, model.targets))
        coLocationTime = coLocationTime + dt;
    end
end
tLog = tLog(1:k); jLog = jLog(1:k);

% ---- Metrics ----
finalJ = jLog(end);
tail = tLog >= 0.8 * p.EndTime;
steadyJ = mean(jLog(tail));
c = polyfit(tLog(tail), jLog(tail), 1);
slopeJ = c(1);

out = struct('finalJ', finalJ, 'steadyJ', steadyJ, 'slopeJ', slopeJ, ...
    'peakR', peakR, 'coLocationTime', coLocationTime, 'policy', pol);
out.model = model;   % the scenario at the end of the run (agents, targets, stations)
out.metricValues = model.metricValues(clock.currentTime);
out.collisions = sum(model.collisions);   % steps in which an agent's motion crossed a wall
% Energy-aware RHC metrics (paper eqs. 3 and 73): Je = sum of int |u|^2 dt (NaN when an agent
% used an FO-0 profile), peak speed and acceleration; Js = finalJ; JT = energyWeight * Je + Js.
out.Je = NaN; out.JT = NaN; out.vmax = NaN; out.umax = NaN;
if ~isempty(model.agents) && all(arrayfun(@(a) isprop(a, 'Je'), model.agents))
    out.Je = sum([model.agents.Je]);
    out.vmax = max([model.agents.peakSpeed]);
    out.umax = max([model.agents.peakAccel]);
    if isprop(pol, 'energyWeight'), out.JT = pol.energyWeight * out.Je + finalJ; end
end
if logOn
    for name = ["pos", "vel", "acc", "ori", "soc", "energy", "Je", "R"]
        L.(name) = L.(name)(1:k, :, :);
    end
    L.t = tLog; L.J = jLog; L.dt = p.Dt; L.starts = starts;
    out.log = L;
    out.metrics = trialMetrics(L);
end
end

function L = emptyLog(n, nA, nT)
% Per-step states: agent pos/vel/acc are n x nA x 2, agent ori/soc/energy/Je n x nA (NaN if the
% type has none), target uncertainties R n x nT
L = struct('pos', zeros(n, nA, 2), 'vel', zeros(n, nA, 2), 'acc', zeros(n, nA, 2), ...
    'ori', zeros(n, nA), 'soc', nan(n, nA), 'energy', nan(n, nA), 'Je', nan(n, nA), 'R', zeros(n, nT));
end

function L = logAgents(L, k, agents)
for a = 1:numel(agents)
    s = agents(a).state;
    L.pos(k, a, :) = s.pos; L.vel(k, a, :) = s.vel; L.acc(k, a, :) = s.acc; L.ori(k, a) = s.ori;
    if isprop(agents(a), 'soc')
        L.soc(k, a) = agents(a).soc;
        L.energy(k, a) = agents(a).e_total;
    end
    if isprop(agents(a), 'Je'), L.Je(k, a) = agents(a).Je; end
end
end

function starts = parseStarts(v, stored)
% AgentStarts arrives as "auto", "layout", a numeric vector, or its text form "[1 5]" (from runSweep)
if isnumeric(v), starts = v(:)'; return; end
v = string(v);
if v == "auto", starts = []; return; end
if v == "layout"
    assert(~isempty(stored), 'AgentStarts "layout": the layout file stores no agent starts');
    starts = stored;
    return;
end
starts = sscanf(char(erase(v, ["[", "]", ","])), '%f')';
end

function p = withDefaults(params)
p = trialDefaults();
if nargin < 1 || isempty(params), return; end
f = fieldnames(params);
for i = 1:numel(f)
    p.(f{i}) = params.(f{i});
end
end
