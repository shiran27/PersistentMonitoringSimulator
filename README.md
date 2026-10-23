# MultiAgentSimulator

A MATLAB simulator for experimenting with multi-agent path planning, navigation policies, and agent behavior. Agents move between targets on a user-drawn graph, avoiding walls, while an objective function tracks how well the swarm covers the environment over time.

The project is designed around five **modular areas** so new algorithms can be dropped in and compared side-by-side from the UI:

- **Path Planner** — how a path between two targets is computed (Dijkstra Corners, Energy Efficient, Grid A*).
- **Agent Policy** — how an agent decides which target to go to next (base class `Policy`).
- **Agent Type** — the agent class itself (its dynamics, state, energy model, rendering, etc.; base class `Agent`).
- **Target Type** — what a target's state is and how it evolves (base class `Target`).
- **Metric** — how a run is scored; the first one is the objective plotted over time (base class `Metric`).

New to the simulator? `Examples/EventDetection/` sets up a new monitoring problem from scratch (see [Getting started](#getting-started-a-new-monitoring-problem)).

## Examples

### Main UI
The application on launch — Modular Components bar on top, Tools panel on the left, Time Control below, canvas on the right.

![Main UI overview](screenshots/overview.png)

### Building a scenario
Targets, edges, and walls drawn on the canvas before running the simulation.

![Scenario with targets, edges, and walls](screenshots/scenario.png)

### Simulation running
Agents following their paths mid-simulation. The Modular Components dropdowns are locked while the sim is active.

![Simulation in progress](screenshots/running.png)

### Objective plot
The objective function `J(t)` tracked over the course of a run.

![Objective function over time](screenshots/objective.png)

## Running the project

1. Open MATLAB and set the working directory to the repo root (`MultiAgentSimulator`).
2. In the Command Window, run:
   ```matlab
   runMainUI
   ```
   This adds all subfolders to the path and opens the UI. `runMainUI('Monitor', 2)` opens the windows on
   another monitor (a row of `get(groot, 'MonitorPositions')`), sized to fit it, and
   `runMainUI('PaperProfile', "Energy paper")` starts with a [paper profile](#paper-profiles).
3. Pick a ready-made scenario from the **Scenario** dropdown (see [Scenarios](#scenarios)), or build your own. Use the dropdowns in the **Tools** panel to pick a Path Planner, Agent Policy, Agent Type, and Uncertainty mode. These can only be changed while the simulation is **not** running.
4. Use the **Tools** panel to add targets, agents, edges, and walls. **Agent Speed**, **Agent Accel** and **Speed Profile** set the next agents you place (see [Agent types](#agent-types)). Use the **Time Control** panel to play/pause the simulation.

## Scenarios

The **Scenario** dropdown (top of the Tools panel; `MainUI.loadScenario(name)` in code) loads a ready-made scenario in one click: background map, targets, edges, walls and agents. The agents take the selected **Agent Type**, so every scenario runs with either agent type and any policy. When the agents do not suit the policy (see the compatibility table below), the status line says what to expect.

| Scenario | Layout | What it is |
|---|---|---|
| Lab floor (Stevens) | `ReportLayout.mat` | Stevens lab floor plan; 11 targets, 5 agents; the simulator paper's reference scenario |
| Office floor | `OfficeFloor.mat` | One floor of an office and lab building: eight rooms with doors (gaps in the walls), a hall and a lobby; 14 targets in rooms and halls, 3 robots |
| Office floor with charging dock | `OfficeDock.mat` | The office floor with a charging station (300 W dock by the lobby) and small batteries (2.5 Wh, about four minutes of operation; the robots stop when empty): the scenario of [Battery-aware ED-RHC](#battery-aware-ed-rhc-and-charging-stations) |
| City patrol | `CityPatrol.mat` | Police patrol on a street grid: 16 intersections and 5 sites (police HQ, bank, school, mall, hospital); every edge runs along a road, so the 4 patrol cars stay on the roads; the nine blocks are obstacles |
| Harbor patrol | `HarborPatrol.mat` | Coast with a port, piers and a breakwater, two islands; 12 buoys and berths, 3 boats |
| Campus | `Campus.mat` | Zoomed-out building complex: six buildings and a pond (obstacles), quad, parking; 11 sites, 5 agents |
| Museum (event detection example) | `Museum.mat` | The beginner's example: four galleries around a hall, event targets (`SimpleTarget`), 3 robots at speed 4; every wall and target on the GUI's click grid |
| JS: ... | `Legacy/JS_*.mat` | The eight presets of the legacy JS simulator (RHC papers), scaled to the canvas |
| Energy: ... | `Energy/Energy_*.mat` | The energy-aware paper's setups (`makeEnergyLayouts`): the office floor with a 1-dock or 2-dock station and 3 robots, and the campus with two 2-dock stations and 6 robots; 120 W stations, robots at 2 m/s with the [full power model](#full-power-model-energy-aware-paper) and 9 Wh batteries |

`Configurations/scenarioCatalog.m` lists them. `Configurations/makeScenarioLayouts.m` draws the maps (`Configurations/backgrounds/*.png`) and writes their layouts from the same geometry, so walls line up with the drawn walls, buildings, coasts and piers. It also restyles the lab floor plan (`LabLayout.png`) in the same palette as `backgrounds/LabFloor.png`. Edit it to change a scenario, or add a row to the catalog for a new layout file.

## Paper profiles

The **Paper profile** dropdown (top of the Tools panel; `MainUI.setPaperProfile(name)` in code) shows only the components, speed profiles and scenarios that one paper uses, so its setups are quick to find. **Full** (the default) shows everything. The profiles are listed in `Configurations/paperProfiles.m`:

| Profile | Shows |
|---|---|
| Full | Everything |
| Simulator paper | Random Walk, ED-RHC, ED-ORHC-SO, ED-ORHC-FO-3, Battery-Aware ED-RHC and the example's classes; Abstract and Physical Agents; every planner, metric and scenario |
| Collaboration paper | ED-RHC and Collab-ED-RHC with Abstract Agents on FO-0 transits; the JS presets, the "Collab: ..." setups and the Lab floor, Office floor, City patrol, Harbor patrol and Campus maps; J(t) |
| Energy paper | ED-RHC, ED-ORHC-SO, ED-ORHC-FO-3, Battery-Aware ED-RHC and Energy-Aware ED-RHC; Abstract and Physical Agents with SO and FO-3 profiles; the "Energy: ..." setups and the Lab floor; J(t) and Minimum SOC |

A profile only filters the dropdowns: when it hides the current choice, the first shown one is selected, and a loaded layout still shows the options it uses. The registry, layouts and `runTrial` ignore profiles, so no result depends on one. To add a profile, add a row to `paperProfiles.m`; a field left empty shows everything of that kind, and a scenario entry ending in `*` matches every scenario whose name starts with the rest (e.g. `"JS: *"`). `extensionTest` checks that every listed name exists.

## Target uncertainty and defaults

Each target's uncertainty follows `dR_i/dt = (A_i - B_i N_i) v_i`, where `N_i` is the number of agents at rest at the target (see [Agent types](#agent-types)) and `v_i` keeps `R_i >= 0`. The defaults match the simulator paper:

| Parameter | Default |
|---|---|
| Uncertainty mode | `Static Uniform` (constant `A_i`) |
| Growth rate `A_i` | 1 for every target |
| Decay rate `B_i` (per agent) | 10 |
| Initial uncertainty `R_i(0)` | 0 |

The other modes are opt-in from the **Uncertainty** dropdown:
- `Static Priority Target` — target 1 grows at `A = 5`, the rest at their `A_i`.
- `Dynamic Uncertainty` — as above, plus periodic spikes: every target's rate jumps to `A_spike = 14` for 10 s every 250 s, staggered by 50 s from one target to the next. With `B = 10`, one agent cannot bring a spiking target down (`14 - 10 > 0`) but two can (`14 - 2*10 < 0`).

The experiment harness (`Experiments/runTrial`, `Experiments/trialDefaults.m`) sets its own uncertainty parameters for every trial (by default `Dynamic Uncertainty` with `B = 20` and `A_spike = 28`, the same one-agent/two-agent balance), so its results do not depend on these GUI defaults.

## Layout files

**Save Layout...** / **Load Layout...** (and `ScenarioModel.exportLayout` / `importLayout`) use a `.mat` file holding a struct `s`:

| Field | Meaning |
|---|---|
| `targets` | `n x 2` target positions |
| `edges` | `m x 2` target index pairs |
| `walls` | `k x 4` wall segments `[x1 y1 x2 y2]` |
| `agentStarts` | start target index of each agent |
| `agentSpeed` | maximum speed of each agent (or one value for all) |
| `agentAccel` | maximum acceleration of each agent (or one value for all) |
| `agentProfile` | speed profile of each agent: `"FO-0"`, `"SO"` or `"FO-3"` |
| `agentBatteryWh` | battery capacity of each agent [Wh] (or one value for all) |
| `agentStopWhenEmpty` | 1 if an agent stops when its battery is empty (or one value for all) |
| `agentMotorC1`, `agentMotorC2`, `agentMotorC3`, `agentMaxNormalAccel` | [full power model](#full-power-model-energy-aware-paper): `c1`, `c2`, `c3` and the turn limit `a_n` of each agent (saved only when set) |
| `agentBatteryR0`, `agentBatteryVoc` | battery internal resistance [Ohm] and open-circuit voltage [V] of each agent (saved only when `R0 > 0`) |
| `chargers`, `chargePower` | targets that are charging stations (`ChargingStation`), and their charging power [W] |
| `chargerDocks` | number of docks of each station (saved only when limited) |
| `background` | background image (a file in `Configurations/backgrounds/`, the project root, or a full path) |
| `uncertaintyMode` | one of the modes above |
| `A`, `B`, `R0` | per-target `A_i`, `B_i`, `R_i(0)` (`n x 1`, or one value for all) |

Only `targets` and `edges` are required, so files saved by older versions still load: missing fields keep the defaults above (agents take the GUI's Agent Accel and Speed Profile) and no agents are placed. Loaded agents use the Agent Type selected in the dropdown. `runTrial` ignores stored agents and places its own from the `AgentStarts` parameter.

`Configurations/ReportLayout.mat` is the reference scenario of the simulator paper (Stevens Smart Lab, background `LabFloor.png`: `LabLayout.png` recolored, without the diamond-shaped block in the middle): 11 targets, 10 edges, 92 walls (3 placed by hand plus 89 traced from the floor plan by `makeScenarioLayouts`, so routes stay on the open floor), and 5 agents at speed 15 starting on targets **1, 6, 2, 3, 5** (agent 1 on target 1, agent 2 on target 6, and so on), in `Static Uniform` mode with `A = 1`, `B = 10`, `R(0) = 0`.

## Smoke test

Run from the repo root before opening a pull request:
```
matlab -batch "addpath(genpath(pwd)); smokeTest"
```
`tests/smokeTest.m` checks the defaults and the layout file round trip, then runs every registered policy x path planner x agent type for 20 s on `ReportLayout.mat` and checks that each run finishes with a finite objective `J`. It also checks the agent types (older names, every speed profile, service only at rest, Physical Agents arriving when planned, smooth heading), that William's Joint RHC harness runs, every catalog scenario, and the battery model (predicted transit energy, Battery-Aware ED-RHC recharging in time while ED-RHC runs out). It prints one line per check and exits with an error if any check fails (about 20 s). New components registered in `ComponentRegistry` are picked up automatically.

`tests/guiTest.m` drives the real GUI with MATLAB's App Testing Framework: it loads `ReportLayout.mat` over the lab background and runs to the end, plays, pauses and resets, saves, clears and reloads a layout, builds a small scenario by clicking, places a charging station, loads every ready-made scenario, filters the menus by paper profile, and checks that a GUI run and a headless `runTrial` give the same `J`. It needs a display; the windows open on a secondary monitor if there is one and close when it finishes (about 2 min):
```
matlab -batch "addpath(genpath(pwd)); assertSuccess(runtests('guiTest'))"
```
Screenshots of each stage are written to `results/gui/` (git-ignored).

`tests/energyModelTest.m` checks the full power model and the stations: the defaults change nothing; the SO and FO-3 profiles match Theorems 1-3 and a QP solve (within 0.02%) in every regime; every profile is rest to rest, within its limits and smooth; an Abstract Agent draws the predicted energy with and without battery losses; the turn limit and Lemma 1; the new settings in `AgentSpec`, layouts and `runTrial`; docks and slots; the energy-optimal transit (Proposition 2); route tables equal to shortest paths; home stations; and the energy layouts:
```
matlab -batch "addpath(genpath(pwd)); energyModelTest"
```

`tests/extensionTest.m` checks the extension points: switching target types, the metrics and `runTrial`'s `metricValues`, the event-detection example, the Grid A* planner on every map (no wall crossings), smooth paths from every planner, the warning for an unreachable edge, charging stations added and reloaded, the wall check during motion, the names in the paper profiles, and the older profile and policy names:
```
matlab -batch "addpath(genpath(pwd)); extensionTest"
```

## Agent types

There are two agent types (`Components/AgentTypes/`): the **Abstract Agent**, an idealized model that executes its plan exactly (the agents of the RHC papers), and the **Physical Agent**, which executes the same plan through feedback control and actuator limits. Both follow the path the planner draws, carry a battery (power model in `Agent.m`, reported as SOC), and record their energy `J_e`, peak speed and peak acceleration.

| | Abstract Agent (`AbstractAgent`) | Physical Agent (`PhysicalAgent`) |
|---|---|---|
| Motion | Follows its speed profile exactly along the path (arc length), at rest on the targets | Double integrator (speed and acceleration saturate at `maxSpeed` and `maxAccel`) with a PD law and feedforward tracking a reference it can follow: the path with its corners rounded, and a speed profile within the full limits (`planMargin` = 1; the acceleration counts its part along the path and the turn's part across it, sqrt(u² + (v²κ)²) ≤ `maxAccel`) that slows down for turns |
| Timing | Every transit takes exactly the planned time | Arrives within a step or two of the planned time, if the plan is at least its `shortestTransit(path)`; otherwise it takes `shortestTransit`. The RHC policies plan with it |
| Model of | The event-driven RHC papers: first-order agent (`"FO-0"` profile) or energy-optimal second-order agent (`"SO"`, `"FO-3"`) | A physical robot executing the same plan with feedback |
| `J_e` | Tangential acceleration only (NaN with `"FO-0"`: impulses) | Full commanded acceleration |

**Speed profiles** (`SpeedProfile.m`, from the agent's `profile` or handed over per transit by a planning policy). The names are those of the energy-aware paper (Welikala and Cassandras, CDC 2021), whose FO-n profiles (n = 0 to 3) make a second-order agent behave approximately like a first-order one:
- `"FO-0"`: constant speed (the first-order agent of the Automatica paper).
- `"SO"`: minimum energy.
- `"FO-3"`: minimum-energy trapezoid.

The older names `"constant"` (and `"constant speed"`) and `"FO"` still work: they are FO-0 and FO-3. Without a planned transit time an Abstract Agent crosses an edge of length `y` in `y/maxSpeed` (`"FO-0"`) or `1.5 y/maxSpeed` (`"SO"`, `"FO-3"`, peak speed `maxSpeed`). On curved paths its profile depends on the path length only; the heading follows the path and turns at `omega = kappa * v` (`state.omega`), as in Theorem 2 of the energy-aware paper.

A Physical Agent follows the planned profile when that profile keeps within its reference limits on the path (`"SO"` and `"FO-3"` usually do on straight edges). Otherwise, it follows its fastest profile within those limits (full acceleration and braking, slower through corners, where the turn takes acceleration too), slowed down uniformly to the planned time. With `"SO"` or `"FO-3"` it takes whichever of the two draws less `int |a|^2 dt` on the path: those shapes minimize energy on a straight line, but on a curvy path their higher mid-edge speed can cost more in the turns. `"FO-0"` never fits, since it jumps to its speed, so with `"FO-0"` a Physical Agent moves at its own fastest pace, `shortestTransit(path)`. Its feedforward is the reference's own change of velocity over each step, so an agent on the reference stays on it.

**Spawning and customizing.** Agents are spawned from an `AgentSpec` (`Components/AgentTypes/AgentSpec.m`):
- the type (a registered name);
- `maxSpeed`, `maxAccel`;
- `profile`;
- the tracking gains `kp` and `kd` (Physical Agents);
- `batteryWh`.

Unset settings keep the type's defaults. One spec serves every place that creates agents:
- **In the GUI:** **Agent Type**, **Agent Speed**, **Agent Accel** and **Speed Profile** form `ScenarioModel.agentSpec`, applied to the next agents placed.
- **Layouts:** a layout stores each agent's speed, acceleration and profile. The type is always the current selection.
- **In `runTrial`:**
  - `AgentType` and `AgentSpeed` set the type and speed.
  - `Agent_<setting>` sets a spec setting (`'Agent_profile', "SO"`, `'Agent_maxAccel', 10`, `'Agent_kp', 16`, `'Agent_batteryWh', 50`, `'Agent_motorC2', 0.05`, `'Agent_maxNormalAccel', 2`, `'Agent_batteryR0', 0.1`).
  - Any other `Agent_<property>` is set on every agent, its state or controller (e.g. `'Agent_stopWhenEmpty', true`).
- **In code:** `spec = AgentSpec(Type="Physical Agent", MaxSpeed=10, Profile="SO"); agent = spec.spawn(index, position)`.

**Parts held by every agent, swappable without subclassing:**
- **Speed profile** (`SpeedProfile`): chosen per transit by a planning policy, or the agent's own `profile`.
- **Controller** (`TrackingController`): PD tracking with acceleration feedforward. Physical Agents use it; replace `agent.controller` to try another law.
- **Battery** (`Battery`): capacity and power model, `agent.battery`. Reported as SOC; with `stopWhenEmpty` the agent stops when it is empty. See [Full power model](#full-power-model-energy-aware-paper) for `c2` and battery losses.

New agent types are subclasses for different platforms (dynamics, limits, sensing), not for profiles; see [New Agent Type](#3-new-agent-type).

**Service.** A target is serviced only by agents at rest there: arrived (no path) or waiting to leave. Agents in transit service nothing, even when they pass close to a target. `ScenarioModel.detectionRadius` (1.2) is only the tolerance for "at the target".

### Full power model (energy-aware paper)

The energy-aware paper (Batch 1 of its simulator specification) uses the DC-motor power model `P = P_m + c1 u^2 + c2 v^2 + c3 v` (Tokekar, Karnad and Isler), with `P_m = P_I` (idle) or `P_M` (moving). The battery's model above is the case `c1 = gammaAcc/eta`, `c3 = alphaVel/eta` and `c2 = 0`. Setting `agent.battery.c2 > 0` (or `AgentSpec(MotorC2=...)`, `'Agent_motorC2'`, layout `agentMotorC2`) switches every agent to the full model. The defaults (`c2 = 0`, no turn limit, no battery losses) reproduce every earlier result bit for bit.
- **Least-loss profiles** (`SpeedProfile.shape(..., nu)` with `nu = sqrt(c2/c1)`). The agent minimizes the motion loss `L = int (c1 u^2 + c2 v^2) dt`.
  - SO without active limits is hyperbolic: `v(t) = A (cosh th - cosh(nu (t - rho/2)))` with `th = nu rho/2`, and `L = c1 nu^3 y^2 / (2 (th - tanh th))` (Theorem 1). As `c2 -> 0` it tends to the cubic.
  - Under a speed limit, hyperbolic arcs reach the limit with `u -> 0`, then the agent cruises (Theorem 2). Under an acceleration limit, `u` stays at the limit at both ends around a hyperbolic middle arc. With both limits, the two combine. Each case keeps `u` continuous.
  - FO-3 accelerates for the root `tau*` in `(0, rho/3]` of `c2 tau^2 (2 tau - rho) = 3 c1 (3 tau - rho)`, clamped to what the limits allow (Theorem 3).
  - `SpeedProfile.loss(name, y, rho, vb, ub, c1, c2)` gives `L` (vectorized in `rho`), and `SpeedProfile.integrals(P)` gives `int u^2` and `int v^2` of any profile.
- **Turn limit** (Assumption A1). `agent.maxNormalAccel` (`a_n`, `Inf` by default) caps a path's speed at `min(maxSpeed, sqrt(a_n / kappa))`, with `kappa` the path's largest curvature (`agent.segmentSpeedLimit(pts)`). `agent.limitTransit(pts)` is the shortest transit within the agent's limits, `y/v + v/u` (Lemma 1).
- **Energy of a plan.** `[E, Eex] = agent.transitPlanEnergy(pts, rho, name)` gives the battery energy `E = P_M rho + c3 y + L` of the least-loss profile within the agent's limits (`agent.planProfile`), and its excess `Eex = E - P_I rho` over idling, the only part a plan can change. `Battery.planEnergy(P)`, `idleEnergy` and `excessEnergy` work on any profile.
- **Battery losses.** With `battery.R0 > 0`, the battery delivers `P + beta P^2` with `beta = R0/Voc^2` (first order in the internal resistance). Both `drain` and the plan energies include it, the latter by quadrature along the profile.
- **Abstract Agents** follow these profiles exactly, so they draw what is predicted (to 0.001% in `energyModelTest`). Physical Agents are unchanged; the energy paper's differential-drive agent is planned.
- ED-ORHC-SO and ED-ORHC-FO-3 keep the CDC 2021 objective `int u^2`, as validated against the JS simulator.
- **Stations with docks** (Sec. 6.2). `ChargingStation.docks` (`m_sigma`, `Inf` by default: every agent at rest charges) limits the agents charging at once; an agent at rest takes a free dock first come, first served and holds it until it leaves. `station.slotLength(agent, socMin)` is the slot `t_s = (Q - E_min)/(P_c - P_I)`, a full charge from the reserve. `model.addStation(pos, power, docks)`; layout field `chargerDocks`.
- **Route tables** (Sec. 6.1, Lemma 2). `R = RouteTable(model, agent)` gives, for every node and station, the least excess route energy `R.D`, the route time `R.T` and the next node `R.next` (`R.route(k, station)` lists the nodes). An edge weighs its least excess energy `min E^ex(rho)` over `rho >= limitTransit`, at the energy-optimal transit time `R.edgeTime` (Proposition 2). The tables come from a synchronous distributed Bellman-Ford, whose cost is in `R.rounds` and `R.messages`.
- **Home stations.** `[home, util, ok] = R.homeStations(model, starts, agent, Pbar)` balances the stations' utilization `n_sigma Pbar/(m_sigma (P_c - P_I))` and then matches agents to stations for the least total route energy from their start nodes; `ok` reports Lemma 4's condition `P_c >= ceil(n_sigma/m_sigma) P_I`.

**Older names.** `"Linear Agent"` (= Abstract, `"FO-0"`), `"Second-Order Agent"` (= Abstract, `"SO"`), `"Default Agent"` and `"Energy Agent"` (= Physical) still work in scripts and layouts, but are not listed in the GUI.

**Policies and agent types.** Each policy states what it assumes of the agents' motion: `agentAssumption` on the policy, or `ComponentRegistry.assumptions` for policies that do not declare it.
- `"any"`: no timing assumption.
- `"plans transit"`: hands each transit's time (and profile) to the agent, no shorter than the agent's `shortestTransit(path)` (0 for an Abstract Agent).
- `"FO-0"`: transit = path length / `maxSpeed` (constant speed), not handed over.

`ScenarioModel.checkAgentFit` turns this into a note in the GUI status line and a warning in `runTrial`.
- ✓ = works as designed.
- ≈ = runs, but the agent's transit times differ from what the policy plans.

| Policy | Abstract | Physical | Notes |
|---|---|---|---|
| Random Walk | ✓ | ✓ | No timing assumptions; dwell from the current `R` at arrival. Each agent moves at its own pace (a Physical Agent: `shortestTransit`) |
| Battery-Aware ED-RHC | ✓ | ✓ | As ED-RHC; predicts each plan's battery energy with the agent's own `transitEnergy` |
| ED-RHC | ✓ | ✓ | Hands the agent its planned transit: `y/maxSpeed`, or the agent's `shortestTransit` if longer (a Physical Agent's own pace). `"FO-0"` is the Automatica model; `"SO"` runs it with the SO shape (the JS SO#) |
| Collab-ED-RHC | ✓ | ✓ | As ED-RHC (hands over `y/maxSpeed`, or the agent's `shortestTransit` if longer); several agents may share a target |
| ED-ORHC-SO / -FO-3 | ✓ | ✓ | Hands over the optimal transit time (no shorter than the agent's `shortestTransit`), profile and limits. A Physical Agent keeps the time; its profile is the planned one where that fits its limits, so its energy can differ from the plan's |
| Classical RHC [1], Divergent RHC, Joint RHC, Joint RHC N | ✓ with `"FO-0"`; ≈ with `"SO"`/`"FO-3"` (1.5× slower than planned) | ≈ | Plan transits as path length / `maxSpeed`, which a Physical Agent cannot keep (it takes its `shortestTransit`). Joint RHC needs exactly 2 agents |

## Random Walk

`Random Walk` (`Components/AgentPolicy/RandomWalkPolicy.m`) is the baseline policy. It uses the same event-driven structure as ED-RHC, but an agent acts only at two of its own events:
- **On arrival**, it dwells for a uniform random time between 0.8 and 1.2 times the time to empty its target, `R_i / (N_i B_i - A_i)`. Here `N_i` counts the agents already there, which are assumed to stay; they keep their own plans.
- **When the dwell ends**, it moves to a neighbor chosen uniformly at random.

Agents may share targets. During a spike that the agents present cannot overcome, the target's baseline growth rate is used instead. The parameters are `dwellSpread` (default 0.2) and `dwellScale` (default 1; 0 means no dwell).

## Event-driven receding horizon control (ED-RHC)

`ED-RHC` (`Components/AgentPolicy/EDRHCPolicy.m`) implements the distributed event-driven receding horizon controller of S. Welikala and C. G. Cassandras, "Event-driven receding horizon control for distributed persistent monitoring in network systems", *Automatica* 127 (2021) 109519.

- **Local and event-driven.** Each agent decides on its own, using only its target `i` and the 1-hop neighbors of `i`: their `R`, `A`, `B`, the edges `(i, j)`, and whether each neighbor is covered.
  - It solves RHCP1 on arrival (active time `u_i`), RHCP2 when `R_i` reaches 0 (inactive time `v_i`), and RHCP3 when it leaves (next target `j`).
  - It re-solves RHCP1/RHCP2 when a neighbor becomes covered or uncovered.
- **Agents avoid each other.** A target is covered while another agent is at it or on its way to it. Covered neighbors are left out, so two agents never share a target.
- **Variable horizon.** Each RHCP plans `[u_i, v_i]` at `i`, the transit to `j`, and `[u_j, v_j]` at `j` over a horizon `w <= H` (`H = 250` by default). It minimizes the neighborhood cost `(1/w) * sum of the integrals of R`.
- **alpha** (a parameter). RHCP3 weights the next target's cost by `alpha` and the others' by `1 - alpha`.
  - The default `"nominal"` is the paper's `alpha = 1/|N_i|^2` (Sec. IV-A, the RHC-alpha method; `N_i` includes `i`), as in the JS simulator.
  - `alpha = 0.5` gives the paper's unweighted RHC method (Sec. III).
  - Set it with `'Policy_alpha'` in `runTrial`.
- **Solver.** The RHCPs are solved numerically (grid search plus `fmincon` on each smooth case) instead of with the paper's closed forms.
- **Agents.** The method assumes first-order agents moving at constant speed: use the **Abstract Agent** with the `"FO-0"` profile (the default), whose transit times equal path length / speed. See the compatibility table under [Agent types](#agent-types) for the others.

`Experiments/validation/validateEDRHC` compares `alpha = 0.5` and the nominal alpha with the legacy JavaScript simulator on its presets (`Configurations/Legacy/`).

### Energy-aware ED-RHC (ED-ORHC-SO, ED-ORHC-FO-3)

`ED-ORHC-SO` and `ED-ORHC-FO-3` (`EDRHCPolicy` with `transit = "SO"` or `"FO-3"`; the older name `ED-ORHC-FO` still works) implement the energy-aware variant of S. Welikala and C. G. Cassandras, "Event-driven receding horizon control of energy-aware dynamic agents for distributed persistent monitoring", arXiv 2102.12963 (IEEE CDC 2021). Use them with the **Abstract Agent**; the policy hands each transit's profile, time and limits to the agent, whatever its own profile.

- **Objective.** `J_T = energyWeight * J_e + J_s`. Here `J_e` is the integral of the squared tangential acceleration `u(t)^2` over all agents, and `J_s` is the usual sensing objective.
  - The default `energyWeight = 7.68e-3` is the paper's `213.3e-6` (for a 600 x 600 space) rescaled to the 100 x 100 canvas. Energy grows with length squared, so the factor is 36.
  - `EDRHCPolicy.weightFromBeta(beta, model)` gives the paper's App. A rule, eq. (77): `beta/(1 - beta) * ybar^2 / vmax^4`, where `ybar` is the mean edge length and `vmax` the agents' `maxSpeed`. With `"accel"` as a third argument it gives `beta/(1 - beta) / umax^2`.
  - In `runTrial`, `'EnergyBeta', beta` (optionally with `'EnergyBound', "accel"`) applies this rule.
- **Transit time as a decision.** Events, locality, covering and alpha are as in ED-RHC. In addition, each RHCP chooses the transit time `rho_ij` and adds the energy `E(rho)` of the speed profile on that edge. In RHCP3 the alpha-weighted sensing cost is doubled, so that `alpha = 0.5` gives the paper's objective.
- **Speed and acceleration limits** (parameters `speedLimit` and `accelLimit`, `Inf` by default; `'Policy_speedLimit'` and `'Policy_accelLimit'` in `runTrial`). The agent uses the minimum-energy profile of the method's family for the planned `y` and `rho`, rest to rest:
  - **SO** (any profile). Without limits, `v(t) = 6 y t (rho - t)/rho^3` with `E = 12 y^2/rho^3` (Theorem 3).
    - Under a speed limit, `u` falls linearly to 0 as the speed reaches the limit, then the agent cruises.
    - Under an acceleration limit, `u` is held at the limit, then falls linearly (the paper's SO-A).
    - Under both, it accelerates at the limit, ramps `u` down to 0 as it reaches the speed limit, cruises, and mirrors this at the end.
  - **FO-3** (trapezoids). Without limits, the trapezoid accelerates for `rho/3`, cruises at `3y/(2 rho)`, brakes for `rho/3`, and has `E = 27 y^2/(2 rho^3)` (Theorem 5). With limits, the cruise speed is clamped to what they allow (the paper's FO-V and FO-A).
  - The RHCP never plans a transit shorter than the limits allow: `y/v + v/u`, or `2 sqrt(y/u)` on short edges.
  - These profiles generalize the paper's Apps. D-E to both limits at once. The speed-limited SO keeps `u` continuous, which uses 2-6% less energy than the paper's SO-V profile.
- **Agents.** The Abstract Agent follows each planned transit exactly and accumulates `J_e` exactly. Under `ED-RHC` it runs each fixed transit `y/maxSpeed` with its own profile (`"SO"`: the JS simulator's SO#). The Physical Agent tracks the same plans with feedback.
- **The energy each agent draws.** The RHCP's `E(rho)` comes from the agent (`Agent.transitEnergyModel`):
  - For the Abstract Agent it is the closed form above (results unchanged).
  - For the Physical Agent it is the `int |a|^2 dt` of the reference that agent will follow. That reference is the fastest profile retimed to `rho`, or, where the planned SO/FO-3 profile fits the agent's limits on the path, whichever of the two draws less. The energy includes the turns' `v^2 kappa` on curved paths, and `E(rho)` falls with `rho` (monotone), so the RHCP's search meets no jump.
  - The model is tabulated on a few transit times per edge, interpolated in log-log coordinates, and cached by the policy.
  - On a curved edge of the lab (8-2), the closed form misses the drawn energy by about 85%, while the agent's model is within 0.2% (0.4% on a nearly straight edge).
  - Over 500 s on the lab (5 Physical Agents), planned and drawn `J_e` agree within 1% (closed form: 18% mean error for SO, 13% for FO-3).
  - The RHCPs now see what turns cost. Compared with the closed form, the energy drawn falls by 30% (SO) and 25% (FO-3), `J` rises by 5.0% and 2.9%, and `runTrial`'s `J_T` falls by 28% and 23%.
- **Metrics.** `runTrial` also returns `out.Je`, `out.JT`, `out.vmax` and `out.umax` (paper eqs. 3 and 73), and `out.log.Je` holds the per-step energy.
- **`clearFirst`** (default `false`, which follows the paper). Set it to `true` for the JS simulator's energy-aware convention: every visit clears the target, there is no RHCP1 on arrival, and plans only include visits that clear `j`.

`Experiments/validation/validateEDORHC` compares both methods (without limits) with the JS simulator (methods 9 and 11) under the JS conventions.

### Battery-aware ED-RHC and charging stations

`Battery-Aware ED-RHC` (`Components/AgentPolicy/BatteryAwareRHCPolicy.m`, a subclass of `EDRHCPolicy`) keeps every agent able to reach a charging station. It replaces Gabriel's `Battery Efficient` heuristic (nearest-neighbor moves with short random dwells, longer at low SOC, never recharging), retired on 2026-10-21; its code is kept in `Old_Design/` for reference, and old scripts that ask for it get an error naming the replacements.

- **Charging stations** (`Components/ChargingStation.m`, a `Target` subclass). A station has no uncertainty (`R = A = 0`) and is drawn as a yellow square. Every agent at rest there recharges at `chargePower` (W) until full; the agent still draws its idle power. Layouts place stations by target index (fields `chargers`, `chargePower`); in the GUI, **Add Station** places one with the power typed next to it (`ScenarioModel.addStation(pos, power)` in code), and Add Edge connects it like a target. Other policies treat a station as an ordinary node.
- **Energy of a plan.** Each agent predicts its battery energy from the battery's power model (`Battery`):
  - at rest: `idlePower` per second;
  - a transit of length `y` in time `rho`: `Agent.transitEnergy(path, rho)` = `movePower * rho + (alphaVel * y + gammaAcc * int |a|^2 dt) / eta`. The speed term integrates to `alphaVel * y` for any speed profile. A Physical Agent computes `int |a|^2 dt` from the reference it will follow.
  - The prediction matches the energy drawn to 0.1% on a wall-routed edge (smoke test).
- **Terminal SOC constraint.** Every RHCP plan `[u_i v_i rho_ij u_j v_j]` must end, at `j`, with enough energy for three things: the cheapest route from `j` to the nearest station, the wait there while every other agent using that station charges fully once (the worst case, when they all arrive together), and a reserve `socMin` (5%) of the capacity:
  `idlePower * (u_i + v_i + u_j + v_j) + E_ij <= E_left - socMin * capacity - E_station(j)`.
  The constraint is linear in the plan, so it is added as one more row to the constraints of each RHCP case (`EDRHCPolicy.planConstraint`). It caps the dwells of RHCP1 and RHCP2, and RHCP3 drops the neighbors the agent cannot afford. Stations are never planned as monitoring visits.
- **Recharging.** When the agent cannot afford any uncovered neighbor, even without dwelling (or cannot afford to wait while every neighbor is covered), it goes to the nearest station along its cheapest route, one edge per decision. It waits (`queueWait`, 1 s) while another agent is at or heading to the station, charges until full, and then resumes ED-RHC with RHCP3. Agents on their way to a station do not block the targets they pass, so an agent leaving a station can always get out.
- **Parameters.** `socMin` (0.05), `queueWait` (1 s), and everything of ED-RHC (`alpha`, `H`, `transit`, ...). `charges` counts the charging sessions; each station records `energyDelivered` and `busyTime`.
- **Metric.** `Minimum SOC` (`MinimumSOC`) is the lowest SOC of any agent so far. Use it with `setMetrics` or `runTrial`'s `Metrics` parameter.
- **Without stations** the policy is ED-RHC. With unlimited batteries on the dock scenario it never charges, and its J is 2% above ED-RHC's (140.6 vs 137.6): it never plans a visit to the station, and its RHCPs carry the extra, inactive constraint.

On **Office floor with charging dock** (1500 s, seed 1, the layout's growth rates; measured at d8dcd23). The paper's Table VI sets `A = 1` on every target, so its `J` values are lower:

| Policy | Agent | J | Lowest SOC | Charges | Batteries empty |
|---|---|---|---|---|---|
| ED-RHC, unlimited batteries | Abstract | 137.6 | 100% | 0 | none |
| Battery-Aware ED-RHC | Abstract | 255.2 | 9.9% | 19 (dock busy 32% of the time) | none |
| ED-RHC | Abstract | 8786 | 0 | 0 | all three, at about 225 s |
| Battery Efficient (retired; measured at ac496ba) | Abstract | 10233 | 0 | 0 | two, at about 243 s |
| Random Walk | Abstract | 9093 | 0 | 0 | all three, at 214-230 s |
| ED-RHC, unlimited batteries | Physical | 238.0 | 100% | 0 | none |
| Battery-Aware ED-RHC | Physical | 616.0 | 15.5% | 32 (dock busy 54% of the time) | none |
| ED-RHC | Physical | 10645 | 0 | 0 | all three, at 96-107 s |

A slower station does not empty the batteries, since the reserve covers the worst-case wait, but it limits monitoring. Here the three robots draw about 40 W each and the dock charges at 300 W. At 100 W, no battery runs out either, but the dock is busy 85% (Abstract) and 96% (Physical) of the time, and `J` rises to 623 and 6581.

### Energy-aware ED-RHC with charging coordination (Energy-Aware ED-RHC)

`Energy-Aware ED-RHC` (`Components/AgentPolicy/EnergyAwareEDRHCPolicy.m`, a subclass of `EDRHCPolicy`) is the method of the energy-aware paper: ED-RHC with the [full power model](#full-power-model-energy-aware-paper), terminal values, and booked charging slots. It keeps ED-RHC's events and ordering cases. Batch 3a of the paper's simulator specification; the station scheduler (reported costs, reassignments, offers) follows.
- **Objective.** For a plan of length `w` with transit time `rho`, `N = J + omega E^ex(rho) + dhhat`: the neighborhood's uncertainty integral, the transit's excess energy over idling (`Agent.transitPlanEnergy`, with battery losses), and the terminal correction.
  - `terminal = "hhat"` (default) adds the targets' relative values `hhat_k(X(t_f)) - hhat_k(X(t))` (`CollabTargetCost.terminal`, as in Collab-ED-RHC); `"alpha"` uses ED-RHC-alpha's weights instead; `"none"` drops them.
  - All three keep the energy value `hhat_E`, which adds `mu_a (E^ex + D_j - D_i)/(P_c - P_I + Pbar_a)`: energy is priced by the charging time it costs (Proposition 3), with `mu_a >= 0` the targets' marginal cost of delay and `Pbar_a` the agent's mean power since its last charge (`priceForm = "renewal"`, the renewal model's slope; `"charging"` drops `Pbar_a`). `priceScale` scales the price and `priceAveraging = true` uses the running mean of `mu_a` over the agent's events.
  - `objectiveForm = "dinkelbach"` (default) minimizes `N - lambda w` with Dinkelbach's iteration on `lambda`; `"ratio"` minimizes `N/w` directly (the same plans; a solver check); `"alphaH"` minimizes `(J + dhhat_targets)/w + alphaH E^ex`. `omega = 0` by default.
  - Without stations, with `terminal = "none"`, `omega = 0` and the ratio form, it makes ED-RHC's decisions (alpha 0.5).
- **Bookings.** Every agent holds a booking: its home station (`RouteTable.homeStations`), a dock, and a slot start `t_a` on the station's grid `k t_s`. A plan to `j` must keep `t_f + T(j, sigma) + delta <= t_a` (a row on the plan length) and `E^ex(rho) + D(j, sigma) + P_I (t_a - t) <= E - E_min` (an interval of `rho`, from `E^ex` tabulated per edge), with `D` and `T` from the route tables and `P_I` with the battery's losses. With a mission end `T`, `t_f <= T`.
  - `bookingStrategy = "a"` (default) books the free slot nearest the predicted need (energy at the mean power since the last charge); `"b"` the latest free slot in the agent's window.
- **Charging.** When it has uncovered neighbors but none is affordable, the agent leaves now if it can reach an earlier free slot (and moves its booking there), and otherwise keeps servicing its target until its latest departure. Then, or at its latest departure, it takes the least-energy route to its station without covering targets, waits there without charging until its slot (`Agent.chargeAllowed`), charges until full or the end of its slot, and books its next slot. With `earlyStart = true` (default) the docks are work-conserving: a booked agent at its station starts as soon as its dock is idle and no other booking on it comes first, and holds the dock through its slot. With `pullForward = true` (default) a station offers time freed on a dock (a session that ended full before its slot's end, or a released booking of the current slot) to the first booked agent in line (one that asked for an earlier slot, then the earliest booking) that is at most `1 - pullMinCharge` full (default half), can reach the dock in that time and can use it until its slot; the agent accepts at its next event, claims the dock and comes now. Once its energy covers its mean power until `T`, it releases its booking.
- **Records.** `charges`, `sessions` (agent, station, dock, slot start and end, arrival, start, stop, energy before and after), `modeTime` (seconds monitoring, on the way, waiting and charging per agent: the monitoring share), `windowMisses` (bookings outside the window, none expected), and `stats` (RHCPs solved, how often the energy or a time row binds at the optimum, "no affordable neighbor" events, planned transits `[rho, rho_min]`, and why an RHCP3 found no next target). `traceEvents = true` records every decision in `trace`.
- **Setups.** Run it on the `"Energy: ..."` scenarios with `runTrial(..., 'AgentSettings', "layout", 'TargetRates', "layout")`, so the layout's robots and rates are used as in the GUI.

### Collaborative ED-RHC (Collab-ED-RHC)

`Collab-ED-RHC` (`Components/AgentPolicy/CollabEDRHCPolicy.m`, target costs in `CollabTargetCost.m`) is the controller of the collaboration paper (W. Getts and S. Welikala, "Collaborative persistent monitoring", in preparation). It generalizes ED-RHC:

- **Sharing.** Agents may share targets (no covering rule). Every target cost follows the Lindley recursion `R(s_{k+1}) = max(0, R(s_k) + sigma_k * Delta_k)`, which holds for any number of agents at a target, any order of arrivals and departures, and `A >= B` (targets only a group can clear).
- **Announced plans.** Every agent announces its planned departure time and tentative next target. The policy keeps them per target as an occupancy schedule, so other agents' visits are predicted from their plans instead of the projections `dR/dt = A` of ED-RHC.
- **Coordination, without negotiation.** At an event at target `i`, the agents at `i` plan jointly (their departure times, next targets and the common horizon end `tf`). With `coordination = "B"` (default), the same solve may also shift the departure times of the agents at neighboring targets, provided no such agent's 1-hop score gets worse than without any shift (individual rationality). The neighbors adopt the new times without replying. `coordination = "A"` plans only the co-located group, on the 1-hop neighborhood.
- **Events.** Arrival (E1), deactivation `R_i -> 0` (E2), departure (E3), a departure from or towards a neighbor (E4), and the start (E5). RHCP-R is solved at E1, E2, E4 and E5, RHCP-D (leave now) at E3.
- **Terminal correction** (`terminal = "hhat"`, default). The objective is the corrected ratio `[J + hhat(X(tf)) - hhat(X(t))] / (tf - t)`. `hhat` sums each target's relative value under a revisit model: the target waits `delta_k`, then its minimum clearing group clears it; the cost until then minus `A_k * gap_k / 2` times the time until then. `delta_k` is the time to the earliest announced arrival (`deltaRule = "plans"`), else the target's mean unattended time `gap_k`. It is truncated at the mission end (`missionEnd`, set by `runTrial` to `EndTime`). `terminal = "none"` gives ED-RHC's ratio; with one agent the policy then reproduces ED-RHC with `alpha = 0.5`.
- **Solver.** The next targets of the group are enumerated. For each assignment, the departure times and `tf` are searched on a dense grid (or sampled for a group of 2 or more), refined by coordinate-wise line searches, and polished with `fmincon`. The shifts of mode B are optimized from the no-shift plan with the IR constraints, and adopted only if they lower the objective.
- **Parameters.** `coordination`, `terminal`, `deltaRule`, `sharing` (false: no sharing, covered next targets excluded), `H` (250), `gridSize`, `traceDecisions` (`decisions`: one row per solve). `stats` counts solves, events by kind, solve CPU time and shifts.

`tests/collabPolicyTest.m` checks the reduction to ED-RHC with one agent, sharing on a target only two agents can clear, and the event counts; `tests/collabCostTest.m` checks the recursion and the terminal correction against a fine time integration.

**Collaboration paper results.** `CollabResults.mlx` (repository root; built from `Experiments/collab/collabResultsSource.m` by `buildCollabResults`) regenerates every figure and result of the collaboration paper: the target-class, neighborhood and setup figures (`collabFigures`), the 99 simulations (`collabExperiments`: the eight JS presets with four methods and three ablations, two configurations with targets that only two agents can clear, two to six agents on the General preset, and the Lab floor, Office floor, City patrol, Harbor patrol and Campus maps with four methods), the tables, plots and the numbers quoted in the text (`collab_macros.tex`). Saved runs (`results/collab/runs/`) are not run again; the runs use parallel workers when the Parallel Computing Toolbox is available (about 45 minutes on a 10-core computer) and give the same results serially. Run it in the Live Editor, or headless:

    matlab -batch "addpath(genpath(pwd)); exportCollabResults"              % report: results/collab/CollabResults.pdf
    matlab -batch "addpath(genpath(pwd)); exportCollabResults(Quick=true)"  % 60 s runs, a few minutes

Every setup is a layout file the GUI lists in its Scenario dropdown: the JS presets (`JS: ...`) and the collaboration setups (`Collab: ...`, `Configurations/Collab/`, built by `Configurations/makeCollabLayouts.m`). The collaboration layouts store per-target `A`, `B` and `R0`; `runTrial` uses them with `'TargetRates', "layout"`. The paper's runs use Collab-ED-RHC with its defaults (mode B, `terminal = "hhat"`, `H = 250`), T = 500 s and dt = 0.02 s; select **Collab-ED-RHC** in the Agent Policy dropdown to run them in the GUI.

## Headless experiments

`Experiments/runTrial(params)` runs one simulation without the GUI. Unset parameters take the values in `Experiments/trialDefaults.m`; component parameters take any `ComponentRegistry` name:

```matlab
out = runTrial(struct('Layout', "ReportLayout.mat", 'AgentStarts', "layout", ...
    'Policy', "ED-RHC", 'Planner', "Energy Efficient", 'AgentType', "Physical Agent", ...
    'UncertaintyMode', "Static Uniform", 'A_base', 1, 'B', 10, 'EndTime', 500, 'Seed', 1, 'Log', true));
```

- `AgentStarts`: `"auto"` (spread evenly), `"layout"` (the start targets stored in the layout file), or target indices such as `[1 5]`.
- `Seed`: the `rng` seed set before simulating (`-1`, the default, leaves `rng` as it is).
- `Policy_<property>`: sets a property of the active policy, e.g. `'Policy_dwellScale', 0` turns off the Random Walk dwell and `'Policy_alpha', 0.5` sets ED-RHC's alpha.
- `TargetRates`: `"params"` (default) gives every target `A_base` and `B`; `"layout"` keeps the layout file's per-target `A`, `B` and `R0`, e.g. the `Configurations/Collab/` setups with targets that only a group of agents can clear (`A_i > B_i`).
- `AgentSettings`: `"params"` (default) places new agents from `AgentType`, `AgentSpeed` and the `Agent_<setting>` parameters; `"layout"` keeps the layout's agents with their stored settings (speed, limits, profile, battery and power model), e.g. the `Configurations/Energy/` setups, and only `Agent_<setting>` parameters given explicitly change them.
- `InitialR`: `R_i(0)` for every target, as a vector, or a number `Rmax` that draws each from `U[0, Rmax]` with its own random stream seeded by `Seed` (the same draws in serial and in `parfor`).
- `out.model` is the scenario at the end of the run (agents, targets, stations).
- `Log = true`: `out.log` holds the per-step time, `J(t)`, agent positions, velocities, accelerations, headings, SOC, battery energy and `J_e`; `out.metrics` holds per-agent path length `L`, heading change `Θ`, integrated squared acceleration `E_a`, final SOC and energy (`Experiments/metrics/`).

`Experiments/runSweep(grid)` runs every combination of parameter values, and `analyzeResults` compares the variants.

## Paper results

`PaperResults.mlx` (repository root) regenerates every result of the simulator paper: the validation tables against the JS simulator (ED-RHC and ED-ORHC on its eight presets), the four case studies (policies × agent types, path planners, the energy trade-off of ED-ORHC-SO over β, and Battery-Aware ED-RHC on the charging-dock office), their tables, plots and the numbers quoted in the text (`macros.tex`), the GUI figures (overview, scenario library, planner routes), and the getting-started figures (it runs `Examples/EventDetection/eventDetectionExample.m`). Open it and click **Run All**, or run it without the Live Editor to get a PDF report of everything:
```
matlab -batch "addpath(genpath(pwd)); exportPaperResults"
matlab -batch "addpath(genpath(pwd)); exportPaperResults(Quick=true)"
```
The full run takes about an hour and writes to `results/paper/`; when the paper's folder sits next to this repository (`../Paper_PerMonSimulator`), the tables, macros and figures are also copied into its `generated/` folder, which the paper includes. `Quick=true` (60 s runs, at most two seeds, one validation problem, a few minutes) writes to `results/paper_quick/` only.

The experiments are defined in `Experiments/paper/paperDefaults.m` (the reference scenario) and `Experiments/paper/paperConditions.m` (what each experiment varies). The live script is binary, so it is built from a plain-text source: edit `Experiments/paper/paperResultsSource.m` and run `buildPaperResults`.

## Path planners

Every edge's path is planned once, when the edge is added (and again when walls or the planner change). Agents follow it; the policies use its length.

| Planner | Method |
|---|---|
| Dijkstra Corners | Shortest path over the wall end points pushed out by `cornerBuffer` = 1.2 (a visibility graph) |
| Energy Efficient | As Dijkstra Corners, with a penalty on heading changes and corners pushed further out (`cornerBuffer` = 2.5) |
| Grid A* | A* on an 8-connected occupancy grid (cell 0.5; cells closer than `clearance` = 1.2 to a wall are blocked), octile heuristic, then line-of-sight shortcuts; the standard grid-based baseline |

Every planner then rounds the corners of its route (`PathSmoother.fillet`): each corner becomes a circular arc tangent to both segments, as large as the segments allow (radius at most `filletRadius` = 10), shrunk until it keeps `filletClearance` (0.6; 1.0 for Energy Efficient) from every wall. The heading is continuous and the curvature bounded, so Physical Agents keep their speed through turns: with the full acceleration limit, their transits on the maps take 14–19% less time than with the earlier pchip-smoothed paths and the 80% margin. When no wall-free route exists, a planner returns the straight line and sets `noRoute`; the GUI says so when the edge is added.

**Walls during motion.** Paths avoid the walls, but an agent can still leave its path (for example, a user-defined agent). After every step the model checks whether an agent's motion crossed a wall; if it did, the agent stops just before the wall and the collision is counted (`ScenarioModel.collisions`, `runTrial` output `collisions`).

## Getting started: a new monitoring problem

`Examples/EventDetection/` builds a different persistent monitoring problem from scratch: security robots patrol a small museum, incidents occur at random at the targets and expire, and a robot at a target detects the incidents in progress. Four small classes define it, one per extension point:

| Class | Base class | What it does |
|---|---|---|
| `SimpleTarget` | `Target` | Events arrive at random (rate `eventRate`) and last `lifetime` s; an agent at the target detects those in progress; `R` = events in progress |
| `SimpleAgent` | `Agent` | Moves at its top speed from path point to path point |
| `SimplePolicy` | `Policy` | Goes to the free neighbor with the most events in progress (else the one visited longest ago) |
| `SimpleMetric` | `Metric` | Share of the events that ended before an agent detected them |

Each is one line in `ComponentRegistry`, so it appears in the GUI's dropdowns (Target Type, Agent Type, Active Policy, Objective) and in `runTrial` (`TargetType`, `AgentType`, `Policy`, `Metrics`). `eventDetectionExample.m` walks through the whole process in seven sections, each doing what the named GUI clicks do: import the floor plan, draw the walls, mark the targets, add the edges (the planner routes them around the walls), choose the new classes and place the robots, run, and compare policies headless with `runTrial`. It saves what the canvas shows after each step to `results/example/`. On the museum, SimplePolicy misses about 13% of the events and Random Walk about 44% (10 runs of 600 s).

## Adding a new modular class

Each area has a dedicated folder under `Components/`. Creating a new algorithm takes two steps:

1. Create a new `.m` class file in the matching folder.
2. Register it in `Components/ComponentRegistry.m` — this makes it appear in the UI dropdown automatically.

### 1. New Path Planner

Folder: `Components/PathPlanner/`

Create a class that inherits from `PathPlanner` and implements `plan`:

```matlab
classdef MyPlanner < PathPlanner
    methods
        function path = plan(obj, pStart, pEnd, walls, bounds)
            % Return an Nx2 matrix of [x y] waypoints from pStart to pEnd.
            path = [pStart; pEnd];
        end
    end
end
```

Register it in `ComponentRegistry.m`:
```matlab
obj.pathPlanners('My Planner') = @() MyPlanner();
```

### 2. New Agent Policy

Folder: `Components/AgentPolicy/`

Subclass `Policy` and implement `plan`, which returns a move command struct:

```matlab
classdef MyPolicy < Policy
    methods
        function cmd = plan(obj, agent, model, adj, curIdx, simTime)
            cmd = struct();
            nbrs = find(adj(curIdx, :));
            if isempty(nbrs), return; end
            cmd.kind = "move";
            cmd.targetIdx = nbrs(1);          % index of the target to move to
            cmd.dwellSeconds = 0.5;           % optional: wait at the current target before leaving
        end
    end
end
```

Returning `struct()` (no `kind`) leaves the agent where it is; it asks again on the next step. Instead of `dwellSeconds`, a policy may set `agent.dwellRemaining` itself, as the RHC policies do. Optional: `onStep(agent, model, simTime)` (called every step, to react to events during a dwell), `resetReservations()` (called on reset), and a constant `agentAssumption` (see the agent types section).

Register it:
```matlab
obj.policies('My Policy') = @() MyPolicy();
```

### 3. New Agent Type

Folder: `Components/AgentTypes/`

Subclass `Agent` and implement `update(dt)`:

```matlab
classdef MyAgent < Agent
    methods
        function obj = MyAgent(index, position, maxSpeed)
            obj@Agent(index, position, maxSpeed);
        end

        function update(obj, dt)
            % Move obj.state along obj.path (N x 2 waypoints from the path planner) for dt
            % seconds; set obj.path = [] once the last waypoint is reached.
        end
    end
end
```

`Agent` holds what every agent shares:
- the state (`state`, a `KinematicState`), `path`, `dwellRemaining` and the start target;
- the speed `profile` and the transit plan a policy may hand over (`transitTime`, `transitProfile`, `transitLimits`);
- the battery (`battery`, a `Battery`; `soc`, `e_total`) and the metrics `Je`, `peakSpeed`, `peakAccel`;
- `resetToInitial`, `draw` and `updateVisuals`.

Helpers for path-following agents:
- `beginLeg()` gives the arc-length parametrized path and the speed profile of the transit starting now.
- `pointAt` and `headingAt` give position, tangent, heading and curvature along it.

Override `resetToInitial` (calling `resetToInitial@Agent(obj)` first) to reset extra state, or `label` to change the text drawn on the agent. `AbstractAgent` is the simplest complete example; `PhysicalAgent` adds feedback control with saturation. Agents of different types can be mixed in one scenario.

Register it:
```matlab
obj.agentTypes('My Agent') = @(idx, pos, sp) MyAgent(idx, pos, sp);
```

That's it — restart `runMainUI` and the new component will appear in its dropdown.

### 4. New Target Type

Folder: anywhere on the path (the example is in `Examples/EventDetection/`)

Subclass `Target` and override `updateUncertainty(dt, simTime, mode)`; `R` is what the target's bar shows and what `J(t)` averages. `residingAgents` lists the agents at rest at the target.

```matlab
classdef MyTarget < Target
    methods
        function obj = MyTarget(index, position)
            obj@Target(index, position);
        end

        function updateUncertainty(obj, dt, simTime, ~)
            if isempty(obj.residingAgents), obj.R = obj.R + dt; else, obj.R = 0; end   % idleness
        end
    end
end
```

Override `reset` (calling `reset@Target(obj)` first) to clear extra state. Register it:
```matlab
obj.targetTypes('My Target') = @(idx, pos) MyTarget(idx, pos);
```
The Target Type dropdown (or `model.setTargetType(name)`) rebuilds the scenario with targets of that type; layouts store it (`targetType`).

### 5. New Metric

Subclass `Metric` and implement `value(model, simTime)`; implement `update(model, simTime, dt)` too if the metric accumulates over the run (it is called after every step), and `reset(model)` to clear it.

```matlab
classdef PeakUncertainty < Metric
    properties
        peak = 0
    end
    methods
        function obj = PeakUncertainty(), obj.label = "Largest R so far"; end
        function reset(obj, ~), obj.peak = 0; end
        function update(obj, model, ~, ~), obj.peak = max([obj.peak, model.targets.R]); end
        function v = value(obj, ~, ~), v = obj.peak; end
    end
end
```

Register it:
```matlab
obj.metrics('Peak Uncertainty') = @() PeakUncertainty();
```
The Objective dropdown plots it over time; `runTrial` returns the final value of every metric named in `Metrics` (`out.metricValues.PeakUncertainty`).
