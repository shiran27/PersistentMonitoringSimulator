classdef (Abstract) Agent < handle & matlab.mixin.Heterogeneous
    % Base class of every agent type. A subclass implements update(dt) (its motion along
    % obj.path) and may extend resetToInitial and label. Sharing this base
    % (matlab.mixin.Heterogeneous) lets agents of different types live in the same
    % ScenarioModel.agents array.
    %
    % What every agent has:
    %   - kinematic state and limits (KinematicState: pos, vel, acc, maxSpeed, maxAccel)
    %   - a speed profile for its transits (SpeedProfile: "FO-0", "SO" or "FO-3"). A policy
    %     that plans a transit may hand the agent its transit time, profile and speed /
    %     acceleration limits before it leaves a target (as RHC policies set dwellRemaining);
    %     otherwise the agent uses its own profile with peak speed maxSpeed. A policy plans
    %     transit times of at least shortestTransit(path), which the agent can keep; legTime is
    %     the planned duration of the transit in progress (or the last one)
    %   - energy metrics: Je = int |u|^2 dt (NaN once an FO-0 transit is used: its
    %     impulsive accelerations make it undefined), peak speed and acceleration
    %   - a battery (Battery: capacity and power model), reported as SOC (about an hour of
    %     operation on the default 26 Wh); with stopWhenEmpty = true the agent stops when it is
    %     empty
    % Agents are spawned from an AgentSpec (type and settings): see ScenarioModel.agentSpec.
    properties
        index
        state                       % KinematicState
        controller                  % TrackingController (feedback law of PhysicalAgent)
        path = []                   % N x 2 waypoints to the destination target
        pathIndex = 1
        current_target_idx = []     % destination target, or the target the agent is at
        dwellRemaining double = 0   % seconds left to wait at the current target
        initialPosition
        initialTargetIdx

        profile string = "FO-0"     % speed profile of its transits: "FO-0", "SO" or "FO-3"
        transitTime = NaN           % planned transit time of the next edge (NaN: from maxSpeed)
        transitProfile string = ""  % profile of the next edge ("" = profile)
        transitLimits = [Inf Inf]   % [speed, acceleration] limits of the next edge's profile

        legTime = NaN               % planned duration of the transit in progress (or the last)

        Je = 0                      % energy objective int |u(t)|^2 dt so far
        peakSpeed = 0               % largest speed so far
        peakAccel = 0               % largest |acceleration| so far

        mode string = "idle"        % "idle", "traveling" or "power_outage" (battery)
        battery                     % Battery (capacity, power model, energy drawn)
        stopWhenEmpty = false       % true: stop moving when the battery is empty (default:
                                    % the battery is only reported, as in the RHC papers)
        maxNormalAccel = Inf        % normal acceleration available in turns, a_n (energy-aware
                                    % paper, A1): curves limit the speed to sqrt(a_n / kappa)
        chargeAllowed = true        % false: a station does not charge it (a policy waiting for
                                    % a booked slot clears it; reset sets it again)

        color = [0.8 0.2 0.2]
        size = 8
        labelFontSize = 7
        graphicHandle
        textHandle
        ax
    end

    properties (Dependent)
        e_total                     % energy drawn from the battery so far [J]
        soc                         % battery state of charge
        battery_percentage
    end

    methods (Abstract)
        update(obj, dt)   % advance the motion by dt along obj.path; clear obj.path on arrival
    end

    methods
        function obj = Agent(index, position, maxSpeed)
            obj.index = index;
            obj.initialPosition = position(:)';
            obj.state = KinematicState(obj.initialPosition, maxSpeed, 15);
            obj.controller = TrackingController();
            obj.battery = Battery();
        end

        function resetToInitial(obj)
            obj.state.pos = obj.initialPosition;
            obj.state.vel = [0 0];
            obj.state.acc = [0 0];
            obj.state.ori = 0;
            obj.state.omega = 0;
            obj.path = [];
            obj.pathIndex = 1;
            obj.dwellRemaining = 0;
            obj.current_target_idx = obj.initialTargetIdx;
            obj.clearTransitPlan();
            obj.legTime = NaN;
            obj.Je = 0;
            obj.peakSpeed = 0;
            obj.peakAccel = 0;
            obj.mode = "idle";
            obj.chargeAllowed = true;
            obj.battery.reset();
        end

        function e = get.e_total(obj), e = obj.battery.energyUsed; end
        function s = get.soc(obj), s = obj.battery.soc; end
        function p = get.battery_percentage(obj), p = obj.battery.percentage; end

        function set.profile(obj, p)
            p = SpeedProfile.canonical(p);   % older names ("constant", "FO") still work
            assert(any(p == SpeedProfile.names), 'Agent:Profile', ...
                'Unknown speed profile "%s" (use %s)', p, strjoin(SpeedProfile.names, ", "));
            obj.profile = p;
        end

        function s = label(obj)
            % Text drawn on the agent: its index and battery level
            s = sprintf('%d\n%.0f%%', obj.index, obj.battery_percentage);
        end

        function draw(obj, ax)
            obj.ax = ax;
            if ~isempty(obj.graphicHandle) && isvalid(obj.graphicHandle), delete(obj.graphicHandle); end
            [x, y] = obj.calculateVertices();
            % Graphics let clicks through to the canvas so a target under an agent stays clickable
            obj.graphicHandle = patch(ax, x, y, obj.color, 'EdgeColor', 'k', 'FaceAlpha', 0.8, 'PickableParts', 'none');
            obj.textHandle = text(ax, obj.state.pos(1), obj.state.pos(2), obj.label(), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', 'w', ...
                'FontSize', obj.labelFontSize, 'PickableParts', 'none');
        end

        function updateVisuals(obj)
            if isempty(obj.graphicHandle) || ~isgraphics(obj.graphicHandle), return; end
            [x, y] = obj.calculateVertices();
            set(obj.graphicHandle, 'XData', x, 'YData', y);
            set(obj.textHandle, 'Position', [obj.state.pos(1), obj.state.pos(2), 0], 'String', obj.label());
        end

        function tf = batteryEmpty(obj)
            % Has the agent stopped for an empty battery? (only with stopWhenEmpty)
            tf = obj.stopWhenEmpty && obj.battery_percentage <= 0;
        end

        function rho = shortestTransit(~, ~)
            % Shortest transit time along a path (N x 2 points) this agent can keep: 0, since an
            % idealized agent follows any plan (PhysicalAgent overrides this)
            rho = 0;
        end

        function E = transitEnergy(obj, pts, rho)
            % Predicted battery energy [J] of a transit along pts (N x 2) taking rho, with the
            % agent's own speed profile (under the full power model and with battery losses
            % when its battery has c2 > 0 or R0 > 0)
            y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
            if obj.battery.c2 == 0 && obj.battery.R0 == 0
                E = obj.battery.transitEnergy(rho, y, obj.accelEnergy(pts, rho));
            else
                E = obj.battery.planEnergy(SpeedProfile.shape(obj.profile, y, rho, Inf, Inf, obj.motionNu()));
            end
        end

        function vb = segmentSpeedLimit(obj, pts)
            % Speed limit on a path (N x 2): maxSpeed, or lower where its curves would need more
            % normal acceleration than maxNormalAccel, min(maxSpeed, sqrt(a_n/kappa)) with kappa
            % the path's largest curvature (energy-aware paper, Assumption A1)
            vb = obj.state.maxSpeed;
            if isfinite(obj.maxNormalAccel)
                vb = min(vb, sqrt(obj.maxNormalAccel / Agent.maxCurvature(pts)));
            end
        end

        function rho = limitTransit(obj, pts)
            % Shortest rest-to-rest transit along pts within the agent's own limits: accelerate
            % at maxAccel, cruise at segmentSpeedLimit, brake (energy-aware paper, Lemma 1)
            y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
            rho = SpeedProfile.minTransit(y, obj.segmentSpeedLimit(pts), obj.state.maxAccel);
        end

        function P = planProfile(obj, pts, rho, name)
            % The least-loss profile <name> (default: the agent's own) of a transit along pts
            % taking rho (at least limitTransit), within the agent's limits and under its power
            % model (SpeedProfile.shape with nu = motionNu)
            if nargin < 4, name = obj.profile; end
            y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
            vb = obj.segmentSpeedLimit(pts);
            if SpeedProfile.canonical(name) == "FO-0"   % jumps to its speed: no acceleration phase
                rho = max(rho, y / vb);
            else
                rho = max(rho, SpeedProfile.minTransit(y, vb, obj.state.maxAccel));
            end
            P = SpeedProfile.shape(name, y, rho, vb, obj.state.maxAccel, obj.motionNu());
        end

        function [E, Eex] = transitPlanEnergy(obj, pts, rho, name)
            % Battery energy E_ij [J] of a transit along pts taking rho with profile name, within
            % the agent's limits and with battery losses, and its excess E^ex over idling for the
            % same time (energy-aware paper, Sec. 3.3)
            if nargin < 4, name = obj.profile; end
            P = obj.planProfile(pts, rho, name);
            E = obj.battery.planEnergy(P);
            Eex = E - obj.battery.idleEnergy(P.rho);
        end

        function nu = motionNu(obj)
            % sqrt(c2/c1) of the agent's power model: 0 unless its battery has c2 > 0
            nu = sqrt(obj.battery.c2 / obj.battery.c1);
        end

        function Ja = accelEnergy(obj, pts, rho, name, vb, ub)
            % int |a|^2 dt of a transit along pts taking rho with profile name (default: the
            % agent's own) and limits vb, ub (default none); 0 for FO-0, whose jumps in
            % speed draw no modeled power
            if nargin < 4, name = obj.profile; end
            if nargin < 6, vb = Inf; ub = Inf; end
            y = sum(vecnorm(diff(pts, 1, 1), 2, 2));
            Ja = SpeedProfile.energy(name, y, rho, vb, ub);
            if ~isfinite(Ja), Ja = 0; end
        end

        function f = transitEnergyModel(~, ~, y, name, vb, ub, ~)
            % The energy objective int |a|^2 dt of a planned transit, as a function f(rho)
            % (vectorized) for a path of length y crossed with profile name and limits vb, ub
            % (args: pts, y, name, vb, ub, rhoRange). An idealized agent follows the profile
            % exactly along the arc length, so this is SpeedProfile.energy, the closed form of
            % the energy-aware RHC paper; PhysicalAgent adds its turns and its own profile
            f = @(rho) SpeedProfile.energy(name, y, rho, vb, ub);
        end

        function k = energyModelKey(~)
            % What transitEnergyModel depends on besides the path and the plan (for caching)
            k = "closed form";
        end

        function calculate_energy(obj, dt)
            % Battery drain over dt from the current speed, acceleration and mode (none once an
            % agent with stopWhenEmpty has emptied its battery: it is switched off)
            if obj.batteryEmpty(), return; end
            obj.battery.drain(norm(obj.state.vel), norm(obj.state.acc), obj.mode == "traveling", dt);
        end
    end

    methods (Access = protected)
        function [x, y] = calculateVertices(obj)
            h = obj.size * sqrt(3)/2;
            xb = [obj.size/2, -obj.size/2, -obj.size/2, obj.size/2];
            yb = [0, h/2, -h/2, 0];
            x = xb*cos(obj.state.ori) - yb*sin(obj.state.ori) + obj.state.pos(1);
            y = xb*sin(obj.state.ori) + yb*cos(obj.state.ori) + obj.state.pos(2);
        end

        function L = beginLeg(obj)
            % The transit that starts now along obj.path: arc lengths from the current position
            % (L.pts, L.s), the speed profile L.P (planned by the policy, or the agent's own
            % with peak speed maxSpeed; it depends on the path length only), and the heading
            % along the path (L.th per segment, turned over a window of half-width L.h around
            % each vertex; see headingAt)
            if isobject(obj.path)
                pts = cell2mat(arrayfun(@(w) w.position(:)', obj.path(:), 'UniformOutput', false));
            else
                pts = obj.path;
            end
            pts = [obj.state.pos; pts];
            s = [0; cumsum(vecnorm(diff(pts, 1, 1), 2, 2))];
            keep = [true; diff(s) > 0];                 % drop repeated points
            pts = pts(keep, :); s = s(keep);
            if numel(s) < 2                             % already there
                pts = [pts; pts]; s = [0; eps];
            end
            y = s(end);
            d = diff(pts, 1, 1);
            th = unwrap(atan2(d(:, 2), d(:, 1)));
            seg = diff(s);
            h = min([0.5 * ones(numel(seg) - 1, 1), seg(1:end-1) / 2, seg(2:end) / 2], [], 2);
            L = struct('pts', pts, 's', s, 'P', obj.plannedProfile(y), 'tau', 0, 'e', 0, 'th', th, 'h', h);
            obj.legTime = L.P.rho;
        end

        function P = plannedProfile(obj, y)
            % The speed profile of a transit of length y: the policy's plan (transit time,
            % profile, limits) or the agent's own profile with peak speed maxSpeed
            name = obj.transitProfile;
            if name == "", name = obj.profile; end
            lim = obj.transitLimits;
            rho = obj.transitTime;
            if ~(rho > 0), rho = SpeedProfile.defaultTransit(name, y, obj.state.maxSpeed); end
            rho = max(rho, SpeedProfile.minTransit(y, lim(1), lim(2)));
            P = SpeedProfile.shape(name, y, rho, lim(1), lim(2), obj.motionNu());
        end

        function [p, dir] = pointAt(~, L, l)
            % Position at arc length l along the leg, and the unit tangent there
            k = min(find(L.s < l, 1, 'last'), numel(L.s) - 1);
            if isempty(k), k = 1; end
            dir = (L.pts(k + 1, :) - L.pts(k, :)) / (L.s(k + 1) - L.s(k));
            p = L.pts(k, :) + (min(l, L.s(end)) - L.s(k)) * dir;
        end

        function [theta, kappa] = headingAt(~, L, l)
            % Heading theta and its rate per unit length kappa (the path curvature) at arc
            % length l. The heading follows the segment directions and turns linearly across a
            % window around each vertex (half-width up to 0.5), so the angular velocity
            % omega = kappa * v stays finite (Welikala and Cassandras, arXiv 2102.12963,
            % Theorem 2: omega = F(l) v). Curvature never enters the speed profile.
            n = numel(L.s);
            k = min(find(L.s < l, 1, 'last'), n - 1);
            if isempty(k), k = 1; end
            theta = L.th(k); kappa = 0;
            if k > 1 && l < L.s(k) + L.h(k - 1)                 % turning at the vertex where k starts
                w = 2 * L.h(k - 1); dth = L.th(k) - L.th(k - 1);
                theta = L.th(k - 1) + dth * (l - L.s(k) + L.h(k - 1)) / w; kappa = dth / w;
            elseif k < n - 1 && l > L.s(k + 1) - L.h(k)         % turning at the vertex where k ends
                w = 2 * L.h(k); dth = L.th(k + 1) - L.th(k);
                theta = L.th(k) + dth * (l - L.s(k + 1) + L.h(k)) / w; kappa = dth / w;
            end
        end

        function clearTransitPlan(obj)
            obj.transitTime = NaN;
            obj.transitProfile = "";
            obj.transitLimits = [Inf Inf];
        end

    end

    methods (Static)
        function k = maxCurvature(pts)
            % Largest curvature of a path (N x 2): the largest 1/radius of the circle through
            % three consecutive points (the planners sample their arcs, so on an arc this is
            % 1/radius); 0 for a straight path
            k = 0;
            for i = 2:size(pts, 1) - 1
                a = pts(i, :) - pts(i - 1, :); b = pts(i + 1, :) - pts(i, :); c = a + b;
                la = norm(a); lb = norm(b); lc = norm(c);
                if la < 1e-12 || lb < 1e-12 || lc < 1e-12, continue; end
                k = max(k, 2 * abs(a(1) * b(2) - a(2) * b(1)) / (la * lb * lc));
            end
        end
    end
end
