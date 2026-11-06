classdef PhysicalAgent < Agent
    % Physical agent (the counterpart of the idealized AbstractAgent): a second-order
    % (double-integrator) agent with speed and acceleration limits (maxSpeed, maxAccel;
    % PhysicalIntegrator) that tracks a planned reference motion with feedback
    % (TrackingController) and arrives when the plan says.
    %
    % The reference keeps within planMargin (1 by default: the full limits) of maxSpeed and
    % maxAccel, the acceleration counting both its parts, sqrt(u^2 + (v^2 kappa)^2) <= maxAccel
    % (the friction circle: the turn takes v^2 kappa across the path, the rest is left for u
    % along it):
    %   - its path is the edge's path with each corner rounded over at most cornerRadius on
    %     either side, so speed and acceleration stay continuous along it
    %   - its speed profile is the planned one (the policy's profile and transit time, or the
    %     agent's own) when that profile keeps within the limits on this path; otherwise it is
    %     the fastest profile that does (full acceleration and braking, slower through corners,
    %     where the turn takes acceleration too), slowed down uniformly to the planned time.
    %     With an energy-aware profile ("SO", "FO-3") the agent takes whichever of the two draws
    %     less int |a|^2 dt here: on a curvy path the planned shape, optimal on a straight
    %     line, can cost more in the turns than the retimed fastest profile
    % A plan shorter than shortestTransit(path), the fastest such profile, cannot be kept, so
    % the agent takes that long instead. The RHC policies plan with shortestTransit, so their
    % transit times are kept. legTime is the duration of the transit in progress; the agent
    % arrives within a step of it.
    %
    % The feedback is a PD law with acceleration feedforward: the reference's own change of
    % velocity over each step (along the path and, in turns, across it). The agent counts as
    % arrived once the reference has stopped and it is within
    % arrivePosTol of the target and almost still; while dwelling it holds its position there.
    % The profile, maxSpeed, maxAccel and the gains (controller.kp, controller.kd) are set per
    % agent (GUI: Agent Speed / Agent Accel / Speed Profile; runTrial: AgentSpeed,
    % Agent_profile, Agent_maxAccel, Agent_kp, Agent_kd). With profile "FO-0" (whose jumps
    % in speed no physical agent can follow) the agent moves at its own fastest pace. Je counts
    % the full commanded acceleration (|a|^2, steering included).
    properties
        planMargin = 1        % share of maxSpeed and maxAccel the reference may use
        cornerRadius = 1.5    % largest stretch of path on either side of a corner that is rounded
    end

    properties (Access = private)
        leg = []          % the edge in progress: geometry G, profile P, elapsed time tau, and
                          % the reference point p and its step velocity v at tau
        holdPoint = []    % where the agent holds while it has no path
    end

    methods
        function obj = PhysicalAgent(index, position, maxSpeed, opts)
            arguments
                index
                position
                maxSpeed
                opts.Profile string = "FO-0"
            end
            obj@Agent(index, position, maxSpeed);
            obj.profile = opts.Profile;
        end

        function resetToInitial(obj)
            resetToInitial@Agent(obj);
            obj.leg = [];
            obj.holdPoint = [];
        end

        function rho = shortestTransit(obj, pts)
            % Duration of the fastest reference along pts (N x 2) this agent can keep
            P = obj.fastestProfile(obj.geometry(pts));
            rho = P.rho;
        end

        function Ja = accelEnergy(obj, pts, rho, name, vb, ub)
            % int |a|^2 dt of the reference along pts for a plan taking rho with profile name
            % (default: the agent's own) and limits vb, ub (default none), along the path and
            % across it, sampled as the agent follows it
            if nargin < 4, name = obj.profile; end
            if nargin < 6, vb = Inf; ub = Inf; end
            G = obj.geometry(pts);
            y = G.s(end);
            rho = max([rho, SpeedProfile.minTransit(y, vb, ub), eps]);
            Ja = obj.referenceEnergy(G, obj.referenceProfile(G, SpeedProfile.shape(name, y, rho, vb, ub)));
        end

        function f = transitEnergyModel(obj, pts, y, name, vb, ub, rhoRange)
            % int |a|^2 dt of the reference this agent follows for a plan (profile name, limits
            % vb, ub, transit time rho in rhoRange) along pts, as a function f(rho) (vectorized),
            % mirroring referenceProfile: the fastest profile retimed to rho, or, from the rho at
            % which the planned profile fits the agent's limits on this path, whichever of the two
            % draws less. Each is tabulated on a few transit times and interpolated in log-log
            % coordinates (exact for the rho^-3 law of a profile shape scaled in time), and
            % extrapolated by that law; the result falls with rho (monotone)
            G = obj.geometry(pts);
            fast = obj.fastestProfile(G);
            lo = max([rhoRange(1), fast.rho, SpeedProfile.minTransit(y, vb, ub), eps]);
            hi = max(rhoRange(end), 1.01 * lo);
            fits = @(r) SpeedProfile.canonical(name) ~= "FO-0" && obj.keepsLimits(SpeedProfile.shape(name, y, r, vb, ub), G);
            if fits(lo)
                rs = lo;
            elseif ~fits(hi)
                rs = Inf;
            else
                a = lo; b = hi;                      % bisection (log scale) for the switch
                for k = 1:14
                    c = sqrt(a * b);
                    if fits(c), b = c; else, a = c; end
                end
                rs = b;
            end
            top = min(hi, 8 * max(lo, min(rs, hi)));
            rF = unique(lo * (max(top, lo) / lo) .^ linspace(0, 1, 5));   % the fastest profile, retimed
            EF = arrayfun(@(r) obj.referenceEnergy(G, SpeedProfile.retime(fast, r)), rF);
            rP = []; EP = [];
            if isfinite(rs)                          % the planned profile
                rP = unique(rs * (min(max(hi, rs), 8 * rs) / rs) .^ linspace(0, 1, 6));
                EP = arrayfun(@(r) obj.referenceEnergy(G, SpeedProfile.shape(name, y, r, vb, ub)), rP);
            end
            if ~any(SpeedProfile.canonical(name) == ["SO", "FO-3"]), rP = []; EP = []; end   % not energy-aware: no choice
            f = @(r) PhysicalAgent.tabulatedEnergy(r, rs, rF, EF, rP, EP);
        end

        function k = energyModelKey(obj)
            k = sprintf("physical|%.10g|%.10g|%.10g|%.10g", obj.state.maxSpeed, obj.state.maxAccel, ...
                obj.planMargin, obj.cornerRadius);
        end

        function update(obj, dt)
            c = obj.controller;
            if obj.batteryEmpty()
                aCmd = -obj.state.vel / dt;                       % brake to a stop
                if norm(aCmd) > obj.state.maxAccel, aCmd = aCmd / norm(aCmd) * obj.state.maxAccel; end
                obj.mode = "power_outage";
            elseif isempty(obj.path)
                if isempty(obj.holdPoint), obj.holdPoint = obj.state.pos; end
                aCmd = c.track(obj.state, obj.holdPoint, [0 0], [0 0]);   % hold at the target
                obj.mode = "idle";
            else
                % Reference point now (p, with velocity v: its last step over dt) and one step
                % ahead; the feedforward is the change of that step velocity, so an agent on the
                % reference stays on it exactly (PhysicalIntegrator: v += a dt, then p += v dt)
                if isempty(obj.leg), obj.startLeg(); end
                L = obj.leg;
                tau = L.tau + dt;
                if tau >= L.P.rho - 1e-9 * max(1, L.P.rho), tau = L.P.rho; end
                pNext = obj.curveAt(L.G, SpeedProfile.at(L.P, tau));
                vNext = (pNext - L.p) / dt;
                aCmd = c.track(obj.state, L.p, L.v, (vNext - L.v) / dt);
                obj.leg.tau = tau; obj.leg.p = pNext; obj.leg.v = vNext;
                obj.mode = "traveling";
            end
            obj.state.acc = aCmd;
            ori0 = obj.state.ori;
            PhysicalIntegrator.update(obj.state, dt);                  % heading from the velocity
            obj.state.omega = angle(exp(1i * (obj.state.ori - ori0))) / dt;
            obj.Je = obj.Je + (aCmd * aCmd') * dt;
            obj.peakSpeed = max(obj.peakSpeed, norm(obj.state.vel));
            obj.peakAccel = max(obj.peakAccel, norm(aCmd));
            if ~isempty(obj.leg) && obj.leg.tau >= obj.leg.P.rho - 1e-9 * max(1, obj.leg.P.rho) ...
                    && c.hasArrived(obj.state, obj.leg.G.pts(end, :))
                obj.holdPoint = obj.leg.G.pts(end, :);              % arrived
                obj.path = [];
                obj.pathIndex = 1;
                obj.leg = [];
                obj.clearTransitPlan();
            end
            obj.calculate_energy(dt);
            obj.updateVisuals();
        end
    end

    methods (Static, Access = private)
        function E = tabulatedEnergy(r, rs, rF, EF, rP, EP)
            % transitEnergyModel's interpolation: the retimed fastest profile's energy, and from
            % rs the lower of it and the planned profile's; log-log linear inside each table, the
            % rho^-3 law outside
            E = part(r, rF, EF);
            if ~isempty(rP)
                in = r >= rs;
                if any(in(:)), E(in) = min(E(in), part(r(in), rP, EP)); end
            end

            function e = part(r, rg, Eg)
                Eg = max(Eg, realmin);
                if isscalar(rg)
                    e = Eg * (rg ./ r).^3;
                    return;
                end
                e = exp(interp1(log(rg), log(Eg), log(r), 'linear'));
                below = r < rg(1); above = r > rg(end);
                e(below) = Eg(1) * (rg(1) ./ r(below)).^3;
                e(above) = Eg(end) * (rg(end) ./ r(above)).^3;
            end
        end
    end

    methods (Access = private)
        function startLeg(obj)
            % The reference of the transit that starts now: from the start of the path (where
            % the agent holds) or, if the agent is away from it, from where it is
            if isobject(obj.path)
                pts = cell2mat(arrayfun(@(w) w.position(:)', obj.path(:), 'UniformOutput', false));
            else
                pts = obj.path;
            end
            if norm(pts(1, :) - obj.state.pos) >= obj.controller.arrivePosTol
                pts = [obj.state.pos; pts];
            end
            G = obj.geometry(pts);
            P = obj.referenceProfile(G, obj.plannedProfile(G.s(end)));
            obj.leg = struct('G', G, 'P', P, 'tau', 0, 'p', G.pts(1, :), 'v', [0 0]);
            obj.legTime = P.rho;
            obj.holdPoint = [];
        end

        function Ja = referenceEnergy(obj, G, P)
            % int |a|^2 dt of the reference (path G, profile P) as the agent follows it: the
            % step velocities and their changes, as in update (0.05 s steps; coarser for very
            % long transits)
            dt = max(0.05, P.rho / 2000);
            t = [unique([0:dt:P.rho, P.rho]), P.rho, P.rho];        % then two steps at rest
            p = zeros(numel(t), 2);
            for k = 1:numel(t), p(k, :) = obj.curveAt(G, SpeedProfile.at(P, t(k))); end
            v = [0 0; diff(p) / dt];
            a = diff(v) / dt;
            Ja = sum(sum(a.^2, 2)) * dt;
        end

        function P = referenceProfile(obj, G, P)
            % The planned profile P if it keeps within the reference limits along G, else the
            % fastest profile that does, slowed down to P's time (if that is longer). For an
            % energy-aware P ("SO", "FO-3"), the retimed fastest profile also when it draws less
            fastest = obj.fastestProfile(G);
            if P.rho < fastest.rho || ~obj.keepsLimits(P, G)
                P = SpeedProfile.retime(fastest, max(P.rho, fastest.rho));
            elseif any(P.name == ["SO", "FO-3"])
                Q = SpeedProfile.retime(fastest, P.rho);
                if obj.referenceEnergy(G, Q) < obj.referenceEnergy(G, P), P = Q; end
            end
        end

        function G = geometry(obj, pts)
            % Arc lengths s, unit directions D of the segments, and the rounded corner at each
            % inner point k: half-width h(k) and curvature bound kap(k) = |D(k) - D(k-1)| / (2 h(k))
            seg = vecnorm(diff(pts, 1, 1), 2, 2);
            keep = [true; seg > 1e-9];                              % drop repeated points
            pts = pts(keep, :);
            if size(pts, 1) < 2, pts = [pts; pts + [1e-6 0]]; end  % already there (eps would vanish at |p| > 1)
            seg = vecnorm(diff(pts, 1, 1), 2, 2);
            n = size(pts, 1);
            G.pts = pts;
            G.s = [0; cumsum(seg)];
            G.D = diff(pts, 1, 1) ./ seg;
            G.h = zeros(n, 1); G.kap = zeros(n, 1);
            if n > 2
                k = (2:n-1)';
                G.h(k) = min([seg(k - 1) / 2, seg(k) / 2, repmat(obj.cornerRadius, n - 2, 1)], [], 2);
                G.kap(k) = vecnorm(G.D(k, :) - G.D(k - 1, :), 2, 2) ./ (2 * G.h(k));
            end
        end

        function p = curveAt(~, G, l)
            % Point p at path distance l along the rounded path (a quadratic Bezier curve over
            % each corner)
            l = min(max(l, 0), G.s(end));
            k = min(find(G.s <= l, 1, 'last'), numel(G.s) - 1);
            j = 0;                                                  % corner whose rounding holds l
            if k > 1 && l - G.s(k) < G.h(k), j = k; end
            if k + 1 < numel(G.s) && G.s(k + 1) - l < G.h(k + 1), j = k + 1; end
            if j == 0
                p = G.pts(k, :) + (l - G.s(k)) * G.D(k, :);
            else
                h = G.h(j); Ti = G.D(j - 1, :); To = G.D(j, :); V = G.pts(j, :);
                q = (l - G.s(j) + h) / (2 * h);
                p = (1 - q)^2 * (V - h * Ti) + 2 * (1 - q) * q * V + q^2 * (V + h * To);
            end
        end

        function kap = curvature(~, G, l)
            % Curvature bound at path distances l (0 outside the rounded corners)
            kap = zeros(size(l));
            for j = find(G.h > 0)'
                in = abs(l - G.s(j)) < G.h(j);
                kap(in) = max(kap(in), G.kap(j));
            end
        end

        function P = fastestProfile(obj, G)
            % The fastest rest-to-rest profile along G within the reference limits: speed at
            % most vb, and acceleration sqrt(u^2 + (v^2 kappa)^2) at most ub (forward pass
            % accelerating, backward pass braking, on a grid that resolves every corner)
            vb = obj.planMargin * obj.state.maxSpeed;
            ub = obj.planMargin * obj.state.maxAccel;
            y = G.s(end);
            corners = find(G.h > 0);
            l = unique([linspace(0, y, min(600, max(2, ceil(y / 0.25) + 1)))'; ...
                G.s(corners) - G.h(corners); G.s(corners) + G.h(corners)]);
            l = l(l >= 0 & l <= y);
            l = l([true; diff(l) > 1e-9]);
            dl = diff(l);
            kap = obj.curvature(G, (l(1:end-1) + l(2:end)) / 2);   % per interval
            vlim = min(vb, sqrt(ub ./ kap));
            v = [0; min(vlim(1:end-1), vlim(2:end)); 0];
            for k = 1:numel(dl)                                     % accelerate
                v(k + 1) = min(v(k + 1), sqrt(v(k)^2 + 2 * sqrt(max(ub^2 - (v(k)^2 * kap(k))^2, 0)) * dl(k)));
            end
            for k = numel(dl):-1:1                                  % brake
                v(k) = min(v(k), sqrt(v(k + 1)^2 + 2 * sqrt(max(ub^2 - (v(k + 1)^2 * kap(k))^2, 0)) * dl(k)));
            end
            P = SpeedProfile.table(l, v);
        end

        function ok = keepsLimits(obj, P, G)
            % Does profile P keep within the reference limits along G? (FO-0 never does: it
            % jumps to its speed)
            if P.name == "FO-0", ok = false; return; end
            vb = obj.planMargin * obj.state.maxSpeed;
            ub = obj.planMargin * obj.state.maxAccel;
            t = linspace(0, P.rho, 400);
            l = zeros(size(t)); v = l; u = l;
            for k = 1:numel(t), [l(k), v(k), u(k)] = SpeedProfile.at(P, t(k)); end
            tol = 1 + 1e-6;
            ok = all(v <= vb * tol) && all(hypot(u, v.^2 .* obj.curvature(G, l)) <= ub * tol);
        end
    end
end
