classdef DiffDriveAgent < Agent
    % Differential-drive robot (energy-aware paper, Sec. 7): the physical agent of the
    % energy-aware paper. PhysicalAgent is unchanged.
    %
    % Model. Pose (p, psi), speed v and yaw rate w; wheels of radius r on a track b, driven by DC
    % motors through gears n with quasi-static electrics, tau = n kt I and V = Rm I + n kt phidot.
    % Rolling resistance Frr, drag cd v and turning resistance scrub w oppose the motion:
    %   m vdot = (tau_R + tau_L)/r - Frr - cd v,   Iz wdot = b (tau_R - tau_L)/(2 r) - scrub w.
    % Each motor draws max(V I, 0) (no regeneration) through the converter (efficiency eta) on
    % top of the processor and communication loads, from a Battery1RC (one-RC circuit, CC-CV
    % charging, SOC estimate by coulomb counting). Rm, Frr, cd and the moving load are fitted to
    % the battery's plan coefficients (c1, c2, c3 and movePower; fitPhysics), so on a straight
    % line the motors draw what the plan model P_M + c1 u^2 + c2 v^2 + c3 v predicts. Turns,
    % saturation, braking without regeneration and the circuit are what the plan leaves out.
    %
    % Control. A transit starts with a turn in place to the path's heading (rate turnRate,
    % angular acceleration turnAccel), whose time shortestTransit includes; then the reference is
    % the planned speed profile, over the rest of the planned time, along the edge's path with
    % each corner rounded over at most cornerRadius. A Kanayama tracking law gives the commanded speed and
    % yaw rate (gains [kx ky ktheta]) with the reference's own as feedforward; an inner loop
    % sets the wheel torques (feedforward from the rigid body plus a proportional term on the
    % wheel speeds), within the motors' current limit and the battery's terminal voltage.
    % Arrival-time regulator: when the robot falls behind the reference by more than lagTol, the
    % rest of the reference is re-timed to end at the planned time, using up to (1 + epsV) and
    % (1 + epsU) of the planning limits (maxSpeed, maxAccel: the policy plans within them); when
    % even that cannot make it, it takes the fastest such profile and counts a delay event.
    %
    % Planning interface: segmentSpeedLimit min(vbar/(1 + b kappa/2), sqrt(a_n/kappa)) over the
    % rounded path's largest curvature kappa (the outer wheel at vbar), and shortestTransit /
    % limitTransit, the fastest profile within it. transitPlanEnergy is the parent's plan model.
    %
    % Records: transits (per transit: planned and actual duration, the plan's energy and the
    % cells' energy, and whether it was delayed), socLog ([time, true SOC, estimate] once a
    % second) and delayEvents.
    properties
        mass = 20                 % [kg]
        wheelRadius = 0.05        % r [m]
        track = 0.3               % b, wheel separation [m]
        gearRatio = 10            % n
        kt = 0.02                 % torque constant = back-EMF constant [N m/A]
        maxCurrent = 6            % per motor [A]
        scrub = 0.3               % turning resistance [N m s]
        epsV = 0.1                % re-timing margins (shares of maxSpeed and maxAccel)
        epsU = 0.2
        gains = [3 6 3]           % Kanayama kx [1/s], ky [1/m^2], ktheta [1/m]
        wheelTime = 0.05          % time constant of the wheel-speed loop [s]
        cornerRadius = 1.5        % largest stretch rounded on either side of a corner [m]
        turnRate = 2              % turn in place at the start of a transit: rate [rad/s] and
        turnAccel = 4             % angular acceleration [rad/s^2]
        arriveTol = 0.15          % arrived within this distance once the reference stops [m]
        lagTol = 0.2              % re-time the rest when this far behind the reference [m]
        substep = 0.005           % integration step of the drive [s]
        transits = struct('planned', {}, 'actual', {}, 'planE', {}, 'cellE', {}, 'delayed', {})
        socLog = zeros(0, 3)      % [time, true SOC, estimated SOC], once a second
        delayEvents = 0
    end

    properties (SetAccess = private)
        phys = struct()           % fitted: Rm, Frr, cd, Iz, loads moving and idle (fitPhysics)
    end

    properties (Access = private)
        leg = []                  % transit in progress
        holdPoint = []
        v = 0                     % speed [m/s]
        w = 0                     % yaw rate [rad/s]
        clock = 0                 % time since reset [s]
        lastLog = -Inf
        fitKey = ""
    end

    methods
        function obj = DiffDriveAgent(index, position, maxSpeed, opts)
            arguments
                index
                position
                maxSpeed
                opts.Profile string = "SO"
            end
            obj@Agent(index, position, maxSpeed);
            obj.profile = opts.Profile;
            obj.state.maxAccel = 1;
            obj.battery = Battery1RC();
            obj.battery.sensorSeed = index;
        end

        function resetToInitial(obj)
            resetToInitial@Agent(obj);
            obj.leg = []; obj.holdPoint = [];
            obj.v = 0; obj.w = 0; obj.clock = 0; obj.lastLog = -Inf;
            obj.transits = obj.transits([]); obj.socLog = zeros(0, 3); obj.delayEvents = 0;
            obj.battery.sensorSeed = obj.index;
            obj.battery.reset();
            obj.fitPhysics();
        end

        function ph = fitPhysics(obj)
            % Physical parameters that reproduce the battery's plan model on a straight line:
            % copper loss K (m u + F)^2 with K = Rm r^2/(2 n^2 kt^2) = c1 eta/m^2, drag cd and
            % rolling resistance Frr from c2 and c3 (cross terms included), and the moving load
            % movePower less the constant copper loss K Frr^2/eta
            b = obj.battery;
            K = b.c1 * b.eta / obj.mass^2;
            if K > 0
                cd = (-1 + sqrt(1 + 4 * K * b.c2 * b.eta)) / (2 * K);
            else
                cd = b.c2 * b.eta;
            end
            Frr = b.c3 * b.eta / (1 + 2 * K * cd);
            ph = struct('K', K, 'Rm', 2 * obj.gearRatio^2 * obj.kt^2 * K / obj.wheelRadius^2, ...
                'Frr', Frr, 'cd', cd, 'Iz', obj.mass * obj.track^2 / 8, ...
                'loadMove', b.movePower() - K * Frr^2 / b.eta, 'loadIdle', b.idlePower(), ...
                'Kw', obj.mass * obj.wheelRadius^2 / (2 * obj.wheelTime));
            assert(ph.loadMove > 0, 'DiffDriveAgent:Fit', ['The copper loss at rolling resistance ' ...
                '(%.1f W) exceeds the moving power; use a larger mass'], K * Frr^2 / b.eta);
            obj.phys = ph;
            obj.fitKey = obj.paramKey();
        end

        function vb = segmentSpeedLimit(obj, pts)
            % The outer wheel at maxSpeed and the normal acceleration a_n over the rounded
            % path's largest curvature
            G = DiffDriveAgent.geometry(pts, obj.cornerRadius);
            kap = max(G.kap);
            vb = obj.state.maxSpeed / (1 + obj.track * kap / 2);
            if isfinite(obj.maxNormalAccel) && kap > 0, vb = min(vb, sqrt(obj.maxNormalAccel / kap)); end
        end

        function rho = shortestTransit(obj, pts)
            % The fastest profile along the rounded path within the planning limits, after the turn
            % in place to the path's heading when the robot is at the path's start
            G = DiffDriveAgent.geometry(pts, obj.cornerRadius);
            P = obj.fastestProfile(G, 1);
            rho = P.rho;
            if norm(pts(1, :) - obj.state.pos) < obj.arriveTol
                rho = rho + obj.turnTime(angle(exp(1i * (atan2(G.D(1, 2), G.D(1, 1)) - obj.state.ori))));
            end
        end

        function t = turnTime(obj, dpsi)
            % Time of a rest-to-rest turn in place by dpsi (a trapezoid in the yaw rate)
            a = abs(dpsi); w = obj.turnRate; u = obj.turnAccel;
            if a < 1e-3, t = 0;
            elseif a <= w^2 / u, t = 2 * sqrt(a / u);
            else, t = a / w + w / u;
            end
        end

        function rho = limitTransit(obj, pts)
            rho = max(limitTransit@Agent(obj, pts), obj.shortestTransit(pts));
        end

        function update(obj, dt)
            if obj.fitKey ~= obj.paramKey(), obj.fitPhysics(); end
            ph = obj.phys;
            obj.clock = obj.clock + dt;
            moving = false;
            if obj.batteryEmpty()
                vc = 0; wc = 0; aff = -obj.v / dt; alff = -obj.w / dt;
                obj.mode = "power_outage";
            elseif isempty(obj.path)
                if isempty(obj.holdPoint), obj.holdPoint = obj.state.pos; end
                vc = 0; wc = 0; aff = -obj.v / dt; alff = -obj.w / dt;
                obj.mode = "idle";
            else
                if isempty(obj.leg), obj.startLeg(); end
                [vc, wc, aff, alff] = obj.followReference(dt);
                obj.mode = "traveling";
                moving = true;
            end
            obj.drive(vc, wc, aff, alff, dt, moving, ph);
            if moving, obj.checkArrival(); end
            if obj.clock - obj.lastLog >= 1
                obj.socLog(end+1, :) = [obj.clock, obj.battery.socTrue, obj.battery.socEstimate];
                obj.lastLog = obj.clock;
            end
            obj.updateVisuals();
        end
    end

    methods (Access = private)
        function k = paramKey(obj)
            b = obj.battery;
            k = sprintf("%.10g|", obj.mass, obj.wheelRadius, obj.track, obj.gearRatio, obj.kt, ...
                obj.wheelTime, b.c1, b.c2, b.c3, b.eta, b.movePower(), b.idlePower());
        end

        function startLeg(obj)
            % The reference of the transit that starts now (from where the robot is, if it is
            % away from the path's start), its plan's energy, and the cells' energy so far
            if isobject(obj.path)
                pts = cell2mat(arrayfun(@(q) q.position(:)', obj.path(:), 'UniformOutput', false));
            else
                pts = obj.path;
            end
            if norm(pts(1, :) - obj.state.pos) >= obj.arriveTol, pts = [obj.state.pos; pts]; end
            G = DiffDriveAgent.geometry(pts, obj.cornerRadius);
            th0 = atan2(G.D(1, 2), G.D(1, 1));
            dpsi = angle(exp(1i * (th0 - obj.state.ori)));
            tTurn = obj.turnTime(dpsi);
            plan = obj.transitTime;
            if plan > 0, obj.transitTime = max(plan - tTurn, eps); end   % the profile gets the rest
            P = obj.plannedProfile(G.s(end));
            obj.transitTime = plan;
            fast = obj.fastestProfile(G, 1);
            if P.rho < fast.rho, P = fast; end   % a plan it cannot keep: the fastest it can
            name = obj.transitProfile; if name == "", name = obj.profile; end
            rhoPlan = tTurn + P.rho;
            if plan > 0, rhoPlan = max(plan, rhoPlan); end
            planE = obj.transitPlanEnergy(G.pts, rhoPlan, name);   % what the policy predicted, turn included
            obj.leg = struct('G', G, 'P', P, 'rho', tTurn + P.rho, 'tau', 0, 'tau0', tTurn, 's0', 0, ...
                'turn', [tTurn, dpsi, obj.state.ori], ...
                'pRef', G.pts(1, :), 'thRef', th0, 'vRef', 0, 'wRef', 0, ...
                'lastRetime', -Inf, 'delayed', false, 'start', obj.clock, ...
                'planE', planE, 'cellE0', obj.battery.energyDrawn, 'vLim', obj.transitLimits(1));
            obj.legTime = tTurn + P.rho;
            if plan > 0, obj.legTime = max(plan, obj.legTime); end
            obj.holdPoint = [];
        end

        function [vc, wc, aff, alff] = followReference(obj, dt)
            % The reference one step ahead, the regulator, and Kanayama's law
            L = obj.leg;
            if L.tau < L.turn(1)   % turning in place to the path's heading
                tau = min(L.tau + dt, L.turn(1));
                [~, w0] = obj.turnAt(L.turn, L.tau); [a1, w1] = obj.turnAt(L.turn, tau);
                k = obj.gains;
                eth = angle(exp(1i * (L.turn(3) + a1 - obj.state.ori)));
                vc = 0; wc = w1 + k(3) * eth; aff = -obj.v / dt; alff = (w1 - w0) / dt;
                L.tau = tau; obj.leg = L;
                return;
            end
            tau = min(L.tau + dt, L.rho);
            lag = dot(L.pRef - obj.state.pos, [cos(L.thRef), sin(L.thRef)]);   % along the path
            if lag > obj.lagTol && tau < L.rho - 0.5 && obj.clock - L.lastRetime > 1
                L = obj.retime(L, lag);
            end
            l = L.s0 + SpeedProfile.at(L.P, max(tau - L.tau0, 0));
            p = DiffDriveAgent.curveAt(L.G, l);
            d = p - L.pRef;
            if norm(d) > 1e-9
                vr = norm(d) / dt; th = atan2(d(2), d(1));
            else
                vr = 0; th = L.thRef;
            end
            wr = angle(exp(1i * (th - L.thRef))) / dt;
            % Kanayama's law on the error to the reference point now
            e = L.pRef - obj.state.pos; psi = obj.state.ori;
            ex = cos(psi) * e(1) + sin(psi) * e(2);
            ey = -sin(psi) * e(1) + cos(psi) * e(2);
            eth = angle(exp(1i * (L.thRef - psi)));
            k = obj.gains;
            vc = L.vRef * cos(eth) + k(1) * ex;
            wc = L.wRef + L.vRef * (k(2) * ey + k(3) * sin(eth));
            vc = min(max(vc, -0.2), (1 + obj.epsV) * obj.state.maxSpeed);
            aff = (vr - L.vRef) / dt; alff = (wr - L.wRef) / dt;
            L.tau = tau; L.pRef = p; L.thRef = th; L.vRef = vr; L.wRef = wr;
            obj.leg = L;
        end

        function [a, w] = turnAt(obj, turn, t)
            % Angle turned and yaw rate at time t of the turn [time, angle, start heading]
            T = turn(1); s = sign(turn(2)); A = abs(turn(2)); u = obj.turnAccel;
            if T <= 0, a = 0; w = 0; return; end
            wp = min(obj.turnRate, sqrt(A * u));   % peak rate (a triangle when the turn is small)
            tr = wp / u;
            if t <= tr, w = u * t; a = u * t^2 / 2;
            elseif t >= T - tr, r = max(T - t, 0); w = u * r; a = A - u * r^2 / 2;
            else, w = wp; a = wp * tr / 2 + wp * (t - tr);
            end
            a = s * min(a, A); w = s * w;
        end

        function L = retime(obj, L, lag)
            % The rest of the reference re-timed to end at the planned time, within (1 + epsV)
            % and (1 + epsU) of the limits; if that cannot make it, the fastest such and a delay
            vmax = (1 + obj.epsV) * min(obj.state.maxSpeed, L.vLim);
            umax = (1 + obj.epsU) * obj.state.maxAccel;
            sNow = L.s0 + SpeedProfile.at(L.P, max(L.tau - L.tau0, 0)) - lag;
            dist = L.G.s(end) - sNow;
            T = L.rho - L.tau;
            v0 = min(max(obj.v, 0), vmax);
            if dist <= 0, return; end
            ramps = @(vc) [abs(vc^2 - v0^2), vc^2] / (2 * umax);
            time = @(vc) abs(vc - v0) / umax + vc / umax + (dist - sum(ramps(vc))) / vc;
            hi = vmax;
            if sum(ramps(hi)) > dist   % no room to reach vmax: the highest peak that fits
                hi = sqrt(umax * dist + v0^2 / 2);
            end
            if time(hi) > T
                vc = hi;
                if ~L.delayed, obj.delayEvents = obj.delayEvents + 1; L.delayed = true; end
            else
                lo = max(dist / (4 * max(T, eps)), 1e-3);
                for it = 1:50
                    mid = (lo + hi) / 2;
                    if time(mid) > T, lo = mid; else, hi = mid; end
                end
                vc = hi;
            end
            r = ramps(vc);
            l = [0, r(1), dist - r(2), dist];
            sp = [v0, vc, vc, 0];
            keep = [true, diff(l) > 1e-9];
            L.P = SpeedProfile.table(l(keep), sp(keep));
            L.s0 = sNow; L.tau0 = L.tau;
            L.rho = L.tau + L.P.rho;
            L.lastRetime = obj.clock;
        end

        function drive(obj, vc, wc, aff, alff, dt, moving, ph)
            % The wheel-torque loop and the rigid body over dt in substeps, and the energy drawn
            r = obj.wheelRadius; b = obj.track; n = obj.gearRatio; kt = obj.kt;
            Fres = @(v) ph.Frr * tanh(v / 0.02) + ph.cd * v;
            load = ph.loadIdle; if moving, load = ph.loadMove; end
            m = max(1, ceil(dt / obj.substep)); h = dt / m;
            Ff = obj.mass * aff + Fres(vc); Mf = ph.Iz * alff + obj.scrub * wc;
            wheel = [(vc + wc * b / 2), (vc - wc * b / 2)] / r;          % wanted, right and left
            Je = 0; acc = [0 0];
            for k = 1:m
                spin = [(obj.v + obj.w * b / 2), (obj.v - obj.w * b / 2)] / r;
                tq = r * [Ff / 2 + Mf / b, Ff / 2 - Mf / b] + ph.Kw * (wheel - spin);
                if ~moving && abs(vc) < 1e-9, tq = ph.Kw * (wheel - spin); end
                I = tq / (n * kt);
                Vt = obj.battery.terminalVoltage(0);
                V = ph.Rm * I + n * kt * spin;
                over = abs(V) > Vt;
                I(over) = (sign(V(over)) * Vt - n * kt * spin(over)) / ph.Rm;
                I = min(max(I, -obj.maxCurrent), obj.maxCurrent);
                V = ph.Rm * I + n * kt * spin;
                tq = n * kt * I;
                F = (tq(1) + tq(2)) / r; M = b * (tq(1) - tq(2)) / (2 * r);
                if abs(obj.v) < 1e-3 && abs(F) <= ph.Frr        % static friction holds it
                    vdot = -obj.v / h;
                else
                    vdot = (F - Fres(obj.v)) / obj.mass;
                end
                wdot = (M - obj.scrub * obj.w) / ph.Iz;
                obj.v = obj.v + vdot * h; obj.w = obj.w + wdot * h;
                psi = obj.state.ori + obj.w * h;
                obj.state.ori = psi;
                obj.state.pos = obj.state.pos + obj.v * [cos(psi), sin(psi)] * h;
                acc = vdot * [cos(psi), sin(psi)] + obj.v * obj.w * [-sin(psi), cos(psi)];
                Je = Je + (acc * acc') * h;
                if ~obj.batteryEmpty()
                    obj.battery.drainPower(load + sum(max(V .* I, 0)) / obj.battery.eta, h);
                end
            end
            obj.state.vel = obj.v * [cos(obj.state.ori), sin(obj.state.ori)];
            obj.state.acc = acc;
            obj.state.omega = obj.w;
            obj.Je = obj.Je + Je;
            obj.peakSpeed = max(obj.peakSpeed, abs(obj.v));
            obj.peakAccel = max(obj.peakAccel, norm(acc));
        end

        function checkArrival(obj)
            % Arrived: the reference has stopped and the robot is at the end, almost still (or
            % 5 s past the plan, counted as delayed); records the transit
            L = obj.leg;
            if L.tau < L.rho - 1e-9, return; end
            near = norm(obj.state.pos - L.G.pts(end, :)) < obj.arriveTol && abs(obj.v) < 0.05;
            late = obj.clock - L.start > L.rho + 5;
            if ~(near || late), return; end
            actual = obj.clock - L.start;
            planned = obj.legTime;
            obj.transits(end+1) = struct('planned', planned, 'actual', actual, 'planE', L.planE, ...
                'cellE', obj.battery.energyDrawn - L.cellE0, 'delayed', L.delayed || late);
            if late && ~L.delayed, obj.delayEvents = obj.delayEvents + 1; end
            obj.holdPoint = L.G.pts(end, :);
            obj.path = []; obj.pathIndex = 1; obj.leg = [];
            obj.clearTransitPlan();
        end

        function P = fastestProfile(obj, G, share)
            % The fastest rest-to-rest profile along G with speed at most share (1 + epsV share)
            % of the outer-wheel and turn limits and |acceleration| at most share maxAccel
            vb = share * obj.state.maxSpeed; ub = share * obj.state.maxAccel;
            y = G.s(end);
            corners = find(G.h > 0);
            l = unique([linspace(0, y, min(600, max(2, ceil(y / 0.25) + 1)))'; ...
                G.s(corners) - G.h(corners); G.s(corners) + G.h(corners)]);
            l = l(l >= 0 & l <= y);
            l = l([true; diff(l) > 1e-9]);
            dl = diff(l);
            kap = DiffDriveAgent.curvature(G, (l(1:end-1) + l(2:end)) / 2);
            vlim = vb ./ (1 + obj.track * kap / 2);
            if isfinite(obj.maxNormalAccel), vlim = min(vlim, sqrt(obj.maxNormalAccel ./ max(kap, eps))); end
            v = [0; min(vlim(1:end-1), vlim(2:end)); 0];
            for k = 1:numel(dl), v(k + 1) = min(v(k + 1), sqrt(v(k)^2 + 2 * ub * dl(k))); end
            for k = numel(dl):-1:1, v(k) = min(v(k), sqrt(v(k + 1)^2 + 2 * ub * dl(k))); end
            P = SpeedProfile.table(l, v);
        end
    end

    methods (Static, Access = private)
        function G = geometry(pts, radius)
            % Arc lengths s, unit directions D, and the rounded corner at each inner point:
            % half-width h(k) and curvature bound kap(k) = |D(k) - D(k-1)| / (2 h(k))
            seg = vecnorm(diff(pts, 1, 1), 2, 2);
            keep = [true; seg > 1e-9];
            pts = pts(keep, :);
            if size(pts, 1) < 2, pts = [pts; pts + [eps 0]]; end
            seg = vecnorm(diff(pts, 1, 1), 2, 2);
            n = size(pts, 1);
            G.pts = pts;
            G.s = [0; cumsum(seg)];
            G.D = diff(pts, 1, 1) ./ seg;
            G.h = zeros(n, 1); G.kap = zeros(n, 1);
            if n > 2
                k = (2:n-1)';
                G.h(k) = min([seg(k - 1) / 2, seg(k) / 2, repmat(radius, n - 2, 1)], [], 2);
                G.kap(k) = vecnorm(G.D(k, :) - G.D(k - 1, :), 2, 2) ./ (2 * G.h(k));
            end
        end

        function p = curveAt(G, l)
            % Point at path distance l along the rounded path (a quadratic Bezier over each corner)
            l = min(max(l, 0), G.s(end));
            k = min(find(G.s <= l, 1, 'last'), numel(G.s) - 1);
            j = 0;
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

        function kap = curvature(G, l)
            % Curvature bound at path distances l (0 outside the rounded corners)
            kap = zeros(size(l));
            for j = find(G.h > 0)'
                in = abs(l - G.s(j)) < G.h(j);
                kap(in) = max(kap(in), G.kap(j));
            end
        end
    end
end
