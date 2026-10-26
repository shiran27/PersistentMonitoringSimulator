classdef SpeedProfile
    % Rest-to-rest speed profiles along an edge of length y crossed in time rho, shared by the
    % agent types (AbstractAgent follows them exactly, PhysicalAgent tracks them)
    % and by the energy-aware RHC policies. A profile family may be bounded by a speed limit vb
    % and an acceleration limit ub (Inf = no limit):
    %   "FO-0"      constant speed y/rho (a first-order agent; the jumps at both ends are
    %               impulses, so the energy int u^2 dt is undefined)
    %   "SO"        the minimum-energy profile (min int u^2 dt). Without limits v(t) =
    %               6y t (rho - t)/rho^3, E = 12 y^2/rho^3 (Welikala and Cassandras, arXiv
    %               2102.12963, Theorem 3). A reached limit changes the shape (App. D,
    %               generalized to both limits):
    %                 speed:  u falls linearly to 0 as v reaches vb, cruise, mirror image (u stays
    %                         continuous, unlike the paper's SO-V, which uses more energy)
    %                 accel:  u = ub, then linear from ub to -ub, then -ub (the paper's SO-A)
    %                 both:   u = ub, then linear down to 0 as v reaches vb, cruise, mirror image
    %   "FO-3"      the minimum-energy trapezoid (accelerate, cruise, brake).
    %               Without limits the cruise speed is 3y/(2 rho), E = 27 y^2/(2 rho^3)
    %               (Theorem 5); with limits it is clamped to what they allow (App. E)
    % The names follow the paper (FO-n: approximately first-order agents, n = 0..3; FO-1 and
    % FO-2 are not implemented). The older names "constant" and "FO" still work (canonical).
    % Limits make transits shorter than minTransit infeasible.
    % Full power model (energy-aware paper, Sec. 3.3-4): an agent drawing
    % P_m + c1 u^2 + c2 v^2 + c3 v minimizes the motion loss int (c1 u^2 + c2 v^2) dt. With
    % c2 > 0, shape(..., nu) with nu = sqrt(c2/c1) gives those profiles: SO is hyperbolic,
    % v(t) = A (cosh th - cosh(nu (t - rho/2))) with th = nu rho/2 (Theorem 1); a speed limit adds
    % hyperbolic arcs that reach vb with u -> 0 and a cruise (Theorem 2); an acceleration limit
    % holds u = ub at both ends around a hyperbolic middle arc; and FO-3 accelerates for the root
    % tau* in (0, rho/3] of c2 tau^2 (2 tau - rho) = 3 c1 (3 tau - rho) (Theorem 3). nu = 0 (the
    % default) gives the profiles above. loss and integrals evaluate the motion loss.
    % A profile can also be a table (kind "table", from SpeedProfile.table): speeds at given
    % distances, with constant acceleration in between. PhysicalAgent uses one for its paths,
    % whose corners limit the speed.
    properties (Constant)
        names = ["FO-0", "SO", "FO-3"]
    end

    methods (Static)
        function P = shape(name, y, rho, vb, ub, nu)
            % The profile <name> on an edge of length y crossed in time rho (at least
            % minTransit). P.kind: "cubic" (unconstrained SO), "sat" (SO with u at its limit, no
            % cruise), or "arc": accelerate at a for s, ramp u from a to 0 over d, cruise at v
            % (FO-0: s = d = 0; SO with the speed limit reached; FO-3 trapezoids: d = 0). With
            % nu = sqrt(c2/c1) > 0, the least motion loss profiles of the full power model:
            % "hyp", "hypV", "hypA", "hypVA" (SO) or an "arc" trapezoid (FO-3)
            if nargin < 4, vb = Inf; end
            if nargin < 5, ub = Inf; end
            if nargin < 6, nu = 0; end
            name = SpeedProfile.canonical(name);
            if nu > 0 && nu * rho / 2 >= 1e-4 && name ~= "FO-0"   % below: the c2 = 0 shapes to 1e-9
                P = SpeedProfile.fullModelShape(name, y, rho, vb, ub, nu);
                return
            end
            P = struct('name', name, 'y', y, 'rho', rho, 'kind', "arc", 'a', 0, 's', 0, 'd', 0, 'v', 0);
            switch name
                case "FO-0"
                    P.v = y / rho;
                case "SO"
                    [~, regime, p] = SpeedProfile.soEnergy(y, rho, vb, ub);
                    switch regime
                        case "none", P.kind = "cubic";
                        case "A",    P.kind = "sat"; P.a = ub; P.s = p;
                        case "V",    P.a = 2 * vb / p; P.d = p; P.v = vb;
                        case "VA",   P.a = ub; P.d = p; P.s = vb / ub - p / 2; P.v = vb;
                    end
                case "FO-3"
                    [~, vc] = SpeedProfile.fo3Energy(y, rho, vb, ub);
                    P.v = vc; P.a = vc^2 / (vc * rho - y); P.s = vc / P.a;
                otherwise
                    error('SpeedProfile:Name', 'Unknown speed profile "%s" (use %s)', name, ...
                        strjoin(SpeedProfile.names, ", "));
            end
        end

        function [l, v, u, e] = at(P, t)
            % Distance l, speed v, tangential acceleration u and energy int_0^t u^2 at time t in
            % [0, rho]; every profile but a table is symmetric about rho/2
            t = min(max(t, 0), P.rho);
            if P.kind == "table"
                k = min(find(P.t <= t, 1, 'last'), numel(P.t) - 1);
                dt = t - P.t(k);
                l = min(P.l(k) + P.v(k) * dt + P.u(k) * dt^2 / 2, P.y);
                v = max(P.v(k) + P.u(k) * dt, 0);
                u = P.u(k);
                e = P.e(k) + u^2 * dt;
            elseif t <= P.rho / 2
                [l, v, u, e] = SpeedProfile.firstHalf(P, t);
            else
                [l, v, u, e] = SpeedProfile.firstHalf(P, P.rho - t);
                [~, ~, ~, half] = SpeedProfile.firstHalf(P, P.rho / 2);
                l = P.y - l; u = -u; e = 2 * half - e;
            end
        end

        function P = table(l, v)
            % The profile through speeds v(k) at distances l(k) (rest to rest: v(1) = v(end) =
            % 0), with constant acceleration between consecutive distances
            l = l(:); v = v(:);
            dl = diff(l);
            dt = 2 * dl ./ max(v(1:end-1) + v(2:end), eps);
            u = (v(2:end).^2 - v(1:end-1).^2) ./ (2 * dl);
            t = [0; cumsum(dt)];
            P = struct('name', "table", 'y', l(end), 'rho', t(end), 'kind', "table", ...
                't', t, 'l', l, 'v', v, 'u', [u; 0], 'e', [0; cumsum(u.^2 .* dt)]);
        end

        function P = retime(P, rho)
            % A table profile slowed down to take rho (at least P.rho): the same path, with
            % speeds scaled by P.rho/rho and accelerations by (P.rho/rho)^2
            c = P.rho / rho;
            P.t = P.t / c; P.v = P.v * c; P.u = P.u * c^2; P.e = P.e * c^3; P.rho = rho;
        end

        function [vPeak, uPeak] = peaks(P)
            % Largest speed and |acceleration| of the profile (impulses not counted)
            if P.kind == "table", vPeak = max(P.v); uPeak = max(abs(P.u)); return; end
            [~, vPeak] = SpeedProfile.at(P, P.rho / 2);
            [~, ~, uPeak] = SpeedProfile.at(P, 0);
            uPeak = abs(uPeak);
        end

        function L = loss(name, y, rho, vb, ub, c1, c2)
            % The motion loss int (c1 u^2 + c2 v^2) dt of the least-loss profile <name> under
            % the full power model, for each transit time in rho (vectorized); FO-0's impulses
            % are not counted (its loss is c2 y^2/rho)
            L = zeros(size(rho));
            for k = 1:numel(rho)
                [Eu, Ev] = SpeedProfile.integrals(SpeedProfile.shape(name, y, rho(k), vb, ub, sqrt(c2 / c1)));
                L(k) = c1 * Eu + c2 * Ev;
            end
        end

        function [Eu, Ev] = integrals(P)
            % int u^2 dt and int v^2 dt over the whole profile (Gauss-Legendre on its smooth
            % pieces, exact to round-off; impulses not counted)
            [Eu, Ev] = SpeedProfile.quad(P, @(v, u) u.^2, @(v, u) v.^2);
        end

        function varargout = quad(P, varargin)
            % int_0^rho f(v, u) dt along P for each given f(v, u), e.g. a power model; every f
            % must be even in u (the profiles but tables are symmetric about rho/2)
            n = numel(varargin);
            varargout = num2cell(zeros(1, n));
            if P.kind == "table"
                [x, w] = SpeedProfile.gaussLegendre();
                for k = 1:numel(P.t) - 1
                    h = P.t(k + 1) - P.t(k);
                    if h <= 0, continue; end
                    v = P.v(k) + P.u(k) * h * (x + 1) / 2;
                    for i = 1:n, varargout{i} = varargout{i} + h / 2 * (w' * varargin{i}(v, P.u(k))); end
                end
                return
            end
            switch P.kind
                case {"sat", "hypA"}, b = P.s;
                case "arc",           b = [P.s, P.s + P.d];
                case "hypV",          b = P.t1;
                case "hypVA",         b = [P.s, P.t1];
                otherwise,            b = [];
            end
            b = unique([0, min(max(b, 0), P.rho / 2), P.rho / 2]);
            [x, w] = SpeedProfile.gaussLegendre();
            for k = 1:numel(b) - 1
                h = b(k + 1) - b(k);
                if h <= 0, continue; end
                t = b(k) + h * (x + 1) / 2;
                v = zeros(size(t)); u = v;
                for j = 1:numel(t), [~, v(j), u(j)] = SpeedProfile.firstHalf(P, t(j)); end
                for i = 1:n, varargout{i} = varargout{i} + h * (w' * varargin{i}(v, u)); end   % x2 halves
            end
        end

        function E = energy(name, y, rho, vb, ub)
            % Energy int u^2 dt over the whole edge, for each transit time in rho (vectorized);
            % NaN for FO-0 (undefined)
            if nargin < 4, vb = Inf; end
            if nargin < 5, ub = Inf; end
            switch SpeedProfile.canonical(name)
                case "SO",       E = SpeedProfile.soEnergy(y, rho, vb, ub);
                case "FO-3",     E = SpeedProfile.fo3Energy(y, rho, vb, ub);
                case "FO-0",     E = nan(size(rho));
                otherwise,       error('SpeedProfile:Name', 'Unknown speed profile "%s"', name);
            end
        end

        function r = minTransit(y, vb, ub)
            % Shortest transit time the limits allow (accelerate at ub, cruise at vb, brake).
            % Under a speed limit alone the energy grows without bound as rho approaches y/vb,
            % so 0.1% is added there.
            if isfinite(vb) && isfinite(ub)
                if y >= vb^2 / ub, r = y / vb + vb / ub; else, r = 2 * sqrt(y / ub); end
            elseif isfinite(vb)
                r = 1.001 * y / vb;
            elseif isfinite(ub)
                r = 2 * sqrt(y / ub);
            else
                r = 0;
            end
        end

        function rho = defaultTransit(name, y, vmax)
            % Transit time when no policy plans one: the shortest whose peak speed is vmax
            % (FO-0: y/vmax; SO and FO-3 peak at 3y/(2 rho): 1.5 y/vmax)
            if SpeedProfile.canonical(name) == "FO-0", rho = y / vmax; else, rho = 1.5 * y / vmax; end
        end

        function name = canonical(name)
            % The current name of a speed profile: the older names "constant" (also "constant
            % speed") and "FO" are FO-0 and FO-3 (renamed on 2026-10-22)
            name = string(name);
            name(lower(name) == "constant" | lower(name) == "constant speed") = "FO-0";
            name(name == "FO") = "FO-3";
        end
    end

    methods (Static, Access = private)
        function [E, regime, p] = soEnergy(y, rho, vb, ub)
            % Minimum energy of any rest-to-rest profile (a convex problem with a unique optimum)
            % and, for scalar rho, the regime ("none", "V", "A", "VA") and its parameter p:
            % t1 for "V", the saturation time s for "A", the ramp time d for "VA"
            E = 12 * y^2 ./ rho.^3;
            todo = 3 * y ./ (2 * rho) > vb | 6 * y ./ rho.^2 > ub;   % a limit would be broken
            regime = "none"; p = 0;
            if isfinite(vb)   % speed limit only: needs peak acceleration 2vb/t1 <= ub
                t1 = 3 * (vb * rho - y) / (2 * vb);
                k = todo & 3 * y ./ (2 * rho) > vb & t1 > 0 & 2 * vb ./ t1 <= ub * (1 + 1e-12);
                E(k) = 16 * vb^3 ./ (9 * (vb * rho(k) - y));
                todo = todo & ~k;
                if isscalar(rho) && k, regime = "V"; p = t1; end
            end
            if isfinite(ub)   % acceleration limit only: needs peak speed ub (s + m/4) <= vb
                s = (rho - sqrt(max(3 * rho.^2 - 12 * y / ub, 0))) / 2;
                k = todo & 6 * y ./ rho.^2 > ub & ub * (s + (rho - 2 * s) / 4) <= vb * (1 + 1e-12);
                E(k) = ub^2 * (rho(k) + 4 * s(k)) / 3;
                todo = todo & ~k;
                if isscalar(rho) && k, regime = "A"; p = s; end
            end
            if any(todo) && isfinite(vb) && isfinite(ub)   % both limits reached
                d = sqrt(max(12 * (vb * rho - y - vb^2 / ub) / ub, 0));
                E(todo) = 2 * ub * vb - ub^2 * d(todo) / 3;
                if isscalar(rho), regime = "VA"; p = d; end
            end
        end

        function [E, vc] = fo3Energy(y, rho, vb, ub)
            % Minimum energy of a trapezoid: the energy 2 vc^3/(vc rho - y) falls with the cruise
            % speed vc up to 3y/(2 rho), so vc is that value clamped to the limits: at most vb,
            % and at least the vc whose acceleration vc^2/(vc rho - y) is ub
            if isfinite(ub)
                vlo = rho * ub / 2 .* (1 - sqrt(max(1 - 4 * y ./ (ub * rho.^2), 0)));
            else
                vlo = 0;
            end
            vc = min(vb, max(vlo, 3 * y ./ (2 * rho)));
            E = 2 * vc.^3 ./ (vc .* rho - y);
        end

        function [l, v, u, e] = firstHalf(P, t)
            y = P.y; rho = P.rho;
            switch P.kind
                case "cubic"
                    q = t / rho;
                    l = y * (3 * q^2 - 2 * q^3);
                    v = 6 * y * t * (rho - t) / rho^3;
                    u = 6 * y * (rho - 2 * t) / rho^3;
                    e = 6 * y^2 / rho^6 * (rho^3 - (rho - 2 * t)^3);
                case "sat"      % u = a until s, then linear to -a at rho - s
                    a = P.a; s = P.s; m = rho - 2 * s;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    else
                        tau = t - s;
                        u = a * (1 - 2 * tau / m);
                        v = a * (s + tau - tau^2 / m);
                        l = a * s^2 / 2 + a * s * tau + a * (tau^2 / 2 - tau^3 / (3 * m));
                        e = a^2 * s + a^2 * m / 6 * (1 - (1 - 2 * tau / m)^3);
                    end
                case "arc"      % u = a until s, then linear to 0 over d, then cruise at v
                    a = P.a; s = P.s; d = P.d;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    elseif t <= s + d
                        tau = t - s;
                        u = a * (1 - tau / d);
                        v = a * s + a * (tau - tau^2 / (2 * d));
                        l = a * s^2 / 2 + a * s * tau + a * (tau^2 / 2 - tau^3 / (6 * d));
                        e = a^2 * s + a^2 * d / 3 * (1 - (1 - tau / d)^3);
                    else
                        u = 0; v = P.v;
                        l = a * s^2 / 2 + a * s * d + a * d^2 / 3 + P.v * (t - s - d);
                        e = a^2 * s + a^2 * d / 3;
                    end
                case "hyp"      % full power model, no active limit (Theorem 1)
                    nu = P.nu; A = P.A; th = P.th; r = nu * (t - rho / 2);
                    v = 2 * A * sinh((th + r) / 2) * sinh((th - r) / 2);
                    u = -A * nu * sinh(r);
                    l = A / nu * ((r + th) * 2 * sinh(th / 2)^2 - SpeedProfile.sm(r) - SpeedProfile.sm(th));
                    e = A^2 * nu * (SpeedProfile.sm(2 * r) + SpeedProfile.sm(2 * th)) / 4;
                case "hypV"     % hyperbolic arc that reaches v = vb with u -> 0 at t1, then cruise
                    nu = P.nu; B = P.B; x = P.x; t1 = P.t1;
                    w = nu * (min(t, t1) - t1);
                    v = P.v + 2 * B * sinh(w / 2)^2;
                    u = B * nu * sinh(w);
                    l = P.v * min(t, t1) + B / nu * (SpeedProfile.sm(w) + SpeedProfile.sm(x)) + P.v * max(t - t1, 0);
                    e = B^2 * nu * (SpeedProfile.sm(2 * w) + SpeedProfile.sm(2 * x)) / 4;
                case "hypA"     % u = a until s, then a hyperbolic arc from a to -a
                    nu = P.nu; a = P.a; s = P.s; z = nu * P.h; C = P.C;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    else
                        r = nu * (t - rho / 2);
                        v = a * s + 2 * C / nu * sinh((z + r) / 2) * sinh((z - r) / 2);
                        u = -C * sinh(r);
                        l = a * s^2 / 2 + a * s * (t - s) + ...
                            C / nu^2 * ((r + z) * 2 * sinh(z / 2)^2 - SpeedProfile.sm(r) - SpeedProfile.sm(z));
                        e = a^2 * s + C^2 * (SpeedProfile.sm(2 * r) + SpeedProfile.sm(2 * z)) / (4 * nu);
                    end
                case "hypVA"    % u = a until s, a hyperbolic arc to v = vb with u -> 0 at t1, cruise
                    nu = P.nu; a = P.a; s = P.s; B = P.B; x = P.x; t1 = P.t1;
                    if t <= s
                        u = a; v = a * t; l = a * t^2 / 2; e = a^2 * t;
                    else
                        w = nu * (min(t, t1) - t1);
                        v = P.v + 2 * B * sinh(w / 2)^2;
                        u = B * nu * sinh(w);
                        l = a * s^2 / 2 + P.v * (min(t, t1) - s) + B / nu * (SpeedProfile.sm(w) + SpeedProfile.sm(x)) ...
                            + P.v * max(t - t1, 0);
                        e = a^2 * s + B^2 * nu * (SpeedProfile.sm(2 * w) + SpeedProfile.sm(2 * x)) / 4;
                    end
            end
        end

        function P = fullModelShape(name, y, rho, vb, ub, nu)
            % The least motion loss profile <name> ("SO" or "FO-3") under the full power model
            % with nu = sqrt(c2/c1) > 0 (energy-aware paper, Theorems 1-3). SO: the free profile
            % if it keeps both limits, else the speed-limited, acceleration-limited or doubly
            % limited one, whichever keeps both (a convex problem: that one is optimal)
            P = struct('name', name, 'y', y, 'rho', rho, 'kind', "arc", 'a', 0, 's', 0, 'd', 0, 'v', 0, 'nu', nu);
            tol = 1 + 1e-12;
            if name == "FO-3"   % trapezoid: tau* clamped to what the limits allow
                g = @(tau) nu^2 * tau.^2 .* (2 * tau - rho) - 9 * tau + 3 * rho;
                tau = fzero(g, [0, rho / 3]);
                hi = rho / 2; lo = 0;
                if isfinite(vb), hi = min(hi, rho - y / vb); end
                if isfinite(ub), lo = (rho - sqrt(max(rho^2 - 4 * y / ub, 0))) / 2; end
                tau = min(max(tau, lo), hi);
                P.v = y / (rho - tau); P.a = P.v / tau; P.s = tau;
                return
            end
            if rho <= SpeedProfile.minTransit(y, vb, ub) * (1 + 1e-9) && isfinite(ub)   % time-optimal
                if isfinite(vb) && y >= vb^2 / ub
                    P.a = ub; P.s = vb / ub; P.v = vb;                  % accelerate, cruise, brake
                else
                    P.kind = "sat"; P.a = ub; P.s = rho / 2;            % accelerate, brake
                end
                return
            end
            th = nu * rho / 2;
            A = y * th / (rho * (2 * th * sinh(th / 2)^2 - SpeedProfile.sm(th)));   % th cosh th - sinh th
            if 2 * A * sinh(th / 2)^2 <= vb * tol && A * nu * sinh(th) <= ub * tol
                P.kind = "hyp"; P.A = A; P.th = th;
                return
            end
            if 2 * A * sinh(th / 2)^2 > vb * tol        % speed limit: arcs t1 = x/nu, then cruise
                rhs = nu * (vb * rho - y) / (2 * vb);   % (sinh x - x)/(cosh x - 1) = rhs
                if rhs > 0 && rhs < 1
                    x = SpeedProfile.rootUp(@(x) SpeedProfile.sm(x) ./ (2 * sinh(x / 2).^2) - rhs, 1);
                    if 2 * x / nu <= rho * tol && vb * nu / tanh(x / 2) <= ub * tol
                        P.kind = "hypV"; P.B = -vb / (2 * sinh(x / 2)^2); P.x = x; P.t1 = x / nu; P.v = vb;
                        return
                    end
                end
            end
            if A * nu * sinh(th) > ub * tol && y <= ub * rho^2 / 4 * tol   % acceleration limit
                zc = @(z) (2 * z .* sinh(z / 2).^2 - SpeedProfile.sm(z)) ./ sinh(z);   % z coth z - 1
                dist = @(h) ub * (rho^2 / 4 - h.^2) + 2 * ub / nu^2 * zc(nu * h);
                h = SpeedProfile.safeRoot(@(h) dist(h) - y, 1e-12 * rho, rho / 2);
                s = rho / 2 - h; C = ub / sinh(nu * h);
                if ub * s + 2 * C / nu * sinh(nu * h / 2)^2 <= vb * tol
                    P.kind = "hypA"; P.a = ub; P.s = s; P.h = h; P.C = C;
                    return
                end
            end
            % both limits: u = ub until s, arc to vb with u -> 0 at t1 = s + x/nu, cruise
            s1 = @(x) vb / ub - tanh(x / 2) / nu;
            t1 = @(x) s1(x) + x / nu;
            xmax = 700;
            if nu * vb / ub < 1, xmax = 2 * atanh(nu * vb / ub); end
            if t1(xmax) > rho / 2, xmax = SpeedProfile.safeRoot(@(x) t1(x) - rho / 2, 0, xmax); end
            dist = @(x) ub * s1(x)^2 + 2 * vb * x / nu - 2 * ub / (nu^2 * sinh(x)) * SpeedProfile.sm(x) ...
                + vb * (rho - 2 * t1(x));
            x = SpeedProfile.safeRoot(@(x) dist(x) - y, 1e-9 * min(xmax, 1), xmax);
            P.kind = "hypVA"; P.a = ub; P.s = s1(x); P.B = -ub / (nu * sinh(x)); P.x = x; P.t1 = t1(x); P.v = vb;
        end

        function x = safeRoot(f, a, b)
            % The root of f in [a, b]; the endpoint nearer to it when round-off leaves no sign
            % change (a profile at the edge of its regime)
            fa = f(a); fb = f(b);
            if fa * fb <= 0
                x = fzero(f, [a, b]);
            elseif abs(fa) <= abs(fb)
                x = a;
            else
                x = b;
            end
        end

        function x = rootUp(f, x0)
            % The root of an increasing f with f(0+) < 0, bracketed by doubling from x0
            hi = x0;
            while f(hi) <= 0 && hi < 700, hi = 2 * hi; end
            x = fzero(f, [1e-12, min(hi, 700)]);
        end

        function f = sm(x)
            % sinh(x) - x without cancellation for small |x| (series to x^13)
            f = sinh(x) - x;
            k = abs(x) < 0.5;
            if any(k(:))
                z = x(k); z2 = z.^2;
                f(k) = z.^3 / 6 .* (1 + z2 / 20 .* (1 + z2 / 42 .* (1 + z2 / 72 .* (1 + z2 / 110 .* (1 + z2 / 156)))));
            end
        end

        function [x, w] = gaussLegendre()
            % 20-point Gauss-Legendre nodes and weights on [-1, 1] (Golub-Welsch)
            persistent X W
            if isempty(X)
                k = (1:19)';
                b = k ./ sqrt(4 * k.^2 - 1);
                [V, D] = eig(diag(b, 1) + diag(b, -1));
                [X, i] = sort(diag(D)); W = 2 * V(1, i)'.^2;
            end
            x = X; w = W;
        end
    end
end
