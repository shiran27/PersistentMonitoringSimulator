classdef Battery1RC < Battery
    % A battery with a one-RC equivalent circuit (energy-aware paper, Sec. 7.1), for DiffDriveAgent:
    % open-circuit voltage V(SOC) = Voc + kV (SOC - 1/2), series resistance R0 and one RC pair
    % (R1, C1). drainPower(P, dt) draws P at the terminals: the cell current I solves
    % (V - V1 - R0 I) I = P, the state of charge falls by I dt/Q and V1 follows the RC pair. The
    % capacity is capacityWh at the nominal voltage Voc (Q = 3600 capacityWh/Voc coulombs), so the
    % stored energy is E(SOC) = Q int_0^SOC V(s) ds and E(1) = 3600 capacityWh.
    %
    % The agent knows its charge only by coulomb counting with a current sensor that has a bias
    % and noise (socEstimate, reset to 1 at the end of each charge). energyUsed, and so soc,
    % remaining and every policy's view of the battery, follows the estimate; socTrue and
    % trueRemaining are the cells.
    %
    % A ChargingStation calls recharge each step at its power P_c. chargeMethod "CC-CV" (default):
    % a constant current P_c/Vmax until the terminal voltage reaches Vmax, then Vmax held while
    % the current tapers, until it falls below cutoff Q/3600 (C/20); "constant power": P_c into
    % the terminals until full. While it charges, the charger also feeds the agent's loads.
    %
    % The plan model is the parent's (planEnergy, idleDraw: P + beta P^2 with beta = R0/Voc^2,
    % the first-order loss of R0 at the nominal voltage). What it leaves out (R1, V(SOC), the
    % estimate) is what a physical agent's margins absorb.
    properties
        kV = 3.2                       % slope of V(SOC) [V] (12.8 to 16.0 V around Voc = 14.4 V)
        R1 = 0.05                      % RC pair [Ohm] and [F]
        C1 = 1000
        Vmax = 16.4                    % charge voltage limit [V]
        cutoff = 0.05                  % CV ends below cutoff Q/3600 [A] (C/20)
        chargeMethod string = "CC-CV"  % or "constant power"
        sensorBias = 0.01              % current sensor bias [A] and noise (standard deviation) [A]
        sensorNoise = 0.05
        sensorSeed = 1                 % seed of the sensor noise (per agent: DiffDriveAgent sets it)
        socTrue = 1                    % the cells' state of charge
        socEstimate = 1                % the agent's estimate (coulomb counting)
        V1 = 0                         % voltage across the RC pair [V]
        energyDrawn = 0                % energy the cells delivered so far, net of charging [J]
        full = true                    % the last charge ended (CV cutoff or SOC 1)
    end

    properties (Access = private)
        rng                            % RandStream of the sensor noise
        charging = false               % a station charged in the last step (it feeds the loads)
        pendingLoad = 0                % load energy drawn while charging, not yet supplied [J]
    end

    properties (Dependent)
        coulombs                       % Q [C]
        trueRemaining                  % E(socTrue) [J]
    end

    methods
        function obj = Battery1RC(opts)
            arguments
                opts.CapacityWh double = 26.0
            end
            obj@Battery(CapacityWh=opts.CapacityWh);
            obj.R0 = 0.1;
            obj.reset();
        end

        function drain(obj, speed, accel, moving, dt)
            % The parent's power model, drawn through the circuit
            if moving
                pLoad = obj.cpuMove + obj.commActive;
            else
                pLoad = obj.cpuIdle + obj.commStandby;
            end
            P = (pLoad + obj.P0 + obj.alphaVel * speed + obj.gammaAcc * accel^2) / obj.eta + obj.c2 * speed^2;
            obj.drainPower(P, dt);
        end

        function drainPower(obj, P, dt)
            % Draw P [W] at the terminals for dt (while charging, the charger supplies it)
            if obj.charging
                obj.pendingLoad = obj.pendingLoad + P * dt;
                obj.charging = false;
                return;
            end
            obj.full = false;
            obj.flow(obj.cellCurrent(P), dt);
        end

        function recharge(obj, power, dt)
            % One step of charging at the station's power [W]; the loads drawn since the last
            % step come first
            pLoad = obj.pendingLoad / dt;
            obj.pendingLoad = 0;
            obj.charging = true;
            if obj.full, return; end
            avail = max(power - pLoad, 0);
            if obj.chargeMethod == "constant power"
                I = obj.cellCurrent(-avail);
                if obj.socTrue - I * dt / obj.coulombs >= 1
                    I = -(1 - obj.socTrue) * obj.coulombs / dt;
                    obj.flow(I, dt);
                    obj.endCharge();
                    return;
                end
            else
                Vo = obj.ocv(obj.socTrue) - obj.V1;
                Ic = max(power / obj.Vmax - pLoad / obj.Vmax, 0);     % CC: the charger's current, less the loads'
                if Vo + obj.R0 * Ic > obj.Vmax                        % CV: hold the terminals at Vmax
                    Ic = max((obj.Vmax - Vo) / max(obj.R0, eps), 0);
                end
                if Ic < obj.cutoff * obj.coulombs / 3600 || obj.socTrue >= 1
                    obj.endCharge();
                    return;
                end
                I = -Ic;
            end
            obj.flow(I, dt);
        end

        function swap(obj)
            % A battery swap: a full battery at rest, and a full estimate
            obj.socTrue = 1; obj.V1 = 0;
            obj.endCharge();
        end

        function V = ocv(obj, s)
            % Open-circuit voltage at state of charge s
            V = obj.Voc + obj.kV * (s - 0.5);
        end

        function E = storedEnergy(obj, s)
            % E(s) = Q int_0^s V [J]
            E = obj.coulombs * ((obj.Voc - obj.kV / 2) * s + obj.kV * s.^2 / 2);
        end

        function V = terminalVoltage(obj, I)
            % Terminal voltage at cell current I (positive: discharging)
            V = obj.ocv(obj.socTrue) - obj.V1 - obj.R0 * I;
        end

        function reset(obj)
            reset@Battery(obj);
            obj.socTrue = 1; obj.socEstimate = 1; obj.V1 = 0; obj.energyDrawn = 0; obj.full = true;
            obj.charging = false; obj.pendingLoad = 0;
            obj.rng = RandStream('mt19937ar', 'Seed', obj.sensorSeed);
        end

        function set.sensorSeed(obj, s)
            obj.sensorSeed = s;
            obj.rng = RandStream('mt19937ar', 'Seed', s);   %#ok<MCSUP>
        end

        function q = get.coulombs(obj), q = obj.capacityWh * 3600 / obj.Voc; end
        function E = get.trueRemaining(obj), E = obj.storedEnergy(obj.socTrue); end
    end

    methods (Access = private)
        function I = cellCurrent(obj, P)
            % The cell current that delivers P at the terminals (negative P: charging);
            % past the circuit's maximum power, the current at that maximum
            Vo = obj.ocv(obj.socTrue) - obj.V1;
            if obj.R0 <= 0, I = P / Vo; return; end
            disc = Vo^2 - 4 * obj.R0 * P;
            I = (Vo - sqrt(max(disc, 0))) / (2 * obj.R0);
        end

        function flow(obj, I, dt)
            % Cell current I for dt: the cells, the RC pair (exact for a constant current), and
            % the estimate from the sensor's reading
            obj.energyDrawn = obj.energyDrawn + obj.ocv(obj.socTrue) * I * dt;
            obj.socTrue = obj.socTrue - I * dt / obj.coulombs;
            a = exp(-dt / (obj.R1 * obj.C1));
            obj.V1 = a * obj.V1 + (1 - a) * obj.R1 * I;
            Im = I + obj.sensorBias + obj.sensorNoise * randn(obj.rng);
            obj.socEstimate = min(obj.socEstimate - Im * dt / obj.coulombs, 1);
            obj.sync();
        end

        function endCharge(obj)
            % The charge ended: the agent resets its estimate to full
            obj.full = true;
            obj.socEstimate = 1;
            obj.sync();
        end

        function sync(obj)
            % energyUsed follows the estimate
            obj.energyUsed = obj.storedEnergy(1) - obj.storedEnergy(obj.socEstimate);
        end
    end
end
