classdef Battery < handle
    % An agent's battery: capacity and power model, and the energy drawn so far.
    %   P = (P_cpu + P_comp + P0 + alphaVel |v| + gammaAcc |a|^2) / eta + c2 |v|^2
    % with the processor and communication loads of a moving or idle agent. In the notation of
    % the energy-aware paper (Sec. 3.3) this is P_m + c1 u^2 + c2 v^2 + c3 v with P_I = idlePower,
    % P_M = movePower, c1 = gammaAcc/eta and c3 = alphaVel/eta; c2 = 0 by default (Ting-You
    % Chung's model). The battery itself delivers P + beta P^2 with beta = R0/Voc^2 (its internal
    % resistance R0, to first order; R0 = 0 by default: no losses). Every agent holds
    % one (Agent.battery); different platforms can use different parameters (AgentSpec
    % batteryWh, or set them on agent.battery). The battery is reported (SOC); an agent stops
    % when it is empty only if its stopWhenEmpty is true. A ChargingStation recharges the
    % batteries of the agents resting at it (recharge). idlePower, movePower and transitEnergy
    % predict the energy of a plan, for battery-aware policies.
    properties
        capacityWh = 26.0                 % capacity [Wh]
        P0 = 5.82                         % base motion power [W]
        alphaVel = 0.02 * 10 * 9.81       % rolling resistance: power per unit speed
        gammaAcc = 0.5                    % power per unit squared acceleration
        cpuMove = 5.5                     % processor power while moving / idle [W]
        cpuIdle = 4.0
        commActive = 11.7                 % communication power while moving / idle [W]
        commStandby = 11.0
        eta = 0.90                        % converter efficiency
        c2 = 0                            % power per unit squared speed [W s^2/m^2] (full model)
        R0 = 0                            % internal resistance [Ohm] (0: no battery losses)
        Voc = 14.4                        % open-circuit voltage [V]
        energyUsed = 0                    % energy drawn so far [J]
    end

    properties (Dependent)
        soc                               % state of charge in [0, 1]
        percentage                        % state of charge in percent, at least 0
        remaining                         % energy left [J] (negative once overdrawn)
        capacity                          % capacity [J]
        c1                                % power per unit squared acceleration, gammaAcc/eta
        c3                                % power per unit speed, alphaVel/eta
        beta                              % loss coefficient R0/Voc^2 [1/W]
    end

    methods
        function obj = Battery(opts)
            arguments
                opts.CapacityWh double = 26.0
            end
            obj.capacityWh = opts.CapacityWh;
        end

        function drain(obj, speed, accel, moving, dt)
            % Energy drawn over dt at the given speed and |acceleration|
            if moving
                pLoad = obj.cpuMove + obj.commActive;
            else
                pLoad = obj.cpuIdle + obj.commStandby;
            end
            pMotion = obj.P0 + obj.alphaVel * speed + obj.gammaAcc * accel^2;
            P = (pLoad + pMotion) / obj.eta + obj.c2 * speed^2;
            obj.energyUsed = obj.energyUsed + (P + obj.beta * P^2) * dt;
        end

        function recharge(obj, power, dt)
            % Charging at power [W] over dt, up to full
            obj.energyUsed = max(0, obj.energyUsed - power * dt);
        end

        function P = idlePower(obj)
            % Power drawn at rest [W]
            P = (obj.cpuIdle + obj.commStandby + obj.P0) / obj.eta;
        end

        function P = movePower(obj)
            % Power drawn while moving, without the speed and acceleration terms [W]
            P = (obj.cpuMove + obj.commActive + obj.P0) / obj.eta;
        end

        function E = transitEnergy(obj, rho, y, Ja, Jv)
            % Energy [J] of a transit of length y in time rho with int |a|^2 dt = Ja and
            % int |v|^2 dt = Jv (default 0): the speed term integrates to alphaVel * y whatever
            % the speed profile. Without battery losses (see planEnergy for them)
            if nargin < 5, Jv = 0; end
            E = obj.movePower() * rho + (obj.alphaVel * y + obj.gammaAcc * Ja) / obj.eta + obj.c2 * Jv;
        end

        function E = planEnergy(obj, P)
            % Battery energy [J] of a transit along speed profile P (SpeedProfile), losses
            % included: int (p + beta p^2) dt with p = P_M + c1 u^2 + c2 v^2 + c3 v. Without
            % losses it is P_M rho + c3 y + L, L the motion loss (energy-aware paper, Sec. 3.3)
            p = @(v, u) obj.movePower() + obj.c1 * u.^2 + obj.c2 * v.^2 + obj.c3 * v;
            if obj.beta == 0
                [Eu, Ev] = SpeedProfile.integrals(P);
                E = obj.movePower() * P.rho + obj.c3 * P.y + obj.c1 * Eu + obj.c2 * Ev;
            else
                E = SpeedProfile.quad(P, @(v, u) p(v, u) + obj.beta * p(v, u).^2);
            end
        end

        function E = idleEnergy(obj, tau)
            % Battery energy [J] of idling for tau, losses included
            Pi = obj.idlePower();
            E = (Pi + obj.beta * Pi^2) * tau;
        end

        function E = excessEnergy(obj, P)
            % The excess E^ex of a transit along P over idling for the same time (battery
            % energy, losses included): only this part depends on the plan
            E = obj.planEnergy(P) - obj.idleEnergy(P.rho);
        end

        function reset(obj)
            obj.energyUsed = 0;
        end

        function s = get.soc(obj)
            s = 1 - obj.energyUsed / (obj.capacityWh * 3600);
        end

        function p = get.percentage(obj)
            p = max(0, 100 * obj.soc);
        end

        function E = get.remaining(obj)
            E = obj.capacityWh * 3600 - obj.energyUsed;
        end

        function E = get.capacity(obj)
            E = obj.capacityWh * 3600;
        end

        function c = get.c1(obj), c = obj.gammaAcc / obj.eta; end
        function c = get.c3(obj), c = obj.alphaVel / obj.eta; end
        function b = get.beta(obj), b = obj.R0 / obj.Voc^2; end
    end
end
