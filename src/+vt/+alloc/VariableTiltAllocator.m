classdef VariableTiltAllocator < vt.alloc.BaseAllocator
    properties
        A_static
        A_pinv
    end

    methods
        function obj = VariableTiltAllocator(cfg)
            obj@vt.alloc.BaseAllocator(cfg);
            obj.A_static = obj.computeStaticMatrix();
            obj.A_pinv = pinv(obj.A_static);
        end

        function [motorSpeeds, tiltAngles] = allocate(obj, wrench)
            comps = obj.A_pinv * wrench;
            thrusts = zeros(6,1);
            tiltAngles = zeros(6,1);
            for i = 1:6
                Fv = comps(2*i-1);
                Fl = comps(2*i);
                thrusts(i) = sqrt(Fv^2 + Fl^2);
                tiltAngles(i) = atan2(Fl, Fv);
            end

            tiltAngles = min(max(tiltAngles, obj.tiltLimits(1)), obj.tiltLimits(2));
            motorSpeeds = obj.thrustToMotorSpeeds(thrusts);
        end

        function A = getAllocationMatrix(obj, tiltAngles)
            if nargin < 2 || isempty(tiltAngles)
                A = obj.A_static;
                return;
            end

            A = zeros(6,6);
            for i = 1:6
                psi = obj.motorAngles(i);
                alpha = tiltAngles(i);
                u = vt.alloc.BaseAllocator.buildThrustDirection(psi, alpha);
                xi = obj.motorPositions(:,i);
                tau = cross(xi, u);
                A(:,i) = [tau; u];
            end
        end
    end

    methods (Access = private)
        function A = computeStaticMatrix(obj)
            A = zeros(6, 12);
            for i = 1:6
                psi = obj.motorAngles(i);

                % vertical component (Fv)
                A(1, 2*i-1) = 0;                 % Tx (no drag torque)
                A(2, 2*i-1) = obj.armLength * sin(psi); % Ty
                A(3, 2*i-1) = -obj.armLength * cos(psi);% Tz
                A(6, 2*i-1) = 1;                 % Fz

                % lateral component (Fl)
                A(2, 2*i)   = obj.armLength * cos(psi); % Ty
                A(3, 2*i)   = obj.armLength * sin(psi); % Tz
                A(4, 2*i)   = sin(psi);                 % Fx
                A(5, 2*i)   = -cos(psi);                % Fy
            end
        end
    end
end
