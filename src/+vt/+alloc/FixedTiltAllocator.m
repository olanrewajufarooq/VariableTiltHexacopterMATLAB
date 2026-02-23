classdef FixedTiltAllocator < vt.alloc.BaseAllocator
    properties
        fixedTiltAngle
        A
        Ainv
        tiltAngles
    end

    methods
        function obj = FixedTiltAllocator(cfg)
            obj@vt.alloc.BaseAllocator(cfg);
            obj.fixedTiltAngle = cfg.act.fixedTiltAngle;
            obj.tiltAngles = obj.fixedTiltAngle * [1 -1 1 -1 1 -1]';
            obj.A = obj.computeAllocationMatrix(obj.tiltAngles);
            obj.Ainv = inv(obj.A);
        end

        function [motorSpeeds, tiltAngles] = allocate(obj, wrench)
            thrusts = obj.Ainv * wrench;
            thrusts = max(thrusts, 0);
            motorSpeeds = obj.thrustToMotorSpeeds(thrusts);
            tiltAngles = obj.tiltAngles;
        end

        function A = getAllocationMatrix(obj, tiltAngles)
            if nargin < 2 || isempty(tiltAngles)
                tiltAngles = obj.tiltAngles;
            else
                tiltAngles = tiltAngles(:);
            end
            A = obj.computeAllocationMatrix(tiltAngles);
        end
    end

    methods (Access = private)
        function A = computeAllocationMatrix(obj, tiltAngles)
            A = zeros(6,6);
            gamma = obj.kDragToThrust;
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
end
