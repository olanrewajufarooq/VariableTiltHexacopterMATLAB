classdef (Abstract) BaseAllocator < handle
    properties
        kThrust
        kDragToThrust
        armLength
        motorDirections
        minMotorSpeed
        maxMotorSpeed
        tiltLimits
        motorAngles
        motorPositions
    end

    methods
        function obj = BaseAllocator(cfg)
            obj.kThrust = cfg.act.kThrust;
            obj.kDragToThrust = cfg.act.kDragToThrust;
            obj.armLength = cfg.act.armLength;
            obj.motorDirections = cfg.act.motorDirections(:);
            obj.minMotorSpeed = cfg.act.minMotorSpeed;
            obj.maxMotorSpeed = cfg.act.maxMotorSpeed;
            obj.tiltLimits = cfg.act.tiltLimits(:);

            obj.motorAngles = (0:5) * (pi/3);
            obj.motorPositions = obj.armLength * [cos(obj.motorAngles); sin(obj.motorAngles); zeros(1,6)];
        end

        function speeds = thrustToMotorSpeeds(obj, thrusts)
            thrusts = max(thrusts(:), 0);
            speeds = sqrt(thrusts / obj.kThrust);
            speeds = max(obj.minMotorSpeed, min(obj.maxMotorSpeed, speeds));
        end

        function thrusts = motorSpeedsToThrust(obj, speeds)
            speeds = speeds(:);
            thrusts = obj.kThrust * speeds.^2;
        end
    end

    methods (Abstract)
        [motorSpeeds, tiltAngles] = allocate(obj, wrench)
        A = getAllocationMatrix(obj, tiltAngles)
    end

    methods (Static)
        function u = buildThrustDirection(psi, alpha)
            u = [sin(psi) * sin(alpha); -cos(psi) * sin(alpha); cos(alpha)];
        end
    end
end
