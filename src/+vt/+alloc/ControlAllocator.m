classdef ControlAllocator < handle
    properties
        impl
    end

    methods
        function obj = ControlAllocator(cfg)
            method = cfg.act.method;
            switch lower(method)
                case 'fixed_tilt'
                    obj.impl = vt.alloc.FixedTiltAllocator(cfg);
                case 'variable_tilt'
                    obj.impl = vt.alloc.VariableTiltAllocator(cfg);
                otherwise
                    error('Unknown allocation method: %s', method);
            end
        end

        function [motorSpeeds, tiltAngles] = allocate(obj, wrench)
            [motorSpeeds, tiltAngles] = obj.impl.allocate(wrench);
        end

        function A = getAllocationMatrix(obj, tiltAngles)
            A = obj.impl.getAllocationMatrix(tiltAngles);
        end

        function thrusts = motorSpeedsToThrust(obj, speeds)
            thrusts = obj.impl.motorSpeedsToThrust(speeds);
        end
    end
end
