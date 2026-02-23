classdef Config < handle
    %CONFIG Manages simulation configuration parameters
    %   Use chainable methods to set up the simulation.
    %   Example: cfg = vt.config.Config().setTrajectory('circle').setController('PD');
    
    properties
        vehicle = struct()
        act = struct()
        sim = struct()
        traj = struct()
        controller = struct()
        viz = struct()
    end
    
    methods
        function obj = Config()
            % Constructor: Initialize with default (baseline) parameters
            obj.initVehicle();
            obj.initActuation();
            obj.initSimulation();
            obj.initVisualization();
            
            % Default trajectory and controller (can be overridden)
            obj.setTrajectory('circle');
            obj.setController('PD');
        end
        
        %% Setters (Fluent Interface)
        
        function obj = setTrajectory(obj, name)
            %SETTRAJECTORY Sets trajectory parameters
            %   name: 'circle', 'hover', 'square', 'infinity', 'takeoffland'
            
            obj.traj.name = name;
            
            % Common defaults
            obj.traj.altitude = 5;
            obj.traj.hoverFrac = 0.1;
            obj.traj.squareExponent = 4;
            
            switch lower(name)
                case 'circle'
                    obj.traj.scale = 5;
                    obj.traj.period = 20;
                    obj.traj.startWithHover = true;
                    
                case 'hover'
                    obj.traj.scale = 0;
                    obj.traj.period = 10;
                    obj.traj.startWithHover = false;
                    
                case 'square'
                    obj.traj.scale = 5;
                    obj.traj.period = 20;
                    obj.traj.startWithHover = true;
                    
                case 'infinity'
                    obj.traj.scale = 5;
                    obj.traj.period = 20;
                    obj.traj.startWithHover = true;
                    
                case 'takeoffland'
                    obj.traj.scale = 5;
                    obj.traj.period = 20;
                    obj.traj.startWithHover = false;
                    
                otherwise
                    error('Unknown trajectory: %s', name);
            end
        end
        
        function obj = setController(obj, type)
            %SETCONTROLLER Sets controller parameters
            %   type: 'PD', 'FeedLin', 'Adaptive'
            
            obj.controller.type = type;
            
            % Common gains (can be specific to types if needed)
            % These are the gains from the paper/config files
            obj.controller.Kp = [5.5, 5.5, 5.5, 35.5, 35.5, 65.28]';
            obj.controller.Kd = [2.05, 2.05, 2.05, 20.0, 20.5, 20.55]';
            obj.controller.potential = 'liealgebra'; % Default
            
            switch lower(type)
                case 'pd'
                    % Uses common gains above
                    
                case 'feedlin'
                    % Uses common gains above
                    
                case 'adaptive'
                    obj.controller.Gamma = 1e-3 * diag([1,1,1,1,1,1,3,0.1,0.1,0.5]);
                    
                otherwise
                    error('Unknown controller type: %s', type);
            end
        end
        
        function obj = setSimParams(obj, dt, duration)
            obj.sim.dt = dt;
            obj.sim.duration = duration;
        end
        
        function obj = setVisualization(obj, enable, dynamicAxis, padding, initialAxis)
            if nargin > 2
                obj.viz.enable = enable;
            end
            if nargin > 3
                obj.viz.dynamicAxis = dynamicAxis;
            end
            if nargin > 4
                obj.viz.axisPadding = padding;
            end
            if nargin > 5
                obj.viz.initialAxis = initialAxis;
            end
        end
        
        function obj = setActuationMethod(obj, method)
            obj.act.method = method;
        end
        
        function obj = setPotentialType(obj, potential)
            obj.controller.potential = potential;
        end
        
    end
    
    methods (Access = private)
        function initVehicle(obj)
            obj.vehicle.g = 9.8;
            obj.vehicle.m = 3.646;
            obj.vehicle.CoG = [0; 0; -0.00229];
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = vt.utils.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
        end
        
        function initActuation(obj)
            obj.act.armLength = 0.229;
            obj.act.kThrust = 8.54858e-06;
            obj.act.kDragToThrust = 0.016;
            obj.act.motorDirections = [-1, 1, -1, 1, -1, 1];
            obj.act.minMotorSpeed = 0;
            obj.act.maxMotorSpeed = 1000;
            obj.act.fixedTiltAngle = 0.5235;
            obj.act.tiltLimits = [-1.57, 1.57];
            obj.act.method = 'fixed_tilt';
        end
        
        function initSimulation(obj)
            obj.sim.dt = 0.005;
            obj.sim.duration = 30;
        end
        
        function initVisualization(obj)
            obj.viz.enable = true;
            obj.viz.dynamicAxis = true;
            obj.viz.axisPadding = 2.0;
            obj.viz.initialAxis = 'auto';
            obj.viz.liveSummary = true;
            obj.viz.updateEvery = 10;
        end
    end
end
