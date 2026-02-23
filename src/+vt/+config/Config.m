classdef Config < handle
    %CONFIG Manages simulation configuration parameters
    %   Use chainable methods to set up the simulation.
    %   Example: cfg = vt.config.Config().setTrajectory('circle').setController('PD');
    
    properties
        vehicle = struct()
        sim = struct()
        traj = struct()
        controller = struct()
        viz = struct()
        payload = struct()
    end
    
    methods
        function obj = Config()
            % Constructor: Initialize with default (baseline) parameters
            obj.initVehicle();
            obj.initSimulation();
            obj.initTrajectory();
            obj.initPayload();
            obj.initVisualization();
            
            % Default trajectory and controller (can be overridden)
            obj.setTrajectory('hover');
            obj.setController('PD');
        end
        
        %% Setters (Fluent Interface)
        
        function obj = setTrajectory(obj, name, cycles)
            %SETTRAJECTORY Sets trajectory parameters
            %   name: 'circle', 'hover', 'infinity', 'infinity3d', 'flip', 'takeoffland'
            %   cycles: number of cycles to run (default 1)

            obj.initTrajectory();

            if nargin > 2
                obj.traj.cycles = cycles;
            end
            
            obj.traj.name = name;
            obj.traj.useDuration = true;

            switch lower(name)
                case 'circle'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;
                    
                case 'hover'
                    obj.traj.scale = 0;
                    obj.traj.startWithHover = true;
                    obj.traj.useDuration = false;
                    
                case 'infinity'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'infinity3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'flip'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;
                    
                case 'takeoffland'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = false;
                    
                otherwise
                    error('Unknown trajectory: %s', name);
            end

            obj.syncTrajectoryPeriod();
        end

        function obj = setTrajectoryMethod(obj, method, lambda)
            if nargin > 1 && ~isempty(method)
                obj.traj.method = lower(method);
            end
            if nargin > 2 && ~isempty(lambda)
                obj.traj.lambda = lambda(:);
            end
        end
        
        function obj = setController(obj, type, potential)
            %SETCONTROLLER Sets controller parameters
            %   type: 'PD', 'FeedLin', 'Feedforward', 'Adaptive'

            if nargin < 3 || isempty(potential)
                if isfield(obj.controller, 'potential')
                    potential = obj.controller.potential;
                else
                    potential = [];
                end
                if isempty(potential)
                    potential = 'liealgebra';
                end
            end

            obj.controller.type = type;
            if ~isfield(obj.controller, 'adaptation') || isempty(obj.controller.adaptation)
                obj.controller.adaptation = 'none';
            end
            
            % Common gains (can be specific to types if needed)
            % These are the gains from the paper/config files
            if strcmpi(obj.controller.adaptation, 'none')
                obj.controller.Kp = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                obj.controller.Kd = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
            else
                % For adaptive controllers, we might want slightly different gains
                obj.controller.Kp = 1*[5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                obj.controller.Kd = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
            end

            obj.controller.potential = potential;
            
            switch lower(type)
                case 'pd'
                    % Uses common gains above
                    
                case 'feedlin'
                    % Uses common gains above

                case 'feedforward'
                    % Uses common gains above
                    
                otherwise
                    error('Unknown controller type: %s', type);
            end
        end

        function obj = setAdaptation(obj, type)
            %SETADAPTATION Sets adaptation type
            %   type: 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
            if nargin < 2 || isempty(type)
                type = 'none';
            end
            obj.controller.adaptation = lower(type);
            if ~strcmpi(type, 'none')
                if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                    obj.controller.Gamma = 1e-3 * diag([1,1,1,1,1,1,3,0.1,0.1,0.5]);
                end
            end
        end
        
        function obj = setSimParams(obj, dt, duration)
            obj.initTrajectory();
            obj.sim.dt = dt;
            obj.sim.duration = duration;
            obj.syncTrajectoryPeriod();
        end

        function obj = setPayload(obj, mass, cog, dropTime, startWithTrueValues)
            %SETPAYLOAD Sets payload parameters
            if nargin > 1
                obj.payload.mass = mass;
            end
            if nargin > 2
                obj.payload.CoG = cog(:);
            end
            if nargin > 3
                obj.payload.dropTime = dropTime;
            end
            if nargin > 4
                obj.payload.startWithTrueValues = logical(startWithTrueValues);
            end
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

        function obj = setLiveView(obj, enable, liveSummary, updateEvery, embedUrdf)
            if nargin > 1
                obj.viz.enable = enable;
            end
            if nargin > 2
                obj.viz.liveSummary = liveSummary;
            end
            if nargin > 3
                obj.viz.updateEvery = updateEvery;
            end
            if nargin > 4
                obj.viz.embedUrdf = embedUrdf;
            end
        end
        
    end
    
    methods (Access = private)
        function syncTrajectoryPeriod(obj)
            if ~isfield(obj.traj, 'useDuration') || ~obj.traj.useDuration
                return;
            end
            if ~isfield(obj.sim, 'duration') || isempty(obj.sim.duration)
                return;
            end

            duration = obj.sim.duration;
            if ~isfinite(duration) || duration <= 0
                return;
            end

            cycles = 1;
            if isfield(obj.traj, 'cycles') && ~isempty(obj.traj.cycles)
                cycles = max(obj.traj.cycles, 1);
            end

            obj.traj.cycles = cycles;
            obj.traj.period = duration / cycles;
        end

        function initVehicle(obj)
            obj.vehicle.g = 9.8;
            obj.vehicle.m = 3.646;
            obj.vehicle.CoG = [0; 0; -0.00229];
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = vt.utils.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
        end
        
        function initSimulation(obj)
            obj.sim.dt = 0.005;
            obj.sim.duration = 30;
            obj.sim.enableSafety = true;
            obj.sim.groundEnable = true;
            obj.sim.groundHeight = 0;
            obj.sim.groundStiffness = 5000;
            obj.sim.groundDamping = 200;
            obj.sim.groundFriction = 0.3;
            obj.sim.minZ = obj.sim.groundHeight - 0.2;
        end

        function initPayload(obj)
            obj.payload.mass = 0;
            obj.payload.CoG = [0; 0; 0];
            obj.payload.dropTime = inf;
            obj.payload.startWithTrueValues = false;
        end
        
        function initVisualization(obj)
            obj.viz.enable = true;
            obj.viz.dynamicAxis = true;
            obj.viz.axisPadding = 2.0;
            obj.viz.initialAxis = 'auto';
            obj.viz.liveSummary = true;
            obj.viz.updateEvery = 10;
            obj.viz.embedUrdf = true;
        end

        function initTrajectory(obj)
            if ~isstruct(obj.traj)
                obj.traj = struct();
            end
            if ~isfield(obj.traj, 'method') || isempty(obj.traj.method)
                obj.traj.method = 'precomputed';
            end
            if ~isfield(obj.traj, 'altitude') || isempty(obj.traj.altitude)
                obj.traj.altitude = 5;
            end
            if ~isfield(obj.traj, 'hoverFrac') || isempty(obj.traj.hoverFrac)
                obj.traj.hoverFrac = 0.1;
            end
            if ~isfield(obj.traj, 'lambda') || isempty(obj.traj.lambda)
                obj.traj.lambda = ones(6,1);
            end
            if ~isfield(obj.traj, 'useDuration') || isempty(obj.traj.useDuration)
                obj.traj.useDuration = true;
            end
            if ~isfield(obj.traj, 'cycles') || isempty(obj.traj.cycles)
                obj.traj.cycles = 1;
            end
        end
    end
end
