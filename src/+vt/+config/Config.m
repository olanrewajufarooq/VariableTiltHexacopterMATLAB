classdef Config < handle
    %CONFIG Centralized configuration for hexacopter simulations.
    %   Stores vehicle, simulation, trajectory, controller, visualization,
    %   and payload settings as nested structs.
    %
    %   Core fields:
    %     vehicle  - mass, CoG, inertia parameters, gravity.
    %     sim      - dt values, duration, and safety settings.
    %     traj     - path type, cycles, timing, and profile options.
    %     controller - gains, potential type, adaptation mode.
    %     viz      - live view and plot layout options.
    %
    %   Typical use:
    %     cfg = vt.config.Config()
    %         .setTrajectory('circle')
    %         .setController('PD')
    %         .setSimParams(0.005, 30);
    
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
            %CONFIG Build a configuration with baseline defaults.
            %   Initializes vehicle, simulation, trajectory, payload, and
            %   visualization settings.
            %
            %   Output:
            %     obj - Config instance with defaults and a hover trajectory.
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
            %SETTRAJECTORY Configure the reference trajectory type and cycle count.
            %   name: 'circle','hover','infinity','infinity3d','infinity3dmod',
            %         'lissajous3d','helix3d','poly3d','takeoffland'
            %   cycles: number of cycles to run (default 1)
            %
            %   Output:
            %     obj - Config instance (for chaining).

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

                case 'infinity3dmod'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'lissajous3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'helix3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'poly3d'
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
            %SETTRAJECTORYMETHOD Choose trajectory generator and optional gains.
            %   method: 'precomputed' or 'modelreference'
            %   lambda: 6x1 filter gains for model-reference trajectories
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1 && ~isempty(method)
                obj.traj.method = lower(method);
            end
            if nargin > 2 && ~isempty(lambda)
                obj.traj.lambda = lambda(:);
            end
        end
        
        function obj = setController(obj, type, potential)
            %SETCONTROLLER Configure controller type and potential function.
            %   type: 'PD', 'FeedLin', 'Feedforward'
            %   potential: 'liealgebra' or 'separate' (optional)
            %
            %   Output:
            %     obj - Config instance (for chaining).

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
                obj.controller.Kp = 1.01*[5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
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
            %SETADAPTATION Select parameter adaptation mode.
            %   type: 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(type)
                type = 'none';
            end
            obj.controller.adaptation = lower(type);
            if ~strcmpi(type, 'none')
                if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                    obj.controller.Gamma = 4e-3 * diag([20,20,30,1,1,1,90,30,30,60]);
                end
            end

        end
        
        function obj = setControlParams(obj, control_dt)
            %SETCONTROLPARAMS Set the controller update period [s].
            %   control_dt: controller update timestep in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(control_dt)
                return;
            end
            obj.sim.control_dt = control_dt;
        end

        function obj = setAdaptationParams(obj, adaptation_dt)
            %SETADAPTATIONPARAMS Set the adaptation update period [s].
            %   adaptation_dt: adaptation update timestep in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(adaptation_dt)
                return;
            end
            obj.sim.adaptation_dt = adaptation_dt;
            obj.sim.adaptation_dt_auto = false;
        end

        function obj = setSimParams(obj, sim_dt, duration)
            %SETSIMPARAMS Set simulation integration step and duration.
            %   sim_dt: integration timestep in seconds.
            %   duration: total run time in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.initTrajectory();
            obj.sim.dt = sim_dt;
            obj.sim.sim_dt_auto = false;
            obj.sim.duration = duration;
        end

        function obj = done(obj)
            %DONE Normalize time steps and sync trajectory period.
            %   Call at the end of config setup to resolve dt values.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.normalizeTimeSteps(true);
            obj.syncTrajectoryPeriod();
        end

        function obj = setPayload(obj, mass, cog, dropTime, startWithTrueValues)
            %SETPAYLOAD Configure payload mass, CoG, and drop timing.
            %   mass: payload mass in kg.
            %   cog: 3x1 payload center-of-gravity offset in meters.
            %   dropTime: seconds into the run to drop payload.
            %   startWithTrueValues: initialize estimates with payload values.
            %
            %   Output:
            %     obj - Config instance (for chaining).
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
            %SETVISUALIZATION Configure visualization toggles and axis behavior.
            %   enable: toggle live visualization.
            %   dynamicAxis: auto-resize axes when enabled.
            %   padding: extra padding for dynamic axis limits.
            %   initialAxis: initial axis limits or 'auto'.
            %
            %   Output:
            %     obj - Config instance (for chaining).
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
            %SETACTUATIONMETHOD Set the actuation mapping strategy.
            %   method: string identifier for allocation/mapping.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.act.method = method;
        end
        
        function obj = setPotentialType(obj, potential)
            %SETPOTENTIALTYPE Select the controller potential model.
            %   potential: 'liealgebra' or 'separate'.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.controller.potential = potential;
        end

        function obj = enableLiveView(obj, enable)
            %ENABLELIVEVIEW Toggle live visualization.
            %   enable: true/false.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.enable = enable;
            end
        end

        function obj = setLiveSummary(obj, liveSummary)
            %SETLIVESUMMARY Toggle live summary plots.
            %   liveSummary: true/false.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.liveSummary = liveSummary;
            end
        end

        function obj = setLiveUpdateRate(obj, updateEvery)
            %SETLIVEUPDATERATE Set live plot update cadence (in control steps).
            %   updateEvery: update interval in control steps.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.updateEvery = updateEvery;
            end
        end

        function obj = setLiveUrdfEmbedding(obj, embedUrdf)
            %SETLIVEURDFEMBEDDING Control URDF embedding for the live view.
            %   embedUrdf: true to embed URDF, false to load from file.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.embedUrdf = embedUrdf;
            end
        end

        function obj = setPlotLayout(obj, layoutType)
            %SETPLOTLAYOUT Set plot grid layout: 'row-major' or 'column-major'.
            %   layoutType: layout string.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(layoutType)
                return;
            end
            layoutType = lower(string(layoutType));
            if layoutType ~= "row-major" && layoutType ~= "column-major"
                error("plotLayout must be 'row-major' or 'column-major'.");
            end
            obj.viz.plotLayout = char(layoutType);
        end
        
    end
    
    methods (Access = private)
        function syncTrajectoryPeriod(obj)
            %SYNCTRAJECTORYPERIOD Match trajectory period to sim duration.
            %   Uses sim.duration and traj.cycles to set traj.period.
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
            %INITVEHICLE Initialize vehicle mass and inertia parameters.
            %   Populates vehicle mass, CoG, inertia parameters, and I6.
            obj.vehicle.g = 9.8;
            obj.vehicle.m = 3.646;
            obj.vehicle.CoG = [0; 0; -0.00229];
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = vt.utils.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
        end
        
        function initSimulation(obj)
            %INITSIMULATION Initialize simulation timing and safety defaults.
            %   Sets dt values, duration, and ground interaction parameters.
            obj.sim.control_dt = 0.005;
            obj.sim.adaptation_dt = obj.sim.control_dt / 5;
            obj.sim.dt = obj.sim.adaptation_dt;
            obj.sim.duration = 30;
            obj.sim.adaptation_dt_auto = true;
            obj.sim.sim_dt_auto = true;
            obj.sim.enableSafety = true;
            obj.sim.groundEnable = true;
            obj.sim.groundHeight = 0;
            obj.sim.groundStiffness = 5000;
            obj.sim.groundDamping = 200;
            obj.sim.groundFriction = 0.3;
            obj.sim.minZ = obj.sim.groundHeight - 0.2;
        end

        function initPayload(obj)
            %INITPAYLOAD Initialize payload parameters.
            %   Defaults to no payload and no drop event.
            obj.payload.mass = 0;
            obj.payload.CoG = [0; 0; 0];
            obj.payload.dropTime = inf;
            obj.payload.startWithTrueValues = false;
        end
        
        function initVisualization(obj)
            %INITVISUALIZATION Initialize visualization defaults.
            %   Enables live view and sets layout/padding defaults.
            obj.viz.enable = true;
            obj.viz.dynamicAxis = true;
            obj.viz.axisPadding = 2.0;
            obj.viz.initialAxis = 'auto';
            obj.viz.liveSummary = true;
            obj.viz.updateEvery = 10;
            obj.viz.embedUrdf = true;
            obj.viz.plotLayout = 'row-major';
        end

        function initTrajectory(obj)
            %INITTRAJECTORY Initialize trajectory defaults.
            %   Seeds trajectory method, altitude, and cycles.
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

        function normalizeTimeSteps(obj, strict)
            %NORMALIZETIMESTEPS Resolve dt values for control/adaptation/sim.
            %   strict: enforce dt ordering constraints if true.
            if nargin < 2
                strict = false;
            end
            if ~isfield(obj.sim, 'control_dt') || isempty(obj.sim.control_dt)
                obj.sim.control_dt = obj.sim.dt;
            end

            adaptationEnabled = isfield(obj.controller, 'adaptation') && ~strcmpi(obj.controller.adaptation, 'none');
            if ~isfield(obj.sim, 'adaptation_dt_auto')
                obj.sim.adaptation_dt_auto = true;
            end
            if ~isfield(obj.sim, 'sim_dt_auto')
                obj.sim.sim_dt_auto = true;
            end

            if adaptationEnabled
                if obj.sim.adaptation_dt_auto
                    obj.sim.adaptation_dt = obj.sim.control_dt / 5;
                end
            else
                if obj.sim.adaptation_dt_auto || strict
                    obj.sim.adaptation_dt = obj.sim.control_dt;
                end
            end

            if obj.sim.sim_dt_auto
                obj.sim.dt = obj.sim.adaptation_dt;
            end

            if strict
                if adaptationEnabled && obj.sim.adaptation_dt >= obj.sim.control_dt
                    error('adaptation_dt must be smaller than control_dt when adaptation is enabled.');
                end

                if obj.sim.dt > obj.sim.adaptation_dt
                    error('sim_dt must be equal to or smaller than adaptation_dt.');
                end
            end
        end
    end
end
