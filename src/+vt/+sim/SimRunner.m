classdef SimRunner < handle
    properties
        cfg
        traj
        ctrl
        alloc
        plant
        log
        plotter
        viewer
        duration
        dt
        N
        resultsDir
        runName
    end
    properties (Access = private)
        figLive
        figFinal
        tCurrent
        kCurrent
        stopped_
        massLog
        comLog
        inertiaLog
        estTimeLog
    end
    events
        StepCompleted
        SimulationFinished
    end

    methods
        function obj = SimRunner(cfg)
            obj.cfg = cfg;
            obj.dt = cfg.sim.dt;
            obj.duration = cfg.sim.duration;
            obj.N = floor(obj.duration / obj.dt) + 1;
            obj.stopped_ = false;
            obj.setupResultsDir();
        end

        function setup(obj)
            fprintf('Initializing components...\n');
            fprintf('  Trajectory: %s\n', obj.cfg.traj.name);
            fprintf('  Controller: %s (%s)\n', obj.cfg.controller.type, obj.cfg.controller.potential);
            fprintf('  Duration: %.1f s, dt: %.4f s\n', obj.duration, obj.dt);

            obj.traj = vt.traj.PathGenerator(obj.cfg);
            obj.alloc = vt.alloc.ControlAllocator(obj.cfg);
            obj.plant = vt.plant.HexacopterPlant(obj.cfg);
            obj.ctrl = obj.createController();
            obj.log = vt.core.Logger();

            [H0, V0, ~] = obj.traj.generate(0);
            obj.plant.reset(H0, V0);
        end

        function run(obj, isAdaptive, payloadMass, payloadCoG, payloadDropTime, startWithTrueValues)
            if nargin < 2
                isAdaptive = false;
            end
            if nargin < 3
                payloadMass = 0;
            end
            if nargin < 4
                payloadCoG = [0;0;0];
            end
            if nargin < 5
                payloadDropTime = inf;
            end
            if nargin < 6
                startWithTrueValues = false;
            end

            obj.setupVisualization();
            obj.setupAdaptivePayload(isAdaptive, payloadMass, payloadCoG, startWithTrueValues);

            obj.tCurrent = 0;
            obj.kCurrent = 1;
            obj.stopped_ = false;
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];

            if isAdaptive
                obj.runAdaptiveLoop(payloadDropTime);
            else
                obj.runNominalLoop();
            end

            obj.finalize(isAdaptive);
        end

        function logs = getLogs(obj)
            logs = obj.log.finalize();
        end

        function stop(obj)
            obj.stopped_ = true;
        end

        function stopped = isStopped(obj)
            stopped = obj.stopped_;
        end
    end

    methods (Access = private)
        function ctrl = createController(obj)
            switch lower(obj.cfg.controller.type)
                case 'adaptive'
                    ctrl = vt.ctrl.AdaptiveController(obj.cfg);
                otherwise
                    ctrl = vt.ctrl.GeometricController(obj.cfg);
            end
        end

        function setupResultsDir(obj)
            baseDir = fullfile(pwd, 'results');
            if ~exist(baseDir, 'dir')
                mkdir(baseDir);
            end

            subfolder = 'nominal';
            if strcmpi(obj.cfg.controller.type, 'adaptive')
                subfolder = 'adaptive';
            end

            runDir = fullfile(baseDir, subfolder);
            if ~exist(runDir, 'dir')
                mkdir(runDir);
            end

            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            trajName = obj.cfg.traj.name;
            ctrlType = obj.cfg.controller.type;
            potential = obj.cfg.controller.potential;
            actMethod = obj.cfg.act.method;

            obj.runName = sprintf('%s_%s_%s_%s_%s', timestamp, trajName, ctrlType, potential, actMethod);
            obj.resultsDir = fullfile(runDir, obj.runName);
            mkdir(obj.resultsDir);
        end

        function setupVisualization(obj)
            obj.plotter = vt.plot.Plotter(obj.resultsDir, struct('savePng', true, 'duration', obj.duration));

            if obj.cfg.viz.enable
                obj.viewer = vt.plot.UrdfViewer();
                obj.viewer.setAxisLimits(vt.plot.defaultAxisLimits(obj.cfg));
                obj.viewer.setDynamicAxis(obj.cfg.viz.dynamicAxis, obj.cfg.viz.axisPadding);
            end

            if obj.cfg.viz.enable && obj.cfg.viz.liveSummary
                obj.figLive = figure('Name','Live View','Position',[50 50 1600 900]);
            else
                obj.figLive = [];
            end
        end

        function setupAdaptivePayload(obj, isAdaptive, payloadMass, payloadCoG, startWithTrueValues)
            if nargin < 5
                startWithTrueValues = false;
            end
            if ~isAdaptive || payloadMass <= 0
                return;
            end

            [m_base, I_base, cog_base] = vt.utils.baseParams(obj.cfg);
            [m_with, I_with, cog_with] = vt.utils.addPayload(m_base, I_base, cog_base, payloadMass, payloadCoG);

            obj.plant.updateParameters(m_with, cog_with, I_with);
            fprintf('Plant mass (with payload): %.3f kg\n', m_with);

            if startWithTrueValues
                obj.ctrl.setPayloadEstimate(payloadMass, payloadCoG);
                fprintf('Controller initialized with TRUE values (mass=%.3f kg)\n', m_with);
            else
                fprintf('Controller initialized with NOMINAL values (will adapt)\n');
            end
        end

        function runNominalLoop(obj)
            fprintf('\nRunning simulation...\n');
            for k = 1:obj.N
                if obj.stopped_
                    break;
                end
                obj.simulationStepNominal(k);
                obj.tCurrent = obj.tCurrent + obj.dt;
                obj.kCurrent = k;
            end
        end

        function runAdaptiveLoop(obj, dropTime)
            dropped = false;
            [m_base, I_base, cog_base] = vt.utils.baseParams(obj.cfg);

            fprintf('\nRunning simulation...\n');
            for k = 1:obj.N
                if obj.stopped_
                    break;
                end
                if ~dropped && obj.tCurrent >= dropTime
                    obj.plant.updateParameters(m_base, cog_base, I_base);
                    dropped = true;
                end
                obj.simulationStepAdaptive(k);
                obj.tCurrent = obj.tCurrent + obj.dt;
                obj.kCurrent = k;
            end
        end

        function simulationStepNominal(obj, k)
            [Hd, Vd, ~] = obj.traj.generate(obj.tCurrent);
            [H, V] = obj.plant.getState();

            W_cmd = obj.ctrl.computeWrench(Hd, H, Vd, V);
            [motorSpeeds, tiltAngles] = obj.alloc.allocate(W_cmd);
            A = obj.alloc.getAllocationMatrix(tiltAngles);
            thrusts = obj.alloc.motorSpeedsToThrust(motorSpeeds);
            W_actual = A * thrusts(:);

            obj.plant.step(obj.dt, W_actual);

            [Hn, Vn] = obj.plant.getState();
            pos_actual = Hn(1:3,4);
            rpy_actual = vt.utils.rotm2rpy(Hn(1:3,1:3)).';
            linVel_actual = Vn(4:6);
            angVel_actual = Vn(1:3);

            pos_des = Hd(1:3,4);
            rpy_des = vt.utils.rotm2rpy(Hd(1:3,1:3)).';
            linVel_des = Vd(4:6);
            angVel_des = Vd(1:3);

            actual = struct('pos', pos_actual', 'rpy', rpy_actual, 'linVel', linVel_actual', 'angVel', angVel_actual');
            desired = struct('pos', pos_des', 'rpy', rpy_des, 'linVel', linVel_des', 'angVel', angVel_des', 'acc6', zeros(1,6));
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)', 'tilt', tiltAngles(:).');

            obj.log.append(obj.tCurrent, actual, desired, cmd);

            obj.updateVisualization(pos_des, pos_actual, Hn, k);
            obj.checkSafety(pos_des, pos_actual);
        end

        function simulationStepAdaptive(obj, k)
            [Hd, Vd, Ad] = obj.traj.generate(obj.tCurrent);
            [H, V] = obj.plant.getState();

            W_cmd = obj.ctrl.computeWrench(Hd, H, Vd, V, Ad, obj.dt);
            [motorSpeeds, tiltAngles] = obj.alloc.allocate(W_cmd);
            A = obj.alloc.getAllocationMatrix(tiltAngles);
            thrusts = obj.alloc.motorSpeedsToThrust(motorSpeeds);
            W_actual = A * thrusts(:);
            obj.plant.step(obj.dt, W_actual);

            [Hn, Vn] = obj.plant.getState();
            pos_actual = Hn(1:3,4);
            rpy_actual = vt.utils.rotm2rpy(Hn(1:3,1:3)).';
            linVel_actual = Vn(4:6);
            angVel_actual = Vn(1:3);

            pos_des = Hd(1:3,4);
            rpy_des = vt.utils.rotm2rpy(Hd(1:3,1:3)).';
            linVel_des = Vd(4:6);
            angVel_des = Vd(1:3);

            actual = struct('pos', pos_actual', 'rpy', rpy_actual, 'linVel', linVel_actual', 'angVel', angVel_actual');
            desired = struct('pos', pos_des', 'rpy', rpy_des, 'linVel', linVel_des', 'angVel', angVel_des', 'acc6', Ad');
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)', 'tilt', tiltAngles(:).');

            obj.log.append(obj.tCurrent, actual, desired, cmd);

            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            obj.estTimeLog(end+1) = obj.tCurrent;
            obj.massLog(end+1) = m_hat;
            obj.comLog(end+1,:) = cog_hat(:).';
            obj.inertiaLog(end+1,:) = Iparams_hat(:).';

            obj.updateVisualization(pos_des, pos_actual, Hn, k);
            obj.checkSafety(pos_des, pos_actual);
        end

        function updateVisualization(obj, pos_des, pos_actual, Hn, k)
            if obj.cfg.viz.enable
                obj.viewer.showPose(Hn);
                obj.viewer.updatePaths(pos_des, pos_actual);
            end

            if obj.cfg.viz.enable && obj.cfg.viz.liveSummary && mod(k, obj.cfg.viz.updateEvery) == 0
                logsNow = obj.log.finalize();
                obj.figLive = obj.plotter.plotLiveNominal(logsNow, obj.figLive);
                drawnow;
            end
        end

        function checkSafety(obj, pos_des, pos_actual)
            if ~strcmpi(obj.cfg.traj.name, 'takeoffland')
                if (pos_des(3) > 0 || obj.tCurrent > 0.2) && pos_actual(3) <= 0
                    warning('Ground contact detected. Stopping simulation.');
                    obj.stopped_ = true;
                end
            end

            if any(~isfinite(pos_actual))
                warning('Numerical issue detected. Stopping simulation.');
                obj.stopped_ = true;
            end
        end

        function finalize(obj, isAdaptive)
            logs = obj.log.finalize();
            logs = vt.utils.cleanNearZero(logs);
            fprintf('Simulation completed.\n\n');

            metrics = vt.metrics.TrackingMetrics.computeAll(logs.actual.pos, logs.des.pos);
            vt.metrics.TrackingMetrics.printReport(metrics, obj.cfg.traj.name);

            fprintf('\nGenerating final plots...\n');
            obj.figFinal = figure('Name','Final Summary','Position',[100 100 1600 1000]);

            if isAdaptive
                obj.figFinal = obj.plotter.plotSummaryAdaptive(logs, obj.getEstimationData(logs), obj.figFinal);
            else
                obj.figFinal = obj.plotter.plotSummaryNominal(logs, obj.figFinal);
            end

            fprintf('Results saved to: %s\n', obj.resultsDir);
        end

        function est = getEstimationData(obj, logs)
            est = struct();
            if ~isempty(obj.massLog)
                est.t = obj.estTimeLog(:);
                est.mass = obj.massLog(:);
                est.com = obj.comLog;
                est.inertia = obj.inertiaLog(:,1:3);
            else
                est.t = logs.t(:);
                est.mass = ones(numel(est.t), 1);
                est.com = zeros(numel(est.t), 3);
                est.inertia = zeros(numel(est.t), 3);
            end
        end
    end
end
