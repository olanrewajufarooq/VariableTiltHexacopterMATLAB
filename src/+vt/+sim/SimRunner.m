classdef SimRunner < handle
    %SIMRUNNER Orchestrates simulation, logging, and visualization.
    %   Handles trajectory generation, control updates, plant integration,
    %   logging, and optional live plotting/URDF viewing.
    %
    %   Typical flow:
    %     runner = vt.sim.SimRunner(cfg);
    %     runner.setup();
    %     runner.run();
    %     runner.save(true, 'summary');
    properties
        cfg
        traj
        ctrl
        plant
        log
        plotter
        viewer
        duration
        dt
        control_dt
        adaptation_dt
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
        lastLogs
        lastMetrics
        lastEst
        lastRunInfo
        lastIsAdaptive
        massLog
        comLog
        inertiaLog
        estTimeLog
        nextControlTime
        nextAdaptTime
        lastControlTime
        lastAdaptTime
        lastWrench
        payloadMass
        payloadCoG
        payloadDropTime
    end
    events
        StepCompleted
        SimulationFinished
    end

    methods
        function obj = SimRunner(cfg)
            %SIMRUNNER Build a runner from a configuration object.
            %   Inputs:
            %     cfg - vt.config.Config instance (or equivalent struct).
            %
            %   Output:
            %     obj - Simulation runner instance.
            obj.cfg = cfg;
            if ismethod(obj.cfg, 'done')
                obj.cfg.done();
            end
            obj.dt = cfg.sim.dt;
            obj.control_dt = cfg.sim.control_dt;
            obj.adaptation_dt = cfg.sim.adaptation_dt;
            obj.duration = cfg.sim.duration;
            obj.N = floor(obj.duration / obj.dt) + 1;
            obj.stopped_ = false;
            obj.setupResultsDir();
        end

        function setup(obj)
            %SETUP Initialize trajectory, plant, controller, and logger.
            %   Prepares the plant state and prints key configuration info.
            fprintf('Initializing components...\n');
            fprintf('  Trajectory   : %s\n', obj.cfg.traj.name);
            fprintf('  Controller   : %s (%s)\n', obj.cfg.controller.type, obj.cfg.controller.potential);
            if isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                fprintf('  Adaptation   : %s\n', obj.cfg.controller.adaptation);
            end
            fprintf('  Duration     : %.1f s\n', obj.duration);
fprintf('  Timesteps    : sim_dt=%.4f s\n', obj.dt);
            fprintf('               : control_dt=%.4f s\n', obj.control_dt);
            if isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                fprintf('               : adaptation_dt=%.4f s\n', obj.adaptation_dt);
            end
            
            % Display gains if available
            if isfield(obj.cfg.controller, 'Kp')
                Kp = obj.cfg.controller.Kp;
                if isvector(Kp) && numel(Kp) == 6
                    fprintf('  Kp Gains     : [%.2f %.2f %.2f %.2f %.2f %.2f]\n', Kp(1), Kp(2), Kp(3), Kp(4), Kp(5), Kp(6));
                end
            end
            if isfield(obj.cfg.controller, 'Kd')
                Kd = obj.cfg.controller.Kd;
                if isvector(Kd) && numel(Kd) == 6
                    fprintf('  Kd Gains     : [%.2f %.2f %.2f %.2f %.2f %.2f]\n', Kd(1), Kd(2), Kd(3), Kd(4), Kd(5), Kd(6));
                end
            end
            if isfield(obj.cfg.controller, 'Gamma') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                Gamma = obj.cfg.controller.Gamma;
                if isscalar(Gamma)
                    fprintf('  Adaptive Gains: %.4f * I\n', Gamma);
                elseif isvector(Gamma) && numel(Gamma) == 10
                    fprintf('  Adaptive Gains: [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]\n', Gamma(1), Gamma(2), Gamma(3), Gamma(4), Gamma(5), Gamma(6), Gamma(7), Gamma(8), Gamma(9), Gamma(10));
                elseif size(Gamma,1) == 10 && size(Gamma,2) == 10
                    diagGamma = diag(Gamma);
                    fprintf('  Adaptive Gains: diag([%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f])\n', diagGamma(1), diagGamma(2), diagGamma(3), diagGamma(4), diagGamma(5), diagGamma(6), diagGamma(7), diagGamma(8), diagGamma(9), diagGamma(10));
                end
            end

            obj.traj = vt.traj.TrajectoryFactory.create(obj.cfg);
            obj.plant = vt.plant.HexacopterPlant(obj.cfg);
            obj.ctrl = obj.createController();
            obj.log = vt.core.Logger();

            [H0, V0, ~] = obj.traj.getInitialState();
            obj.plant.reset(H0, V0);
        end

        function run(obj, isAdaptive, payloadMass, payloadCoG, payloadDropTime, startWithTrueValues)
            %RUN Execute a nominal or adaptive simulation loop.
            %   Inputs:
            %     isAdaptive - true for adaptive loop, false for nominal.
            %     payloadMass - payload mass [kg] (optional).
            %     payloadCoG - 3x1 payload CoG offset [m] (optional).
            %     payloadDropTime - time to drop payload [s] (optional).
            %     startWithTrueValues - seed estimator with payload (optional).
            if nargin < 2
                if isfield(obj.cfg.controller, 'adaptation')
                    isAdaptive = ~strcmpi(obj.cfg.controller.adaptation, 'none');
                else
                    isAdaptive = false;
                end
            end
            if nargin < 3
                payloadMass = obj.getPayloadField('mass', 0);
            end
            if nargin < 4
                payloadCoG = obj.getPayloadField('CoG', [0;0;0]);
            end
            if nargin < 5
                payloadDropTime = obj.getPayloadField('dropTime', inf);
            end
            if nargin < 6
                startWithTrueValues = obj.getPayloadField('startWithTrueValues', false);
            end

            obj.payloadMass = payloadMass;
            obj.payloadCoG = payloadCoG(:);
            obj.payloadDropTime = payloadDropTime;

            obj.lastLogs = [];
            obj.lastMetrics = [];
            obj.lastEst = [];
            obj.lastRunInfo = [];
            obj.lastIsAdaptive = isAdaptive;

            obj.setupVisualization();
            obj.setupAdaptivePayload(isAdaptive, payloadMass, payloadCoG, startWithTrueValues);

            obj.tCurrent = 0;
            obj.kCurrent = 1;
            obj.stopped_ = false;
            obj.resetEstimationLogs();
            obj.nextControlTime = 0;
            obj.nextAdaptTime = 0;
            obj.lastControlTime = 0;
            obj.lastAdaptTime = 0;
            obj.lastWrench = zeros(6,1);

            if isAdaptive
                obj.runAdaptiveLoop(payloadDropTime);
            else
                obj.runNominalLoop();
            end

            obj.finalize(isAdaptive);
        end

        function save(obj, data, plotType)
            %SAVE Persist logs/metrics and generate plots.
            %   Inputs:
            %     data - true to save .mat file (default false).
            %     plotType - 'none','summary','all' (default 'summary').
            if nargin < 2 || isempty(data)
                data = false;
            end
            if nargin < 3 || isempty(plotType)
                plotType = 'summary';
            end
            plotType = lower(string(plotType));
            if plotType ~= "none" && plotType ~= "summary" && plotType ~= "all"
                error("plotType must be 'none', 'summary', or 'all'.");
            end

            logs = obj.lastLogs;
            if isempty(logs)
                logs = obj.log.finalize();
                logs = vt.utils.cleanNearZero(logs);
                obj.lastLogs = logs;
            end

            isAdaptive = obj.lastIsAdaptive;
            if isempty(isAdaptive)
                if isfield(obj.cfg.controller, 'adaptation')
                    isAdaptive = ~strcmpi(obj.cfg.controller.adaptation, 'none');
                else
                    isAdaptive = false;
                end
            end

            metrics = obj.lastMetrics;
            if isempty(metrics)
                metricsObj = vt.metrics.TrackingMetrics(logs, obj.cfg.traj.name);
                metrics = metricsObj.computeAll();
                metricsObj.printReport();
                obj.lastMetrics = metrics;
            end

            est = obj.lastEst;
            if isAdaptive && isempty(est)
                est = obj.getEstimationData(logs);
                obj.lastEst = est;
            end

            runInfo = obj.lastRunInfo;
            if isempty(runInfo)
                runInfo = struct('isAdaptive', isAdaptive, 'duration', obj.duration, 'dt', obj.dt, ...
                    'control_dt', obj.control_dt, 'adaptation_dt', obj.adaptation_dt, 'runName', obj.runName);
                obj.lastRunInfo = runInfo;
            end

            if data
                dataPath = fullfile(obj.resultsDir, 'sim_data.mat');
                cfg = obj.cfg;
                save(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfg');
            end

            if plotType ~= "none"
                layoutType = 'row-major';
                if isfield(obj.cfg.viz, 'plotLayout') && ~isempty(obj.cfg.viz.plotLayout)
                    layoutType = obj.cfg.viz.plotLayout;
                end

                if isAdaptive
                    obj.figFinal = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
                    obj.figFinal = obj.plotter.plotSummaryAdaptive(logs, est, obj.figFinal, layoutType);
                else
                    obj.figFinal = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
                    obj.figFinal = obj.plotter.plotSummaryNominal(logs, obj.figFinal, layoutType);
                end

                if plotType == "all"
                    if isAdaptive
                        obj.plotter.plotStandaloneSubplotsAdaptive(logs, est);
                        obj.plotter.plotStackedEstimation(est);
                        obj.plotter.plotStackedInertia(est);
                    else
                        obj.plotter.plotStandaloneSubplotsNominal(logs);
                    end
                    obj.plotter.plotStackedAllState(logs);
                    obj.plotter.plotStackedPositionOrientation(logs);
                    obj.plotter.plotStackedVelocity(logs);
                    obj.plotter.plotStackedWrench(logs);
                end

                if ~isempty(obj.figLive) && isvalid(obj.figLive)
                    obj.plotter.saveFigure(obj.figLive, 'live_summary');
                end
                if ~isempty(obj.viewer) && isvalid(obj.viewer.fig)
                    obj.plotter.saveFigure(obj.viewer.fig, 'urdf_view');
                end
            end

            fprintf('Results saved to: %s\n', obj.resultsDir);
        end

        function logs = getLogs(obj)
            %GETLOGS Return the finalized log structure.
            %   Output:
            %     logs - struct of time-series arrays.
            logs = obj.log.finalize();
        end

        function stop(obj)
            %STOP Request a graceful simulation stop.
            %   Sets a flag checked by the main loop.
            obj.stopped_ = true;
        end

        function stopped = isStopped(obj)
            %ISSTOPPED Report whether the simulation was stopped.
            %   Output:
            %     stopped - true if stop was requested.
            stopped = obj.stopped_;
        end
    end

    methods (Access = private)
        function ctrl = createController(obj)
            %CREATECONTROLLER Instantiate the configured controller.
            %   Output:
            %     ctrl - vt.ctrl.WrenchController instance.
            ctrl = vt.ctrl.WrenchController(obj.cfg);
        end

        function setupResultsDir(obj)
            %SETUPRESULTSDIR Create a run-specific results folder.
            %   Uses timestamp, trajectory, and controller metadata.
            baseDir = fullfile(pwd, 'results');
            if ~exist(baseDir, 'dir')
                mkdir(baseDir);
            end

            subfolder = 'nominal';
            if isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
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

            obj.runName = sprintf('%s_%s_%s_%s', timestamp, trajName, ctrlType, potential);
            obj.resultsDir = fullfile(runDir, obj.runName);
            mkdir(obj.resultsDir);
        end

        function setupVisualization(obj)
            %SETUPVISUALIZATION Initialize plotter and URDF viewer.
            %   Honors cfg.viz flags and layout preferences.
            obj.plotter = vt.plot.Plotter(obj.resultsDir, struct('savePng', true, 'duration', obj.duration));
            embedUrdf = false;
            if isfield(obj.cfg.viz, 'embedUrdf')
                embedUrdf = logical(obj.cfg.viz.embedUrdf);
            end
            useRobotics = embedUrdf;

            layoutType = 'row-major';
            if isfield(obj.cfg.viz, 'plotLayout') && ~isempty(obj.cfg.viz.plotLayout)
                layoutType = obj.cfg.viz.plotLayout;
            end

            if obj.cfg.viz.enable && obj.cfg.viz.liveSummary
                obj.figLive = figure('Name','Live View','Position',[50 50 1600 900]);
                if obj.lastIsAdaptive
                    if embedUrdf
                        obj.plotter.plotLiveAdaptive(struct('t', []), [], obj.figLive, true, layoutType);
                    end
                else
                    if embedUrdf
                        obj.plotter.plotLiveNominal(struct('t', []), obj.figLive, true, layoutType);
                    end
                end
            else
                obj.figLive = [];
            end

            if obj.cfg.viz.enable
                if embedUrdf && ~isempty(obj.figLive)
                    ax = obj.plotter.getLiveUrdfAxes();
                    obj.viewer = vt.plot.UrdfViewer([], ax, useRobotics);
                    if isgraphics(obj.viewer.ax)
                        title(obj.viewer.ax, 'URDF View');
                    end
                else
                    obj.viewer = vt.plot.UrdfViewer([], [], useRobotics);
                end
                obj.viewer.setAxisLimits(vt.plot.defaultAxisLimits(obj.cfg));
                obj.viewer.setDynamicAxis(obj.cfg.viz.dynamicAxis, obj.cfg.viz.axisPadding);
            end
        end

        function setupAdaptivePayload(obj, isAdaptive, payloadMass, payloadCoG, startWithTrueValues)
            %SETUPADAPTIVEPAYLOAD Apply payload and init estimates.
            %   Inputs:
            %     isAdaptive - true if adaptation is enabled.
            %     payloadMass - payload mass [kg].
            %     payloadCoG - 3x1 CoG offset [m].
            %     startWithTrueValues - seed estimator if true.
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
            %RUNNOMINALLOOP Main loop for nominal control.
            %   Integrates plant dynamics with fixed parameters.
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
            %RUNADAPTIVELOOP Main loop for adaptive control.
            %   Handles payload drop timing and updates estimates.
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

        function resetEstimationLogs(obj)
            %RESETESTIMATIONLOGS Clear adaptation history buffers.
            %   Resets mass/CoG/inertia time series.
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];
        end

        function simulationStepNominal(obj, k)
            %SIMULATIONSTEPNOMINAL Single-step nominal update.
            %   Inputs:
            %     k - step index.
            [H, V] = obj.plant.getState();
            [Hd, Vd, Ad] = obj.traj.generate(obj.tCurrent, H, V, struct());

            obj.maybeUpdateAdaptation(Hd, H, Vd, V, Ad);
            W_cmd = obj.computeControl(Hd, H, Vd, V, Ad);
            obj.logStep(Hd, Vd, Ad, W_cmd, k);
            obj.recordEstimate();
        end

        function simulationStepAdaptive(obj, k)
            %SIMULATIONSTEPADAPTIVE Single-step adaptive update.
            %   Inputs:
            %     k - step index.
            [H, V] = obj.plant.getState();
            params = obj.getAdaptiveParams();
            [Hd, Vd, Ad] = obj.traj.generate(obj.tCurrent, H, V, params);
            obj.maybeUpdateAdaptation(Hd, H, Vd, V, Ad);
            W_cmd = obj.computeControl(Hd, H, Vd, V, Ad);
            obj.logStep(Hd, Vd, Ad, W_cmd, k);
            obj.recordEstimate();
        end

        function W_cmd = computeControl(obj, Hd, H, Vd, V, Ad)
            %COMPUTECONTROL Compute and apply wrench command.
            %   Inputs: desired/actual pose/velocity/acceleration.
            %   Output: W_cmd - 6x1 body wrench applied to plant.
            W_cmd = obj.maybeUpdateControl(Hd, H, Vd, V, Ad);
            obj.plant.step(obj.dt, W_cmd);
        end

        function maybeUpdateAdaptation(obj, Hd, H, Vd, V, Ad)
            %MAYBEUPDATEADAPTATION Update adaptation on schedule.
            %   Uses adaptation_dt to rate-limit updates.
            if obj.shouldUpdate(obj.nextAdaptTime)
                obj.ctrl.updateAdaptation(Hd, H, Vd, V, Ad, obj.adaptation_dt);
                obj.lastAdaptTime = obj.tCurrent;
                obj.nextAdaptTime = obj.nextAdaptTime + obj.adaptation_dt;
            end
        end

        function W_cmd = maybeUpdateControl(obj, Hd, H, Vd, V, Ad)
            %MAYBEUPDATECONTROL Update controller on schedule.
            %   Output: W_cmd - last or updated wrench command.
            if obj.shouldUpdate(obj.nextControlTime) || isempty(obj.lastWrench)
                obj.lastWrench = obj.ctrl.computeWrench(Hd, H, Vd, V, Ad, obj.control_dt);
                obj.lastControlTime = obj.tCurrent;
                obj.nextControlTime = obj.nextControlTime + obj.control_dt;
            end
            W_cmd = obj.lastWrench;
        end

        function doUpdate = shouldUpdate(obj, nextTime)
            %SHOULDUPDATE Check timing guard for updates.
            %   Input: nextTime - scheduled update time.
            %   Output: doUpdate - true when tCurrent passes nextTime.
            tol = max(1e-12, obj.dt * 1e-9);
            doUpdate = obj.tCurrent + tol >= nextTime;
        end

        function actual = buildActual(obj, H, V)
            %BUILDACTUAL Package actual state for logging.
            %   Inputs: H (pose), V (body velocity).
            %   Output: actual struct (pos, rpy, linVel, angVel).
            pos = H(1:3,4);
            rpy = vt.utils.rotm2rpy(H(1:3,1:3)).';
            actual = struct('pos', pos', 'rpy', rpy, 'linVel', V(4:6)', 'angVel', V(1:3)');
        end

        function desired = buildDesired(obj, Hd, Vd, Ad)
            %BUILDDESIRED Package desired state for logging.
            %   Inputs: Hd, Vd, Ad desired pose/velocity/acceleration.
            %   Output: desired struct for logger.
            pos = Hd(1:3,4);
            rpy = vt.utils.rotm2rpy(Hd(1:3,1:3)).';
            Ad = Ad(:);
            desired = struct('pos', pos', 'rpy', rpy, 'linVel', Vd(4:6)', 'angVel', Vd(1:3)', 'acc6', Ad.');
        end

        function cmd = buildCmd(obj, W_cmd)
            %BUILDCMD Package command wrench for logging.
            %   Input: W_cmd 6x1 wrench.
            %   Output: cmd struct with force/torque components.
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)');
        end

        function logStep(obj, Hd, Vd, Ad, W_cmd, k)
            %LOGSTEP Append logs and update visualization/safety.
            %   Inputs: desired state, command, and step index.
            [Hn, Vn] = obj.plant.getState();
            actual = obj.buildActual(Hn, Vn);
            desired = obj.buildDesired(Hd, Vd, Ad);
            cmd = obj.buildCmd(W_cmd);
            timing = struct('controlTime', obj.lastControlTime, 'adaptationTime', obj.lastAdaptTime);
            obj.log.append(obj.tCurrent, actual, desired, cmd, timing);

            pos_actual = Hn(1:3,4);
            pos_des = Hd(1:3,4);
            obj.updateVisualization(pos_des, pos_actual, Hn, k);
            obj.checkSafety(pos_actual);
        end

        function recordEstimate(obj)
            %RECORDESTIMATE Save latest parameter estimates.
            %   Stores mass, CoG, and inertia time series if available.
            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            if isempty(m_hat)
                return;
            end
            obj.estTimeLog(end+1) = obj.tCurrent;
            obj.massLog(end+1) = m_hat;
            obj.comLog(end+1,:) = cog_hat(:).';
            obj.inertiaLog(end+1,:) = Iparams_hat(:).';
        end

        function params = getAdaptiveParams(obj)
            %GETADAPTIVEPARAMS Build parameter struct from estimates.
            %   Output: params struct with m, cog, and Iparams.
            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            params = struct('m', m_hat, 'cog', cog_hat, 'Iparams', Iparams_hat);
        end

        function value = getPayloadField(obj, fieldName, defaultValue)
            %GETPAYLOADFIELD Read payload config with fallback.
            %   Inputs: fieldName, defaultValue.
            %   Output: value from cfg.payload or default.
            value = defaultValue;
            if isprop(obj.cfg, 'payload') && isfield(obj.cfg.payload, fieldName)
                candidate = obj.cfg.payload.(fieldName);
                if ~isempty(candidate)
                    value = candidate;
                end
            end
        end

        function updateVisualization(obj, pos_des, pos_actual, Hn, k)
            %UPDATEVISUALIZATION Render live plots/URDF if enabled.
            %   Inputs: desired/actual position, pose, and step index.
            updateEvery = 1;
            if isfield(obj.cfg.viz, 'updateEvery') && ~isempty(obj.cfg.viz.updateEvery)
                updateEvery = max(1, obj.cfg.viz.updateEvery);
            end
            doUpdate = obj.cfg.viz.enable && mod(k, updateEvery) == 0;

            if doUpdate && ~isempty(obj.viewer) && isgraphics(obj.viewer.ax)
                obj.viewer.showPose(Hn);
                obj.viewer.updatePaths(pos_des, pos_actual);
            end

            if doUpdate && obj.cfg.viz.liveSummary
                logsNow = obj.log.finalize();
                embedUrdf = isfield(obj.cfg.viz, 'embedUrdf') && obj.cfg.viz.embedUrdf;
                layoutType = 'row-major';
                if isfield(obj.cfg.viz, 'plotLayout') && ~isempty(obj.cfg.viz.plotLayout)
                    layoutType = obj.cfg.viz.plotLayout;
                end
                if obj.lastIsAdaptive
                    estNow = obj.getEstimationData(logsNow);
                    obj.figLive = obj.plotter.plotLiveAdaptive(logsNow, estNow, obj.figLive, embedUrdf, layoutType);
                else
                    obj.figLive = obj.plotter.plotLiveNominal(logsNow, obj.figLive, embedUrdf, layoutType);
                end
            end

            if doUpdate
                drawnow limitrate;
            end
        end

        function checkSafety(obj, pos_actual)
            %CHECKSAFETY Enforce numeric and ground-contact checks.
            %   Stops the sim if values are invalid or below ground.
            if any(~isfinite(pos_actual))
                warning('Numerical issue detected. Stopping simulation.');
                obj.stopped_ = true;
                return;
            end

            enableSafety = true;
            if isfield(obj.cfg.sim, 'enableSafety')
                enableSafety = logical(obj.cfg.sim.enableSafety);
            end
            if ~enableSafety
                return;
            end

            minZ = -0.005;
            if isfield(obj.cfg.sim, 'minZ') && ~isempty(obj.cfg.sim.minZ)
                minZ = obj.cfg.sim.minZ;
            end

            if obj.tCurrent > 1.0 && pos_actual(3) < minZ
                warning('Ground contact detected. Stopping simulation.');
                obj.stopped_ = true;
            end
        end

        function finalize(obj, isAdaptive)
            %FINALIZE Compute metrics and cache run artifacts.
            %   Inputs: isAdaptive - whether adaptation was used.
            logs = obj.log.finalize();
            logs = vt.utils.cleanNearZero(logs);
            fprintf('Simulation completed.\n\n');

            est = [];
            if isAdaptive
                est = obj.getEstimationData(logs);
                logs.est = est;
            end

            metricsObj = vt.metrics.TrackingMetrics(logs, obj.cfg.traj.name);
            metrics = metricsObj.computeAll();
            metricsObj.printReport();

            obj.lastLogs = logs;
            obj.lastMetrics = metrics;
            obj.lastIsAdaptive = isAdaptive;
            obj.lastEst = est;

            obj.lastRunInfo = struct('isAdaptive', isAdaptive, 'duration', obj.duration, 'dt', obj.dt, ...
                'control_dt', obj.control_dt, 'adaptation_dt', obj.adaptation_dt, 'runName', obj.runName);
        end

        function est = getEstimationData(obj, logs)
            %GETESTIMATIONDATA Assemble estimation time series.
            %   Inputs: logs struct for time vector fallback.
            %   Output: est struct of estimated and actual parameters.
            est = struct();
            if ~isempty(obj.massLog)
                t = obj.estTimeLog(:);
                est.mass = obj.massLog(:);
                est.com = obj.comLog;
                est.inertia = obj.inertiaLog;
            else
                t = logs.t(:);
                est.mass = ones(numel(t), 1);
                est.com = zeros(numel(t), 3);
                est.inertia = zeros(numel(t), 6);
            end

            est.t = t;

            [m_base, I_base, cog_base] = vt.utils.baseParams(obj.cfg);
            m_base = m_base(:).';
            I_base = I_base(:).';
            cog_base = cog_base(:);

            m_with = m_base;
            I_with = I_base;
            cog_with = cog_base;
            if obj.payloadMass > 0
                [m_with, I_with, cog_with] = vt.utils.addPayload(m_base, I_base, cog_base, obj.payloadMass, obj.payloadCoG);
                m_with = m_with(:).';
                I_with = I_with(:).';
                cog_with = cog_with(:).';
            end

            if obj.payloadMass > 0 && isfinite(obj.payloadDropTime)
                est.dropTime = obj.payloadDropTime;
                est.massActual = m_with * ones(numel(t), 1);
                est.comActual = repmat(cog_with, numel(t), 1);
                est.inertiaActual = repmat(I_with, numel(t), 1);

                idx = t >= obj.payloadDropTime;
                est.massActual(idx) = m_base;
                est.comActual(idx,:) = repmat(cog_base(:).', sum(idx), 1);
                est.inertiaActual(idx,:) = repmat(I_base, sum(idx), 1);
            else
                est.massActual = m_with * ones(numel(t), 1);
                est.comActual = repmat(cog_with, numel(t), 1);
                est.inertiaActual = repmat(I_with, numel(t), 1);
            end
        end
    end
end
