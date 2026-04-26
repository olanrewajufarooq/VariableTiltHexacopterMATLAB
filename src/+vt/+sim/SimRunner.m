classdef SimRunner < handle
    %SIMRUNNER Orchestrates simulation, logging, and visualization.
    %   Handles trajectory generation, control updates, plant integration,
    %   logging, and optional live plotting/URDF viewing.
    %
    %   Typical flow:
    %     runner = vt.sim.SimRunner(cfg);
    %     runner.setup();
    %     runner.run();
    %     runner.plot('summary');
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
        batchSize
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
        batchResultDirs
        pendingRunArgs
        commandLogPath
        commandLogActive
        captureConsoleExternally
        consoleCaptureCallback
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
            obj.batchSize = obj.resolveBatchSize();
            obj.batchResultDirs = {};
            obj.pendingRunArgs = {};
            obj.commandLogPath = '';
            obj.commandLogActive = false;
            obj.captureConsoleExternally = isfield(cfg.sim, 'captureConsoleExternally') ...
                && logical(cfg.sim.captureConsoleExternally);
            obj.consoleCaptureCallback = [];
            obj.setupResultsDir();
        end

        function setup(obj)
            %SETUP Initialize trajectory, plant, controller, and logger.
            %   Prepares the plant state and prints key configuration info.
            if obj.isBatchMode()
                fprintf('Configured batch simulation with %d runs.\n', obj.batchSize);
                return;
            end

            obj.beginCommandWindowCapture();
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
                if isvector(Gamma) && numel(Gamma) == 10
                    fprintf('  Adaptive Gains: [%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f]\n', Gamma(1), Gamma(2), Gamma(3), Gamma(4), Gamma(5), Gamma(6), Gamma(7), Gamma(8), Gamma(9), Gamma(10));
                end
            end

            obj.traj = vt.traj.TrajectoryFactory.create(obj.cfg);
            obj.plant = vt.plant.HexacopterPlant(obj.cfg);
            obj.ctrl = obj.createController();
            obj.log = vt.core.Logger();

            [H0, V0] = obj.resolveInitialPlantState();
            obj.plant.reset(H0, V0);
        end

        function run(obj, isAdaptive, payloadMass, payloadCoG, payloadDropTime, estimateInitialization)
            %RUN Execute a nominal or adaptive simulation loop.
            %   Inputs:
            %     isAdaptive - true for adaptive loop, false for nominal.
            %     payloadMass - payload mass [kg] (optional).
            %     payloadCoG - 3x1 payload CoG offset [m] (optional).
            %     payloadDropTime - time to drop payload [s] (optional).
            %     estimateInitialization - initialization mode/spec (optional).
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
                estimateInitialization = obj.getEstimateInitialization();
            end

            if obj.isBatchMode()
                obj.pendingRunArgs = {isAdaptive, payloadMass, payloadCoG, payloadDropTime, estimateInitialization};
                obj.runBatch();
                return;
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
            obj.setupAdaptivePayload(isAdaptive, payloadMass, payloadCoG, estimateInitialization);

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

        function plot(obj, plotType)
            %PLOT Generate plots from saved simulation results on disk.
            %   Input:
            %     plotType - 'none','summary','all' (default 'summary').
            if nargin < 2 || isempty(plotType)
                plotType = 'summary';
            end
            plotType = lower(string(plotType));
            if plotType ~= "none" && plotType ~= "summary" && plotType ~= "all"
                error("plotType must be 'none', 'summary', or 'all'.");
            end

            if obj.isBatchMode()
                obj.plotBatch(char(plotType));
                return;
            end

            obj.plotSavedRun(obj.resultsDir, char(plotType));
        end

        function save(obj, varargin)
            %SAVE Deprecated compatibility wrapper for plot().
            if nargin == 2
                plotType = varargin{1};
            elseif nargin >= 3
                plotType = varargin{2};
            else
                plotType = 'summary';
            end
            obj.plot(plotType);
        end

        function logs = getLogs(obj)
            %GETLOGS Return the finalized log structure.
            %   Output:
            %     logs - struct of time-series arrays.
            if ~isempty(obj.log)
                logs = obj.log.finalize();
            else
                saved = obj.loadSavedRun(obj.resultsDir);
                logs = saved.logs;
            end
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
        function [H0, V0] = resolveInitialPlantState(obj)
            %RESOLVEINITIALPLANTSTATE Choose the plant initial condition.
            %   If startWithHover is disabled, start from ground origin with
            %   level attitude and zero twist instead of the trajectory's
            %   initial desired state.
            if isprop(obj.cfg, 'traj') && isfield(obj.cfg.traj, 'startWithHover') ...
                    && ~logical(obj.cfg.traj.startWithHover)
                H0 = eye(4);
                H0(1:3,4) = [0; 0; 0];
                V0 = zeros(6,1);
                return;
            end

            [H0, V0, ~] = obj.traj.getInitialState();
        end

        function ctrl = createController(obj)
            %CREATECONTROLLER Instantiate the configured controller.
            %   Output:
            %     ctrl - vt.ctrl.WrenchController instance.
            ctrl = vt.ctrl.WrenchController(obj.cfg);
        end

        function setupResultsDir(obj)
            %SETUPRESULTSDIR Create a run-specific results folder.
            %   Uses timestamp, trajectory, and controller metadata.
            if isfield(obj.cfg.sim, 'resultsDirOverride') && ~isempty(obj.cfg.sim.resultsDirOverride)
                obj.resultsDir = obj.cfg.sim.resultsDirOverride;
                if ~exist(obj.resultsDir, 'dir')
                    mkdir(obj.resultsDir);
                end
                [~, obj.runName] = fileparts(obj.resultsDir);
                return;
            end

            baseDir = fullfile(obj.repoRoot(), 'results');
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

            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            trajName = obj.getBatchTrajectoryLabel();
            if obj.isBatchMode()
                obj.runName = sprintf('%s_%s', timestamp, trajName);
            else
                ctrlType = obj.getCompactControllerLabel();
                potential = obj.getCompactPotentialLabel();
                obj.runName = sprintf('%s_%s_%s_%s', timestamp, trajName, ctrlType, potential);
            end
            obj.resultsDir = fullfile(runDir, obj.runName);
            mkdir(obj.resultsDir);
        end

        function batchCount = resolveBatchSize(obj)
            %RESOLVEBATCHSIZE Resolve requested run count from config gains.
            batchCount = 1;
            if ismethod(obj.cfg, 'getBatchCount')
                batchCount = obj.cfg.getBatchCount();
            end
        end

        function tf = isBatchMode(obj)
            %ISBATCHMODE Return true when multiple runs are configured.
            tf = obj.batchSize > 1;
        end

        function runBatch(obj)
            %RUNBATCH Execute all requested runs and capture per-run console output.
            cfgs = obj.cfg.expandBatchConfigs(obj.resultsDir);
            obj.batchResultDirs = cell(obj.batchSize, 1);
            for i = 1:obj.batchSize
                child = vt.sim.SimRunner(cfgs{i});
                runLog = obj.captureConsole(@() obj.executeChildRun(child));
                childLogPath = fullfile(child.resultsDir, 'command_window.txt');
                obj.writeTextFile(childLogPath, strtrim(runLog));
                obj.batchResultDirs{i} = child.resultsDir;
                clear child
            end
            obj.writeBatchArtifactsFromSavedRuns();
            obj.releaseBatchMemory();
            fprintf('Batch results saved to: %s\n', obj.resultsDir);
        end

        function executeChildRun(obj, child)
            %EXECUTECHILDRUN Run the setup and simulation steps for one child runner.
            child.setup();
            child.run(obj.pendingRunArgs{:});
        end

        function plotBatch(obj, plotType)
            %PLOTBATCH Generate plots for each saved batch run and rebuild reports.
            if isempty(obj.batchResultDirs)
                obj.batchResultDirs = obj.findBatchResultDirs();
            end
            if isempty(obj.batchResultDirs)
                error('SimRunner:BatchNotRun', 'Batch simulation has not been run yet.');
            end
            for i = 1:numel(obj.batchResultDirs)
                obj.plotSavedRun(obj.batchResultDirs{i}, char(plotType));
            end
            obj.writeBatchArtifactsFromSavedRuns();
        end

        function root = repoRoot(~)
            %REPOROOT Return repository root path.
            p = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(p))));
        end

        function label = getBatchTrajectoryLabel(obj)
            %GETBATCHTRAJECTORYLABEL Return a trajectory label for the results root.
            if isfield(obj.cfg.traj, 'batch') && isstruct(obj.cfg.traj.batch) ...
                    && isfield(obj.cfg.traj.batch, 'names') && ~isempty(obj.cfg.traj.batch.names)
                if numel(obj.cfg.traj.batch.names) > 1
                    label = 'multi_traj';
                    return;
                end
                label = obj.getCompactTrajectoryLabel(obj.cfg.traj.batch.names{1});
                return;
            end
            label = obj.getCompactTrajectoryLabel(obj.cfg.traj.name);
        end

        function label = getCompactTrajectoryLabel(~, name)
            %GETCOMPACTTRAJECTORYLABEL Build a compact label for root folders.
            switch lower(name)
                case 'circle'
                    label = 'circle';
                case 'infinity'
                    label = 'inf';
                case 'infinity3d'
                    label = 'inf3d';
                case 'infinity3dmod'
                    label = 'inf3dmod';
                case 'lissajous3d'
                    label = 'liss3d';
                case 'helix3d'
                    label = 'helix3d';
                case 'poly3d'
                    label = 'poly3d';
                case 'takeoffland'
                    label = 'tkoffland';
                otherwise
                    label = regexprep(lower(char(string(name))), '[^a-z0-9]+', '');
            end
            if strlength(string(label)) > 12
                label = char(extractBefore(string(label), 13));
            end
        end

        function label = getCompactControllerLabel(obj)
            %GETCOMPACTCONTROLLERLABEL Build a compact controller label.
            if ~isfield(obj.cfg.controller, 'type') || isempty(obj.cfg.controller.type)
                label = 'ctrl';
                return;
            end
            switch lower(obj.cfg.controller.type)
                case 'feedforward'
                    label = 'ff';
                case 'feedlin'
                    label = 'fl';
                case 'pd'
                    label = 'pd';
                otherwise
                    label = regexprep(lower(char(string(obj.cfg.controller.type))), '[^a-z0-9]+', '');
            end
        end

        function label = getCompactPotentialLabel(obj)
            %GETCOMPACTPOTENTIALLABEL Build a compact potential label.
            if ~isfield(obj.cfg.controller, 'potential') || isempty(obj.cfg.controller.potential)
                label = 'pot';
                return;
            end
            switch lower(obj.cfg.controller.potential)
                case 'liealgebra'
                    label = 'lie';
                case 'separate'
                    label = 'sep';
                otherwise
                    label = regexprep(lower(char(string(obj.cfg.controller.potential))), '[^a-z0-9]+', '');
            end
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

        function setupAdaptivePayload(obj, isAdaptive, payloadMass, payloadCoG, estimateInitialization)
            %SETUPADAPTIVEPAYLOAD Apply payload and init estimates.
            %   Inputs:
            %     isAdaptive - true if adaptation is enabled.
            %     payloadMass - payload mass [kg].
            %     payloadCoG - 3x1 CoG offset [m].
            %     estimateInitialization - initialization mode/spec.
            if nargin < 5
                estimateInitialization = obj.getEstimateInitialization();
            end
            if ~isAdaptive
                return;
            end

            [m_base, I_base, cog_base] = vt.utils.baseParams(obj.cfg);
            [m_with, I_with, cog_with] = vt.utils.addPayload(m_base, I_base, cog_base, payloadMass, payloadCoG);

            obj.plant.updateParameters(m_with, cog_with, I_with);
            if payloadMass > 0
                fprintf('Plant mass (with payload): %.3f kg\n', m_with);
            else
                fprintf('Plant mass: %.3f kg\n', m_with);
            end

            [theta0, initLabel] = obj.resolveEstimateInitializationTheta( ...
                estimateInitialization, m_base, I_base, cog_base, m_with, I_with, cog_with);
            obj.ctrl.setEstimateTheta(theta0);
            fprintf('Controller initialized with %s values (mass=%.3f kg)\n', initLabel, theta0(7));
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

        function actual = buildActual(~, H, V)
            %BUILDACTUAL Package actual state for logging.
            %   Inputs: H (pose), V (body velocity).
            %   Output: actual struct (pos, rpy, linVel, angVel).
            pos = H(1:3,4);
            rpy = vt.utils.rotm2rpy(H(1:3,1:3)).';
            actual = struct('pos', pos', 'rpy', rpy, 'linVel', V(4:6)', 'angVel', V(1:3)');
        end

        function desired = buildDesired(~, Hd, Vd, Ad)
            %BUILDDESIRED Package desired state for logging.
            %   Inputs: Hd, Vd, Ad desired pose/velocity/acceleration.
            %   Output: desired struct for logger.
            pos = Hd(1:3,4);
            rpy = vt.utils.rotm2rpy(Hd(1:3,1:3)).';
            Ad = Ad(:);
            desired = struct('pos', pos', 'rpy', rpy, 'linVel', Vd(4:6)', 'angVel', Vd(1:3)', 'acc6', Ad.');
        end

        function cmd = buildCmd(~, W_cmd)
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

        function [theta0, initLabel] = resolveEstimateInitializationTheta( ...
                obj, initCfg, m_base, I_base, cog_base, m_true, I_true, cog_true)
            %RESOLVEESTIMATEINITIALIZATIONTHETA Build initial adaptive theta.
            mode = 'nominal';
            spec = [];
            if isstruct(initCfg)
                if isfield(initCfg, 'mode') && ~isempty(initCfg.mode)
                    mode = char(lower(string(initCfg.mode)));
                end
                if isfield(initCfg, 'spec')
                    spec = initCfg.spec;
                end
            elseif ischar(initCfg) || (isstring(initCfg) && isscalar(initCfg))
                mode = char(lower(string(initCfg)));
            end

            switch mode
                case 'nominal'
                    theta0 = obj.packEstimateTheta(m_base, I_base, cog_base);
                    initLabel = 'NOMINAL';
                case 'true'
                    theta0 = obj.packEstimateTheta(m_true, I_true, cog_true);
                    initLabel = 'TRUE';
                case 'fixed'
                    if isempty(spec)
                        theta0 = obj.buildDefaultFixedEstimateTheta(m_true, I_true, cog_true);
                    else
                        validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                        theta0 = spec(:);
                    end
                    initLabel = 'FIXED';
                case 'random'
                    theta0 = obj.buildRandomEstimateTheta(m_true, I_true, cog_true, spec);
                    initLabel = 'RANDOM';
                otherwise
                    error('SimRunner:InvalidEstimateInitializationMode', ...
                        'Unknown estimate initialization mode: %s', mode);
            end
        end

        function theta = buildRandomEstimateTheta(obj, m_true, I_true, cog_true, spec)
            %BUILDRANDOMESTIMATETHETA Build a deterministic random initial theta.
            seed = 1729;
            if nargin >= 5 && ~isempty(spec)
                if isnumeric(spec) && isscalar(spec)
                    seed = double(spec);
                elseif isstruct(spec) && isfield(spec, 'seed') && ~isempty(spec.seed)
                    seed = double(spec.seed);
                else
                    error('SimRunner:InvalidRandomEstimateSpec', ...
                        'Random estimate spec must be empty, a scalar seed, or a struct with a seed field.');
                end
            end

            rngState = rng;
            cleanup = onCleanup(@() rng(rngState));
            cleanupObj = cleanup; %#ok<NASGU>
            rng(seed, 'twister');

            inertiaScale = 1 + 0.10 * (2 * rand(6,1) - 1);
            massScale = 1 + 0.10 * (2 * rand() - 1);
            cogDelta = 0.01 * (2 * rand(3,1) - 1);

            I_rand = I_true(:) .* inertiaScale;
            m_rand = max(1e-6, m_true * massScale);
            cog_rand = cog_true(:) + cogDelta;
            theta = obj.packEstimateTheta(m_rand, I_rand, cog_rand);
        end

        function theta = buildDefaultFixedEstimateTheta(obj, m_true, I_true, cog_true)
            %BUILDDEFAULTFIXEDESTIMATETHETA Build the repo default fixed theta.
            inertiaScale = [0.96; 1.04; 0.98; 1.03; 0.97; 1.02];
            massScale = 0.97;
            cogDelta = [0.004; -0.003; 0.002];

            I_fixed = I_true(:) .* inertiaScale;
            m_fixed = massScale * m_true;
            cog_fixed = cog_true(:) + cogDelta;
            theta = obj.packEstimateTheta(m_fixed, I_fixed, cog_fixed);
        end

        function theta = packEstimateTheta(~, m, Iparams, CoG)
            %PACKESTIMATETHETA Convert physical parameters into theta form.
            theta = [Iparams(:); m; m * CoG(:)];
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
            obj.persistCurrentRun();
            obj.endCommandWindowCapture();
            obj.releaseRunMemory();
        end

        function beginCommandWindowCapture(obj)
            %BEGINCOMMANDWINDOWCAPTURE Start writing command-window output to file.
            if obj.captureConsoleExternally || obj.commandLogActive
                return;
            end
            obj.commandLogPath = fullfile(obj.resultsDir, 'command_window.txt');
            diary(obj.commandLogPath);
            obj.commandLogActive = true;
        end

        function endCommandWindowCapture(obj)
            %ENDCOMMANDWINDOWCAPTURE Stop command-window capture when active.
            if obj.captureConsoleExternally || ~obj.commandLogActive
                return;
            end
            diary off;
            obj.commandLogActive = false;
        end

        function writeTextFile(~, path, content)
            %WRITETEXTFILE Write UTF-8 text content to a file.
            fid = fopen(path, 'w');
            if fid < 0
                error('SimRunner:WriteFailed', 'Unable to write file: %s', path);
            end
            cleanup = onCleanup(@() fclose(fid));
            fprintf(fid, '%s\n', content);
        end

        function persistCurrentRun(obj)
            %PERSISTCURRENTRUN Save finalized run artifacts immediately.
            logs = obj.lastLogs;
            metrics = obj.lastMetrics;
            est = obj.lastEst;
            runInfo = obj.lastRunInfo;
            cfgSnapshot = obj.cfg;
            dataPath = fullfile(obj.resultsDir, 'sim_data.mat');
            save(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
            fprintf('Run data saved to: %s\n', dataPath);
        end

        function releaseRunMemory(obj)
            %RELEASERUNMEMORY Clear heavy in-memory run state after persistence.
            obj.lastLogs = [];
            obj.lastMetrics = [];
            obj.lastEst = [];
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];
            obj.log = [];
        end

        function releaseBatchMemory(obj)
            %RELEASEBATCHMEMORY Drop batch execution state after persistence.
            obj.pendingRunArgs = {};
            obj.consoleCaptureCallback = [];
        end

        function plotSavedRun(obj, resultsDir, plotType)
            %PLOTSAVEDRUN Generate plots for one saved run directory.
            if string(plotType) == "none"
                return;
            end
            saved = obj.loadSavedRun(resultsDir);
            cfgSnapshot = saved.cfgSnapshot;
            logs = saved.logs;
            est = saved.est;
            runInfo = saved.runInfo;
            isAdaptive = logical(runInfo.isAdaptive);

            layoutType = 'row-major';
            if isprop(cfgSnapshot, 'viz') && isfield(cfgSnapshot.viz, 'plotLayout') && ~isempty(cfgSnapshot.viz.plotLayout)
                layoutType = cfgSnapshot.viz.plotLayout;
            end

            plotterObj = vt.plot.Plotter(resultsDir, struct('savePng', true, 'duration', runInfo.duration));
            if isAdaptive
                summaryFig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
                plotterObj.plotSummaryAdaptive(logs, est, summaryFig, layoutType);
            else
                summaryFig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
                plotterObj.plotSummaryNominal(logs, summaryFig, layoutType);
            end

            if string(plotType) == "all"
                if isAdaptive
                    plotterObj.plotStandaloneSubplotsAdaptive(logs, est);
                    plotterObj.plotStackedEstimation(est);
                    plotterObj.plotStackedInertia(est);
                else
                    plotterObj.plotStandaloneSubplotsNominal(logs);
                end
                plotterObj.plotStackedAllState(logs);
                plotterObj.plotStackedPositionOrientation(logs);
                plotterObj.plotStackedVelocity(logs);
                plotterObj.plotStackedWrench(logs);
            end
        end

        function saved = loadSavedRun(~, resultsDir)
            %LOADSAVEDRUN Load saved run data from disk.
            dataPath = fullfile(resultsDir, 'sim_data.mat');
            if ~exist(dataPath, 'file')
                error('SimRunner:MissingSavedData', 'Missing sim_data.mat in %s', resultsDir);
            end
            saved = load(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
        end

        function dirs = findBatchResultDirs(obj)
            %FINDBATCHRESULTDIRS Discover saved batch child result directories.
            dirs = {};
            if ~exist(obj.resultsDir, 'dir')
                return;
            end
            listing = dir(fullfile(obj.resultsDir, '**', 'sim_data.mat'));
            if isempty(listing)
                return;
            end
            dirs = cell(numel(listing), 1);
            for i = 1:numel(listing)
                dirs{i} = listing(i).folder;
            end
            dirs = sort(dirs);
        end

        function writeBatchArtifactsFromSavedRuns(obj)
            %WRITEBATCHARTIFACTSFROMSAVEDRUNS Rebuild aggregate logs and reports from files.
            if isempty(obj.batchResultDirs)
                obj.batchResultDirs = obj.findBatchResultDirs();
            end
            aggregateChunks = cell(numel(obj.batchResultDirs), 1);
            for i = 1:numel(obj.batchResultDirs)
                saved = obj.loadSavedRun(obj.batchResultDirs{i});
                childLogPath = fullfile(obj.batchResultDirs{i}, 'command_window.txt');
                childLog = obj.readTextFile(childLogPath);
                aggregateChunks{i} = sprintf('===== Trajectory: %s | %s =====\n%s\n', ...
                    saved.cfgSnapshot.traj.name, obj.getRunLabelFromSaved(saved, obj.batchResultDirs{i}), strtrim(childLog));
            end
            aggregatePath = fullfile(obj.resultsDir, 'command_window.txt');
            obj.writeTextFile(aggregatePath, strjoin(aggregateChunks, newline));
            if obj.isAdaptiveBatchFromFiles()
                summaryPath = fullfile(obj.resultsDir, 'adaptive_report.txt');
                obj.writeTextFile(summaryPath, obj.buildBatchSummaryTableFromFiles());
            end
        end

        function content = readTextFile(~, path)
            %READTEXTFILE Read a UTF-8 text file.
            if ~exist(path, 'file')
                content = '';
                return;
            end
            content = fileread(path);
        end

        function tableText = buildBatchSummaryTableFromFiles(obj)
            %BUILDBATCHSUMMARYTABLEFROMFILES Build an aligned summary table from saved runs.
            nRuns = numel(obj.batchResultDirs);
            headers = {'Trajectory', 'Run', 'Track RMSE', 'Track Score', ...
                'Mass RMSE', 'Mass Score', 'CoG RMSE', 'CoG Score', ...
                'Inertia RMSE', 'Inertia Score'};
            rawRows = cell(nRuns, numel(headers));
            numericValues = nan(nRuns, numel(headers));
            trajectoryNames = cell(nRuns, 1);
            betterIsLower = [false, false, true, false, true, false, true, false, true, false];

            for i = 1:nRuns
                saved = obj.loadSavedRun(obj.batchResultDirs{i});
                metrics = saved.metrics;
                trajectoryNames{i} = saved.cfgSnapshot.traj.name;
                rawRows{i,1} = trajectoryNames{i};
                rawRows{i,2} = obj.getRunLabelFromSaved(saved, obj.batchResultDirs{i});

                rawRows{i,3} = obj.formatMetric(metrics.combined.rmse_total, 4);
                rawRows{i,4} = obj.formatMetric(metrics.combined.tracking_score, 2);
                numericValues(i,3) = metrics.combined.rmse_total;
                numericValues(i,4) = metrics.combined.tracking_score;

                if isfield(metrics, 'parameters')
                    rawRows{i,5} = obj.formatMetric(metrics.parameters.mass.rmse, 4);
                    rawRows{i,6} = obj.formatMetric(metrics.parameters.mass.tracking_score, 2);
                    rawRows{i,7} = obj.formatMetric(metrics.parameters.cog.rmse_total, 4);
                    rawRows{i,8} = obj.formatMetric(metrics.parameters.cog.tracking_score, 2);
                    rawRows{i,9} = obj.formatMetric(metrics.parameters.inertia.rmse_total, 4);
                    rawRows{i,10} = obj.formatMetric(metrics.parameters.inertia.tracking_score, 2);
                    numericValues(i,5) = metrics.parameters.mass.rmse;
                    numericValues(i,6) = metrics.parameters.mass.tracking_score;
                    numericValues(i,7) = metrics.parameters.cog.rmse_total;
                    numericValues(i,8) = metrics.parameters.cog.tracking_score;
                    numericValues(i,9) = metrics.parameters.inertia.rmse_total;
                    numericValues(i,10) = metrics.parameters.inertia.tracking_score;
                else
                    rawRows(i,5:10) = {'N/A'};
                end
            end

            [~, ~, trajectoryGroups] = unique(trajectoryNames, 'stable');
            for groupId = 1:max(trajectoryGroups)
                groupRows = find(trajectoryGroups == groupId);
                for col = 3:numel(headers)
                    values = numericValues(groupRows, col);
                    validMask = isfinite(values);
                    if ~any(validMask)
                        continue;
                    end
                    validValues = values(validMask);
                    if betterIsLower(col)
                        bestValue = min(validValues);
                    else
                        bestValue = max(validValues);
                    end
                    bestMask = validMask & abs(values - bestValue) <= max(1e-12, abs(bestValue) * 1e-12);
                    bestRows = groupRows(bestMask);
                    for row = bestRows.'
                        rawRows{row,col} = sprintf('%s (best)', rawRows{row,col});
                    end
                end
            end

            widths = cellfun(@strlength, headers);
            for col = 1:numel(headers)
                for row = 1:nRuns
                    widths(col) = max(widths(col), strlength(string(rawRows{row,col})));
                end
            end

            lines = strings(nRuns + 5, 1);
            lineIdx = 1;
            lines(lineIdx) = "Batch Run Summary"; lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSeparator(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildAlignedRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSeparator(widths); lineIdx = lineIdx + 1;
            for row = 1:nRuns
                lines(lineIdx) = obj.buildAlignedRow(rawRows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = obj.buildSeparator(widths);
            tableText = strjoin(cellstr(lines), newline);
        end

        function tf = isAdaptiveBatchFromFiles(obj)
            %ISADAPTIVEBATCHFROMFILES Return true when all saved batch runs use adaptation.
            tf = ~isempty(obj.batchResultDirs);
            if ~tf
                return;
            end
            for i = 1:numel(obj.batchResultDirs)
                saved = obj.loadSavedRun(obj.batchResultDirs{i});
                if ~isfield(saved.cfgSnapshot.controller, 'adaptation') || strcmpi(saved.cfgSnapshot.controller.adaptation, 'none')
                    tf = false;
                    return;
                end
            end
        end

        function initCfg = getEstimateInitialization(obj)
            %GETESTIMATEINITIALIZATION Read estimate-init config with fallback.
            initCfg = struct('mode', 'nominal', 'spec', []);
            if isprop(obj.cfg, 'controller') && isfield(obj.cfg.controller, 'estimateInitialization')
                candidate = obj.cfg.controller.estimateInitialization;
                if isstruct(candidate) && isfield(candidate, 'mode') && ~isempty(candidate.mode)
                    initCfg = candidate;
                end
            end
        end

        function text = formatMetric(~, value, decimals)
            %FORMATMETRIC Format a numeric metric with fixed decimals.
            if ~isfinite(value)
                text = 'N/A';
                return;
            end
            text = sprintf(['%0.' num2str(decimals) 'f'], value);
        end

        function line = buildAlignedRow(~, values, widths)
            %BUILDALIGNEDROW Build one padded plain-text table row.
            parts = cell(1, numel(values));
            for i = 1:numel(values)
                parts{i} = char(pad(string(values{i}), widths(i), 'right'));
            end
            line = sprintf('| %s |', strjoin(parts, ' | '));
        end

        function line = buildSeparator(~, widths)
            %BUILDSEPARATOR Build a horizontal separator for the table.
            parts = cell(1, numel(widths));
            for i = 1:numel(widths)
                parts{i} = repmat('-', 1, widths(i));
            end
            line = sprintf('+-%s-+', strjoin(parts, '-+-'));
        end

        function label = getRunLabelFromSaved(~, saved, resultsDir)
            %GETRUNLABELFROMSAVED Return a stable run label for a saved batch child.
            runIndex = [];
            if isfield(saved.cfgSnapshot, 'sim') && isfield(saved.cfgSnapshot.sim, 'batchRunIndex') ...
                    && ~isempty(saved.cfgSnapshot.sim.batchRunIndex)
                runIndex = saved.cfgSnapshot.sim.batchRunIndex;
            else
                [~, savedRunName] = fileparts(resultsDir);
                token = regexp(savedRunName, 'run_(\d+)$', 'tokens', 'once');
                if ~isempty(token)
                    runIndex = str2double(token{1});
                end
            end
            if isempty(runIndex) || ~isfinite(runIndex)
                [~, label] = fileparts(resultsDir);
            else
                label = sprintf('Run %d', runIndex);
            end
        end

        function output = captureConsole(obj, callback)
            %CAPTURECONSOLE Capture command-window output using evalc.
            obj.consoleCaptureCallback = callback;
            cleanup = onCleanup(@() obj.clearConsoleCaptureCallback());
            output = evalc('obj.invokeConsoleCaptureCallback();');
            fprintf('%s', output);
            clear cleanup
        end

        function invokeConsoleCaptureCallback(obj)
            %INVOKECONSOLECAPTURECALLBACK Execute the stored console callback.
            if isempty(obj.consoleCaptureCallback)
                return;
            end
            obj.consoleCaptureCallback();
        end

        function clearConsoleCaptureCallback(obj)
            %CLEARCONSOLECAPTURECALLBACK Reset the stored console callback.
            obj.consoleCaptureCallback = [];
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
