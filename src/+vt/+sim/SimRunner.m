classdef SimRunner < handle
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
            if isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                fprintf('  Adaptation: %s\n', obj.cfg.controller.adaptation);
            end
            fprintf('  Duration: %.1f s, dt: %.4f s\n', obj.duration, obj.dt);

            obj.traj = vt.traj.PathGenerator(obj.cfg);
            obj.plant = vt.plant.HexacopterPlant(obj.cfg);
            obj.ctrl = obj.createController();
            obj.log = vt.core.Logger();

            [H0, V0, ~] = obj.traj.generate(0);
            obj.plant.reset(H0, V0);
        end

        function run(obj, isAdaptive, payloadMass, payloadCoG, payloadDropTime, startWithTrueValues)
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

            obj.setupVisualization();
            obj.setupAdaptivePayload(isAdaptive, payloadMass, payloadCoG, startWithTrueValues);

            obj.tCurrent = 0;
            obj.kCurrent = 1;
            obj.stopped_ = false;
            obj.resetEstimationLogs();

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
            ctrl = vt.ctrl.WrenchController(obj.cfg);
        end

        function setupResultsDir(obj)
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
            actMethod = obj.cfg.act.method;

            obj.runName = sprintf('%s_%s_%s_%s_%s', timestamp, trajName, ctrlType, potential, actMethod);
            obj.resultsDir = fullfile(runDir, obj.runName);
            mkdir(obj.resultsDir);
        end

        function setupVisualization(obj)
            obj.plotter = vt.plot.Plotter(obj.resultsDir, struct('savePng', true, 'duration', obj.duration));
            embedUrdf = false;
            if isfield(obj.cfg.viz, 'embedUrdf')
                embedUrdf = logical(obj.cfg.viz.embedUrdf);
            end
            useRobotics = embedUrdf;

            if obj.cfg.viz.enable && obj.cfg.viz.liveSummary
                obj.figLive = figure('Name','Live View','Position',[50 50 1600 900]);
                if embedUrdf
                    obj.plotter.plotLiveNominal(struct('t', []), obj.figLive, true);
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

        function resetEstimationLogs(obj)
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];
        end

        function simulationStepNominal(obj, k)
            [Hd, Vd, ~] = obj.traj.generate(obj.tCurrent);
            [H, V] = obj.plant.getState();
            Ad = zeros(6,1);

            W_cmd = obj.computeControl(Hd, H, Vd, V, Ad);
            obj.logStep(Hd, Vd, Ad, W_cmd, k);
            obj.recordEstimate();
        end

        function simulationStepAdaptive(obj, k)
            [Hd, Vd, Ad] = obj.traj.generate(obj.tCurrent);
            [H, V] = obj.plant.getState();
            W_cmd = obj.computeControl(Hd, H, Vd, V, Ad);
            obj.logStep(Hd, Vd, Ad, W_cmd, k);
            obj.recordEstimate();
        end

        function W_cmd = computeControl(obj, Hd, H, Vd, V, Ad)
            W_cmd = obj.ctrl.computeWrench(Hd, H, Vd, V, Ad, obj.dt);
            obj.plant.step(obj.dt, W_cmd);
        end

        function actual = buildActual(obj, H, V)
            pos = H(1:3,4);
            rpy = vt.utils.rotm2rpy(H(1:3,1:3)).';
            actual = struct('pos', pos', 'rpy', rpy, 'linVel', V(4:6)', 'angVel', V(1:3)');
        end

        function desired = buildDesired(obj, Hd, Vd, Ad)
            pos = Hd(1:3,4);
            rpy = vt.utils.rotm2rpy(Hd(1:3,1:3)).';
            Ad = Ad(:);
            desired = struct('pos', pos', 'rpy', rpy, 'linVel', Vd(4:6)', 'angVel', Vd(1:3)', 'acc6', Ad.');
        end

        function cmd = buildCmd(obj, W_cmd)
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)');
        end

        function logStep(obj, Hd, Vd, Ad, W_cmd, k)
            [Hn, Vn] = obj.plant.getState();
            actual = obj.buildActual(Hn, Vn);
            desired = obj.buildDesired(Hd, Vd, Ad);
            cmd = obj.buildCmd(W_cmd);
            obj.log.append(obj.tCurrent, actual, desired, cmd);

            pos_actual = Hn(1:3,4);
            pos_des = Hd(1:3,4);
            obj.updateVisualization(pos_des, pos_actual, Hn, k);
            obj.checkSafety(pos_actual);
        end

        function recordEstimate(obj)
            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            if isempty(m_hat)
                return;
            end
            obj.estTimeLog(end+1) = obj.tCurrent;
            obj.massLog(end+1) = m_hat;
            obj.comLog(end+1,:) = cog_hat(:).';
            obj.inertiaLog(end+1,:) = Iparams_hat(:).';
        end

        function value = getPayloadField(obj, fieldName, defaultValue)
            value = defaultValue;
            if isfield(obj.cfg, 'payload') && isfield(obj.cfg.payload, fieldName)
                candidate = obj.cfg.payload.(fieldName);
                if ~isempty(candidate)
                    value = candidate;
                end
            end
        end

        function updateVisualization(obj, pos_des, pos_actual, Hn, k)
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
                obj.figLive = obj.plotter.plotLiveNominal(logsNow, obj.figLive, embedUrdf);
            end

            if doUpdate
                drawnow limitrate;
            end
        end

        function checkSafety(obj, pos_actual)
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

            est = [];
            if isAdaptive
                est = obj.getEstimationData(logs);
            end

            runInfo = struct('isAdaptive', isAdaptive, 'duration', obj.duration, 'dt', obj.dt, 'runName', obj.runName);
            dataPath = fullfile(obj.resultsDir, 'sim_data.mat');
            cfg = obj.cfg;
            save(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfg');

            if ~isempty(obj.figLive) && isvalid(obj.figLive)
                obj.plotter.saveFigure(obj.figLive, 'live_summary');
            end
            if ~isempty(obj.viewer) && isvalid(obj.viewer.fig)
                obj.plotter.saveFigure(obj.viewer.fig, 'urdf_view');
            end

            fprintf('Results saved to: %s\n', obj.resultsDir);
        end

        function est = getEstimationData(obj, logs)
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
            cog_base = cog_base(:).';

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
                est.comActual(idx,:) = repmat(cog_base, sum(idx), 1);
                est.inertiaActual(idx,:) = repmat(I_base, sum(idx), 1);
            else
                est.massActual = m_with * ones(numel(t), 1);
                est.comActual = repmat(cog_with, numel(t), 1);
                est.inertiaActual = repmat(I_with, numel(t), 1);
            end
        end
    end
end
