function run_nominal_demo()
%RUN_NOMINAL_DEMO Nominal (non-adaptive) simulation

    clear; close all;
    startup;

    % ---- User-tunable parameters (via Config class)
    cfg = vt.config.Config();
    
    cfg.setTrajectory('hover') ...              % 'hover','circle','square','infinity','takeoffland'
       .setController('PD') ...                  % 'PD' or 'FeedLin'
       .setSimParams(0.005, 30) ...              % dt, duration
       .setActuationMethod('fixed_tilt') ...     % 'fixed_tilt' or 'variable_tilt'
       .setPotentialType('liealgebra');          % 'liealgebra' or 'separate'

    fprintf('Initializing components...\n');
    fprintf('  Trajectory: %s\n', cfg.traj.name);
    fprintf('  Controller: %s (%s)\n', cfg.controller.type, cfg.controller.potential);
    fprintf('  Duration: %.1f s, dt: %.4f s\n', cfg.sim.duration, cfg.sim.dt);

    traj = vt.traj.PathGenerator(cfg);
    ctrl = vt.ctrl.GeometricController(cfg);
    alloc = vt.alloc.ControlAllocator(cfg);
    plant = vt.plant.HexacopterPlant(cfg);

    [H0, V0, ~] = traj.generate(0);
    plant.reset(H0, V0);

    log = vt.core.Logger();
    plotter = vt.plot.Plotter(pwd, struct('savePng', false));
    if cfg.viz.enable
        viewer = vt.viz.UrdfViewer();
        viewer.setAxisLimits(vt.viz.defaultAxisLimits(cfg));
        viewer.setDynamicAxis(cfg.viz.dynamicAxis, cfg.viz.axisPadding);
    end
    if cfg.viz.enable && cfg.viz.liveSummary
        figSummary = figure('Position',[100 100 1400 800]);
    else
        figSummary = [];
    end
    N = floor(cfg.sim.duration / cfg.sim.dt) + 1;
    t = 0;

    fprintf('\nRunning simulation...\n');
    for k = 1:N
        [Hd, Vd, ~] = traj.generate(t);
        [H, V] = plant.getState();

        W_cmd = ctrl.computeWrench(Hd, H, Vd, V);
        [motorSpeeds, tiltAngles] = alloc.allocate(W_cmd);
        A = alloc.getAllocationMatrix(tiltAngles);
        thrusts = alloc.motorSpeedsToThrust(motorSpeeds);
        W_actual = A * thrusts(:);

        plant.step(cfg.sim.dt, W_actual);

        [Hn, Vn] = plant.getState();
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

        log.append(t, actual, desired, cmd);
        if cfg.viz.enable
            viewer.showPose(Hn);
            viewer.updatePaths(pos_des, pos_actual);
        end
        if cfg.viz.enable && cfg.viz.liveSummary && mod(k, cfg.viz.updateEvery) == 0
            logsNow = log.finalize();
            figSummary = plotter.plotSummaryNominal(logsNow, figSummary);
            drawnow;
        end

        if ~strcmpi(cfg.traj.name, 'takeoffland') && (pos_des(3) > 0.05 || t > 0.2) && pos_actual(3) <= 0
            warning('Ground contact detected. Stopping simulation.');
            break;
        end

        if any(~isfinite(pos_actual))
            warning('Numerical issue detected. Stopping simulation.');
            break;
        end
        t = t + cfg.sim.dt;
    end

    logs = log.finalize();
    fprintf('Simulation completed.\n\n');

    metrics = vt.metrics.TrackingMetrics.computeAll(logs.actual.pos, logs.des.pos);
    vt.metrics.TrackingMetrics.printReport(metrics, cfg.traj.name);

    fprintf('\nGenerating plots...\n');
    plotter.plotSummaryNominal(logs, figSummary);
    fprintf('Plotting done.\n');
end
