function run_adaptive_demo()

    clear; close all;
    startup;

    % ---- User-tunable parameters (via Config class)
    cfg = vt.config.Config();
    
    cfg.setTrajectory('hover') ...               % 'hover','circle','square','infinity','takeoffland'
       .setController('Adaptive') ...             % 'Adaptive'
       .setSimParams(0.005, 30) ...               % dt, duration
       .setActuationMethod('fixed_tilt') ...      % 'fixed_tilt' or 'variable_tilt'
       .setPotentialType('liealgebra');           % 'liealgebra' or 'separate'

    payloadMass = 0.75;
    payloadCoG = [0.005; 0.001; -0.025];
    payloadDropTime = 20;

    fprintf('========================================\n');
    fprintf('Adaptive Control Demo with Payload Drop\n');
    fprintf('========================================\n');
    fprintf('  Trajectory: %s\n', cfg.traj.name);
    fprintf('  Duration: %.1f s\n', cfg.sim.duration);
    fprintf('  Payload mass: %.2f kg\n', payloadMass);
    fprintf('  Drop time: %.1f s\n', payloadDropTime);
    fprintf('========================================\n\n');

    traj = vt.traj.PathGenerator(cfg);
    ctrl = vt.ctrl.AdaptiveController(cfg);
    alloc = vt.alloc.ControlAllocator(cfg);
    plant = vt.plant.HexacopterPlant(cfg);

    % Apply payload to plant
    [m_base, I_base, cog_base] = vt.utils.baseParams(cfg);
    plant.updateParameters(m_base, cog_base, I_base);

    [m_with, I_with, cog_with] = vt.utils.addPayload(m_base, I_base, cog_base, payloadMass, payloadCoG);
    plant.updateParameters(m_with, cog_with, I_with);
    ctrl.setPayloadEstimate(payloadMass, payloadCoG);

    fprintf('Initial total mass: %.3f kg\n', m_with);

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
        figSummary = figure('Position',[50 50 1500 900]);
    else
        figSummary = [];
    end
    N = floor(cfg.sim.duration / cfg.sim.dt) + 1;
    theta_log = zeros(10, N);
    mass_log = zeros(1, N);

    t = 0;
    dropped = false;
    k_end = 0;
    for k = 1:N
        if ~dropped && t >= payloadDropTime
            plant.updateParameters(m_base, cog_base, I_base);
            dropped = true;
        end

        [Hd, Vd, Ad] = traj.generate(t);
        [H, V] = plant.getState();

        W_cmd = ctrl.computeWrench(Hd, H, Vd, V, Ad, cfg.sim.dt);
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
        desired = struct('pos', pos_des', 'rpy', rpy_des, 'linVel', linVel_des', 'angVel', angVel_des', 'acc6', Ad');
        cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)', 'tilt', tiltAngles(:).');
        log.append(t, actual, desired, cmd);
        if cfg.viz.enable
            viewer.showPose(Hn);
            viewer.updatePaths(pos_des, pos_actual);
        end

        [m_hat, ~, ~] = ctrl.getEstimate();
        theta_log(:, k) = ctrl.theta_hat;
        mass_log(k) = m_hat;

        if cfg.viz.enable && cfg.viz.liveSummary && mod(k, cfg.viz.updateEvery) == 0
            logsNow = log.finalize();
            t_now = logsNow.t(:);
            theta_now = theta_log(:, 1:numel(t_now));
            mass_now = mass_log(1:numel(t_now));
            com_now = [theta_now(8,:)./mass_now; theta_now(9,:)./mass_now; theta_now(10,:)./mass_now]';
            inertia_now = [theta_now(1,:); theta_now(2,:); theta_now(3,:); theta_now(4,:); theta_now(6,:); theta_now(5,:)]';
            estNow.t = t_now;
            estNow.mass = mass_now(:);
            estNow.com = com_now;
            estNow.inertia = inertia_now;
            estNow.dropTime = payloadDropTime;
            figSummary = plotter.plotSummaryAdaptive(logsNow, estNow, figSummary);
            drawnow;
        end

        if ~strcmpi(cfg.traj.name, 'takeoffland') && (pos_des(3) > 0.05 || t > 0.2) && pos_actual(3) <= 0
            warning('Ground contact detected. Stopping simulation.');
            k_end = k;
            break;
        end

        t = t + cfg.sim.dt;
        k_end = k;
    end

    logs = log.finalize();
    if k_end == 0
        k_end = numel(logs.t);
    end
    logs.t = logs.t(1:k_end);

    metrics = vt.metrics.TrackingMetrics.computeAll(logs.actual.pos, logs.des.pos);
    vt.metrics.TrackingMetrics.printReport(metrics, [cfg.traj.name ' (Adaptive)']);

    t_vec = logs.t(:);
    theta_log = theta_log(:, 1:numel(t_vec));
    mass_log = mass_log(1:numel(t_vec));

    com_log = [theta_log(8,:)./mass_log; theta_log(9,:)./mass_log; theta_log(10,:)./mass_log]';
    inertia_log = [theta_log(1,:); theta_log(2,:); theta_log(3,:); theta_log(4,:); theta_log(6,:); theta_log(5,:)]';

    est.t = t_vec;
    est.mass = mass_log(:);
    est.com = com_log;
    est.inertia = inertia_log;
    est.dropTime = payloadDropTime;

    fprintf('\nGenerating plots...\n');
    plotter.plotSummaryAdaptive(logs, est, figSummary);
end
