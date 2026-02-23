function run_adaptive_demo()
%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

    clear; close all;
    startup;

    cfg = vt.config.Config();
    
    cfg.setTrajectory('hover') ...               % 'hover','circle','square','infinity','takeoffland'
       .setController('Adaptive') ...             % 'Adaptive'
       .setSimParams(0.005, 30) ...               % dt, duration
       .setActuationMethod('fixed_tilt') ...      % 'fixed_tilt' or 'variable_tilt'
       .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
       .setLiveView(true, true, 200);              % enable, liveSummary, updateEvery, embedUrdf (optional)

    payloadMass = 0.75;                           % [kg]
    payloadCoG = [0.005; 0.001; -0.025];         % [m] offset from CoM
    payloadDropTime = 20;                         % [s]
    startWithTrueValues = false;                  % true=init with true, false=nominal

    sim = vt.sim.SimRunner(cfg);
    sim.setup();
    isAdaptive = true;
    sim.run(isAdaptive, payloadMass, payloadCoG, payloadDropTime, startWithTrueValues);
end
