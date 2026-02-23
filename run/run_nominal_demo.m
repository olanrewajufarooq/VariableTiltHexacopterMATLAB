function run_nominal_demo()
%RUN_NOMINAL_DEMO Nominal (non-adaptive) simulation

    clear; close all;
    startup;

    cfg = vt.config.Config();
    
    cfg.setTrajectory('hover') ...               % 'hover','circle','square','infinity','takeoffland'
       .setController('PD') ...                   % 'PD','Geometric','Feedforward','Adaptive'
       .setSimParams(0.005, 30) ...               % dt, duration
       .setActuationMethod('fixed_tilt') ...      % 'fixed_tilt' or 'variable_tilt'
       .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
       .setLiveView(true, true, 10);              % enable, liveSummary, updateEvery

    sim = vt.sim.SimRunner(cfg);
    sim.setup();
    isAdaptive = false;
    sim.run(isAdaptive);
end
