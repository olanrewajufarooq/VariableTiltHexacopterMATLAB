%RUN_NOMINAL_DEMO Nominal (non-adaptive) simulation

clear; close all;
startup;

cfg = vt.config.Config();

cfg.setSimParams(0.005, 60) ...                  % dt, duration
   .setTrajectory('square', 1.25) ...               % 'hover','circle','square','infinity','takeoffland'
   .setController('Feedforward') ...                   % 'PD','FeedLin','Feedforward'
   .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
   .setAdaptation('none') ...                 % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
   .setLiveView(true, true, 50, false);       % enable, liveSummary, updateEvery, embedUrdf

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
