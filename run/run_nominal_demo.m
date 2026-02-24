%RUN_NOMINAL_DEMO Nominal (non-adaptive) simulation

clear; close all;
startup;

cfg = vt.config.Config();

cfg.setControlParams(0.005);                 % control dt
cfg.setSimParams(0.005, 30);                 % sim dt, duration
cfg.setTrajectory('lissajous3d', 1.2);         % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland',
cfg.setTrajectoryMethod('precomputed');             % 'precomputed','modelreference'
cfg.setController('Feedforward');                   % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'
cfg.setAdaptation('none');                 % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
cfg.setLiveView(true, true, 100, false);       % enable, liveSummary, updateEvery, embedUrdf
cfg.done();

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
