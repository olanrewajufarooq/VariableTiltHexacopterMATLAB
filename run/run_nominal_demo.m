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

cfg.enableLiveView(true);               % enable live view
cfg.setLiveSummary(true);             % show live summary
cfg.setLiveUpdateRate(100);           % update live view every N control steps (lower is more responsive, but more computationally expensive)
cfg.setLiveUrdfEmbedding(false);      % embed urdf in live view (true) or load from file (false; faster startup, but won't reflect changes to urdf)
cfg.setPlotLayout('column-major');    % layout for live view plots; 'column-major' (default) fills columns first, 'row-major' fills rows first
cfg.done();

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save( ...
  true, ...            % whether to save the simulation data
  'summary' ...        % plotting mode: 'summary' (default), 'all' (plot all possible plots), 'none' (no plots)
);         % first arg is whether to save, second arg is plotting mode: 'summary' (default), 'all' (plots all saved data), 'none' (no plots) 
