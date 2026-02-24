%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

clear; close all;
startup;

cfg = vt.config.Config();

duration = 30; % seconds

cfg.setSimParams(0.005, duration);          % sim dt, duration
cfg.setAdaptationParams(0.005);             % adaptation dt
cfg.setControlParams(0.01);                 % control dt

cfg.setTrajectory('lissajous3d', 1.25);                % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland',
cfg.setTrajectoryMethod('precomputed');          % 'precomputed','modelreference'

cfg.setController('Feedforward');                  % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'
cfg.setAdaptation('euclidean');            % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'

cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(500);
cfg.setLiveUrdfEmbedding(true);
cfg.setPlotLayout('column-major');

cfg.setPayload( ...
   1.5, ...                     % mass
   [0.115; 0.05; -0.05], ...   % comOffset
   2*duration/3, ...               % dropTime
   false ...                     % initialize adaptive estimate with payload properties
);
cfg.done();

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save( ...
   true, ...            % whether to save the simulation data
   'summary' ...        % plotting mode: 'summary' (default), 'all' (plot all possible plots), 'none' (no plots)
); 
