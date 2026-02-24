%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

clear; close all;
startup;

cfg = vt.config.Config();

duration = 30; % seconds

cfg.setSimParams(0.005, duration) ...                  % dt, duration
   .setTrajectory('lissajous3d', 1.25) ...               % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland',
   .setTrajectoryMethod('precomputed') ...          % 'precomputed','modelreference'
   .setController('Feedforward') ...                   % 'PD','FeedLin','Feedforward'
   .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
   .setAdaptation('euclidean') ...            % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
   .setLiveView(true, true, 100, true);      % enable, liveSummary, updateEvery, embedUrdf

cfg.setPayload( ...
   1.5, ...                     % mass
   [0.115; 0.05; -0.05], ...   % comOffset
   2*duration/3, ...               % dropTime
   false ...                     % initialize adaptive estimate with payload properties
);

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
