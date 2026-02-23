%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

clear; close all;
startup;

cfg = vt.config.Config();

duration = 30; % seconds

cfg.setSimParams(0.005, duration) ...                  % dt, duration
   .setTrajectory('infinity3d', 1.25) ...               % 'hover','circle','infinity','infinity3d','flip','takeoffland'
   .setTrajectoryMethod('modelreference') ...          % 'precomputed','modelreference'
   .setController('Feedforward') ...                   % 'PD','FeedLin','Feedforward'
   .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
   .setAdaptation('euclidean') ...            % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
   .setLiveView(true, true, 100, false);      % enable, liveSummary, updateEvery, embedUrdf

cfg.setPayload( ...
   1.5, ...                     % mass
   [0.005; 0.001; -0.025], ...   % comOffset
   duration/2, ...               % dropTime
   false ...                     % initialize adaptive estimate with payload properties
);

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
