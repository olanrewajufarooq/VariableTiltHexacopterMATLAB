%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

clear; close all;
startup;

cfg = vt.config.Config();

duration = 30; % seconds

cfg.setSimParams(0.005, duration) ...                  % dt, duration
   .setTrajectory('circle', 1.25) ...               % 'hover','circle','infinity','infinity3d','takeoffland'
   .setTrajectoryMethod('precomputed') ...          % 'precomputed','modelreference'
   .setController('Feedforward') ...                   % 'PD','FeedLin','Feedforward'
   .setPotentialType('separate') ...        % 'liealgebra' or 'separate'
   .setAdaptation('euclidean') ...            % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
   .setLiveView(true, true, 100, true);      % enable, liveSummary, updateEvery, embedUrdf

cfg.setPayload( ...
   0.5, ...                     % mass
   [0.015; 0.05; -0.025], ...   % comOffset
   duration/2, ...               % dropTime
   false ...                     % initialize adaptive estimate with payload properties
);

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
