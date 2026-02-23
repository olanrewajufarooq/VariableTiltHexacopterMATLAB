%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop

clear; close all;
startup;

cfg = vt.config.Config();

duration = 30; % seconds

cfg.setSimParams(0.005, duration) ...                  % dt, duration
   .setTrajectory('takeoffland') ...               % 'hover','circle','square','infinity','takeoffland'
   .setController('FeedLin') ...                   % 'PD','FeedLin','Feedforward'
   .setPotentialType('liealgebra') ...        % 'liealgebra' or 'separate'
   .setAdaptation('euclidean') ...            % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
   .setLiveView(true, true, 100, false);      % enable, liveSummary, updateEvery, embedUrdf

cfg.setPayload( ...
   1.75, ...                     % mass
   [0.005; 0.001; -0.025], ...   % comOffset
   duration/2, ...               % dropTime
   true ...                      % visualize in live view after drop (default false)
);

sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
