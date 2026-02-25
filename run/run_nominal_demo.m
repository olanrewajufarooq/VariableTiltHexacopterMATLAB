%RUN_NOMINAL_DEMO Nominal hexacopter simulation (no adaptation).
%   Configures a baseline tracking scenario and produces summary plots.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = vt.config.Config();

% Timing and simulation horizon.
cfg.setControlParams(0.005);                 % control dt
cfg.setSimParams(0.005, 3);                 % sim dt, duration

% Reference trajectory setup.
cfg.setTrajectory('lissajous3d', 1.2);        % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland'
cfg.setTrajectoryMethod('precomputed');      % 'precomputed','modelreference'

% Controller and potential selection.
cfg.setController('Feedforward');          % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'
cfg.setAdaptation('none');                 % 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'

% Live visualization preferences.
cfg.enableLiveView(true);              % enable live view
cfg.setLiveSummary(true);              % show live summary
cfg.setLiveUpdateRate(100);            % update every N control steps
cfg.setLiveUrdfEmbedding(false);       % embed URDF (true) or load from file (false)
cfg.setPlotLayout('column-major');     % 'column-major' or 'row-major'
cfg.done();

% Run the simulation and save summary plots.
sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save( ...
  true, ...            % whether to save the simulation data
  'summary' ...        % plotting mode: 'summary' (default), 'all' (plot all possible plots), 'none' (no plots)
);         % first arg is whether to save, second arg is plotting mode: 'summary' (default), 'all' (plots all saved data), 'none' (no plots) 
