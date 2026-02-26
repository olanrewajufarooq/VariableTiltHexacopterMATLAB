%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop.
%   Demonstrates online parameter adaptation and payload change.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = vt.config.Config();

% Scenario duration in seconds.
duration = 60;

% Timing parameters.
cfg.setSimParams(0.005, duration);          % sim dt, duration
cfg.setAdaptationParams(0.005);             % adaptation dt
cfg.setControlParams(0.01);                 % control dt

% Reference trajectory setup.
cfg.setTrajectory('infinity3dmod', 1.25);    % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland'

% Controller and potential selection.
cfg.setController('Feedforward');          % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'

cfg.setAdaptation('euclidean');            % 'none','euclidean','geo-aware'
cfg.setAdaptiveGains(1e-2 * [8,8,12,0.4,0.4,0.4,36,12,12,12]); % adaptation gains for position and attitude
% cfg.setAdaptiveGains(1e-2 * [360,360,360,40,40,40,72,12,12,12]); % adaptation gains for position and attitude
% cfg.setAdaptiveGains(1e-2 * [8,8,12,0.4,0.4,0.4,36,120,120,120]); % adaptation gains for position and attitude

% Payload schedule (mass drop event).
cfg.setPayload( ...
   1.5, ...                    % mass
   [0.115; 0.05; -0.05], ...    % comOffset
   2*duration/3, ...            % dropTime
   false ...                    % initialize adaptive estimate with payload properties
);

cfg.done();

% Run the simulation and save summary plots.
sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run();
sim.save( ...
   false, ...            % whether to save the simulation data
   'summary' ...        % plotting mode: 'summary' (default), 'all' (plot all possible plots), 'none' (no plots)
); 
