classdef TestConfig < matlab.unittest.TestCase
    %TESTCONFIG Unit tests for the Config fluent API and validation logic.

    methods (Test)
        function testDefaultConstruction(testCase)
            cfg = vt.config.Config();
            testCase.verifyEqual(cfg.traj.name, 'hover');
            testCase.verifyEqual(cfg.controller.type, 'PD');
            testCase.verifyEqual(cfg.controller.adaptation, 'none');
            testCase.verifyEqual(cfg.vehicle.g, 9.8);
            testCase.verifyTrue(cfg.vehicle.m > 0);
        end

        function testSetTrajectoryValid(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            testCase.verifyEqual(cfg.traj.name, 'circle');
        end

        function testSetTrajectoryWithCycles(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('infinity', 3);
            testCase.verifyEqual(cfg.traj.name, 'infinity');
            testCase.verifyEqual(cfg.traj.cycles, 3);
        end

        function testSetControllerPD(testCase)
            cfg = vt.config.Config();
            cfg.setController('PD');
            testCase.verifyEqual(lower(cfg.controller.type), 'pd');
        end

        function testSetControllerFeedLin(testCase)
            cfg = vt.config.Config();
            cfg.setController('FeedLin');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedlin');
        end

        function testSetControllerFeedforward(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedforward');
        end

        function testSetControllerInvalidThrows(testCase)
            cfg = vt.config.Config();
            testCase.verifyError(@() cfg.setController('BadType'), ...
                'MATLAB:error');
        end

        function testSetPotentialType(testCase)
            cfg = vt.config.Config();
            cfg.setController('PD', 'separate');
            testCase.verifyEqual(cfg.controller.potential, 'separate');
        end

        function testDefaultPotentialIsLieAlgebra(testCase)
            cfg = vt.config.Config();
            cfg.setController('PD');
            testCase.verifyEqual(cfg.controller.potential, 'liealgebra');
        end

        function testSetAdaptationEuclidean(testCase)
            cfg = vt.config.Config();
            cfg.setAdaptation('euclidean');
            testCase.verifyEqual(cfg.controller.adaptation, 'euclidean');
            testCase.verifyTrue(~isempty(cfg.controller.Gamma));
        end

        function testSetAdaptationNone(testCase)
            cfg = vt.config.Config();
            cfg.setAdaptation('none');
            testCase.verifyEqual(cfg.controller.adaptation, 'none');
        end

        function testSetSimParams(testCase)
            cfg = vt.config.Config();
            cfg.setSimParams(0.01, 20);
            testCase.verifyEqual(cfg.sim.dt, 0.01);
            testCase.verifyEqual(cfg.sim.duration, 20);
        end

        function testSetKpGainsVector(testCase)
            cfg = vt.config.Config();
            Kp = [1; 2; 3; 4; 5; 6];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp);
        end

        function testSetKpGainsRowVector(testCase)
            cfg = vt.config.Config();
            Kp = [1, 2, 3, 4, 5, 6];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp(:));
        end

        function testSetKpGainsMatrix(testCase)
            cfg = vt.config.Config();
            Kp = [1 2 3 4 5 6; 7 8 9 10 11 12];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp);
        end

        function testSetKdGainsVector(testCase)
            cfg = vt.config.Config();
            Kd = [1; 2; 3; 4; 5; 6];
            cfg.setKdGains(Kd);
            testCase.verifyEqual(cfg.controller.Kd, Kd);
        end

        function testSetAdaptiveGainsVector(testCase)
            cfg = vt.config.Config();
            Gamma = ones(10, 1);
            cfg.setAdaptiveGains(Gamma);
            testCase.verifyEqual(cfg.controller.Gamma, Gamma);
        end

        function testSetAdaptiveGainsMatrix(testCase)
            cfg = vt.config.Config();
            Gamma = [ones(1, 10); 2*ones(1, 10)];
            cfg.setAdaptiveGains(Gamma);
            testCase.verifyEqual(cfg.controller.Gamma, Gamma);
        end

        function testFluentChaining(testCase)
            cfg = vt.config.Config();
            cfg = cfg.setTrajectory('circle') ...
                     .setController('Feedforward') ...
                     .setSimParams(0.005, 30);
            testCase.verifyEqual(cfg.traj.name, 'circle');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedforward');
            testCase.verifyEqual(cfg.sim.duration, 30);
        end

        function testDoneNormalizesDefaults(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.done();
            testCase.verifyTrue(~isempty(cfg.controller.Kp));
            testCase.verifyTrue(~isempty(cfg.controller.Kd));
            testCase.verifyEqual(numel(cfg.controller.Kp), 6);
            testCase.verifyEqual(numel(cfg.controller.Kd), 6);
        end

        function testDoneAdaptiveGetsGamma(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.done();
            testCase.verifyTrue(~isempty(cfg.controller.Gamma));
            testCase.verifyEqual(numel(cfg.controller.Gamma), 10);
        end

        function testCopyIsIndependent(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfgCopy = cfg.copy();
            cfgCopy.setTrajectory('hover');
            testCase.verifyEqual(cfg.traj.name, 'circle');
            testCase.verifyEqual(cfgCopy.traj.name, 'hover');
        end

        function testGetBatchCountSingle(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 1);
        end

        function testGetBatchCountMultiTrajectory(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory({'circle', 'infinity'});
            cfg.setController('PD');
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
        end

        function testGetBatchCountMultiGain(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.setKpGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.setKdGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
        end

        function testInconsistentBatchCountsThrow(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.setKpGains([1 2 3 4 5 6; 7 8 9 10 11 12]);  % 2 runs
            cfg.setKdGains([1 2 3 4 5 6; 7 8 9 10 11 12; 1 1 1 1 1 1]);  % 3 runs
            testCase.verifyError(@() cfg.getBatchCount(), ...
                'Config:InconsistentBatchCounts');
        end

        function testSetPayloadScenario(testCase)
            cfg = vt.config.Config();
            cfg.setPayloadScenario(0.5, [0.01; 0; -0.02], 15);
            testCase.verifyEqual(cfg.payload.mass, 0.5);
            testCase.verifyEqual(cfg.payload.CoG, [0.01; 0; -0.02]);
            testCase.verifyEqual(cfg.payload.dropTime, 15);
        end

        function testSetEstimateInitializationNominal(testCase)
            cfg = vt.config.Config();
            cfg.setEstimateInitialization('nominal');
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'nominal');
        end

        function testSetEstimateInitializationFixed(testCase)
            cfg = vt.config.Config();
            theta = (1:10)';
            cfg.setEstimateInitialization(theta);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, theta);
        end

        function testSetEstimateInitializationInvalidModeThrows(testCase)
            cfg = vt.config.Config();
            testCase.verifyError(@() cfg.setEstimateInitialization('badmode'), ...
                'Config:InvalidEstimateInitializationMode');
        end

        function testSetPlotLayoutValid(testCase)
            cfg = vt.config.Config();
            cfg.setPlotLayout('row-major');
            testCase.verifyEqual(cfg.viz.plotLayout, 'row-major');
        end

        function testSetControlParams(testCase)
            cfg = vt.config.Config();
            cfg.setControlParams(0.01);
            testCase.verifyEqual(cfg.sim.control_dt, 0.01);
        end

        function testSetAdaptationParams(testCase)
            cfg = vt.config.Config();
            cfg.setAdaptationParams(0.002);
            testCase.verifyEqual(cfg.sim.adaptation_dt, 0.002);
            testCase.verifyFalse(cfg.sim.adaptation_dt_auto);
        end

        function testVehicleI6Computed(testCase)
            cfg = vt.config.Config();
            testCase.verifyTrue(isfield(cfg.vehicle, 'I6'));
            testCase.verifyEqual(size(cfg.vehicle.I6), [6, 6]);
            % I6 should be symmetric positive-definite
            testCase.verifyEqual(cfg.vehicle.I6, cfg.vehicle.I6', 'AbsTol', 1e-14);
            testCase.verifyTrue(all(eig(cfg.vehicle.I6) > 0));
        end

        function testVisualizationDefaults(testCase)
            cfg = vt.config.Config();
            testCase.verifyFalse(cfg.viz.enable);
            testCase.verifyTrue(cfg.viz.dynamicAxis);
            testCase.verifyEqual(cfg.viz.plotLayout, 'column-major');
        end
    end
end
