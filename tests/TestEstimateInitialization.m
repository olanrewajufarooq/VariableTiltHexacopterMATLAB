classdef TestEstimateInitialization < matlab.unittest.TestCase
    %TESTESTIMATEINITIALIZATION Unit tests for estimate init mode behavior.

    methods (Test)
        function testWrenchControllerSetEstimateThetaUpdatesEuclideanAdaptation(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            ctrl = vt.ctrl.WrenchController(cfg);
            theta = (1:10).';
            ctrl.setEstimateTheta(theta);
            [m_hat, cog_hat, I_hat] = ctrl.getEstimate();

            testCase.verifyEqual(I_hat(:), theta(1:6));
            testCase.verifyEqual(m_hat, theta(7));
            testCase.verifyEqual(cog_hat(:), theta(8:10) ./ theta(7), 'AbsTol', 1e-12);
        end

        function testEuclideanAdaptationRegressorUsesPositiveCoriolisSign(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([1; zeros(9,1)]);
            cfg.setAdaptationParams(0.001);
            cfg.done();

            adapt = vt.ctrl.adapt.EuclideanAdaptation(cfg);
            Hd = eye(4);
            H = eye(4);
            Vd = zeros(6,1);
            V = [0.3; -0.2; 0.1; 0.4; -0.5; 0.6];
            Ades = zeros(6,1);
            dt = 0.001;

            [~, ~, I_before] = adapt.getEstimate();
            adapt.update(Hd, H, Vd, V, Ades, dt);
            [~, ~, I_after] = adapt.getEstimate();

            B1 = zeros(6,6);
            B1(1,1) = 1;
            expectedColumn = vt.se3.adV(V)' * B1 * V;
            expectedDelta = dt * (expectedColumn.' * V);

            testCase.verifyEqual(I_after(1) - I_before(1), expectedDelta, 'AbsTol', 1e-12);
        end

        function testSetEstimateInitializationAcceptsFixedHigher(testCase)
            cfg = vt.config.Config();
            cfg.setEstimateInitialization('fixed-higher');
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed-higher');
        end

        function testSetEstimateInitializationCustomVectorStoredAsFixed(testCase)
            cfg = vt.config.Config();
            theta = 1:10;
            cfg.setEstimateInitialization(theta);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, theta(:));
        end

        function testSetEstimateInitializationRandomWithSeedSpecStored(testCase)
            cfg = vt.config.Config();
            cfg.setEstimateInitialization('random', 1234);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'random');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, 1234);
        end

        function testSetEstimateInitializationFixedHigherWithExplicitSpec(testCase)
            cfg = vt.config.Config();
            theta = (21:30).';
            cfg.setEstimateInitialization('fixed-higher', theta);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed-higher');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, theta);
        end
    end
end
