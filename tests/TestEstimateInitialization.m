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
