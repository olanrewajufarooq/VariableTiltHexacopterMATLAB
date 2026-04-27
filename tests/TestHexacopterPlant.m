classdef TestHexacopterPlant < matlab.unittest.TestCase
    %TESTHEXACOPTERPLANT Unit tests for plant state and ground defaults.

    methods (Test)
        function testConstructorReadsGroundDefaultsFromConfig(testCase)
            cfg = vt.config.Config();
            plant = vt.plant.HexacopterPlant(cfg);
            testCase.verifyTrue(plant.groundEnable);
            testCase.verifyEqual(plant.groundHeight, 0);
            testCase.verifyEqual(plant.groundStiffness, 5000);
            testCase.verifyEqual(plant.groundDamping, 200);
            testCase.verifyEqual(plant.groundFriction, 0.3);
        end

        function testResetSetsState(testCase)
            cfg = vt.config.Config();
            plant = vt.plant.HexacopterPlant(cfg);
            H0 = eye(4);
            H0(1:3,4) = [1; 2; 3];
            V0 = (1:6).';

            plant.reset(H0, V0);
            [H, V] = plant.getState();
            testCase.verifyEqual(H, H0);
            testCase.verifyEqual(V, V0);
        end

        function testStepWithZeroWrenchIsFinite(testCase)
            cfg = vt.config.Config();
            plant = vt.plant.HexacopterPlant(cfg);
            plant.reset(eye(4), zeros(6,1));
            plant.step(0.005, zeros(6,1));
            [H, V] = plant.getState();
            testCase.verifyTrue(all(isfinite(H), 'all'));
            testCase.verifyTrue(all(isfinite(V)));
        end

        function testUpdateParametersChangesMassCogAndInertia(testCase)
            cfg = vt.config.Config();
            plant = vt.plant.HexacopterPlant(cfg);
            newMass = cfg.vehicle.m + 1.0;
            newCoG = [0.1; -0.05; 0.02];
            newI = cfg.vehicle.I_params(:) + 0.01;

            plant.updateParameters(newMass, newCoG, newI);

            testCase.verifyEqual(plant.m, newMass);
            testCase.verifyEqual(plant.CoG, newCoG);
            testCase.verifyEqual(size(plant.I6), [6 6]);
            testCase.verifyTrue(all(isfinite(plant.I6), 'all'));
        end
    end
end
