classdef TestResultsManager < matlab.unittest.TestCase
    %TESTRESULTSMANAGER Unit tests for result persistence helper behavior.

    methods (Test)
        function testBuildRunNameSingleAndBatch(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('Feedforward', 'liealgebra');

            singleName = vt.sim.ResultsManager.buildRunName(cfg, false);
            batchName = vt.sim.ResultsManager.buildRunName(cfg, true);

            testCase.verifyTrue(~isempty(regexp(singleName, '^\d{8}_\d{6}_circle_ff_lie$', 'once')));
            testCase.verifyTrue(~isempty(regexp(batchName, '^\d{8}_\d{6}_circle$', 'once')));
        end

        function testWriteAndReadTextFileRoundtrip(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            filePath = fullfile(rootDir, 'notes.txt');

            vt.sim.ResultsManager.writeTextFile(filePath, sprintf('hello%sworld', newline));
            content = vt.sim.ResultsManager.readTextFile(filePath);
            testCase.verifyTrue(contains(content, 'hello'));
            testCase.verifyTrue(contains(content, 'world'));
        end

        function testPersistAndLoadRunRoundtrip(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            logs = struct('actual', struct('pos', [0 0 0]), 'des', struct('pos', [0 0 0]));
            metrics = struct('combined', struct('rmse_total', 0.1, 'tracking_score', 99));
            est = struct('mass', 3.5);
            runInfo = struct('isAdaptive', true, 'duration', 1, 'dt', 0.01);
            cfg = vt.config.Config();

            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);
            saved = vt.sim.ResultsManager.loadRun(rootDir);

            testCase.verifyEqual(saved.metrics.combined.rmse_total, 0.1);
            testCase.verifyEqual(saved.est.mass, 3.5);
            testCase.verifyEqual(saved.runInfo.isAdaptive, true);
            testCase.verifyEqual(saved.cfgSnapshot.traj.name, cfg.traj.name);
        end

        function testLoadRunMissingDataThrows(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            testCase.verifyError(@() vt.sim.ResultsManager.loadRun(rootDir), ...
                'ResultsManager:MissingSavedData');
        end

        function testFindChildResultDirsReturnsSortedRunDirs(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            run1 = fullfile(rootDir, 't02_inf3d', 'run_002');
            run2 = fullfile(rootDir, 't01_circle', 'run_001');
            mkdir(run1);
            mkdir(run2);
            save(fullfile(run1, 'sim_data.mat'), 'rootDir');
            save(fullfile(run2, 'sim_data.mat'), 'rootDir');

            dirs = vt.sim.ResultsManager.findChildResultDirs(rootDir);
            testCase.verifyEqual(dirs, sort({run1; run2}));
        end

        function testCreateResultsDirUsesAdaptiveSubfolder(testCase)
            repoRoot = tempname;
            mkdir(repoRoot);
            c = onCleanup(@() rmdir(repoRoot, 's'));

            cfg = vt.config.Config();
            cfg.setAdaptation('euclidean');
            runDir = vt.sim.ResultsManager.createResultsDir(cfg, repoRoot, 'sample_run');
            testCase.verifyTrue(exist(runDir, 'dir') == 7);
            testCase.verifyTrue(contains(runDir, fullfile('results', 'adaptive', 'sample_run')));
        end
    end
end
