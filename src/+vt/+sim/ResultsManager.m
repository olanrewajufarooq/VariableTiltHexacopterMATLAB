classdef ResultsManager
    %RESULTSMANAGER Handles results directory setup, persistence, and loading.
    %   Manages creation of timestamped results directories, saving/loading
    %   simulation data (.mat), and text file I/O for run artifacts.

    methods (Static)
        function resultsDir = createResultsDir(cfg, repoRoot, runName)
            %CREATERESULTSDIR Create a run-specific results folder.
            %   Inputs:
            %     cfg - config with controller and traj fields.
            %     repoRoot - repository root path.
            %     runName - pre-built run name string.
            %   Output:
            %     resultsDir - full path to the new results directory.
            baseDir = fullfile(repoRoot, 'results');
            if ~exist(baseDir, 'dir')
                mkdir(baseDir);
            end

            subfolder = 'nominal';
            if isfield(cfg.controller, 'adaptation') && ~strcmpi(cfg.controller.adaptation, 'none')
                subfolder = 'adaptive';
            end

            runDir = fullfile(baseDir, subfolder);
            if ~exist(runDir, 'dir')
                mkdir(runDir);
            end

            resultsDir = fullfile(runDir, runName);
            mkdir(resultsDir);
        end

        function runName = buildRunName(cfg, isBatch)
            %BUILDRUNNAME Build a timestamped run name from config.
            %   Inputs:
            %     cfg - config with traj and controller fields.
            %     isBatch - true for batch mode.
            %   Output:
            %     runName - string like '20260426_123456_inf3d_ff_lie'.
            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            trajName = vt.sim.NamingUtils.batchTrajectoryLabel(cfg);
            if isBatch
                runName = sprintf('%s_%s', timestamp, trajName);
            else
                ctrlType = vt.sim.NamingUtils.controllerLabel(cfg);
                potential = vt.sim.NamingUtils.potentialLabel(cfg);
                runName = sprintf('%s_%s_%s_%s', timestamp, trajName, ctrlType, potential);
            end
        end

        function persistRun(resultsDir, logs, metrics, est, runInfo, cfgSnapshot)
            %PERSISTRUN Save finalized run artifacts to disk.
            %   Inputs: resultsDir, logs, metrics, est, runInfo, cfgSnapshot.
            dataPath = fullfile(resultsDir, 'sim_data.mat');
            save(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
            fprintf('Run data saved to: %s\n', dataPath);
        end

        function saved = loadRun(resultsDir)
            %LOADRUN Load saved run data from disk.
            %   Input: resultsDir - path containing sim_data.mat.
            %   Output: saved struct with logs, metrics, est, runInfo, cfgSnapshot.
            dataPath = fullfile(resultsDir, 'sim_data.mat');
            if ~exist(dataPath, 'file')
                error('ResultsManager:MissingSavedData', 'Missing sim_data.mat in %s', resultsDir);
            end
            saved = load(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
        end

        function writeTextFile(path, content)
            %WRITETEXTFILE Write UTF-8 text content to a file.
            fid = fopen(path, 'w');
            if fid < 0
                error('ResultsManager:WriteFailed', 'Unable to write file: %s', path);
            end
            cleanup = onCleanup(@() fclose(fid));
            fprintf(fid, '%s\n', content);
        end

        function content = readTextFile(path)
            %READTEXTFILE Read a UTF-8 text file.
            if ~exist(path, 'file')
                content = '';
                return;
            end
            content = fileread(path);
        end

        function plotSavedRun(resultsDir, plotType)
            %PLOTSAVEDRUN Generate plots for one saved run directory.
            if string(plotType) == "none"
                return;
            end
            saved = vt.sim.ResultsManager.loadRun(resultsDir);
            cfgSnapshot = saved.cfgSnapshot;
            logs = saved.logs;
            est = saved.est;
            runInfo = saved.runInfo;
            isAdaptive = logical(runInfo.isAdaptive);

            layoutType = 'row-major';
            if isprop(cfgSnapshot, 'viz') && isfield(cfgSnapshot.viz, 'plotLayout') && ~isempty(cfgSnapshot.viz.plotLayout)
                layoutType = cfgSnapshot.viz.plotLayout;
            end

            plotterObj = vt.plot.Plotter(resultsDir, struct('savePng', true, 'duration', runInfo.duration));
            if isAdaptive
                summaryFig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
                plotterObj.plotSummaryAdaptive(logs, est, summaryFig, layoutType);
            else
                summaryFig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
                plotterObj.plotSummaryNominal(logs, summaryFig, layoutType);
            end

            if string(plotType) == "all"
                if isAdaptive
                    plotterObj.plotStandaloneSubplotsAdaptive(logs, est);
                    plotterObj.plotStackedEstimation(est);
                    plotterObj.plotStackedInertia(est);
                else
                    plotterObj.plotStandaloneSubplotsNominal(logs);
                end
                plotterObj.plotStackedAllState(logs);
                plotterObj.plotStackedPositionOrientation(logs);
                plotterObj.plotStackedVelocity(logs);
                plotterObj.plotStackedWrench(logs);
            end
        end

        function dirs = findChildResultDirs(resultsDir)
            %FINDCHILDRESULTDIRS Discover saved batch child result directories.
            dirs = {};
            if ~exist(resultsDir, 'dir')
                return;
            end
            listing = dir(fullfile(resultsDir, '**', 'sim_data.mat'));
            if isempty(listing)
                return;
            end
            dirs = cell(numel(listing), 1);
            for i = 1:numel(listing)
                dirs{i} = listing(i).folder;
            end
            dirs = sort(dirs);
        end

        function root = repoRoot()
            %REPOROOT Return repository root path.
            p = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(p))));
        end
    end
end
