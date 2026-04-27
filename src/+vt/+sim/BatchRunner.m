classdef BatchRunner < handle
    %BATCHRUNNER Orchestrates multi-trajectory and multi-gain batch runs.
    %   Manages child SimRunner creation, per-run console capture, aggregate
    %   report generation, and batch plot dispatch.
    %
    %   Usage:
    %     br = vt.sim.BatchRunner(cfg, resultsDir, batchSize);
    %     br.runAll(runArgs);
    %     br.plotAll('summary');

    properties (Access = private)
        cfg
        resultsDir
        batchSize
        childDirs
        console
    end

    methods
        function obj = BatchRunner(cfg, resultsDir, batchSize)
            %BATCHRUNNER Create a batch runner.
            %   Inputs:
            %     cfg - vt.config.Config instance.
            %     resultsDir - root results directory for this batch.
            %     batchSize - total number of runs.
            obj.cfg = cfg;
            obj.resultsDir = resultsDir;
            obj.batchSize = batchSize;
            obj.childDirs = {};
            obj.console = vt.sim.ConsoleCapture();
        end

        function runAll(obj, runArgs)
            %RUNALL Execute all batch runs with per-run console capture.
            %   Input: runArgs - cell array of arguments for child.run().
            cfgs = obj.cfg.expandBatchConfigs(obj.resultsDir);
            obj.childDirs = cell(obj.batchSize, 1);
            for i = 1:obj.batchSize
                child = vt.sim.SimRunner(cfgs{i});
                runLog = obj.console.capture(@() obj.executeChild(child, runArgs));
                childLogPath = fullfile(child.resultsDir, 'command_window.txt');
                vt.sim.ResultsManager.writeTextFile(childLogPath, strtrim(runLog));
                obj.childDirs{i} = child.resultsDir;
                clear child
            end
            obj.writeAggregateArtifacts();
            fprintf('Batch results saved to: %s\n', obj.resultsDir);
        end

        function plotAll(obj, plotType)
            %PLOTALL Generate plots for each saved batch run.
            if isempty(obj.childDirs)
                obj.childDirs = vt.sim.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            if isempty(obj.childDirs)
                error('BatchRunner:NotRun', 'Batch simulation has not been run yet.');
            end
            for i = 1:numel(obj.childDirs)
                vt.sim.ResultsManager.plotSavedRun(obj.childDirs{i}, char(plotType));
            end
            obj.writeAggregateArtifacts();
        end

        function dirs = getChildDirs(obj)
            %GETCHILDDIRS Return the list of child result directories.
            dirs = obj.childDirs;
        end
    end

    methods (Access = private)
        function executeChild(~, child, runArgs)
            %EXECUTECHILD Run setup and simulation for one child runner.
            child.setup();
            child.run(runArgs{:});
        end

        function writeAggregateArtifacts(obj)
            %WRITEAGGREGATEARTIFACTS Rebuild aggregate logs and reports from saved runs.
            if isempty(obj.childDirs)
                obj.childDirs = vt.sim.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            aggregateChunks = cell(numel(obj.childDirs), 1);
            for i = 1:numel(obj.childDirs)
                saved = vt.sim.ResultsManager.loadRun(obj.childDirs{i});
                childLogPath = fullfile(obj.childDirs{i}, 'command_window.txt');
                childLog = vt.sim.ResultsManager.readTextFile(childLogPath);
                label = vt.sim.NamingUtils.runLabel(saved, obj.childDirs{i});
                aggregateChunks{i} = sprintf('===== Trajectory: %s | %s =====\n%s\n', ...
                    saved.cfgSnapshot.traj.name, label, strtrim(childLog));
            end
            aggregatePath = fullfile(obj.resultsDir, 'command_window.txt');
            vt.sim.ResultsManager.writeTextFile(aggregatePath, strjoin(aggregateChunks, newline));
            if obj.isAdaptiveBatch()
                summaryPath = fullfile(obj.resultsDir, 'adaptive_report.txt');
                vt.sim.ResultsManager.writeTextFile(summaryPath, obj.buildSummaryTable());
            end
        end

        function tf = isAdaptiveBatch(obj)
            %ISADAPTIVEBATCH Return true when all saved batch runs use adaptation.
            tf = ~isempty(obj.childDirs);
            if ~tf, return; end
            for i = 1:numel(obj.childDirs)
                saved = vt.sim.ResultsManager.loadRun(obj.childDirs{i});
                if ~isfield(saved.cfgSnapshot.controller, 'adaptation') || strcmpi(saved.cfgSnapshot.controller.adaptation, 'none')
                    tf = false;
                    return;
                end
            end
        end

        function tableText = buildSummaryTable(obj)
            %BUILDSUMMARYTABLE Build an aligned summary table from saved runs.
            nRuns = numel(obj.childDirs);
            headers = {'Trajectory', 'Run', 'Track RMSE', 'Track Score', ...
                'Mass RMSE', 'Mass Score', 'CoG RMSE', 'CoG Score', ...
                'Inertia RMSE', 'Inertia Score'};
            rawRows = cell(nRuns, numel(headers));
            numericValues = nan(nRuns, numel(headers));
            trajectoryNames = cell(nRuns, 1);
            betterIsLower = [false, false, true, false, true, false, true, false, true, false];

            for i = 1:nRuns
                saved = vt.sim.ResultsManager.loadRun(obj.childDirs{i});
                metrics = saved.metrics;
                trajectoryNames{i} = saved.cfgSnapshot.traj.name;
                rawRows{i,1} = trajectoryNames{i};
                rawRows{i,2} = vt.sim.NamingUtils.runLabel(saved, obj.childDirs{i});

                rawRows{i,3} = obj.fmtMetric(metrics.combined.rmse_total, 4);
                rawRows{i,4} = obj.fmtMetric(metrics.combined.tracking_score, 2);
                numericValues(i,3) = metrics.combined.rmse_total;
                numericValues(i,4) = metrics.combined.tracking_score;

                if isfield(metrics, 'parameters')
                    rawRows{i,5} = obj.fmtMetric(metrics.parameters.mass.rmse, 4);
                    rawRows{i,6} = obj.fmtMetric(metrics.parameters.mass.tracking_score, 2);
                    rawRows{i,7} = obj.fmtMetric(metrics.parameters.cog.rmse_total, 4);
                    rawRows{i,8} = obj.fmtMetric(metrics.parameters.cog.tracking_score, 2);
                    rawRows{i,9} = obj.fmtMetric(metrics.parameters.inertia.rmse_total, 4);
                    rawRows{i,10} = obj.fmtMetric(metrics.parameters.inertia.tracking_score, 2);
                    numericValues(i,5) = metrics.parameters.mass.rmse;
                    numericValues(i,6) = metrics.parameters.mass.tracking_score;
                    numericValues(i,7) = metrics.parameters.cog.rmse_total;
                    numericValues(i,8) = metrics.parameters.cog.tracking_score;
                    numericValues(i,9) = metrics.parameters.inertia.rmse_total;
                    numericValues(i,10) = metrics.parameters.inertia.tracking_score;
                else
                    rawRows(i,5:10) = {'N/A'};
                end
            end

            [~, ~, trajectoryGroups] = unique(trajectoryNames, 'stable');
            for groupId = 1:max(trajectoryGroups)
                groupRows = find(trajectoryGroups == groupId);
                for col = 3:numel(headers)
                    values = numericValues(groupRows, col);
                    validMask = isfinite(values);
                    if ~any(validMask), continue; end
                    validValues = values(validMask);
                    if betterIsLower(col)
                        bestValue = min(validValues);
                    else
                        bestValue = max(validValues);
                    end
                    bestMask = validMask & abs(values - bestValue) <= max(1e-12, abs(bestValue) * 1e-12);
                    bestRows = groupRows(bestMask);
                    for row = bestRows.'
                        rawRows{row,col} = sprintf('%s (best)', rawRows{row,col});
                    end
                end
            end

            widths = cellfun(@strlength, headers);
            for col = 1:numel(headers)
                for row = 1:nRuns
                    widths(col) = max(widths(col), strlength(string(rawRows{row,col})));
                end
            end

            lines = strings(nRuns + 5, 1);
            lineIdx = 1;
            lines(lineIdx) = "Batch Run Summary"; lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            for row = 1:nRuns
                lines(lineIdx) = obj.buildRow(rawRows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = obj.buildSep(widths);
            tableText = strjoin(cellstr(lines), newline);
        end

        function text = fmtMetric(~, value, decimals)
            %FMTMETRIC Format a numeric metric with fixed decimals.
            if ~isfinite(value)
                text = 'N/A';
                return;
            end
            text = sprintf(['%0.' num2str(decimals) 'f'], value);
        end

        function line = buildRow(~, values, widths)
            %BUILDROW Build one padded plain-text table row.
            parts = cell(1, numel(values));
            for i = 1:numel(values)
                parts{i} = char(pad(string(values{i}), widths(i), 'right'));
            end
            line = sprintf('| %s |', strjoin(parts, ' | '));
        end

        function line = buildSep(~, widths)
            %BUILDSEP Build a horizontal separator for the table.
            parts = cell(1, numel(widths));
            for i = 1:numel(widths)
                parts{i} = repmat('-', 1, widths(i));
            end
            line = sprintf('+-%s-+', strjoin(parts, '-+-'));
        end
    end
end
