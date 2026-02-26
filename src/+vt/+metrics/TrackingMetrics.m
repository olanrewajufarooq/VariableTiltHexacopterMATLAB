classdef TrackingMetrics < handle
    %TRACKINGMETRICS Compute and report trajectory tracking metrics.
    %   Usage:
    %     metricsObj = vt.metrics.TrackingMetrics(logger, 'Run 1');
    %     metrics = metricsObj.computeAll();
    %     metricsObj.printReport(metrics);

    properties (SetAccess = private)
        Name
        ActualPos
        DesiredPos
        ActualRpy
        DesiredRpy
        EstMass
        EstMassActual
        EstCoG
        EstCoGActual
        EstInertia
        EstInertiaActual
        PositionMetrics
        OrientationMetrics
        CombinedMetrics
        ParameterMetrics
    end

    properties (Access = private)
        IsPositionErrComputed = false
        IsOrientationErrComputed = false
        IsCombinedErrComputed = false
        IsParameterErrComputed = false
    end

    methods
        function obj = TrackingMetrics(logs, name)
            if nargin < 1
                logs = [];
            end
            if nargin < 2
                name = '';
            end

            obj.Name = name;
            if ~isempty(logs)
                obj.setLogs(logs);
            end
        end

        function setLogs(obj, logs)
            if isobject(logs) && ismethod(logs, 'finalize')
                logs = logs.finalize();
            elseif ~isstruct(logs)
                error('TrackingMetrics:InvalidLogs', 'Logs must be a Logger object or a logs struct.');
            end
            obj.ActualPos = logs.actual.pos;
            obj.DesiredPos = logs.des.pos;
            obj.ActualRpy = logs.actual.rpy;
            obj.DesiredRpy = logs.des.rpy;
            obj.EstMass = [];
            obj.EstMassActual = [];
            obj.EstCoG = [];
            obj.EstCoGActual = [];
            obj.EstInertia = [];
            obj.EstInertiaActual = [];
            if isfield(logs, 'est') && isstruct(logs.est)
                if isfield(logs.est, 'mass') && isfield(logs.est, 'massActual')
                    obj.EstMass = logs.est.mass;
                    obj.EstMassActual = logs.est.massActual;
                end
                if isfield(logs.est, 'com') && isfield(logs.est, 'comActual')
                    obj.EstCoG = logs.est.com;
                    obj.EstCoGActual = logs.est.comActual;
                end
                if isfield(logs.est, 'inertia') && isfield(logs.est, 'inertiaActual')
                    obj.EstInertia = logs.est.inertia;
                    obj.EstInertiaActual = logs.est.inertiaActual;
                end
            end
            obj.PositionMetrics = struct();
            obj.OrientationMetrics = struct();
            obj.CombinedMetrics = struct();
            obj.ParameterMetrics = struct();
            obj.IsPositionErrComputed = false;
            obj.IsOrientationErrComputed = false;
            obj.IsCombinedErrComputed = false;
            obj.IsParameterErrComputed = false;
        end

        function metrics = computeAll(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            metrics = struct();
            metrics.name = obj.Name;
            metrics.position = obj.computePosition();
            metrics.orientation = obj.computeOrientation();
            metrics.combined = obj.computeCombined();
            metrics.parameters = obj.computeParameterEstimation();
        end

        function metrics = computePosition(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsPositionErrComputed
                metrics = obj.PositionMetrics;
                return;
            end

            posErr = obj.ActualPos - obj.DesiredPos;
            metrics = struct();
            metrics.rmse_xyz = sqrt(mean(posErr .^ 2, 1));
            metrics.rmse_total = sqrt(mean(sum(posErr .^ 2, 2)));

            range_xyz = max(obj.DesiredPos, [], 1) - min(obj.DesiredPos, [], 1);
            range_xyz(range_xyz == 0) = 1;
            metrics.nrmse_xyz = metrics.rmse_xyz ./ range_xyz;
            metrics.nrmse_total = mean(metrics.nrmse_xyz);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            metrics.max_error = max(vecnorm(posErr, 2, 2));
            metrics.mean_error = mean(vecnorm(posErr, 2, 2));
            metrics.std_error = std(vecnorm(posErr, 2, 2));

            obj.PositionMetrics = metrics;
            obj.IsPositionErrComputed = true;
        end

        function metrics = computeOrientation(obj)
            if isempty(obj.ActualRpy)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsOrientationErrComputed
                metrics = obj.OrientationMetrics;
                return;
            end

            [oriErr, oriErrNorm] = obj.computeSO3Error(obj.ActualRpy, obj.DesiredRpy);
            metrics = struct();
            metrics.rmse_rpy = sqrt(mean(oriErr .^ 2, 1));
            metrics.rmse_total = sqrt(mean(oriErrNorm .^ 2));

            denom = pi;
            metrics.nrmse_rpy = metrics.rmse_rpy ./ denom;
            metrics.nrmse_total = mean(metrics.nrmse_rpy);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            metrics.max_error = max(oriErrNorm);
            metrics.mean_error = mean(oriErrNorm);
            metrics.std_error = std(oriErrNorm);

            obj.OrientationMetrics = metrics;
            obj.IsOrientationErrComputed = true;
        end

        function metrics = computeCombined(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsCombinedErrComputed
                metrics = obj.CombinedMetrics;
                return;
            end

            se3Err = obj.computeSE3Error(obj.ActualPos, obj.DesiredPos, obj.ActualRpy, obj.DesiredRpy);
            metrics = struct();
            metrics.rmse_total = sqrt(mean(se3Err .^ 2));
            metrics.max_error = max(se3Err);
            metrics.mean_error = mean(se3Err);
            metrics.std_error = std(se3Err);

            posMetrics = obj.computePosition();
            oriMetrics = obj.computeOrientation();
            metrics.nrmse_total = mean([posMetrics.nrmse_total, oriMetrics.nrmse_total]);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            obj.CombinedMetrics = metrics;
            obj.IsCombinedErrComputed = true;
        end

        function printReport(obj)
            metrics = obj.computeAll();
            name = metrics.name;
            if isempty(name)
                name = 'Tracking';
            end

            fprintf('========================================\n');
            fprintf('Tracking Metrics: %s\n', name);
            fprintf('========================================\n');

            obj.printPosition();
            obj.printOrientation();
            obj.printCombined();
            if isfield(metrics, 'parameters')
                obj.printParameterEstimation();
            end

            fprintf('========================================\n');
        end

        function printPosition(obj)
            metrics = obj.computePosition();

            fprintf('Position Metrics\n');
            fprintf('----------------------------------------\n');
            fprintf('RMSE Total:      %.4f m\n', metrics.rmse_total);
            fprintf('NRMSE Total:     %.4f\n', metrics.nrmse_total);
            fprintf('Tracking Score:  %.2f %%\n', metrics.tracking_score);
            fprintf('RMSE XYZ:        [%.4f %.4f %.4f] m\n', metrics.rmse_xyz(1), metrics.rmse_xyz(2), metrics.rmse_xyz(3));
            fprintf('Max Error:       %.4f m\n', metrics.max_error);
            fprintf('Mean Error:      %.4f m\n', metrics.mean_error);
            fprintf('Std Error:       %.4f m\n\n', metrics.std_error);
        end

        function printOrientation(obj)
            metrics = obj.computeOrientation();

            fprintf('Orientation Metrics (SO(3))\n');
            fprintf('----------------------------------------\n');
            fprintf('RMSE Total:      %.4f rad\n', metrics.rmse_total);
            fprintf('NRMSE Total:     %.4f\n', metrics.nrmse_total);
            fprintf('Tracking Score:  %.2f %%\n', metrics.tracking_score);
            fprintf('RMSE RPY:        [%.4f %.4f %.4f] rad\n', metrics.rmse_rpy(1), metrics.rmse_rpy(2), metrics.rmse_rpy(3));
            fprintf('Max Error:       %.4f rad\n', metrics.max_error);
            fprintf('Mean Error:      %.4f rad\n', metrics.mean_error);
            fprintf('Std Error:       %.4f rad\n\n', metrics.std_error);
        end

        function printCombined(obj)
            metrics = obj.computeCombined();

            fprintf('Combined Pose Metrics (SE(3))\n');
            fprintf('----------------------------------------\n');
            fprintf('RMSE Total:      %.4f\n', metrics.rmse_total);
            fprintf('NRMSE Total:     %.4f\n', metrics.nrmse_total);
            fprintf('Tracking Score:  %.2f %%\n', metrics.tracking_score);
            fprintf('Max Error:       %.4f\n', metrics.max_error);
            fprintf('Mean Error:      %.4f\n', metrics.mean_error);
            fprintf('Std Error:       %.4f\n\n', metrics.std_error);
        end

        function printParameterEstimation(obj)
            metrics = obj.computeParameterEstimation();

            fprintf('Parameter Estimation Metrics\n');
            fprintf('----------------------------------------\n');

            fprintf('Mass RMSE:       %.4f kg\n', metrics.mass.rmse);
            fprintf('Mass NRMSE:      %.4f\n', metrics.mass.nrmse);
            fprintf('Mass Score:      %.2f %%\n', metrics.mass.tracking_score);

            fprintf('CoG RMSE:        [%.4f %.4f %.4f] m\n', metrics.cog.rmse_xyz(1), metrics.cog.rmse_xyz(2), metrics.cog.rmse_xyz(3));
            fprintf('CoG NRMSE:       %.4f\n', metrics.cog.nrmse_total);
            fprintf('CoG Score:       %.2f %%\n', metrics.cog.tracking_score);

            fprintf('Inertia RMSE:    [%.4f %.4f %.4f %.4f %.4f %.4f]\n', metrics.inertia.rmse_params(1), metrics.inertia.rmse_params(2), metrics.inertia.rmse_params(3), metrics.inertia.rmse_params(4), metrics.inertia.rmse_params(5), metrics.inertia.rmse_params(6));
            fprintf('Inertia NRMSE:   %.4f\n', metrics.inertia.nrmse_total);
            fprintf('Inertia Score:   %.2f %%\n', metrics.inertia.tracking_score);

            fprintf('\n');
        end
    end

    methods (Access = private)
        function [oriErr, oriErrNorm] = computeSO3Error(obj, actualRpy, desiredRpy)
            n = size(actualRpy, 1);
            oriErr = zeros(n, 3);
            oriErrNorm = zeros(n, 1);

            for i = 1:n
                R = obj.rpyToRotm(actualRpy(i, :));
                Rd = obj.rpyToRotm(desiredRpy(i, :));
                Rerr = Rd' * R;

                T = eye(4);
                T(1:3, 1:3) = Rerr;
                zeta = vt.se3.logSE3(T);
                oriErr(i, :) = zeta(1:3).';
                oriErrNorm(i, 1) = norm(vt.se3.hat3(zeta(1:3)), 'fro');
            end
        end

        function se3Err = computeSE3Error(obj, actualPos, desiredPos, actualRpy, desiredRpy)
            n = size(actualPos, 1);
            se3Err = zeros(n, 1);

            for i = 1:n
                H = obj.buildSE3(actualPos(i, :), actualRpy(i, :));
                Hd = obj.buildSE3(desiredPos(i, :), desiredRpy(i, :));
                He = vt.se3.invSE3(Hd) * H;
                zeta = vt.se3.logSE3(He);
                se3mat = vt.se3.hat6(zeta);
                se3Err(i, 1) = norm(se3mat, 'fro');
            end
        end

        function H = buildSE3(obj, pos, rpy)
            R = obj.rpyToRotm(rpy);
            H = eye(4);
            H(1:3, 1:3) = R;
            H(1:3, 4) = pos(:);
        end

        function R = rpyToRotm(obj, rpy)
            R = vt.utils.rpy2rotm(rpy(:));
        end

        function metrics = computeParameterEstimation(obj)
            if obj.IsParameterErrComputed
                metrics = obj.ParameterMetrics;
                return;
            end

            metrics = struct();
            metrics.mass = struct('rmse', NaN, 'nrmse', NaN, 'tracking_score', NaN);
            metrics.cog = struct('rmse_xyz', [NaN NaN NaN], 'rmse_total', NaN, ...
                'nrmse_xyz', [NaN NaN NaN], 'nrmse_total', NaN, 'tracking_score', NaN);
            metrics.inertia = struct('rmse_params', [NaN NaN NaN NaN NaN NaN], 'rmse_total', NaN, ...
                'nrmse_params', [NaN NaN NaN NaN NaN NaN], 'nrmse_total', NaN, 'tracking_score', NaN);

            if ~isempty(obj.EstMass) && ~isempty(obj.EstMassActual)
                err = obj.EstMass - obj.EstMassActual;
                rmse = sqrt(mean(err .^ 2));
                range_val = max(obj.EstMassActual) - min(obj.EstMassActual);
                if range_val == 0
                    range_val = 1;
                end
                nrmse = rmse / range_val;
                metrics.mass = struct('rmse', rmse, 'nrmse', nrmse, ...
                    'tracking_score', max(0, (1 - nrmse) * 100));
            end

            if ~isempty(obj.EstCoG) && ~isempty(obj.EstCoGActual)
                err = obj.EstCoG - obj.EstCoGActual;
                rmse_xyz = sqrt(mean(err .^ 2, 1));
                rmse_total = sqrt(mean(sum(err .^ 2, 2)));
                range_xyz = max(obj.EstCoGActual, [], 1) - min(obj.EstCoGActual, [], 1);
                range_xyz(range_xyz == 0) = 1;
                nrmse_xyz = rmse_xyz ./ range_xyz;
                nrmse_total = mean(nrmse_xyz);
                metrics.cog = struct('rmse_xyz', rmse_xyz, 'rmse_total', rmse_total, ...
                    'nrmse_xyz', nrmse_xyz, 'nrmse_total', nrmse_total, ...
                    'tracking_score', max(0, (1 - nrmse_total) * 100));
            end

            if ~isempty(obj.EstInertia) && ~isempty(obj.EstInertiaActual)
                err = obj.EstInertia - obj.EstInertiaActual;
                rmse_params = sqrt(mean(err .^ 2, 1));
                rmse_total = sqrt(mean(sum(err .^ 2, 2)));
                range_params = max(obj.EstInertiaActual, [], 1) - min(obj.EstInertiaActual, [], 1);
                range_params(range_params == 0) = 1;
                nrmse_params = rmse_params ./ range_params;
                nrmse_total = mean(nrmse_params);
                metrics.inertia = struct('rmse_params', rmse_params, 'rmse_total', rmse_total, ...
                    'nrmse_params', nrmse_params, 'nrmse_total', nrmse_total, ...
                    'tracking_score', max(0, (1 - nrmse_total) * 100));
            end

            obj.ParameterMetrics = metrics;
            obj.IsParameterErrComputed = true;
        end
    end
end
