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
        PositionMetrics
        OrientationMetrics
        CombinedMetrics
    end

    properties (Access = private)
        IsPositionErrComputed = false
        IsOrientationErrComputed = false
        IsCombinedErrComputed = false
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
            obj.PositionMetrics = struct();
            obj.OrientationMetrics = struct();
            obj.CombinedMetrics = struct();
            obj.IsPositionErrComputed = false;
            obj.IsOrientationErrComputed = false;
            obj.IsCombinedErrComputed = false;
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
    end
end
