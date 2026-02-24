classdef TrackingMetrics
    %TRACKINGMETRICS Compute and report trajectory tracking metrics.
    %   Provides RMSE/NRMSE summaries and a simple tracking score in percent.
    %
    %   Usage:
    %     metrics = vt.metrics.TrackingMetrics.computeAll(actual, desired);
    %     vt.metrics.TrackingMetrics.printReport(metrics, 'Nominal');
    methods (Static)
        function metrics = computeAll(actualPos, desiredPos)
            %COMPUTEALL Return RMSE, NRMSE, and summary error stats.
            %   Inputs:
            %     actualPos - Nx3 actual position samples [m].
            %     desiredPos - Nx3 desired position samples [m].
            %
            %   Output:
            %     metrics - struct with rmse/nrmse and error statistics.
            err = actualPos - desiredPos;
            metrics.rmse_xyz = sqrt(mean(err.^2, 1));
            metrics.rmse_total = sqrt(mean(sum(err.^2, 2)));

            range_xyz = max(desiredPos, [], 1) - min(desiredPos, [], 1);
            range_xyz(range_xyz == 0) = 1;
            metrics.nrmse_xyz = metrics.rmse_xyz ./ range_xyz;
            metrics.nrmse_total = mean(metrics.nrmse_xyz);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            metrics.max_error = max(vecnorm(err, 2, 2));
            metrics.mean_error = mean(vecnorm(err, 2, 2));
            metrics.std_error = std(vecnorm(err, 2, 2));
        end

        function printReport(metrics, name)
            %PRINTREPORT Print a formatted metrics summary to console.
            %   Inputs:
            %     metrics - struct returned by computeAll.
            %     name - label for report header.
            fprintf('========================================\n');
            fprintf('Tracking Metrics: %s\n', name);
            fprintf('========================================\n');
            fprintf('RMSE Total:      %.4f m\n', metrics.rmse_total);
            fprintf('NRMSE Total:     %.4f\n', metrics.nrmse_total);
            fprintf('Tracking Score:  %.2f %%\n', metrics.tracking_score);
            fprintf('----------------------------------------\n');
            fprintf('RMSE X:          %.4f m\n', metrics.rmse_xyz(1));
            fprintf('RMSE Y:          %.4f m\n', metrics.rmse_xyz(2));
            fprintf('RMSE Z:          %.4f m\n', metrics.rmse_xyz(3));
            fprintf('----------------------------------------\n');
            fprintf('Max Error:       %.4f m\n', metrics.max_error);
            fprintf('Mean Error:      %.4f m\n', metrics.mean_error);
            fprintf('Std Error:       %.4f m\n', metrics.std_error);
            fprintf('========================================\n');
        end
    end
end
