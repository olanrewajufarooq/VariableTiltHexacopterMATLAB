classdef Plotter < handle
    properties
        outDir
        savePng
        lineWidth
        rgb
        duration
    end

    methods
        function obj = Plotter(outDir, opts)
            if nargin < 2
                opts = struct();
            end
            obj.outDir = outDir;
            obj.savePng = isfield(opts,'savePng') && opts.savePng;
            obj.duration = [];
            if isfield(opts, 'duration')
                obj.duration = opts.duration;
            end
            obj.lineWidth = 1.5;
            obj.rgb = [1 0 0; 0 1 0; 0 0 1];
        end

        function fig = plotLiveNominal(obj, logs, fig)
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1400 700]);
            else
                clf(fig);
            end
            
            subplot(4, 3, 1); obj.plot3DView(logs);
            subplot(4, 3, 2); obj.plotXYView(logs);
            subplot(4, 3, 3); obj.plotZvsT(logs);
            
            subplot(4, 3, 4); obj.plotPosition(logs);
            subplot(4, 3, 7); obj.plotOrientation(logs);
            
            subplot(4, 3, 5); obj.plotLinearVel(logs);
            subplot(4, 3, 8); obj.plotAngularVel(logs);
            
            subplot(4, 3, 6); obj.plotForce(logs);
            subplot(4, 3, 9); obj.plotTorque(logs);
            
            sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
        end

        function fig = plotSummaryNominal(obj, logs, fig)
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 700]);
            else
                clf(fig);
            end
            
            subplot(4, 3, 1); obj.plot3DView(logs);
            subplot(4, 3, 2); obj.plotXYView(logs);
            subplot(4, 3, 3); obj.plotZvsT(logs);
            
            subplot(4, 3, 4); obj.plotPosition(logs);
            subplot(4, 3, 7); obj.plotOrientation(logs);
            
            subplot(4, 3, 5); obj.plotLinearVel(logs);
            subplot(4, 3, 8); obj.plotAngularVel(logs);
            
            subplot(4, 3, 6); obj.plotForce(logs);
            subplot(4, 3, 9); obj.plotTorque(logs);
            
            obj.saveFigure(fig, 'summary_nominal');
        end

        function fig = plotSummaryAdaptive(obj, logs, est, fig)
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 950]);
            else
                clf(fig);
            end
            
            subplot(5, 3, 1); obj.plot3DView(logs);
            subplot(5, 3, 2); obj.plotXYView(logs);
            subplot(5, 3, 3); obj.plotZvsT(logs);
            
            subplot(5, 3, 4); obj.plotPosition(logs);
            subplot(5, 3, 7); obj.plotOrientation(logs);
            
            subplot(5, 3, 5); obj.plotLinearVel(logs);
            subplot(5, 3, 8); obj.plotAngularVel(logs);
            
            subplot(5, 3, 6); obj.plotForce(logs);
            subplot(5, 3, 9); obj.plotTorque(logs);
            
            subplot(5, 3, 10); obj.plotMass(est);
            subplot(5, 3, 13); obj.plotCoG(est);
            
            subplot(5, 3, 11); obj.plotPrincipalInertia(est);
            subplot(5, 3, 12); obj.plotOffDiagInertia(est);
            
            obj.saveFigure(fig, 'summary_adaptive');
        end
    end

    methods (Access = private)
        function plot3DView(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            view(ax, 3);
            title(ax, '3D Path');
            plot3(ax, logs.des.pos(:,1), logs.des.pos(:,2), logs.des.pos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            plot3(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), logs.actual.pos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            plot3(ax, logs.actual.pos(1,1), logs.actual.pos(1,2), logs.actual.pos(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            plot3(ax, logs.actual.pos(end,1), logs.actual.pos(end,2), logs.actual.pos(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            legend(ax, {'Desired','Actual','Start','End'}, 'Location', 'best', 'FontSize', 7);
            axis(ax, 'equal');
        end

        function plotXYView(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'XY Path');
            plot(ax, logs.des.pos(:,1), logs.des.pos(:,2), 'k--', 'LineWidth', obj.lineWidth);
            plot(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), 'b-', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
            legend(ax, {'Desired','Actual'}, 'Location', 'best', 'FontSize', 7);
            axis(ax, 'equal');
        end

        function plotZvsT(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Altitude');
            plot(ax, logs.t, logs.actual.pos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            plot(ax, logs.t, logs.des.pos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Z [m]');
            legend(ax, {'Actual','Desired'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, logs.t);
        end

        function plotPosition(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Position [m]');
            plot(ax, logs.t, logs.actual.pos(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.pos(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.pos(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.pos(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.pos(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.pos(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'x','y','z'}, 'Location', 'best', 'FontSize', 7);
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
        end

        function plotOrientation(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Orientation [deg]');
            rpy_act = logs.actual.rpy * (180/pi);
            rpy_des = logs.des.rpy * (180/pi);
            plot(ax, logs.t, rpy_act(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_act(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_act(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, rpy_des(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_des(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_des(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            xlabel(ax, 'Time [s]');
            legend(ax, {'roll','pitch','yaw'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, logs.t);
        end

        function plotLinearVel(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Linear Vel [m/s]');
            plot(ax, logs.t, logs.actual.linVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.linVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.linVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.linVel(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.linVel(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.linVel(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'vx','vy','vz'}, 'Location', 'best', 'FontSize', 7);
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
        end

        function plotAngularVel(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Angular Vel [rad/s]');
            plot(ax, logs.t, logs.actual.angVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.angVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.angVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.angVel(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.angVel(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.angVel(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            xlabel(ax, 'Time [s]');
            legend(ax, {'wx','wy','wz'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, logs.t);
        end

        function plotForce(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Force [N]');
            plot(ax, logs.t, logs.cmd.wrenchF(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            set(ax, 'XTickLabel', []);
            legend(ax, {'Fx','Fy','Fz'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, logs.t);
        end

        function plotTorque(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Torque [Nm]');
            plot(ax, logs.t, logs.cmd.wrenchT(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            xlabel(ax, 'Time [s]');
            legend(ax, {'Tx','Ty','Tz'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, logs.t);
        end

        function plotMass(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Mass [kg]');
            plot(ax, est.t, est.mass, 'b-', 'LineWidth', obj.lineWidth);
            if isfield(est, 'massActual')
                plot(ax, est.t, est.massActual, 'k--', 'LineWidth', obj.lineWidth);
                legend(ax, {'Estimated','Actual'}, 'Location', 'best', 'FontSize', 7);
            end
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotCoG(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'CoG [m]');
            plot(ax, est.t, est.com(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.com(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.com(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'comActual')
                plot(ax, est.t, est.comActual(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.comActual(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.comActual(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            end
            xlabel(ax, 'Time [s]');
            legend(ax, {'x','y','z'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotPrincipalInertia(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Principal Inertia [kg·m²]');
            plot(ax, est.t, est.inertia(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.inertia(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.inertia(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'inertiaActual')
                plot(ax, est.t, est.inertiaActual(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertiaActual(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertiaActual(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            end
            xlabel(ax, 'Time [s]');
            legend(ax, {'Ixx','Iyy','Izz'}, 'Location', 'best', 'FontSize', 7);
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotOffDiagInertia(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Off-diag Inertia [kg·m²]');
            if size(est.inertia, 2) >= 6
                plot(ax, est.t, est.inertia(:,4), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertia(:,5), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertia(:,6), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                if isfield(est, 'inertiaActual') && size(est.inertiaActual, 2) >= 6
                    plot(ax, est.t, est.inertiaActual(:,4), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                    plot(ax, est.t, est.inertiaActual(:,5), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                    plot(ax, est.t, est.inertiaActual(:,6), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                end
                legend(ax, {'Ixy','Iyz','Izx'}, 'Location', 'best', 'FontSize', 7);
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function setXLim(obj, ax, tVec)
            if ~isempty(obj.duration)
                xlim(ax, [0 obj.duration]);
            else
                xlim(ax, [0 max(tVec)]);
            end
        end

        function saveFigure(obj, fig, filename)
            if obj.savePng
                if ~isempty(obj.outDir) && ~exist(obj.outDir, 'dir')
                    mkdir(obj.outDir);
                end
                filepath = fullfile(obj.outDir, [filename '.png']);
                saveas(fig, filepath);
            end
        end
    end
end
