classdef Plotter < handle
    properties
        outDir
        savePng
        lineWidth
        rgb
    end

    methods
        function obj = Plotter(outDir, opts)
            if nargin < 2
                opts = struct();
            end
            obj.outDir = outDir;
            obj.savePng = isfield(opts,'savePng') && opts.savePng;
            obj.lineWidth = 1.5;
            obj.rgb = [1 0 0; 0 1 0; 0 0 1];
        end

        function fig = plotSummaryNominal(obj, logs, fig)
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Position',[100 100 1400 800]);
            else
                clf(fig);
                set(fig, 'Position', [100 100 1400 800]);
            end
            t = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

            % Row 1
            ax1 = nexttile(t,1); obj.plot3DPath(ax1, logs.actual.pos, logs.des.pos);
            ax2 = nexttile(t,2); obj.plotXY(ax2, logs.actual.pos, logs.des.pos);
            ax3 = nexttile(t,3); obj.plotHeight(ax3, logs.t, logs.actual.pos, logs.des.pos);

            % Row 2 (stacked)
            axPos = obj.stackedAxes(nexttile(t,4));
            obj.plotXYZ(axPos(1), logs.t, logs.actual.pos, logs.t, logs.des.pos, 'Position [m]', {'x','y','z'});
            rpy_act = logs.actual.rpy * (180/pi);
            rpy_des = logs.des.rpy * (180/pi);
            obj.plotXYZ(axPos(2), logs.t, rpy_act, logs.t, rpy_des, 'Orientation [deg]', {'roll','pitch','yaw'});

            axVel = obj.stackedAxes(nexttile(t,5));
            obj.plotXYZ(axVel(1), logs.t, logs.actual.linVel, logs.t, logs.des.linVel, 'Linear Vel [m/s]', {'vx','vy','vz'});
            obj.plotXYZ(axVel(2), logs.t, logs.actual.angVel, logs.t, logs.des.angVel, 'Angular Vel [rad/s]', {'wx','wy','wz'});

            axW = obj.stackedAxes(nexttile(t,6));
            obj.plotXYZ(axW(1), logs.t, logs.cmd.wrenchF, [], [], 'Force [N]', {'fx','fy','fz'});
            obj.plotXYZ(axW(2), logs.t, logs.cmd.wrenchT, [], [], 'Torque [Nm]', {'tx','ty','tz'});

            obj.saveFigure(fig, 'summary_nominal');
        end

        function fig = plotSummaryAdaptive(obj, logs, est, fig)
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Position',[50 50 1500 900]);
            else
                clf(fig);
                set(fig, 'Position', [50 50 1500 900]);
            end
            t = tiledlayout(fig, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

            % Row 1
            ax1 = nexttile(t,1); obj.plot3DPath(ax1, logs.actual.pos, logs.des.pos);
            ax2 = nexttile(t,2); obj.plotXY(ax2, logs.actual.pos, logs.des.pos);
            ax3 = nexttile(t,3); obj.plotHeight(ax3, logs.t, logs.actual.pos, logs.des.pos);

            % Row 2 (stacked)
            axPos = obj.stackedAxes(nexttile(t,4));
            obj.plotXYZ(axPos(1), logs.t, logs.actual.pos, logs.t, logs.des.pos, 'Position [m]', {'x','y','z'});
            rpy_act = logs.actual.rpy * (180/pi);
            rpy_des = logs.des.rpy * (180/pi);
            obj.plotXYZ(axPos(2), logs.t, rpy_act, logs.t, rpy_des, 'Orientation [deg]', {'roll','pitch','yaw'});

            axVel = obj.stackedAxes(nexttile(t,5));
            obj.plotXYZ(axVel(1), logs.t, logs.actual.linVel, logs.t, logs.des.linVel, 'Linear Vel [m/s]', {'vx','vy','vz'});
            obj.plotXYZ(axVel(2), logs.t, logs.actual.angVel, logs.t, logs.des.angVel, 'Angular Vel [rad/s]', {'wx','wy','wz'});

            axW = obj.stackedAxes(nexttile(t,6));
            obj.plotXYZ(axW(1), logs.t, logs.cmd.wrenchF, [], [], 'Force [N]', {'fx','fy','fz'});
            obj.plotXYZ(axW(2), logs.t, logs.cmd.wrenchT, [], [], 'Torque [Nm]', {'tx','ty','tz'});

            % Row 3
            ax7 = nexttile(t,7); obj.plotMass(ax7, est.t, est.mass, est);
            ax8 = nexttile(t,8); obj.plotCoM(ax8, est.t, est.com);
            ax9 = nexttile(t,9); obj.plotInertia(ax9, est.t, est.inertia);

            obj.saveFigure(fig, 'summary_adaptive');
        end
    end

    methods (Access = private)
        function axPair = stackedAxes(~, axBase)
            pos = axBase.Position;
            delete(axBase);
            gap = 0.04;
            h = (pos(4) - gap) / 2;
            axTop = axes('Position', [pos(1), pos(2)+h+gap, pos(3), h]);
            axBot = axes('Position', [pos(1), pos(2), pos(3), h]);
            axPair = [axTop; axBot];
        end

        function plotXYZ(obj, ax, t1, actual, t2, desired, ylabelText, names)
            [t1, actual] = obj.alignTime(t1, actual);
            if nargin < 5 || isempty(desired)
                t2 = []; desired = [];
            else
                [t2, desired] = obj.alignTime(t2, desired);
            end
            hold(ax,'on');
            for i = 1:3
                plot(ax, t1, actual(:,i), 'LineWidth', obj.lineWidth, 'Color', obj.rgb(i,:));
                if ~isempty(desired)
                    plot(ax, t2, desired(:,i), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(i,:));
                end
            end
            ylabel(ax, ylabelText);
            xlabel(ax, 'Time [s]');
            grid(ax,'on');
            if isempty(desired)
                legend(ax, {names{1}, names{2}, names{3}}, 'Location', 'best');
            else
                legend(ax, {names{1}, [names{1} ' (des)'], names{2}, [names{2} ' (des)'], names{3}, [names{3} ' (des)']}, 'Location', 'best');
            end
        end

        function plotHeight(obj, ax, t, actualPos, desiredPos)
            [t, actualPos] = obj.alignTime(t, actualPos);
            [t2, desiredPos] = obj.alignTime(t, desiredPos);
            hold(ax,'on');
            plot(ax, t, actualPos(:,3), 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, t2, desiredPos(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Z [m]'); grid(ax,'on');
            legend(ax, {'z', 'z (des)'}, 'Location', 'best');
        end

        function plot3DPath(obj, ax, actualPos, desiredPos)
            hold(ax,'on'); view(ax,3); grid(ax,'on');
            plot3(ax, desiredPos(:,1), desiredPos(:,2), desiredPos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            plot3(ax, actualPos(:,1), actualPos(:,2), actualPos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            legend(ax, {'Desired','Actual'}, 'Location', 'best');
            axis(ax,'equal');
            maxXY = max(abs([actualPos(:,1); actualPos(:,2); desiredPos(:,1); desiredPos(:,2)]));
            if maxXY > 0
                xlim(ax, [-maxXY maxXY]);
                ylim(ax, [-maxXY maxXY]);
            end
        end

        function plotXY(obj, ax, actualPos, desiredPos)
            hold(ax,'on');
            plot(ax, desiredPos(:,1), desiredPos(:,2), 'k--', 'LineWidth', obj.lineWidth);
            plot(ax, actualPos(:,1), actualPos(:,2), 'b-', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
            axis(ax,'equal'); grid(ax,'on');
            legend(ax, {'Desired','Actual'}, 'Location', 'best');
        end

        function plotMass(obj, ax, t, mass, est)
            [t, mass] = obj.alignTime(t, mass);
            hold(ax,'on');
            plot(ax, t, mass, 'LineWidth', obj.lineWidth, 'Color', [0 0 0]);
            if nargin >= 5 && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'k:', 'LineWidth', 1.0);
            end
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Mass [kg]'); grid(ax,'on');
        end

        function plotCoM(obj, ax, t, com)
            [t, com] = obj.alignTime(t, com);
            hold(ax,'on');
            for i = 1:3
                plot(ax, t, com(:,i), 'LineWidth', obj.lineWidth, 'Color', obj.rgb(i,:));
            end
            xlabel(ax, 'Time [s]'); ylabel(ax, 'CoM [m]'); grid(ax,'on');
            legend(ax, {'x','y','z'}, 'Location', 'best');
        end

        function plotInertia(obj, ax, t, inertia)
            [t, inertia] = obj.alignTime(t, inertia);
            pos = ax.Position; delete(ax);
            gap = 0.04;
            h = (pos(4) - gap) / 2;
            axTop = axes('Position', [pos(1), pos(2)+h+gap, pos(3), h]);
            axBot = axes('Position', [pos(1), pos(2), pos(3), h]);

            hold(axTop,'on'); hold(axBot,'on');
            for i = 1:3
                plot(axTop, t, inertia(:,i), 'LineWidth', obj.lineWidth, 'Color', obj.rgb(i,:));
            end
            for i = 1:3
                plot(axBot, t, inertia(:,3+i), 'LineWidth', obj.lineWidth, 'Color', obj.rgb(i,:));
            end
            xlabel(axBot, 'Time [s]');
            ylabel(axTop, 'I_{xx}, I_{yy}, I_{zz}');
            ylabel(axBot, 'I_{xy}, I_{yz}, I_{xz}');
            grid(axTop,'on'); grid(axBot,'on');
            legend(axTop, {'Ixx','Iyy','Izz'}, 'Location', 'best');
            legend(axBot, {'Ixy','Iyz','Ixz'}, 'Location', 'best');
        end

        function [t, sig] = alignTime(~, t, sig)
            n = size(sig,1);
            if numel(t) > n
                t = t(1:n);
            elseif numel(t) < n
                sig = sig(1:numel(t), :);
            end
            t = t(:);
        end

        function saveFigure(obj, fig, filename)
            if obj.savePng
                if ~exist(obj.outDir, 'dir')
                    mkdir(obj.outDir);
                end
                saveas(fig, fullfile(obj.outDir, [filename '.png']));
            end
        end
    end
end
