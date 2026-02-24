classdef Plotter < handle
    properties
        outDir
        savePng
        lineWidth
        rgb
        duration
    end

    properties (Access = private)
        liveAxes
        liveUseUrdf
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

        function saveFigure(obj, fig, filename)
            obj.saveFigureInternal(fig, filename);
        end

        function fig = plotLiveNominal(obj, logs, fig, useUrdfSlot)
            if nargin < 4
                useUrdfSlot = false;
            end
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1400 600]);
                obj.liveAxes = struct();
            end

            if useUrdfSlot
                obj.ensureLiveLayout(fig, useUrdfSlot);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    obj.renderLiveAxes(logs, useUrdfSlot);
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                else
                    sgtitle(fig, 'Live View');
                end
            else
                clf(fig);
                obj.drawNominalLayout(fig, logs);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                end
            end
        end

        function ax = getLiveUrdfAxes(obj)
            ax = [];
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'urdf') && isgraphics(obj.liveAxes.urdf)
                ax = obj.liveAxes.urdf;
            end
        end

        function fig = plotSummaryNominal(obj, logs, fig)
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
            else
                clf(fig);
            end
            
            obj.drawNominalLayout(fig, logs);
            obj.saveFigureInternal(fig, 'summary_nominal');
        end

        function fig = plotSummaryAdaptive(obj, logs, est, fig)
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
            else
                clf(fig);
            end
            
            obj.drawAdaptiveLayout(fig, logs, est);
            obj.saveFigureInternal(fig, 'summary_adaptive');
        end
    end

    methods (Access = private)
        function drawNominalLayout(obj, fig, logs)
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.08; mb = 0.08;
            gh = 0.03; gv = 0.06;
            pw = (1 - ml - mr - 2*gh) / 3;
            rh = (1 - mt - mb - gv) / 2;
            
            pos = struct();
            pos.row1_1 = [ml,                    mt + rh + gv, pw, rh];
            pos.row1_2 = [ml + pw + gh,          mt + rh + gv, pw, rh];
            pos.row1_3 = [ml + 2*(pw + gh),      mt + rh + gv, pw, rh];
            pos.row2_1 = [ml,                    mt,           pw, rh];
            pos.row2_2 = [ml + pw + gh,          mt,           pw, rh];
            pos.row2_3 = [ml + 2*(pw + gh),      mt,           pw, rh];
            
            subplot('Position', pos.row1_1); obj.plot3DView(logs);
            subplot('Position', pos.row1_2); obj.plotXYView(logs);
            subplot('Position', pos.row1_3); obj.plotZvsT(logs);
            
            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, pos.row2_1, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, pos.row2_2, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, pos.row2_3, logs);
        end

        function drawAdaptiveLayout(obj, fig, logs, est)
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.06; mb = 0.08;
            gh = 0.03; gv = 0.04;
            pw = (1 - ml - mr - 2*gh) / 3;
            rh = (1 - mt - mb - 2*gv) / 3;
            
            pos = struct();
            pos.row1_1 = [ml,                    mt + 2*rh + 2*gv, pw, rh];
            pos.row1_2 = [ml + pw + gh,          mt + 2*rh + 2*gv, pw, rh];
            pos.row1_3 = [ml + 2*(pw + gh),      mt + 2*rh + 2*gv, pw, rh];
            pos.row2_1 = [ml,                    mt + rh + gv,     pw, rh];
            pos.row2_2 = [ml + pw + gh,          mt + rh + gv,     pw, rh];
            pos.row2_3 = [ml + 2*(pw + gh),      mt + rh + gv,     pw, rh];
            pos.row3_1 = [ml,                    mt,               pw, rh];
            pos.row3_2 = [ml + pw + gh,          mt,               pw, rh];
            pos.row3_3 = [ml + 2*(pw + gh),      mt,               pw, rh];
            
            subplot('Position', pos.row1_1); obj.plot3DView(logs);
            subplot('Position', pos.row1_2); obj.plotXYView(logs);
            subplot('Position', pos.row1_3); obj.plotZvsT(logs);
            
            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, pos.row2_1, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, pos.row2_2, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, pos.row2_3, logs);
            
            obj.plotStackedEst(@obj.plotMass, @obj.plotCoG, pos.row3_1, est);
            subplot('Position', pos.row3_2); obj.plotPrincipalInertia(est);
            subplot('Position', pos.row3_3); obj.plotOffDiagInertia(est);
        end

        function ensureLiveLayout(obj, fig, useUrdfSlot)
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'row1_2') && isgraphics(obj.liveAxes.row1_2)
                if isequal(obj.liveUseUrdf, useUrdfSlot)
                    return;
                end
            end

            obj.liveUseUrdf = useUrdfSlot;
            obj.liveAxes = struct();
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.08; mb = 0.08;
            gh = 0.03; gv = 0.06;
            pw = (1 - ml - mr - 2*gh) / 3;
            rh = (1 - mt - mb - gv) / 2;

            pos = struct();
            pos.row1_1 = [ml,                    mt + rh + gv, pw, rh];
            pos.row1_2 = [ml + pw + gh,          mt + rh + gv, pw, rh];
            pos.row1_3 = [ml + 2*(pw + gh),      mt + rh + gv, pw, rh];
            pos.row2_1 = [ml,                    mt,           pw, rh];
            pos.row2_2 = [ml + pw + gh,          mt,           pw, rh];
            pos.row2_3 = [ml + 2*(pw + gh),      mt,           pw, rh];

            if useUrdfSlot
                obj.liveAxes.urdf = axes('Parent', fig, 'Position', pos.row1_1);
            else
                obj.liveAxes.row1_1 = axes('Parent', fig, 'Position', pos.row1_1);
            end
            obj.liveAxes.row1_2 = axes('Parent', fig, 'Position', pos.row1_2);
            obj.liveAxes.row1_3 = axes('Parent', fig, 'Position', pos.row1_3);

            [topPos, botPos] = obj.splitStacked(pos.row2_1);
            obj.liveAxes.posTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.posBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(pos.row2_2);
            obj.liveAxes.velTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.velBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(pos.row2_3);
            obj.liveAxes.forceTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.forceBot = axes('Parent', fig, 'Position', botPos);
        end

        function renderLiveAxes(obj, logs, useUrdfSlot)
            if ~useUrdfSlot && isfield(obj.liveAxes, 'row1_1') && isgraphics(obj.liveAxes.row1_1)
                axes(obj.liveAxes.row1_1);
                cla(obj.liveAxes.row1_1);
                obj.plot3DView(logs, false);
            end

            axes(obj.liveAxes.row1_2);
            cla(obj.liveAxes.row1_2);
            obj.plotXYView(logs);

            axes(obj.liveAxes.row1_3);
            cla(obj.liveAxes.row1_3);
            obj.plotZvsT(logs);

            axes(obj.liveAxes.posTop);
            cla(obj.liveAxes.posTop);
            obj.plotPosition(logs);

            axes(obj.liveAxes.posBot);
            cla(obj.liveAxes.posBot);
            obj.plotOrientation(logs);

            axes(obj.liveAxes.velTop);
            cla(obj.liveAxes.velTop);
            obj.plotLinearVel(logs);

            axes(obj.liveAxes.velBot);
            cla(obj.liveAxes.velBot);
            obj.plotAngularVel(logs);

            axes(obj.liveAxes.forceTop);
            cla(obj.liveAxes.forceTop);
            obj.plotForce(logs);

            axes(obj.liveAxes.forceBot);
            cla(obj.liveAxes.forceBot);
            obj.plotTorque(logs);
        end

        function plotStacked(obj, topFunc, botFunc, pos, logs)
            [topPos, botPos] = obj.splitStacked(pos);
            
            subplot('Position', topPos); topFunc(logs);
            subplot('Position', botPos); botFunc(logs);
        end

        function plotStackedEst(obj, topFunc, botFunc, pos, est)
            [topPos, botPos] = obj.splitStacked(pos);
            
            subplot('Position', topPos); topFunc(est);
            subplot('Position', botPos); botFunc(est);
        end

        function [topPos, botPos] = splitStacked(~, pos)
            h = pos(4) / 2 - 0.01;
            topPos = [pos(1), pos(2) + h + 0.02, pos(3), h];
            botPos = [pos(1), pos(2), pos(3), h];
        end

        function plot3DView(obj, logs, showEnd)
            if nargin < 3
                showEnd = false;
            end
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            view(ax, 3);
            title(ax, '3D Path');
            plot3(ax, logs.des.pos(:,1), logs.des.pos(:,2), logs.des.pos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            plot3(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), logs.actual.pos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            plot3(ax, logs.actual.pos(1,1), logs.actual.pos(1,2), logs.actual.pos(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
            if showEnd
                plot3(ax, logs.actual.pos(end,1), logs.actual.pos(end,2), logs.actual.pos(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
                legend(ax, {'Des','Act','Start','End'}, 'Location', 'best', 'FontSize', 6);
            else
                legend(ax, {'Des','Act','Start'}, 'Location', 'best', 'FontSize', 6);
            end
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            axis(ax, 'equal');
        end

        function plotXYView(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'XY Path');
            plot(ax, logs.des.pos(:,1), logs.des.pos(:,2), 'k--', 'LineWidth', obj.lineWidth);
            plot(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), 'b-', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
            legend(ax, {'Des','Act'}, 'Location', 'best', 'FontSize', 6);
            axis(ax, 'equal');
        end

        function plotZvsT(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Altitude');
            plot(ax, logs.t, logs.actual.pos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            plot(ax, logs.t, logs.des.pos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Z [m]');
            legend(ax, {'Act','Des'}, 'Location', 'best', 'FontSize', 6);
            obj.setXLim(ax, logs.t);
        end

        function plotPosition(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Position [m]', 'FontSize', 9);
            plot(ax, logs.t, logs.actual.pos(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.pos(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.pos(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.pos(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.pos(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.pos(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'x','y','z','x_d','y_d','z_d'}, 'Location', 'best', 'FontSize', 6);
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
        end

        function plotOrientation(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Orientation [deg]', 'FontSize', 9);
            rpy_act = logs.actual.rpy * (180/pi);
            rpy_des = logs.des.rpy * (180/pi);
            plot(ax, logs.t, rpy_act(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_act(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_act(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, rpy_des(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_des(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_des(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'roll','pitch','yaw','roll_d','pitch_d','yaw_d'}, 'Location', 'best', 'FontSize', 6);
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
        end

        function plotLinearVel(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Linear Vel [m/s]', 'FontSize', 9);
            plot(ax, logs.t, logs.actual.linVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.linVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.linVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.linVel(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.linVel(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.linVel(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'v_x','v_y','v_z','v_{x,d}','v_{y,d}','v_{z,d}'}, 'Location', 'best', 'FontSize', 6);
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
        end

        function plotAngularVel(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Angular Vel [rad/s]', 'FontSize', 9);
            plot(ax, logs.t, logs.actual.angVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.angVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.angVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.angVel(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.angVel(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.angVel(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'\omega_x','\omega_y','\omega_z','\omega_{x,d}','\omega_{y,d}','\omega_{z,d}'}, 'Location', 'best', 'FontSize', 6);
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
        end

        function plotForce(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Force [N]', 'FontSize', 9);
            plot(ax, logs.t, logs.cmd.wrenchF(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'F_x','F_y','F_z'}, 'Location', 'best', 'FontSize', 6);
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
        end

        function plotTorque(obj, logs)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Torque [Nm]', 'FontSize', 9);
            plot(ax, logs.t, logs.cmd.wrenchT(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'\tau_x','\tau_y','\tau_z'}, 'Location', 'best', 'FontSize', 6);
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
        end

        function plotMass(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Mass [kg]', 'FontSize', 9);
            plot(ax, est.t, est.mass, 'b-', 'LineWidth', obj.lineWidth);
            if isfield(est, 'massActual')
                plot(ax, est.t, est.massActual, 'k--', 'LineWidth', obj.lineWidth);
                legend(ax, {'est','actual'}, 'Location', 'best', 'FontSize', 6);
            else
                legend(ax, {'est'}, 'Location', 'best', 'FontSize', 6);
            end
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotCoG(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'CoG [m]', 'FontSize', 9);
            plot(ax, est.t, est.com(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.com(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.com(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'comActual')
                plot(ax, est.t, est.comActual(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.comActual(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.comActual(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                legend(ax, {'x','y','z','x_{act}','y_{act}','z_{act}'}, 'Location', 'best', 'FontSize', 6);
            else
                legend(ax, {'x','y','z'}, 'Location', 'best', 'FontSize', 6);
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotPrincipalInertia(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Principal Inertia [kg·m²]', 'FontSize', 9);
            plot(ax, est.t, est.inertia(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.inertia(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.inertia(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'inertiaActual')
                plot(ax, est.t, est.inertiaActual(:,1), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertiaActual(:,2), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertiaActual(:,3), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                legend(ax, {'Ixx','Iyy','Izz','Ixx_{act}','Iyy_{act}','Izz_{act}'}, 'Location', 'best', 'FontSize', 6);
            else
                legend(ax, {'Ixx','Iyy','Izz'}, 'Location', 'best', 'FontSize', 6);
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5);
            end
        end

        function plotOffDiagInertia(obj, est)
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Off-diag Inertia [kg·m²]', 'FontSize', 9);
            if size(est.inertia, 2) >= 6
                plot(ax, est.t, est.inertia(:,4), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertia(:,6), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertia(:,5), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                if isfield(est, 'inertiaActual') && size(est.inertiaActual, 2) >= 6
                    plot(ax, est.t, est.inertiaActual(:,4), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                    plot(ax, est.t, est.inertiaActual(:,6), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                    plot(ax, est.t, est.inertiaActual(:,5), '--', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                    legend(ax, {'Ixy','Iyz','Izx','Ixy_{act}','Iyz_{act}','Izx_{act}'}, 'Location', 'best', 'FontSize', 6);
                else
                    legend(ax, {'Ixy','Iyz','Izx'}, 'Location', 'best', 'FontSize', 6);
                end
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

        function saveFigureInternal(obj, fig, filename)
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
