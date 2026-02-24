classdef Plotter < handle
    %PLOTTER Generate live and summary plots for simulations.
    %   Supports live updating layouts, summary figures, and standalone
    %   stacked plots. Works with log structs from vt.core.Logger.
    %
    %   Plot outputs are saved under results/<run-id>/ by default.
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
        liveLayoutType
    end

    methods
        function obj = Plotter(outDir, opts)
            %PLOTTER Configure plotting output and styling options.
            %   Inputs:
            %     outDir - output directory for saved plots.
            %     opts - struct with fields: savePng, duration.
            %
            %   Output:
            %     obj - Plotter instance.
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
            %SAVEFIGURE Save a figure using configured options.
            %   Inputs:
            %     fig - figure handle.
            %     filename - file base name (no extension).
            obj.saveFigureInternal(fig, filename);
        end

        function fig = plotLiveNominal(obj, logs, fig, useUrdfSlot, layoutType)
            %PLOTLIVENOMINAL Render or update live nominal layout.
            %   Inputs:
            %     logs - logger struct (may be empty during initialization).
            %     fig - existing figure handle (optional).
            %     useUrdfSlot - reserve axes for URDF view (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - live view figure handle.
            if nargin < 4
                useUrdfSlot = false;
            end
            if nargin < 5
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1400 600]);
                obj.liveAxes = struct();
            end

            if useUrdfSlot
                obj.ensureLiveLayout(fig, useUrdfSlot, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    obj.renderLiveAxes(logs, useUrdfSlot);
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                else
                    sgtitle(fig, 'Live View');
                end
            else
                clf(fig);
                obj.drawNominalLayout(fig, logs, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                end
            end
        end

        function fig = plotLiveAdaptive(obj, logs, est, fig, useUrdfSlot, layoutType)
            %PLOTLIVEADAPTIVE Render or update live adaptive layout.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            %     fig - existing figure handle (optional).
            %     useUrdfSlot - reserve axes for URDF view (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - live view figure handle.
            if nargin < 5
                useUrdfSlot = false;
            end
            if nargin < 6
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1600 900]);
                obj.liveAxes = struct();
            end

            if useUrdfSlot
                obj.ensureLiveLayoutAdaptive(fig, useUrdfSlot, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    if nargin < 3 || isempty(est)
                        est = [];
                    end
                    obj.renderLiveAxesAdaptive(logs, est, useUrdfSlot);
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                else
                    sgtitle(fig, 'Live View');
                end
            else
                clf(fig);
                obj.drawAdaptiveLayout(fig, logs, est, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                end
            end
        end

        function ax = getLiveUrdfAxes(obj)
            %GETLIVEURDFAXES Return axes handle reserved for URDF view.
            %   Output:
            %     ax - axes handle or empty.
            ax = [];
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'urdf') && isgraphics(obj.liveAxes.urdf)
                ax = obj.liveAxes.urdf;
            end
        end

        function fig = plotSummaryNominal(obj, logs, fig, layoutType)
            %PLOTSUMMARYNOMINAL Plot final summary for nominal run.
            %   Inputs:
            %     logs - logger struct.
            %     fig - existing figure handle (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - summary figure handle.
            if nargin < 4
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
            else
                clf(fig);
            end
            
            obj.drawNominalLayout(fig, logs, layoutType);
            obj.saveFigureInternal(fig, 'summary_nominal');
        end

        function fig = plotSummaryAdaptive(obj, logs, est, fig, layoutType)
            %PLOTSUMMARYADAPTIVE Plot final summary for adaptive run.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            %     fig - existing figure handle (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - summary figure handle.
            if nargin < 5
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
            else
                clf(fig);
            end
            
            obj.drawAdaptiveLayout(fig, logs, est, layoutType);
            obj.saveFigureInternal(fig, 'summary_adaptive');
        end

        function plotStandaloneSubplotsNominal(obj, logs)
            %PLOTSTANDALONESUBLOTSNOMINAL Save separate nominal plots.
            %   Input:
            %     logs - logger struct with nominal signals.
            fig = figure('Name','Standalone - 3D Path');
            obj.plot3DView(logs);
            obj.saveFigureInternal(fig, 'standalone_3d');

            fig = figure('Name','Standalone - XY Path');
            obj.plotXYView(logs);
            obj.saveFigureInternal(fig, 'standalone_xy');

            fig = figure('Name','Standalone - Altitude');
            obj.plotZvsT(logs);
            obj.saveFigureInternal(fig, 'standalone_z');

            fig = figure('Name','Standalone - Position & Orientation');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_pos_orient');

            fig = figure('Name','Standalone - Velocity');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotLinearVel(logs);
            axes(ax(2)); obj.plotAngularVel(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_vel');

            fig = figure('Name','Standalone - Wrench');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotForce(logs);
            axes(ax(2)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_wrench');
        end

        function plotStandaloneSubplotsAdaptive(obj, logs, est)
            %PLOTSTANDALONESUBPLOTSADAPTIVE Save separate adaptive plots.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            obj.plotStandaloneSubplotsNominal(logs);

            fig = figure('Name','Standalone - Mass & CoG');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotMass(est);
            axes(ax(2)); obj.plotCoG(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_mass_cog');

            fig = figure('Name','Standalone - Principal Inertia');
            obj.plotPrincipalInertia(est);
            obj.saveFigureInternal(fig, 'standalone_inertia_principal');

            fig = figure('Name','Standalone - Off-diag Inertia');
            obj.plotOffDiagInertia(est);
            obj.saveFigureInternal(fig, 'standalone_inertia_offdiag');
        end

        function plotStackedAllState(obj, logs)
            %PLOTSTACKEDALLSTATE Plot stacked state and wrench signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - States');
            ax = obj.createVerticalAxes(fig, 6);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            axes(ax(3)); obj.plotLinearVel(logs);
            axes(ax(4)); obj.plotAngularVel(logs);
            axes(ax(5)); obj.plotForce(logs);
            axes(ax(6)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_all_state');
        end

        function plotStackedPositionOrientation(obj, logs)
            %PLOTSTACKEDPOSITIONORIENTATION Plot stacked pose signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Position & Orientation');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_pos_orient');
        end

        function plotStackedVelocity(obj, logs)
            %PLOTSTACKEDVELOCITY Plot stacked velocity signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Velocity');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotLinearVel(logs);
            axes(ax(2)); obj.plotAngularVel(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_vel');
        end

        function plotStackedWrench(obj, logs)
            %PLOTSTACKEDWRENCH Plot stacked force/torque signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Force & Torque');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotForce(logs);
            axes(ax(2)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_wrench');
        end

        function plotStackedEstimation(obj, est)
            %PLOTSTACKEDESTIMATION Plot stacked estimation signals.
            %   Input:
            %     est - estimation struct.
            fig = figure('Name','Stacked - Mass, CoG, Inertia');
            ax = obj.createVerticalAxes(fig, 4);
            axes(ax(1)); obj.plotMass(est);
            axes(ax(2)); obj.plotCoG(est);
            axes(ax(3)); obj.plotPrincipalInertia(est);
            axes(ax(4)); obj.plotOffDiagInertia(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_estimation');
        end

        function plotStackedInertia(obj, est)
            %PLOTSTACKEDINERTIA Plot stacked inertia estimates.
            %   Input:
            %     est - estimation struct.
            fig = figure('Name','Stacked - Inertia');
            ax = obj.createVerticalAxes(fig, 2);
            axes(ax(1)); obj.plotPrincipalInertia(est);
            axes(ax(2)); obj.plotOffDiagInertia(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_inertia');
        end
    end

    methods (Access = private)
        function drawNominalLayout(obj, fig, logs, layoutType)
            %DRAWNOMINALLAYOUT Draw nominal summary layout.
            %   Inputs:
            %     fig - figure handle.
            %     logs - logger struct.
            %     layoutType - 'row-major' or 'column-major'.
            layoutType = obj.normalizeLayoutType(layoutType);
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.08; mb = 0.08;
            gh = 0.03; gv = 0.09;

            if strcmp(layoutType, 'column-major')
                posGridT = obj.buildGridPositions(3, 2, ml, mr, mt, mb, gh, gv);
                posList = cell(1, 6);
                for i = 1:6
                    r = ceil(i / 3);
                    c = i - (r - 1) * 3;
                    posList{i} = posGridT{c, r};
                end
            else
                posGrid = obj.buildGridPositions(2, 3, ml, mr, mt, mb, gh, gv);
                posList = obj.orderPositions(posGrid, 'row-major');
            end

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{2}); obj.plotXYView(logs);
            subplot('Position', posList{3}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{4}, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{5}, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{6}, logs);
        end

        function drawAdaptiveLayout(obj, fig, logs, est, layoutType)
            %DRAWADAPTIVELAYOUT Draw adaptive summary layout.
            %   Inputs:
            %     fig - figure handle.
            %     logs - logger struct.
            %     est - estimation struct.
            %     layoutType - 'row-major' or 'column-major'.
            layoutType = obj.normalizeLayoutType(layoutType);
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.06; mb = 0.08;
            gh = 0.03; gv = 0.07;

            posGrid = obj.buildGridPositions(3, 3, ml, mr, mt, mb, gh, gv);
            posList = obj.orderPositions(posGrid, layoutType);

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{2}); obj.plotXYView(logs);
            subplot('Position', posList{3}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{4}, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{5}, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{6}, logs);

            obj.plotStackedEst(@obj.plotMass, @obj.plotCoG, posList{7}, est);
            subplot('Position', posList{8}); obj.plotPrincipalInertia(est);
            subplot('Position', posList{9}); obj.plotOffDiagInertia(est);
        end

        function ensureLiveLayout(obj, fig, useUrdfSlot, layoutType)
            %ENSURELIVELAYOUT Build live axes for nominal runs.
            %   Inputs:
            %     fig - figure handle.
            %     useUrdfSlot - true to reserve a URDF axes slot.
            %     layoutType - layout string.
            layoutType = obj.normalizeLayoutType(layoutType);
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'xy') && isgraphics(obj.liveAxes.xy)
                if isequal(obj.liveUseUrdf, useUrdfSlot) && strcmp(obj.liveLayoutType, layoutType)
                    return;
                end
            end

            obj.liveUseUrdf = useUrdfSlot;
            obj.liveLayoutType = layoutType;
            obj.liveAxes = struct();
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.08; mb = 0.08;
            gh = 0.03; gv = 0.06;

            if strcmp(layoutType, 'column-major')
                posGridT = obj.buildGridPositions(3, 2, ml, mr, mt, mb, gh, gv);
                posList = cell(1, 6);
                for i = 1:6
                    r = ceil(i / 3);
                    c = i - (r - 1) * 3;
                    posList{i} = posGridT{c, r};
                end
            else
                posGrid = obj.buildGridPositions(2, 3, ml, mr, mt, mb, gh, gv);
                posList = obj.orderPositions(posGrid, 'row-major');
            end

            if useUrdfSlot
                obj.liveAxes.urdf = axes('Parent', fig, 'Position', posList{1});
            else
                obj.liveAxes.view3d = axes('Parent', fig, 'Position', posList{1});
            end
            obj.liveAxes.xy = axes('Parent', fig, 'Position', posList{2});
            obj.liveAxes.z = axes('Parent', fig, 'Position', posList{3});

            [topPos, botPos] = obj.splitStacked(posList{4});
            obj.liveAxes.posTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.posBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{5});
            obj.liveAxes.velTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.velBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{6});
            obj.liveAxes.forceTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.forceBot = axes('Parent', fig, 'Position', botPos);
        end

        function ensureLiveLayoutAdaptive(obj, fig, useUrdfSlot, layoutType)
            %ENSURELIVELAYOUTADAPTIVE Build live axes for adaptive runs.
            %   Inputs:
            %     fig - figure handle.
            %     useUrdfSlot - true to reserve a URDF axes slot.
            %     layoutType - layout string.
            layoutType = obj.normalizeLayoutType(layoutType);
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'xy') && isgraphics(obj.liveAxes.xy)
                if isequal(obj.liveUseUrdf, useUrdfSlot) && strcmp(obj.liveLayoutType, layoutType)
                    return;
                end
            end

            obj.liveUseUrdf = useUrdfSlot;
            obj.liveLayoutType = layoutType;
            obj.liveAxes = struct();
            clf(fig);
            ml = 0.06; mr = 0.02; mt = 0.06; mb = 0.08;
            gh = 0.03; gv = 0.04;

            posGrid = obj.buildGridPositions(3, 3, ml, mr, mt, mb, gh, gv);
            posList = obj.orderPositions(posGrid, layoutType);

            if useUrdfSlot
                obj.liveAxes.urdf = axes('Parent', fig, 'Position', posList{1});
            else
                obj.liveAxes.view3d = axes('Parent', fig, 'Position', posList{1});
            end
            obj.liveAxes.xy = axes('Parent', fig, 'Position', posList{2});
            obj.liveAxes.z = axes('Parent', fig, 'Position', posList{3});

            [topPos, botPos] = obj.splitStacked(posList{4});
            obj.liveAxes.posTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.posBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{5});
            obj.liveAxes.velTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.velBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{6});
            obj.liveAxes.forceTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.forceBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{7});
            obj.liveAxes.estTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.estBot = axes('Parent', fig, 'Position', botPos);

            obj.liveAxes.inertiaP = axes('Parent', fig, 'Position', posList{8});
            obj.liveAxes.inertiaO = axes('Parent', fig, 'Position', posList{9});
        end

        function renderLiveAxes(obj, logs, useUrdfSlot)
            %RENDERLIVEAXES Update nominal live axes with latest logs.
            %   Inputs:
            %     logs - logger struct.
            %     useUrdfSlot - true when URDF occupies one subplot.
            if ~useUrdfSlot && isfield(obj.liveAxes, 'view3d') && isgraphics(obj.liveAxes.view3d)
                axes(obj.liveAxes.view3d);
                cla(obj.liveAxes.view3d);
                obj.plot3DView(logs, false);
            end

            axes(obj.liveAxes.xy);
            cla(obj.liveAxes.xy);
            obj.plotXYView(logs);

            axes(obj.liveAxes.z);
            cla(obj.liveAxes.z);
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

        function renderLiveAxesAdaptive(obj, logs, est, useUrdfSlot)
            %RENDERLIVEAXESADAPTIVE Update adaptive live axes with logs.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            %     useUrdfSlot - true when URDF occupies one subplot.
            if ~useUrdfSlot && isfield(obj.liveAxes, 'view3d') && isgraphics(obj.liveAxes.view3d)
                axes(obj.liveAxes.view3d);
                cla(obj.liveAxes.view3d);
                obj.plot3DView(logs, false);
            end

            axes(obj.liveAxes.xy);
            cla(obj.liveAxes.xy);
            obj.plotXYView(logs);

            axes(obj.liveAxes.z);
            cla(obj.liveAxes.z);
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

            if nargin >= 3 && ~isempty(est)
                axes(obj.liveAxes.estTop);
                cla(obj.liveAxes.estTop);
                obj.plotMass(est);

                axes(obj.liveAxes.estBot);
                cla(obj.liveAxes.estBot);
                obj.plotCoG(est);

                axes(obj.liveAxes.inertiaP);
                cla(obj.liveAxes.inertiaP);
                obj.plotPrincipalInertia(est);

                axes(obj.liveAxes.inertiaO);
                cla(obj.liveAxes.inertiaO);
                obj.plotOffDiagInertia(est);
            end
        end

        function plotStacked(obj, topFunc, botFunc, pos, logs)
            %PLOTSTACKED Helper to draw stacked nominal plots.
            %   Inputs:
            %     topFunc - function handle for top plot.
            %     botFunc - function handle for bottom plot.
            %     pos - subplot position.
            %     logs - logger struct.
            [topPos, botPos] = obj.splitStacked(pos);
            
            subplot('Position', topPos); topFunc(logs);
            subplot('Position', botPos); botFunc(logs);
        end

        function plotStackedEst(obj, topFunc, botFunc, pos, est)
            %PLOTSTACKEDEST Helper to draw stacked estimate plots.
            %   Inputs:
            %     topFunc - function handle for top plot.
            %     botFunc - function handle for bottom plot.
            %     pos - subplot position.
            %     est - estimation struct.
            [topPos, botPos] = obj.splitStacked(pos);
            
            subplot('Position', topPos); topFunc(est);
            subplot('Position', botPos); botFunc(est);
        end

        function [topPos, botPos] = splitStacked(~, pos)
            %SPLITSTACKED Split a subplot position into two rows.
            %   Input:
            %     pos - 1x4 position vector.
            %   Outputs:
            %     topPos - top subplot position.
            %     botPos - bottom subplot position.
            h = pos(4) / 2 - 0.01;
            topPos = [pos(1), pos(2) + h + 0.02, pos(3), h];
            botPos = [pos(1), pos(2), pos(3), h];
        end

        function posGrid = buildGridPositions(~, rows, cols, ml, mr, mt, mb, gh, gv)
            %BUILDGRIDPOSITIONS Build normalized subplot positions.
            %   Inputs:
            %     rows, cols - grid dimensions.
            %     ml,mr,mt,mb - margins.
            %     gh,gv - gaps between subplots.
            %   Output:
            %     posGrid - rows x cols cell of positions.
            pw = (1 - ml - mr - (cols - 1) * gh) / cols;
            rh = (1 - mt - mb - (rows - 1) * gv) / rows;
            posGrid = cell(rows, cols);
            for r = 1:rows
                for c = 1:cols
                    x = ml + (c - 1) * (pw + gh);
                    y = mt + (rows - r) * (rh + gv);
                    posGrid{r, c} = [x, y, pw, rh];
                end
            end
        end

        function posList = orderPositions(~, posGrid, layoutType)
            %ORDERPOSITIONS Flatten grid by row- or column-major order.
            %   Inputs:
            %     posGrid - grid of positions.
            %     layoutType - 'row-major' or 'column-major'.
            %   Output:
            %     posList - linear cell array of positions.
            [rows, cols] = size(posGrid);
            posList = cell(1, rows * cols);
            idx = 1;
            if strcmp(layoutType, 'row-major')
                for r = 1:rows
                    for c = 1:cols
                        posList{idx} = posGrid{r, c};
                        idx = idx + 1;
                    end
                end
            else
                for c = 1:cols
                    for r = 1:rows
                        posList{idx} = posGrid{r, c};
                        idx = idx + 1;
                    end
                end
            end
        end

        function layoutType = normalizeLayoutType(~, layoutType)
            %NORMALIZELAYOUTTYPE Validate layout type string.
            %   Input:
            %     layoutType - requested layout (optional).
            %   Output:
            %     layoutType - validated layout string.
            if nargin < 2 || isempty(layoutType)
                layoutType = 'row-major';
                return;
            end
            layoutType = lower(char(layoutType));
            if ~strcmp(layoutType, 'row-major') && ~strcmp(layoutType, 'column-major')
                layoutType = 'row-major';
            end
        end

        function ax = createVerticalAxes(~, fig, nRows)
            %CREATEVERTICALAXES Create vertically stacked axes.
            %   Inputs:
            %     fig - figure handle.
            %     nRows - number of axes rows.
            %   Output:
            %     ax - array of axes handles.
            ml = 0.08; mr = 0.04; mt = 0.06; mb = 0.08; gv = 0.03;
            totalH = 1 - mt - mb - (nRows - 1) * gv;
            h = totalH / nRows;
            ax = gobjects(nRows, 1);
            for i = 1:nRows
                y = 1 - mt - i * h - (i - 1) * gv;
                ax(i) = axes('Parent', fig, 'Position', [ml, y, 1 - ml - mr, h]);
            end
        end

        function finalizeStackedAxes(~, ax, xlabelText)
            %FINALIZESTACKEDAXES Finalize stacked axis labels and ticks.
            %   Inputs:
            %     ax - axes array.
            %     xlabelText - label for x-axis.
            if isempty(ax)
                return;
            end
            for i = 1:numel(ax)
                set(ax(i), 'TitleFontSizeMultiplier', 0.9);
                set(ax(i), 'LabelFontSizeMultiplier', 0.9);
            end
            for i = 1:numel(ax) - 1
                set(ax(i), 'XTickLabel', []);
                xlabel(ax(i), '');
            end
            xlabel(ax(end), xlabelText);
        end

        function plot3DView(obj, logs, showEnd)
            %PLOT3DVIEW Plot desired vs actual 3D trajectory.
            %   Inputs:
            %     logs - logger struct.
            %     showEnd - true to mark end point (optional).
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
            %PLOTXYVIEW Plot desired vs actual XY projection.
            %   Input:
            %     logs - logger struct.
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'XY Path');
            plot(ax, logs.des.pos(:,1), logs.des.pos(:,2), 'k--', 'LineWidth', obj.lineWidth);
            plot(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), 'b-', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
            legend(ax, {'Des','Act'}, 'Location', 'best', 'FontSize', 6);
            axis(ax, 'equal');
        end

        function plotZvsT(obj, logs)
            %PLOTZVST Plot altitude over time.
            %   Input:
            %     logs - logger struct.
            ax = gca; hold(ax, 'on'); grid(ax, 'on');
            title(ax, 'Altitude');
            plot(ax, logs.t, logs.actual.pos(:,3), 'b-', 'LineWidth', obj.lineWidth);
            plot(ax, logs.t, logs.des.pos(:,3), 'k--', 'LineWidth', obj.lineWidth);
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Z [m]');
            legend(ax, {'Act','Des'}, 'Location', 'best', 'FontSize', 6);
            obj.setXLim(ax, logs.t);
        end

        function plotPosition(obj, logs)
            %PLOTPOSITION Plot position components over time.
            %   Input:
            %     logs - logger struct.
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
            %PLOTORIENTATION Plot roll/pitch/yaw over time.
            %   Input:
            %     logs - logger struct.
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
            %PLOTLINEARVEL Plot linear velocity components.
            %   Input:
            %     logs - logger struct.
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
            %PLOTANGULARVEL Plot angular velocity components.
            %   Input:
            %     logs - logger struct.
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
            %PLOTFORCE Plot commanded forces over time.
            %   Input:
            %     logs - logger struct.
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
            %PLOTTORQUE Plot commanded torques over time.
            %   Input:
            %     logs - logger struct.
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
            %PLOTMASS Plot estimated mass over time.
            %   Input:
            %     est - estimation struct.
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
            %PLOTCOG Plot estimated CoG components.
            %   Input:
            %     est - estimation struct.
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
            %PLOTPRINCIPALINERTIA Plot principal inertia estimates.
            %   Input:
            %     est - estimation struct.
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
            %PLOTOFFDIAGINERTIA Plot off-diagonal inertia estimates.
            %   Input:
            %     est - estimation struct.
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
            %SETXLIM Apply time-axis limits.
            %   Inputs:
            %     ax - axes handle.
            %     tVec - time vector.
            if ~isempty(obj.duration)
                xlim(ax, [0 obj.duration]);
            else
                xlim(ax, [0 max(tVec)]);
            end
        end

        function saveFigureInternal(obj, fig, filename)
            %SAVEFIGUREINTERNAL Save figure to output directory.
            %   Inputs:
            %     fig - figure handle.
            %     filename - base file name.
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
