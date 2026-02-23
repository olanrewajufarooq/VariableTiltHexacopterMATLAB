classdef UrdfViewer < handle
    properties
        urdfPath
        robot
        fig
        ax
        baseTf
        hasRobotics
        hFallback
        axisLimits
        armLength
        dynamicAxis
        axisPadding
        hOrigin
        hPathDesired
        hPathActual
        pathDesired
        pathActual
    end

    methods
        function obj = UrdfViewer(urdfPath)
            if nargin < 1 || isempty(urdfPath)
                urdfPath = obj.defaultUrdfPath();
            end
            obj.urdfPath = urdfPath;
            obj.hasRobotics = exist('importrobot','file') == 2 && exist(urdfPath,'file') == 2;

            obj.axisLimits = [-1 1 -1 1 -0.1 2];
            obj.armLength = 0.229;
            obj.dynamicAxis = false;
            obj.axisPadding = 1.5;
            obj.pathDesired = zeros(0,3);
            obj.pathActual = zeros(0,3);

            obj.fig = figure('Name','URDF Viewer','Position',[100 100 800 600]);
            obj.ax = axes(obj.fig);
            grid(obj.ax,'on'); axis(obj.ax,'equal'); view(obj.ax,3);
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
            xlabel(obj.ax,'X'); ylabel(obj.ax,'Y'); zlabel(obj.ax,'Z');
            hold(obj.ax,'on');
            obj.hOrigin = plot3(obj.ax, 0, 0, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5);
            obj.hPathDesired = plot3(obj.ax, NaN, NaN, NaN, 'k--', 'LineWidth', 1.2);
            obj.hPathActual = plot3(obj.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);

            if obj.hasRobotics
                try
                    obj.robot = importrobot(urdfPath, 'DataFormat','column');
                    obj.robot.Gravity = [0 0 -9.81];
                    cfg = homeConfiguration(obj.robot);
                    h = show(obj.robot, cfg, 'Parent', obj.ax, 'PreservePlot', false, 'Frames','off');
                    obj.baseTf = hgtransform('Parent', obj.ax);
                    set(h, 'Parent', obj.baseTf);
                    axis(obj.ax, obj.axisLimits);
                    axis(obj.ax, 'manual');
                catch err
                    try
                        urdfFixed = obj.sanitizeUrdf(urdfPath);
                        obj.robot = importrobot(urdfFixed, 'DataFormat','column');
                        obj.robot.Gravity = [0 0 -9.81];
                        cfg = homeConfiguration(obj.robot);
                        h = show(obj.robot, cfg, 'Parent', obj.ax, 'PreservePlot', false, 'Frames','off');
                        obj.baseTf = hgtransform('Parent', obj.ax);
                        set(h, 'Parent', obj.baseTf);
                        axis(obj.ax, obj.axisLimits);
                        axis(obj.ax, 'manual');
                    catch err2
                        warning('URDF import failed (%s). Using fallback visualization.', err2.message);
                        obj.hasRobotics = false;
                        obj.hFallback = obj.drawFallbackModel(eye(4));
                    end
                end
            else
                obj.hFallback = obj.drawFallbackModel(eye(4));
            end
        end

        function showPose(obj, H)
            if obj.hasRobotics
                obj.baseTf.Matrix = H;
            else
                obj.updateFallbackModel(H);
            end
            if obj.dynamicAxis
                obj.updateAxis(H(1:3,4));
            else
                axis(obj.ax, obj.axisLimits);
                axis(obj.ax, 'manual');
            end
            drawnow;
        end

        function updatePaths(obj, pDesired, pActual)
            if nargin >= 2 && ~isempty(pDesired)
                obj.pathDesired(end+1,:) = pDesired(:).';
                set(obj.hPathDesired, 'XData', obj.pathDesired(:,1), 'YData', obj.pathDesired(:,2), 'ZData', obj.pathDesired(:,3));
            end
            if nargin >= 3 && ~isempty(pActual)
                obj.pathActual(end+1,:) = pActual(:).';
                set(obj.hPathActual, 'XData', obj.pathActual(:,1), 'YData', obj.pathActual(:,2), 'ZData', obj.pathActual(:,3));
            end
        end

        function setAxisLimits(obj, limits)
            obj.axisLimits = limits(:).';
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
        end

        function setDynamicAxis(obj, enable, padding)
            obj.dynamicAxis = logical(enable);
            if nargin >= 3 && ~isempty(padding)
                obj.axisPadding = padding;
            end
        end
    end

    methods (Access = private)
        function urdfPath = defaultUrdfPath(obj)
            root = obj.repoRoot();
            urdfPath = fullfile(root, 'assets', 'hexacopter_description', 'urdf', 'variable_tilt_hexacopter.urdf');
            if ~exist(urdfPath,'file')
                alt = fullfile(root, 'assets', 'hexacopter_description', 'urdf', 'variable_tilt_hexacopter_adaptive.urdf');
                if exist(alt,'file')
                    urdfPath = alt;
                end
            end
        end

        function root = repoRoot(~)
            p = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(p))));
        end

        function h = drawFallbackModel(obj, H)
            p = H(1:3,4);
            R = H(1:3,1:3);
            angles = (0:5) * (pi/3);
            ends = [obj.armLength * cos(angles); obj.armLength * sin(angles); zeros(1,6)];
            endsW = R * ends + p;

            hold(obj.ax,'on');
            h.center = plot3(obj.ax, p(1), p(2), p(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
            h.arms = gobjects(6,1);
            for i = 1:6
                h.arms(i) = plot3(obj.ax, [p(1) endsW(1,i)], [p(2) endsW(2,i)], [p(3) endsW(3,i)], 'b-', 'LineWidth', 2);
            end
        end

        function updateFallbackModel(obj, H)
            p = H(1:3,4);
            R = H(1:3,1:3);
            angles = (0:5) * (pi/3);
            ends = [obj.armLength * cos(angles); obj.armLength * sin(angles); zeros(1,6)];
            endsW = R * ends + p;

            set(obj.hFallback.center, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
            for i = 1:6
                set(obj.hFallback.arms(i), 'XData', [p(1) endsW(1,i)], 'YData', [p(2) endsW(2,i)], 'ZData', [p(3) endsW(3,i)]);
            end
        end

        function updateAxis(obj, p)
            pad = obj.axisPadding;
            lim = obj.axisLimits;

            halfXY = max(abs(lim(1:4)));
            if abs(p(1)) > halfXY || abs(p(2)) > halfXY
                newHalf = max(abs(p(1)), abs(p(2))) + pad;
                lim(1) = -newHalf;
                lim(2) =  newHalf;
                lim(3) = -newHalf;
                lim(4) =  newHalf;
            end

            if p(3) > lim(6)
                lim(6) = p(3) + pad;
            end

            lim(5) = 0;
            lim(6) = max(lim(6), 0);

            obj.axisLimits = lim;
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
        end

        function outPath = sanitizeUrdf(~, inPath)
            txt = fileread(inPath);

            txt = regexprep(txt, '<box>\s*<size>([^<]+)</size>\s*</box>', '<box size="$1"/>');
            txt = regexprep(txt, '<cylinder>\s*<radius>([^<]+)</radius>\s*<length>([^<]+)</length>\s*</cylinder>', '<cylinder radius="$1" length="$2"/>');
            txt = regexprep(txt, '<sphere>\s*<radius>([^<]+)</radius>\s*</sphere>', '<sphere radius="$1"/>');

            txt = regexprep(txt, '<mass>\s*([^<]+)\s*</mass>', '<mass value="$1"/>');
            txt = regexprep(txt, '<inertia>\s*<ixx>([^<]+)</ixx>\s*<ixy>([^<]+)</ixy>\s*<ixz>([^<]+)</ixz>\s*<iyy>([^<]+)</iyy>\s*<iyz>([^<]+)</iyz>\s*<izz>([^<]+)</izz>\s*</inertia>', ...
                '<inertia ixx="$1" ixy="$2" ixz="$3" iyy="$4" iyz="$5" izz="$6"/>');

            txt = regexprep(txt, '<material>\s*', '<material name="default">');

            outPath = fullfile(tempdir, 'vt_hexacopter_sanitized.urdf');
            fid = fopen(outPath, 'w');
            fwrite(fid, txt);
            fclose(fid);
        end
    end
end
