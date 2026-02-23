classdef PathGenerator < handle
    properties
        name
        scale
        period
        altitude
        startWithHover
        hoverFrac
        squareExponent
    end

    methods
        function obj = PathGenerator(cfg)
            obj.name = cfg.traj.name;
            obj.scale = cfg.traj.scale;
            obj.period = cfg.traj.period;
            obj.altitude = cfg.traj.altitude;
            obj.startWithHover = cfg.traj.startWithHover;
            obj.hoverFrac = 0.1;
            obj.squareExponent = 4;

            if isfield(cfg.traj, 'hoverFrac')
                obj.hoverFrac = cfg.traj.hoverFrac;
            end
            if isfield(cfg.traj, 'squareExponent')
                obj.squareExponent = cfg.traj.squareExponent;
            end
        end

        function [H, V, A] = generate(obj, t)
            if obj.startWithHover
                hover_time = obj.hoverFrac * obj.period;
                if t < hover_time
                    [s, sd, sdd] = obj.smoothTimeScaling(t, hover_time);
                    z = obj.altitude * s;
                    p = [0; 0; z];
                    v = [0; 0; obj.altitude * sd];
                    a = [0; 0; obj.altitude * sdd];
                    yaw = 0; wyaw = 0; wyawdot = 0;
                    H = obj.poseFromYawPos(yaw, p);
                    V = [0;0;wyaw; v];
                    A = [0;0;wyawdot; a];
                    return;
                else
                    t = t - hover_time;
                end
            end

            % time within one cycle
            T = obj.period;
            tmod = mod(t, T);
            s = tmod / T;
            sdot = 1 / T;
            sddot = 0;

            switch lower(obj.name)
                case 'hover'
                    p = [0; 0; obj.altitude];
                    v = [0; 0; 0];
                    a = [0; 0; 0];
                    yaw = 0; wyaw = 0; wyawdot = 0;

                case 'circle'
                    r = obj.scale;
                    theta = 2*pi*s;
                    p = [r*(cos(theta)-1); r*sin(theta); obj.altitude];
                    dp = 2*pi * [-r*sin(theta); r*cos(theta); 0];
                    d2p = (2*pi)^2 * [-r*cos(theta); -r*sin(theta); 0];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'square'
                    a0 = obj.scale;
                    n = obj.squareExponent;
                    theta = 2*pi*s;
                    c = cos(theta); ss = sin(theta);
                    p = [a0 * sign(c) * abs(c)^n; a0 * sign(ss) * abs(ss)^n; obj.altitude];

                    dp_dth = [-a0 * n * abs(c)^(n-1) * ss; a0 * n * abs(ss)^(n-1) * c; 0];
                    d2p_dth2 = [-a0 * n * (abs(c)^(n-1) * c + (n-1) * abs(c)^(n-2) * sign(c) * ss^2); ...
                                 a0 * n * (-abs(ss)^(n-1) * ss + (n-1) * abs(ss)^(n-2) * sign(ss) * c^2); 0];

                    dp = 2*pi * dp_dth;
                    d2p = (2*pi)^2 * d2p_dth2;
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'infinity'
                    a0 = obj.scale;
                    theta = 2*pi*s;
                    p = [a0*sin(theta); a0*sin(2*theta)/2; obj.altitude];
                    dp = 2*pi * [a0*cos(theta); a0*cos(2*theta); 0];
                    d2p = (2*pi)^2 * [-a0*sin(theta); -2*a0*sin(2*theta); 0];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'takeoffland'
                    t1 = 0.25*T; t2 = 0.75*T;
                    if tmod < t1
                        [s1, sd1, sdd1] = obj.smoothTimeScaling(tmod, t1);
                        z = obj.altitude * s1;
                        p = [0; 0; z];
                        v = [0; 0; obj.altitude * sd1];
                        a = [0; 0; obj.altitude * sdd1];
                    elseif tmod < t2
                        [s2, sd2, sdd2] = obj.smoothTimeScaling(tmod - t1, t2 - t1);
                        x = obj.scale * s2;
                        p = [x; 0; obj.altitude];
                        v = [obj.scale * sd2; 0; 0];
                        a = [obj.scale * sdd2; 0; 0];
                    else
                        [s3, sd3, sdd3] = obj.smoothTimeScaling(tmod - t2, T - t2);
                        z = obj.altitude * (1 - s3);
                        p = [obj.scale; 0; z];
                        v = [0; 0; -obj.altitude * sd3];
                        a = [0; 0; -obj.altitude * sdd3];
                    end
                    yaw = 0; wyaw = 0; wyawdot = 0;

                otherwise
                    error('Unknown trajectory: %s', obj.name);
            end

            H = obj.poseFromYawPos(yaw, p);
            R = H(1:3,1:3);
            v_des = R' * v;
            omega_des = [0; 0; wyaw];
            a_des = R' * a - cross(omega_des, v_des);

            V = [0;0;wyaw; v_des];
            A = [0;0;wyawdot; a_des];
        end
    end

    methods (Access = private)
        function [s, sd, sdd] = smoothTimeScaling(~, t, T)
            if T <= 0
                s = 1; sd = 0; sdd = 0;
                return;
            end
            tau = min(max(t / T, 0), 1);
            s = 10*tau^3 - 15*tau^4 + 6*tau^5;
            sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
            sdd = (60*tau - 180*tau^2 + 120*tau^3) / (T^2);
        end

        function [yaw, wyaw, wyawdot] = yawFromVelocity(~, v, a)
            vx = v(1); vy = v(2);
            ax = a(1); ay = a(2);
            if hypot(vx, vy) < 1e-6
                yaw = 0; wyaw = 0; wyawdot = 0;
                return;
            end
            yaw = atan2(vy, vx);
            wyaw = (vx * ay - vy * ax) / (vx^2 + vy^2);
            wyawdot = 0;
        end

        function H = poseFromYawPos(~, yaw, p)
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            H = [Rz, p; 0 0 0 1];
        end
    end
end
