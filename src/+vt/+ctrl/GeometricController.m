classdef GeometricController < handle
    properties
        Kp
        Kd
        Kp_att
        Kp_pos
        m
        g
        CoG
        potType
        method
        I6
    end

    methods
        function obj = GeometricController(cfg)
            obj.m = cfg.vehicle.m;
            obj.g = cfg.vehicle.g;
            obj.CoG = cfg.vehicle.CoG(:);

            obj.potType = cfg.controller.potential;
            obj.method = cfg.controller.type;

            kp = cfg.controller.Kp(:);
            kd = cfg.controller.Kd(:);
            obj.Kp = diag(kp);
            obj.Kd = diag(kd);
            obj.Kp_att = diag(kp(1:3));
            obj.Kp_pos = diag(kp(4:6));

            if isfield(cfg.vehicle,'I6') && ~isempty(cfg.vehicle.I6)
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = vt.utils.getGeneralizedInertia(cfg.vehicle.m, cfg.vehicle.I_params, obj.CoG);
            end
        end

        function W = computeWrench(obj, Hd, H, Vd, V)
            He = vt.se3.invSE3(Hd) * H;
            AdInvHe = vt.se3.Ad_inv(He);
            Ve = V - AdInvHe * Vd;

            Wg = obj.gravityWrench(H);
            Wp = obj.proportionalWrench(Hd, H);
            Wd = -obj.Kd * Ve;

            W = -Wg - Wp + Wd;

            if strcmpi(obj.method, 'FeedLin')
                C = vt.se3.adV(V)' * obj.I6 * V;
                W = W + C;
            end
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj, H)
            R = H(1:3,1:3);
            gvec = [0; 0; -obj.g];
            f_g = obj.m * (R' * gvec);
            tau_g = cross(obj.CoG, f_g);
            Wg = [tau_g; f_g];
        end

        function Wp = proportionalWrench(obj, Hd, H)
            He = vt.se3.invSE3(Hd) * H;

            if strcmpi(obj.potType, 'liealgebra')
                zeta = vt.se3.logSE3(He);
                Wp = obj.Kp * zeta;
            else
                R = H(1:3,1:3);
                p = H(1:3,4);
                Rd = Hd(1:3,1:3);
                pd = Hd(1:3,4);

                R_err = Rd' * R;
                e_p = p - pd;

                F = obj.Kp_pos * e_p;
                T = -vt.se3.vee3(0.5 * (obj.Kp_att * R_err - (obj.Kp_att * R_err)'));
                Wp = [T; F];
            end
        end
    end
end
