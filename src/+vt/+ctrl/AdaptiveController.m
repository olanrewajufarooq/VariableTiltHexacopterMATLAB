classdef AdaptiveController < handle
    properties
        Kp
        Kd
        Kp_att
        Kp_pos
        potType
        g
        theta_hat
        Gamma
        I_basis
        G_basis
    end

    properties (Access = private)
        dt
    end

    methods
        function obj = AdaptiveController(cfg)
            obj.potType = cfg.controller.potential;
            obj.g = cfg.vehicle.g;

            kp = cfg.controller.Kp(:);
            kd = cfg.controller.Kd(:);

            obj.dt = cfg.sim.dt;

            obj.Kp = diag(kp);
            obj.Kd = diag(kd);
            obj.Kp_att = diag(kp(1:3));
            obj.Kp_pos = diag(kp(4:6));

            I = cfg.vehicle.I_params;
            m = cfg.vehicle.m;
            CoG = cfg.vehicle.CoG(:);
            % I params order: [Ixx Iyy Izz Ixy Ixz Iyz]
            obj.theta_hat = [I(1); I(2); I(3); I(4); I(5); I(6); m; m*CoG(1); m*CoG(2); m*CoG(3)];

            obj.Gamma = obj.buildGamma(cfg.controller.Gamma);
            obj.constructBases();
        end

        function W = computeWrench(obj, Hd, H, Vd, V, Ades, dt)
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end
            if nargin < 7 || isempty(dt)
                dt = obj.dt;
            end

            H_err = vt.se3.invSE3(Hd) * H;
            Ad_inv_err = vt.se3.Ad_inv(H_err);
            V_e = V - Ad_inv_err * Vd;

            Y = obj.regressor(H_err, H, V, Vd, Ades);
            obj.theta_hat = obj.theta_hat + obj.Gamma * (Y.' * V_e) * dt;

            [m_hat, Iparams_hat, cog_hat] = obj.unpackTheta();
            I_hat = vt.utils.getGeneralizedInertia(m_hat, Iparams_hat, cog_hat);
            Wg_hat = obj.gravityWrenchHat(H, m_hat, cog_hat);

            Wp = obj.proportionalWrench(Hd, H);
            Wd = -obj.Kd * V_e;
            C = vt.se3.adV(V)' * I_hat * V;
            ff = I_hat * Ad_inv_err * (Ades - vt.se3.adV(Vd) * (vt.se3.Ad(H_err) * V_e));

            W = C + ff - Wg_hat - Wp + Wd;
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            % Adjust initial estimates by payload (mass and CoG offset)
            if isempty(obj.theta_hat) || numel(obj.theta_hat) < 10
                return;
            end
            m = obj.theta_hat(7) + m_payload;
            mcog = obj.theta_hat(8:10) + m_payload * CoG_payload(:);
            obj.theta_hat(7) = m;
            obj.theta_hat(8:10) = mcog;
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            [m_hat, Iparams_hat, cog_hat] = obj.unpackTheta();
        end
    end

    methods (Access = private)
        function Gamma = buildGamma(~, gamma)
            if isscalar(gamma)
                Gamma = gamma * eye(10);
            elseif isvector(gamma) && numel(gamma) == 10
                Gamma = diag(gamma);
            else
                Gamma = gamma;
            end
        end

        function constructBases(obj)
            obj.I_basis = cell(10,1);
            obj.G_basis = cell(4,1);

            % Diagonal moments
            for k = 1:3
                E = zeros(3,3); E(k,k) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{k} = Gi;
            end

            % Off-diagonal (xy, yz, xz)
            pairs = [1 2; 2 3; 1 3];
            for idx = 1:3
                i = pairs(idx,1); j = pairs(idx,2);
                E = zeros(3,3); E(i,j) = 1; E(j,i) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{3+idx} = Gi;
            end

            % Mass term
            Gi = zeros(6,6); Gi(4:6,4:6) = eye(3);
            obj.I_basis{7} = Gi;

            % m*CoG cross terms
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6);
                Gi(1:3,4:6) = S;
                Gi(4:6,1:3) = S;
                obj.I_basis{7+ax} = Gi;
            end

            % Gravity bases (7..10)
            G7 = zeros(6,6); G7(4:6,4:6) = eye(3);
            obj.G_basis{1} = G7;
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6); Gi(1:3,1:3) = S;
                obj.G_basis{1+ax} = Gi;
            end
        end

        function [m, Iparams, cog] = unpackTheta(obj)
            th = obj.theta_hat(:);
            Ixx = th(1); Iyy = th(2); Izz = th(3); Ixy = th(4); Ixz = th(5); Iyz = th(6);
            m = th(7);
            cog = [th(8); th(9); th(10)] / max(m, 1e-9);
            Iparams = [Ixx, Iyy, Izz, Ixy, Ixz, Iyz];
        end

        function Wg = gravityWrenchHat(obj, H, m, cog)
            R = H(1:3,1:3);
            gvec = [0;0;-obj.g];
            f_g = m * (R' * gvec);
            tau_g = cross(cog(:), f_g);
            Wg = [tau_g; f_g];
        end

        function Y = regressor(obj, H_err, H, V, Vd, Ades)
            Ad_inv_err = vt.se3.Ad_inv(H_err);
            V_e = V - Ad_inv_err * Vd;
            a_bar = Ades - vt.se3.adV(Vd) * (vt.se3.Ad(H_err) * V_e);

            R = H(1:3,1:3);
            g_body = R' * [0;0;-obj.g];
            gvec = [g_body; g_body];

            cols = cell(10,1);
            for i = 1:10
                term = -vt.se3.adV(V).' * obj.I_basis{i} * V ...
                       - obj.I_basis{i} * Ad_inv_err * a_bar;
                if i >= 7
                    term = term + obj.G_basis{i-6} * gvec;
                end
                cols{i} = term;
            end
            Y = [cols{:}];
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
