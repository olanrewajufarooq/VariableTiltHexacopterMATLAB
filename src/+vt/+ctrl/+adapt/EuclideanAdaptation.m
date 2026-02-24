classdef EuclideanAdaptation < vt.ctrl.adapt.AdaptationBase
    properties (Access = private)
        g
        theta_hat
        Gamma
        I_basis
        G_basis
        dt
        m_hat
        cog_hat
        Iparams_hat
    end

    methods
        function obj = EuclideanAdaptation(cfg)
            obj.g = cfg.vehicle.g;
            obj.dt = cfg.sim.adaptation_dt;

            I_params = cfg.vehicle.I_params(:);
            m = cfg.vehicle.m;
            CoG = cfg.vehicle.CoG(:);
            obj.theta_hat = [I_params(1:6); m; (m * CoG(:))];

            obj.Gamma = obj.buildGamma(cfg.controller.Gamma);
            obj.constructBases();
            obj.updateEstimates();
        end

        function params = update(obj, Hd, H, Vd, V, Ades, dt)
            if nargin < 7 || isempty(dt)
                dt = obj.dt;
            end
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end

            H_err = vt.se3.invSE3(Hd) * H;
            Ad_inv_err = vt.se3.Ad_inv(H_err);
            V_e = V - Ad_inv_err * Vd;

            Y = obj.regressor(H_err, H, V, Vd, Ades);
            obj.theta_hat = obj.theta_hat + obj.Gamma * (Y.' * V_e) * dt;
            obj.updateEstimates();

            params = obj.getParams();
        end

        function params = getParams(obj)
            params = struct('m', obj.m_hat, 'CoG', obj.cog_hat, ...
                'Iparams', obj.Iparams_hat, 'I6', vt.utils.getGeneralizedInertia(obj.m_hat, obj.Iparams_hat, obj.cog_hat));
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            m_hat = obj.m_hat;
            cog_hat = obj.cog_hat;
            Iparams_hat = obj.Iparams_hat;
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            if isempty(obj.theta_hat) || numel(obj.theta_hat) < 10
                return;
            end
            m = obj.theta_hat(7) + m_payload;
            mcog = obj.theta_hat(8:10) + m_payload * CoG_payload(:);
            obj.theta_hat(7) = m;
            obj.theta_hat(8:10) = mcog;
            obj.updateEstimates();
        end
    end

    methods (Access = private)
        function updateEstimates(obj)
            [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackTheta(obj.theta_hat);
        end

        function Gamma = buildGamma(obj, gamma)
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

            for k = 1:3
                E = zeros(3,3); E(k,k) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{k} = Gi;
            end

            pairs = [1 2; 2 3; 1 3];
            for idx = 1:3
                i = pairs(idx,1); j = pairs(idx,2);
                E = zeros(3,3); E(i,j) = 1; E(j,i) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{3+idx} = Gi;
            end

            Gi = zeros(6,6); Gi(4:6,4:6) = eye(3);
            obj.I_basis{7} = Gi;

            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6);
                Gi(1:3,4:6) = S;
                Gi(4:6,1:3) = S;
                obj.I_basis{7+ax} = Gi;
            end

            G7 = zeros(6,6); G7(4:6,4:6) = eye(3);
            obj.G_basis{1} = G7;
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6); Gi(1:3,1:3) = S;
                obj.G_basis{1+ax} = Gi;
            end
        end

        function [m_hat, cog_hat, Iparams_hat] = unpackTheta(obj, theta)
            th = theta(:);
            Iparams_hat = [th(1), th(2), th(3), th(4), th(5), th(6)];
            m_hat = max(th(7), 1e-9);
            cog_hat = [th(8); th(9); th(10)] / m_hat;
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
    end
end
