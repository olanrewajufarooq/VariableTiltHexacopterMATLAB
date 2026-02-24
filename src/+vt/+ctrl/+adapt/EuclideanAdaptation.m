classdef EuclideanAdaptation < vt.ctrl.adapt.AdaptationBase
    %EUCLIDEANADAPTATION Gradient adaptation in Euclidean parameter space.
    %   Estimates mass, CoG, and inertia parameters using a linear regressor.
    %
    %   Parameters are stored as a 10x1 vector and unpacked into physical
    %   quantities each update.
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
            %EUCLIDEANADAPTATION Initialize estimates and gain matrix.
            %   Input:
            %     cfg - configuration with vehicle and controller fields.
            %   Output:
            %     obj - EuclideanAdaptation instance.
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
            %UPDATE Step parameter estimates using the regressor.
            %   Inputs:
            %     Hd, H - desired and actual pose.
            %     Vd, V - desired and actual body velocity.
            %     Ades - desired body acceleration (optional).
            %     dt - timestep [s] (optional).
            %   Output:
            %     params - struct with updated parameters.
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
            %GETPARAMS Return current estimated parameters.
            %   Output:
            %     params - struct with m, CoG, Iparams, I6.
            params = struct('m', obj.m_hat, 'CoG', obj.cog_hat, ...
                'Iparams', obj.Iparams_hat, 'I6', vt.utils.getGeneralizedInertia(obj.m_hat, obj.Iparams_hat, obj.cog_hat));
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE Return mass, CoG, and inertia estimates.
            %   Outputs:
            %     m_hat - mass estimate.
            %     cog_hat - 3x1 CoG estimate.
            %     Iparams_hat - inertia parameter estimate.
            m_hat = obj.m_hat;
            cog_hat = obj.cog_hat;
            Iparams_hat = obj.Iparams_hat;
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            %SETPAYLOADESTIMATE Shift estimates based on payload guess.
            %   Inputs:
            %     m_payload - payload mass [kg].
            %     CoG_payload - 3x1 payload CoG offset [m].
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
            %UPDATEESTIMATES Unpack parameter vector into estimates.
            [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackTheta(obj.theta_hat);
        end

        function Gamma = buildGamma(obj, gamma)
            %BUILDGAMMA Build gain matrix from scalar/vector/matrix.
            %   Input:
            %     gamma - scalar, 10x1 vector, or 10x10 matrix.
            %   Output:
            %     Gamma - 10x10 gain matrix.
            if isscalar(gamma)
                Gamma = gamma * eye(10);
            elseif isvector(gamma) && numel(gamma) == 10
                Gamma = diag(gamma);
            else
                Gamma = gamma;
            end
        end

        function constructBases(obj)
            %CONSTRUCTBASES Build inertia and gravity basis matrices.
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
            %UNPACKTHETA Convert parameter vector into physical values.
            %   Input:
            %     theta - 10x1 parameter vector.
            %   Outputs:
            %     m_hat - mass estimate.
            %     cog_hat - 3x1 CoG estimate.
            %     Iparams_hat - inertia parameter estimate.
            th = theta(:);
            Iparams_hat = [th(1), th(2), th(3), th(4), th(5), th(6)];
            m_hat = max(th(7), 1e-9);
            cog_hat = [th(8); th(9); th(10)] / m_hat;
        end

        function Y = regressor(obj, H_err, H, V, Vd, Ades)
            %REGRESSOR Build parameter regressor from tracking data.
            %   Inputs:
            %     H_err - pose error.
            %     H - current pose.
            %     V, Vd - actual/desired body velocity.
            %     Ades - desired acceleration.
            %   Output:
            %     Y - 6x10 regressor matrix.
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
