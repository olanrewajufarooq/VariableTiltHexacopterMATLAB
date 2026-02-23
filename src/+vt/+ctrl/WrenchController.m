classdef WrenchController < handle
    properties (Access = private)
        mode
        potential
        adaptation
        Kd
        g
        dt
    end

    methods
        function obj = WrenchController(cfg)
            obj.mode = lower(cfg.controller.type);
            obj.potential = vt.ctrl.potential.PotentialFactory.create(cfg);
            obj.adaptation = obj.createAdaptation(cfg);
            obj.Kd = diag(cfg.controller.Kd(:));
            obj.g = cfg.vehicle.g;
            obj.dt = cfg.sim.dt;
        end

        function W = computeWrench(obj, Hd, H, Vd, V, Ades, dt)
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end
            if nargin < 7 || isempty(dt)
                dt = obj.dt;
            end

            He = vt.se3.invSE3(Hd) * H;
            AdInvHe = vt.se3.Ad_inv(He);
            Ve = V - AdInvHe * Vd;

            params = obj.adaptation.update(Hd, H, Vd, V, Ades, dt);
            I6 = params.I6;
            Wg = obj.gravityWrench(H, params.m, params.CoG);
            Wp = obj.potential.computeWrench(Hd, H);
            Wd = -obj.Kd * Ve;

            switch obj.mode
                case 'pd'
                    W = -Wg - Wp + Wd;
                case 'feedlin'
                    C = vt.se3.adV(V)' * I6 * V;
                    W = C - Wg - Wp + Wd;
                case 'feedforward'
                    C = vt.se3.adV(V)' * I6 * V;
                    ff = I6 * AdInvHe * (Ades - vt.se3.adV(Vd) * (vt.se3.Ad(He) * Ve));
                    W = C + ff - Wg - Wp + Wd;
                otherwise
                    error('Unknown controller type: %s', obj.mode);
            end
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            [m_hat, cog_hat, Iparams_hat] = obj.adaptation.getEstimate();
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            if ismethod(obj.adaptation, 'setPayloadEstimate')
                obj.adaptation.setPayloadEstimate(m_payload, CoG_payload);
            end
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj, H, m, CoG)
            R = H(1:3,1:3);
            gvec = [0; 0; -obj.g];
            f_g = m * (R' * gvec);
            tau_g = cross(CoG(:), f_g);
            Wg = [tau_g; f_g];
        end

        function adaptation = createAdaptation(obj, cfg)
            adaptType = 'none';
            if isfield(cfg.controller, 'adaptation')
                adaptType = cfg.controller.adaptation;
            end

            if strcmpi(adaptType, 'none')
                adaptation = vt.ctrl.adapt.NoAdaptation(cfg);
                return;
            end

            switch lower(adaptType)
                case 'euclidean'
                    adaptation = vt.ctrl.adapt.EuclideanAdaptation(cfg);
                case 'euclidean-boxed'
                    adaptation = vt.ctrl.adapt.EuclideanBoxedAdaptation(cfg);
                case 'geo-enforced'
                    adaptation = vt.ctrl.adapt.GeoEnforcedAdaptation(cfg);
                case 'geo-aware'
                    adaptation = vt.ctrl.adapt.GeoAwareAdaptation(cfg);
                otherwise
                    warning('Adaptation type %s not implemented; using euclidean.', adaptType);
                    adaptation = vt.ctrl.adapt.EuclideanAdaptation(cfg);
            end
        end
    end
end
