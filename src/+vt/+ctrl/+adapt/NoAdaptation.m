classdef NoAdaptation < vt.ctrl.adapt.AdaptationBase
    properties (Access = private)
        m
        CoG
        Iparams
        I6
    end

    methods
        function obj = NoAdaptation(cfg)
            obj.m = cfg.vehicle.m;
            obj.CoG = cfg.vehicle.CoG(:);
            obj.Iparams = cfg.vehicle.I_params(:);
            if isfield(cfg.vehicle, 'I6')
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = vt.utils.getGeneralizedInertia(obj.m, obj.Iparams, obj.CoG);
            end
        end

        function params = update(obj, ~, ~, ~, ~, ~, ~)
            params = obj.getParams();
        end

        function params = getParams(obj)
            params = struct('m', obj.m, 'CoG', obj.CoG, 'Iparams', obj.Iparams, 'I6', obj.I6);
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            m_hat = [];
            cog_hat = [];
            Iparams_hat = [];
        end
    end
end
