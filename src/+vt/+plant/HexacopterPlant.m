classdef HexacopterPlant < handle
    properties
        H
        V
        m
        g
        CoG
        I6
    end

    methods
        function obj = HexacopterPlant(cfg)
            obj.m = cfg.vehicle.m;
            obj.g = cfg.vehicle.g;
            obj.CoG = cfg.vehicle.CoG(:);
            if isfield(cfg.vehicle,'I6') && ~isempty(cfg.vehicle.I6)
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = vt.utils.getGeneralizedInertia(obj.m, cfg.vehicle.I_params, obj.CoG);
            end
            obj.H = eye(4);
            obj.V = zeros(6,1);
        end

        function reset(obj, H0, V0)
            if nargin < 2 || isempty(H0)
                H0 = eye(4);
            end
            if nargin < 3 || isempty(V0)
                V0 = zeros(6,1);
            end
            obj.H = H0;
            obj.V = V0;
        end

        function [H, V] = getState(obj)
            H = obj.H;
            V = obj.V;
        end

        function step(obj, dt, Wprop)
            Wg = obj.gravityWrench();
            C = vt.se3.adV(obj.V)' * obj.I6 * obj.V;
            Vdot = obj.I6 \ (-C + Wg + Wprop);

            Vmid = obj.V + 0.5 * Vdot * dt;
            obj.H = obj.H * vt.se3.expSE3(vt.se3.hat6(Vmid * dt));
            obj.V = obj.V + Vdot * dt;
        end

        function updateParameters(obj, m, CoG, Iparams)
            if nargin >= 2 && ~isempty(m)
                obj.m = m;
            end
            if nargin >= 3 && ~isempty(CoG)
                obj.CoG = CoG(:);
            end
            if nargin >= 4 && ~isempty(Iparams)
                obj.I6 = vt.utils.getGeneralizedInertia(obj.m, Iparams, obj.CoG);
            end
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj)
            R = obj.H(1:3,1:3);
            gvec = [0;0;-obj.g];
            f_g = obj.m * (R' * gvec);
            tau_g = cross(obj.CoG, f_g);
            Wg = [tau_g; f_g];
        end
    end
end
