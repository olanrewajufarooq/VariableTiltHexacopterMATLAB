classdef HexacopterPlant < handle
    properties
        H
        V
        m
        g
        CoG
        I6
        groundEnable
        groundHeight
        groundStiffness
        groundDamping
        groundFriction
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

            obj.groundEnable = false;
            obj.groundHeight = 0;
            obj.groundStiffness = 0;
            obj.groundDamping = 0;
            obj.groundFriction = 0;
            if isprop(cfg, 'sim') && ~isempty(cfg.sim)
                sim = cfg.sim;
                if isfield(sim, 'groundEnable')
                    obj.groundEnable = logical(sim.groundEnable);
                end
                if isfield(sim, 'groundHeight')
                    obj.groundHeight = sim.groundHeight;
                end
                if isfield(sim, 'groundStiffness')
                    obj.groundStiffness = sim.groundStiffness;
                end
                if isfield(sim, 'groundDamping')
                    obj.groundDamping = sim.groundDamping;
                end
                if isfield(sim, 'groundFriction')
                    obj.groundFriction = sim.groundFriction;
                end
            end
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
            Wground = obj.groundWrench();
            Vdot = obj.I6 \ (-C + Wg + Wprop + Wground);

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

        function Wground = groundWrench(obj)
            Wground = zeros(6,1);
            if ~obj.groundEnable
                return;
            end
            if obj.groundStiffness <= 0
                return;
            end

            p = obj.H(1:3,4);
            depth = obj.groundHeight - p(3);
            if depth <= 0
                return;
            end

            R = obj.H(1:3,1:3);
            v_world = R * obj.V(4:6);
            vz = v_world(3);

            fn = obj.groundStiffness * depth;
            if obj.groundDamping > 0
                fn = fn - obj.groundDamping * min(vz, 0);
            end
            if fn <= 0
                return;
            end

            f_world = [0; 0; fn];
            mu = obj.groundFriction;
            if mu > 0
                v_xy = v_world(1:2);
                speed = norm(v_xy);
                if speed > 1e-6
                    f_tan = mu * fn;
                    f_world(1:2) = -f_tan * (v_xy / speed);
                end
            end

            f_body = R' * f_world;
            Wground = [0; 0; 0; f_body];
        end
    end
end
