classdef ModelReferenceTrajectory < vt.traj.TrajectoryBase
    properties (Access = private)
        pathGenerator
        lambda
        dt
        Hd
        Vd
        initialized_
    end

    methods
        function obj = ModelReferenceTrajectory(cfg)
            obj.pathGenerator = vt.traj.PreComputedTrajectory(cfg);
            obj.dt = cfg.sim.dt;
            obj.initialized_ = false;

            if isfield(cfg.traj, 'lambda') && ~isempty(cfg.traj.lambda)
                obj.lambda = cfg.traj.lambda(:);
            else
                obj.lambda = ones(6,1);
            end
        end

        function [Hd, Vd, Ad] = generate(obj, t, H, V, params)
            if ~obj.initialized_
                obj.initialize(H, V);
            end
            if nargin < 5
                params = struct();
            end

            [H_path, ~, ~] = obj.pathGenerator.generate(t, [], [], []);

            He = vt.se3.invSE3(H_path) * obj.Hd;
            xi = vt.se3.logSE3(He);

            lambda = obj.getLambda(params);
            Vd_dot = -2 * lambda .* obj.Vd - lambda.^2 .* xi;

            obj.Vd = obj.Vd + Vd_dot * obj.dt;
            obj.Hd = obj.Hd * vt.se3.expSE3(vt.se3.hat6(obj.Vd * obj.dt));

            Hd = obj.Hd;
            Vd = obj.Vd;
            Ad = Vd_dot;
        end

        function reset(obj, H0, V0)
            obj.Hd = H0;
            obj.Vd = V0(:);
            obj.initialized_ = true;
        end
    end

    methods (Access = private)
        function initialize(obj, H, V)
            obj.Hd = H;
            obj.Vd = V(:);
            obj.initialized_ = true;
        end

        function lambda = getLambda(obj, params)
            lambda = obj.lambda;
            if nargin < 2 || isempty(params)
                return;
            end
        end
    end
end
