classdef TrajectoryFactory
    methods (Static)
        function traj = create(cfg)
            method = 'precomputed';
            if isfield(cfg.traj, 'method') && ~isempty(cfg.traj.method)
                method = lower(cfg.traj.method);
            end

            switch method
                case 'precomputed'
                    traj = vt.traj.PreComputedTrajectory(cfg);
                case 'modelreference'
                    traj = vt.traj.ModelReferenceTrajectory(cfg);
                otherwise
                    error('Unknown trajectory method: %s', method);
            end
        end
    end
end
