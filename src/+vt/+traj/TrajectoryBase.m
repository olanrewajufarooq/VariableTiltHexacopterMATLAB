classdef (Abstract) TrajectoryBase < handle
    methods (Abstract)
        [Hd, Vd, Ad] = generate(obj, t, H, V, params)
        reset(obj, H0, V0)
    end

    methods
        function [H0, V0, A0] = getInitialState(obj)
            [H0, V0, A0] = obj.generate(0, eye(4), zeros(6,1), struct());
        end
    end
end
