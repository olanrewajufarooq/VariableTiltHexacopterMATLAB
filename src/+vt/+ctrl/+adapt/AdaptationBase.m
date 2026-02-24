classdef (Abstract) AdaptationBase < handle
    %ADAPTATIONBASE Interface for parameter adaptation laws.
    %   Implementations estimate mass, CoG, and inertia for the controller.
    %
    %   Output struct fields:
    %     m, CoG, Iparams, I6.
    methods (Abstract)
        %UPDATE Advance adaptation using tracking data.
        %   Inputs:
        %     Hd, H - desired and actual pose.
        %     Vd, V - desired and actual body velocity.
        %     Ad - desired body acceleration.
        %     dt - timestep [s].
        %   Output:
        %     params - struct with updated parameters.
        params = update(obj, Hd, H, Vd, V, Ad, dt)
        %GETPARAMS Return parameters for the controller/plant.
        %   Output:
        %     params - struct with m, CoG, Iparams, and I6.
        params = getParams(obj)
        %GETESTIMATE Return estimated mass/CoG/inertia parameters.
        %   Outputs:
        %     m_hat - mass estimate [kg].
        %     cog_hat - 3x1 CoG estimate [m].
        %     Iparams_hat - inertia parameter estimate.
        [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
    end
end
