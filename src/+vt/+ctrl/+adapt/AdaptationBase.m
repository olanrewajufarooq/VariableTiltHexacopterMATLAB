classdef (Abstract) AdaptationBase < handle
    methods (Abstract)
        params = update(obj, Hd, H, Vd, V, Ad, dt)
        params = getParams(obj)
        [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
    end
end
