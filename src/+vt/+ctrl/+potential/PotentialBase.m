classdef (Abstract) PotentialBase < handle
    methods (Abstract)
        Wp = computeWrench(obj, Hd, H)
    end
end
