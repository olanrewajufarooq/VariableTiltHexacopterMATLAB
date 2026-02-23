classdef GeoEnforcedAdaptation < vt.ctrl.adapt.EuclideanAdaptation
    methods
        function obj = GeoEnforcedAdaptation(cfg)
            obj@vt.ctrl.adapt.EuclideanAdaptation(cfg);
            warning('Geo-enforced adaptation not implemented; using euclidean.');
        end
    end
end
