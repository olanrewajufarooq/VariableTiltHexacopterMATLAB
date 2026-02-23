classdef GeoAwareAdaptation < vt.ctrl.adapt.EuclideanAdaptation
    methods
        function obj = GeoAwareAdaptation(cfg)
            obj@vt.ctrl.adapt.EuclideanAdaptation(cfg);
            warning('Geo-aware adaptation not implemented; using euclidean.');
        end
    end
end
