classdef EuclideanBoxedAdaptation < vt.ctrl.adapt.EuclideanAdaptation
    methods
        function obj = EuclideanBoxedAdaptation(cfg)
            obj@vt.ctrl.adapt.EuclideanAdaptation(cfg);
            warning('Euclidean-boxed adaptation not implemented; using euclidean.');
        end
    end
end
