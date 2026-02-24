classdef GeoEnforcedAdaptation < vt.ctrl.adapt.EuclideanAdaptation
    %GEOENFORCEDADAPTATION Placeholder for constrained adaptation.
    %   Currently falls back to Euclidean adaptation.
    %
    %   This class is kept for future extension.
    methods
        function obj = GeoEnforcedAdaptation(cfg)
            %GEOENFORCEDADAPTATION Initialize (falls back to Euclidean).
            %   Input:
            %     cfg - configuration struct.
            %   Output:
            %     obj - GeoEnforcedAdaptation instance.
            obj@vt.ctrl.adapt.EuclideanAdaptation(cfg);
            warning('Geo-enforced adaptation not implemented; using euclidean.');
        end
    end
end
