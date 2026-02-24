classdef AdaptationFactory
    methods (Static)
        function adaptation = create(cfg)
            adaptType = 'none';
            if isfield(cfg.controller, 'adaptation')
                adaptType = cfg.controller.adaptation;
            end

            if strcmpi(adaptType, 'none')
                adaptation = vt.ctrl.adapt.NoAdaptation(cfg);
                return;
            end

            switch lower(adaptType)
                case 'euclidean'
                    adaptation = vt.ctrl.adapt.EuclideanAdaptation(cfg);
                case 'geo-enforced'
                    adaptation = vt.ctrl.adapt.GeoEnforcedAdaptation(cfg);
                case 'geo-aware'
                    adaptation = vt.ctrl.adapt.GeoAwareAdaptation(cfg);
                otherwise
                    warning('Adaptation type %s not implemented; using euclidean.', adaptType);
                    adaptation = vt.ctrl.adapt.EuclideanAdaptation(cfg);
            end
        end
    end
end
