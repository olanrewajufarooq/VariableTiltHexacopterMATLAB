function limits = defaultAxisLimits(cfg)
%DEFAULTAXISLIMITS Compute initial axis limits for visualization
    mode = 'auto';
    if isfield(cfg, 'viz') && isfield(cfg.viz, 'initialAxis')
        mode = cfg.viz.initialAxis;
    end

    if strcmpi(mode, 'fixed10')
        half = 5;
        limits = [-half half -half half -half half];
        return;
    end

    scale = cfg.traj.scale;
    alt = cfg.traj.altitude;
    pad = 2.0;
    if isfield(cfg, 'viz') && isfield(cfg.viz, 'axisPadding')
        pad = cfg.viz.axisPadding;
    end

    switch lower(cfg.traj.name)
        case 'circle'
            xMin = -2*scale; xMax = 0;
            yMin = -scale;   yMax = scale;
        case 'infinity'
            xMin = -scale; xMax = scale;
            yMin = -scale/2; yMax = scale/2;
        case 'infinity3d'
            xMin = -scale; xMax = scale;
            yMin = -scale/2; yMax = scale/2;
        case 'takeoffland'
            xMin = 0; xMax = scale;
            yMin = 0; yMax = 0;
        otherwise
            xMin = 0; xMax = 0;
            yMin = 0; yMax = 0;
    end

    maxXY = max(abs([xMin xMax yMin yMax]));
    maxXY = maxXY + pad;

    zMin = 0; zMax = alt;
    if any(strcmpi(cfg.traj.name, {'infinity3d'}))
        zMin = min(zMin, alt - scale);
        zMax = max(zMax, alt + scale);
    end
    zMax = zMax + pad;
    zMax = max(zMax, 0);

    limits = [-maxXY maxXY -maxXY maxXY zMin zMax];
end
