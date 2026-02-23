function startup()
    runDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(runDir);
    addpath(repoRoot);
    rootStartup = fullfile(repoRoot, 'startup.m');
    if exist(rootStartup, 'file') == 2
        run(rootStartup);
    else
        error('Root startup.m not found at %s', rootStartup);
    end
end
