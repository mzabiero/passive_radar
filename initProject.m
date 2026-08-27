function initProject()
    restoredefaultpath;
    
    projectRoot = fileparts(mfilename('fullpath'));
    projectPaths = genpath(projectRoot);
    
    addpath(projectPaths);
    
    fprintf('Środowisko zresetowane. Załadowano ścieżki projektu z:\n%s\n', projectRoot);
end