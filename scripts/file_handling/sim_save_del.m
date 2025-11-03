function sim_save_del(infile,x_ref,x_surv, saveDir)
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end
    
    [~,name] = fileparts(infile);
    outRef = fullfile(saveDir, [name '_ref.mat']);
    outSurv = fullfile(saveDir, [name '_surv.mat']);

    % save_comlex_binary(x_ref,outRef);
    % save_comlex_binary(x_surv,outSurv);

    save(outRef,'x_ref');
    save(outSurv, 'x_surv');
    
    % fid = fopen(outRef, 'wb');
    % fwrite(fid,x_ref);
    % fclose(fid);
    % 
    % fid = fopen(outSurv, 'wb');
    % fwrite(fid, x_surv);
    % fclose(fid);

    if isfile(infile)
        delete(infile);
    end

    fprintf('Saved:\n  %s\n  %s\n', outRef, outSurv);
end