classdef LatticeFilter < Algorithms.Filters.BaseFilter
    properties
        ForgettingFactor
        BlockLen
    end

    methods
        function survClean = apply(obj, ref, surv)
            disp("Clutter removal running...");

            N = length(ref);
            M = obj.BlockLen;
            nBlocks = floor(N/M);
            backFiltLen = obj.backFiltLength;
            forgettingFact = obj.ForgettingFactor;
            filtOrder = obj.filterLength;
            
            if backFiltLen > 0
                ref_shifted = [ref(backFiltLen+1:end); zeros(backFiltLen,1)];
            else
                backFiltLen = abs(backFiltLen);
                ref_shifted = [zeros(backFiltLen,1); ref(1:end-backFiltLen)];
                %ref_shifted = ref;
            end

            survClean = complex(zeros(N,1));

            lattice = dsp.AdaptiveLatticeFilter(...
                "Method","Least-squares Lattice",...
                "ForgettingFactor",forgettingFact,...
                "Length",filtOrder);

            idx = 1;
            hWait = waitbar(0, 'Rozpoczynanie filtracji...', 'Name', 'Lattice Filter');
            for k = 1:nBlocks
                fprintf("%d / %d\n",k,nBlocks);
                r_block = ref_shifted(idx : idx+M-1);
                s_block = surv(idx : idx+M-1);

                [~, e_block] = lattice(r_block, s_block);

                survClean(idx:idx+M-1) = e_block;

                idx = idx + M;

                
                waitbar(k/nBlocks, hWait, sprintf('Filtracja Lattice: %d / %d', k, nBlocks));

            end
            if ishandle(hWait), close(hWait); end
        end

        function setParams(obj, paramsStruct)
            if isfield(paramsStruct, 'FilterLength')
                obj.filterLength = paramsStruct.FilterLength;
            end
            if isfield(paramsStruct, "ForgettingFactor")
                obj.ForgettingFactor = paramsStruct.ForgettingFactor;
            end
            if isfield(paramsStruct, "BlockLength")
                obj.BlockLen = paramsStruct.BlockLength;
            end
            if isfield(paramsStruct, "BackLength")
                obj.backFiltLength = paramsStruct.BackLength;
            end
        end
    end
end
