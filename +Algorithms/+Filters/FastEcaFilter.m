classdef FastEcaFilter < Algorithms.Filters.BaseFilter
    properties (Access = protected)
        BatchSize (1,1) double = 100000
        NegativeLags (1,1) double = 100
    end
    
    methods
        function setParams(obj, paramsStruct)
            if isfield(paramsStruct, 'filterLength')
                obj.filterLength = paramsStruct.filterLength;
            elseif isfield(paramsStruct, 'FilterLength')
                obj.filterLength = paramsStruct.FilterLength;
            end
            
            if isfield(paramsStruct, 'BatchSize')
                obj.BatchSize = paramsStruct.BatchSize;
            elseif isfield(paramsStruct, 'batchSize')
                obj.BatchSize = paramsStruct.batchSize;
            end
            
            if isfield(paramsStruct, 'BackFilterLength')
                obj.backFiltLength = paramsStruct.BackFilterLength;
            end
        end
        
        function survClean = apply(obj, ref, surv)
            ref = ref(:);
            surv = surv(:);
            N = length(ref);
            K = obj.filterLength;
            B = obj.BatchSize;
            L = obj.backFiltLength;
            
            if L > 0
                surv_shifted = [zeros(L, 1, 'like', surv); surv(1:end-L)];
            else
                surv_shifted = surv;
            end
            
            numBlocks = ceil(N / B);
            survCleanCells = cell(numBlocks, 1);
            
            N_fft = 2^nextpow2(B + K - 1);
            
            parfor m = 1:numBlocks
                idxStart = (m - 1) * B + 1;
                idxEnd = min(m * B, N);
                currB = idxEnd - idxStart + 1;
                
                ref_matched = ref(idxStart : idxEnd);
                surv_matched = surv_shifted(idxStart : idxEnd);
                
                r_xx = xcorr(ref_matched, K-1, 'none');
                r_xs = xcorr(surv_matched, ref_matched, K-1, 'none');
                
                r_xx_pos = r_xx(K:end);
                p = r_xs(K:end);
                
                R = toeplitz(r_xx_pos, conj(r_xx_pos));
                delta = 1e-6 * max(diag(abs(R)));
                R = R + eye(K, 'like', R) * delta;
                
                w = R \ p; 
                w = w(:);
                
                refStart = idxStart - K + 1;
                if refStart < 1
                    refBlock = [zeros(1 - refStart, 1); ref(1 : idxEnd)];
                else
                    refBlock = ref(refStart : idxEnd);
                end
                
                W_f = fft(w, N_fft);
                Ref_f = fft(refBlock, N_fft);
                
                Cancel_f = Ref_f .* W_f;
                cancel_t = ifft(Cancel_f);
                
                valid_cancel = cancel_t(K : K + currB - 1);
                
                survCleanCells{m} = surv_matched - valid_cancel;
            end
            
            survClean_shifted = cell2mat(survCleanCells);
            
            if L > 0
                survClean = [survClean_shifted(L+1:end); surv(end-L+1:end)];
            else
                survClean = survClean_shifted;
            end
        end
    end
end