% Plik: +Algorithms/EcaFilter.m
classdef EcaFilter < handle
    properties
        FilterLength (1,1) double = 500 % K - Ile opóźnień (rzędu tysięcy)
        BatchSize (1,1) double = 5e13   % B - Rozmiar paczki 
    end
    
    methods
        function obj = EcaFilter(filterLen, batchSize)
            if nargin > 0, obj.FilterLength = filterLen; end
            if nargin > 1, obj.BatchSize = batchSize; end
        end
        
        function survClean = apply(obj, ref, surv)
            ref = ref(:); 
            surv = surv(:);
            N = length(ref);
            K = obj.FilterLength;
            B = obj.BatchSize;
            
            if B <= K
                error('BatchSize musi być większy niż FilterLength!');
            end
            
            survClean = zeros(N, 1, 'like', surv);
            numBlocks = ceil(N / B);
            
            for m = 1:numBlocks
                idxStart = (m-1)*B + 1;
                idxEnd = min(m*B, N);
                currentBlockSize = idxEnd - idxStart + 1;
                
                refStart = idxStart - K + 1;
                if refStart < 1
                    padLen = 1 - refStart;
                    refChunk = [zeros(padLen, 1); ref(1 : idxEnd)];
                else
                    refChunk = ref(refStart : idxEnd);
                end
                
                X_m = zeros(currentBlockSize, K, 'like', ref);
                for k = 1:K
                    X_m(:, k) = refChunk(K - k + 1 : end - k + 1);
                end
                
                s_m = surv(idxStart : idxEnd);
                
                w_m = X_m \ s_m; 
                fprintf("%d / %d blocks in filter done \n", m, numBlocks);
                survClean(idxStart : idxEnd) = s_m - X_m * w_m;
            end
        end
    end
end