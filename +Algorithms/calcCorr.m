function [corrVec, lags] = calcCorr(sig1, sig2, maxLag)
    if nargin < 3 || isempty(maxLag)
        [corrVec, lags] = xcorr(sig1(:), sig2(:));
    else
        [corrVec, lags] = xcorr(sig1(:), sig2(:), maxLag);
    end
end