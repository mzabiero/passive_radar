function [bladeLength, rpm] = calcDroneParams(v0, v1, vLast, N, lambda, cosPhi)
    if nargin < 5
        lambda = 0.5;
    end
    
    if nargin < 6
        cosPhi = 1.0;
    end
    
    deltaV = abs(v1 - v0);
    vMax = abs(vLast - v0);
    
    deltaF = deltaV / lambda;
    fMax = vMax / lambda;
    
    fRot = deltaF / N;
    rpm = fRot * 60;
    
    bladeLength = (fMax * lambda) / (4 * pi * fRot * cosPhi);
end