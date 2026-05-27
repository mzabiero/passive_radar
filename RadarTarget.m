classdef RadarTarget < handle
    
    properties (Access = public)
        ID (1,1)        uint8
        range_m (1,1)   double 
        vel_ms  (1,1)   double
        atten (1,1)     double
    end

    methods (Access = public)
        function obj = RadarTarget(id,range,vel,att)
            obj.ID = id; 
            obj.range_m = range; 
            obj.vel_ms = vel; 
            obj.atten = att;
        end
    end
end