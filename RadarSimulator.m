classdef RadarSimulator < handle
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = public)
        SigConfig   struct
        TxConfig    struct
        RxConfig    struct

        Targets (1,:) RadarTarget
    end

    properties (Access = private)
        TimeVector  double
        RefSignal   double
        SurvSignal  double
    end
    
    methods (Access = public)
        function obj = RadarSimulator()
            obj.Targets = RadarTarget.empty();
        end

        function addTarget(obj, targetObj)
            obj.Targets(end+1) = targetObj;
        end

        function clearTargets(obj)
            obj.Targets = RadarTarget.empty();
        end

    end
end