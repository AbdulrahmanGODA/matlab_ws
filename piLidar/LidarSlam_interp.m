classdef LidarSlam_interp < matlab.System

    properties
      
    end
    
    properties(DiscreteState)
        
    end

    properties(Access = private)
        slamAlg = lidarSLAM(20);
    end
    
    methods(Access = protected)
        function setupImpl(obj) 
            mapResolution = 20;
            maxLidarRange = 8;
            LoopClosureThreshold = 210;
            LoopClosureSearchRadius = 6;
            maxNumScans = 15;
            obj.slamAlg = lidarSLAM(mapResolution, maxLidarRange,maxNumScans);
            obj.slamAlg.LoopClosureThreshold = LoopClosureThreshold;
            obj.slamAlg.LoopClosureSearchRadius = LoopClosureSearchRadius;
        end
        
        function [isScanAdded,pose, mapData] = stepImpl(obj,ranges,angles,relPoseEst)
            
            scan = lidarScan(ranges,angles);
            isScanAdded = addScan(obj.slamAlg, scan, relPoseEst);  

            [scans,poses] = scansAndPoses(obj.slamAlg);
            pose=poses(end,:);

            mapResolution=20;
            maxLidarRange = 8;
            map = buildMap(scans, poses, mapResolution, maxLidarRange);
            mapData = occupancyMatrix(map);
        end

        function resetImpl(obj)
            
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(~)
            out1 = [1 1];
            out2 = [1 3];
            out3 = [1000 1000];
        end
        
        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            out1 = "boolean";
            out2 = "double";
            out3 = "double";
        end
        
        function [out1, out2, out3] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
            out3 = false;
        end
        
        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
            out3 = false;
        end
        

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", 0.01);
        end    
        
    end
    
    
end
