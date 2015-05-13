classdef CSimLrfN
    %CSimLrfN Class for simulated composed Lidar object
    % This class stores the calibration through poses and provide methods
    % to simulate intersection with polygons in 3D space.
    % This class contains several usual LRF objects.
    %
    % Constructor:
    %   Lidar = CSimLrfN( pose, cLrfs )
    %
    % Input:
    %   pose - a CPose3D object
    %   cLrfs - cell array with N CSimLrf objects, each standing for
    %   a LRF in the device (the extrinsic calibration given by the pose)
    %
    % See also CLrf, CSimLrf.
    
    properties
        o % List of LRF contained in the composed device
        pose % Pose of the device wrt the world frame 
    end
    
    properties (Access=private)
        no % Number of internal LRFs
    end
    
    methods
        % Constructor
        function this = CSimLrfN( pose, cLrfs )
            
            % Set pose of reference frame wrt the world
            this.pose = pose;
            
            % Set the number of distinctive Lrfs composing the device
            this.no = numel(cLrfs);
            
            this.o = repmat(CSimLrf,1,this.no);
            % The pose of each LRF is that wrt the reference frame in the device
            % This is equivalent to the extrinsic calibration
            for k=1:this.no
                this.o(k) = cLrfs{k};
            end
        end
                
        function [cXy, cObjectId] = scan(this, object)
            % [cXy, cObjectId] = scan( this, object )
            % Simulate sampling of a generic object, if it is CScannable,
            % by each LRF inside the device
            
            % Preallocate method output
            [cXy,cObjectId] = deal(cell(1,this.no));
            % For each LRF in the device scan the input object (surfaces)
            for k=1:this.no
                currentLrf = this.o(k);
                % Transform to world reference frame for scanning surface
                originalPose = currentLrf.pose;
                currentLrf.pose = this.pose * currentLrf.pose;
                
                % Scan object
                [cXy{k},cObjectId{k}] = currentLrf.scan(object);
                
                % Set original relative pose inside device
                currentLrf.pose = originalPose;
            end
        end
                        
        % Plotting functions
        function vH = plotScan3D( this, cXy, color )
            % h = plotScan( this, xy, color )
            % Plot the array of sampled points in the current LRF pose
            % This function does not make sense if the input xy
            % does not come from the calling LRF object
            
            if ~exist('color','var')
                color = 'k';
            end
            
            vH = zeros(1,this.no);
            cColor = {'r','g','b','m','y','c','k','k','k'};
            for k=1:this.no
                xy = cXy{k};
                color = cColor{k};
                
                % Transform to world reference frame for scanning
                originalPose = this.o(k).pose;
                this.o(k).pose = this.pose * this.o(k).pose;
                
                % Plot in world
                vH(k) = this.o(k).plotScan3D( xy, color );
                
                % Set original relative pose inside device
                this.o(k).pose = originalPose;
            end
        end
    end
end