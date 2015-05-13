classdef CSimLrf < CLrf
    %CSimLrf Class for simulated Lidar object
    % This class stores configuration parameters and pose and ease methods
    % to simulate intersection with polygons in 3D space
    %   Constructor:
    %   Lidar = CSimLrf( R, t, N, FOVd, sd, d_range )
    % This class inherits from CBaseLidar
    %
    % Own methods:
    % [xy, range, angles, idxs] = scanPolygon( CPolygon )
    %   Get 2D points, range, angles and indexes for scan on polygon
    %
    % This handle object is copyable.
    % 
    % See also CBaseLidar.
    
    properties
        % Empty
    end
    
    methods
        % Constructor
        function this = CSimLrf( pose, config )
            if nargin == 0
                args = {};
            else
                args = { pose, config };
            end
            this = this@CLrf( args{:} );
        end
        
        function [xy, idxs] = scanPolygon( this, polygon )
            % [xy, idxs] = scanPolygon( this, polygon )
            % Simulate sampling of a planar polygon
            %
            % Input:
            %   polygon - a CPolygon object to scan
            % Output:
            %   xy - a 2xNpts array with all valid scanned points
            %   idxs - a 1xNpts vector with the corresponding ray indeces

            % Scan an infinite plane with all the LRF rays
            rho = this.scanPlane( polygon.plane );
            
            % Get corresponding 2D points
            pts2DinLRF = this.rho2xy( rho );
            
            % Transform 2D points in the LRF frame to 3D world points
            pts3D = this.pose + this.make3D( pts2DinLRF );
            
            % Transform 3D world points to 2D points in the polygon frame
            pts2DinPolygon = polygon.transform3Dto2D( pts3D );
            
            % Check which points are inside the polygon
            in_mask = polygon.isInside( pts2DinPolygon );
                                    
            % Add generated noise to sampled ranges
            rho = this.addNoiseToRange( rho );
            
            % Compute xy coordinates of points from corrupted ranges
            xy = this.rho2xy( rho );
            
            % Collect only measurements inside the polygon
            idxs = find( in_mask );
            xy = xy( :, in_mask );
        end
        
        function homLine = intersectPlane( this, plane )
            % homLine = intersectPlane( this, plane )
            % Simulate intersection of the LRF plane
            % with the scanned plane
                        
            % Get parameterization matrix of the plane in P²
            % (as explained by Zisserman)
            M = this.pose.T(:,[1 2 4]);
            
            % Compute homogeneous line in P² from intersection
            % with the observed plane
            homLine = lnormalize( M' * plane );
        end
                        
        function [xy, out2] = scan(this, object, outputFormat)
            % [xy, objectId] = scan( this, object, 'asArray' )
            % [cXy, cIdxs] = scan( this, object, 'asCell' )
            % Simulate sampling of a generic object, if it is CScannable
            
            if ~isa(object,'CScannable')
                error('The input object is not CScannable. Inherit from CScannable and implement getScanBy method.');
            end
            
            if ~exist('outputFormat','var')
                outputFormat = 'asArray';
            end
            
            switch outputFormat
                case 'asArray'
                    [xy, objectId] = object.getScanBy( this, 'asArray' );
                    out2 = objectId;
                case 'asCell' % For legacy code
                    [cXy, cIdxs] = object.getScanBy( this, 'asCell' );
                    xy = cXy;
                    out2 = cIdxs;
                otherwise
                    error('Unknown outputFormat %s, use asArray or asCell', outputFormat);
            end
        end                
    end
    
    methods(Access=private)
        % Internal methods for simulation
        function rho = scanPlane( this, plane )
            % rho = scanPlane( this, plane )
            % Simulate sampling of a plane (infinite) by all scanning rays
            %
            % Input:
            %   plane - the P³ vector of the plane in World coordinates
            % Output:
            %   rho - vector of range measurements
            
            % Transform the input plane to the reference frame of the LRF
            % X₁ = ¹T₂ * X₂ → Π₁ = (¹T₂)⁻ᵀ Π₂
            % Here 2 is the SR in the LRF and 1 is the SR in the World
            % so: Π₂ = (¹T₂)ᵀ Π₁
            transformedPlane = this.pose.T' * plane;
            
            % Parameterize ray as ρ·v and impose plane equation to solve ρ
            % Note: The third component of v is zero by definition
            % of the LRF scan plane (Z=0)
            rho = -transformedPlane(4) ./ (transformedPlane(1:2)'*this.v2);
            
            % Set as invalid negative or inf measures
            % Note: the range rho is negative
            % for irreal collisions (behind ray)
            invalidPoints = (rho < this.config.minRange) | (rho > this.config.maxRange);
            rho(invalidPoints) = NaN;
        end     
        
        % Noise function
        function r = addNoiseToRange( this, r )
            r = r + this.config.sd * randn(1,length(r));
        end
    end
end