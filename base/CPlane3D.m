classdef CPlane3D < handle
    %CPlane3D Plane object in a 3D space
    %   Plane object is fully defined through its orientation and 
    %   translation wrt reference frame
    %
    %   Constructor:
    %   plane = CPlane3D( R, t )
    %   plane = CPlane3D( T )
    %   plane = CPlane3D( pose )
   
    properties
        pose
    end
    properties (SetAccess = protected, Dependent) % (Read-only)
        n       % 3x1 plane normal vector
        plane   % 4x1 plane vector (normal and distance)
    end
    properties (Dependent)
        M       % 4x3 matrix for conversion from 2D to 3D frame
                % See Zisserman's 3.2.1 Planes → Parametrized points on a plane
    end
    
    methods
        % Constructor
        function this = CPlane3D( varargin )
            
            this.pose = CPose3D( varargin{:} );
        end
        
        function pts2D = transform3Dto2D( this, pts3D )
            % pts2D = this.transform3Dto2D( pts3D )
            % Convert points in 3D to plane 2D frame (checking if they are inside)
            
            numPoints = size( pts3D, 2 );
            rel = pts3D - repmat( this.pose.t, 1, numPoints );
            
            % Check that all points belong to plane
            distanceToPlane = this.n' * rel;
            max_prod = max( abs( distanceToPlane ) );
            if max_prod > 1e-6 % micron distance
                error('[CPolygon::transform2D] Points outside plane. max(n''·(p3D-t)=%e',max_prod);
            end
            
            % Compute 2D coordinates in the plane's inner frame
            pts2D = this.pose.R(:,1:2)' * rel;
        end
        
        function pts3D = transform2Dto3D( this, pts2D )
            % pts3D = this.transform2Dto3D( pts2D )
            % Convert points from plane's 2D frame to 3D world frame
            pts3D = makeinhomogeneous( this.M * makehomogeneous( pts2D ) );
        end
        
        % Get methods
        function n = get.n( this )
            % n = get.n( this )
            % Plane's unitary normal vector
            n = this.pose.R(:,3);
        end
        function plane = get.plane( this )
            % plane = get.plane( this )
            % Plane's homogeneous P³ vector
            plane = [ this.n ; -this.n' * this.pose.t ];
        end
        function M = get.M( this )
            % M = get.M( this )
            % A 4x3 parameterization matrix of a P³ plane in P² space
            M = this.pose.T(:,[1 2 4]); % Remove Z column
        end
        
    end
end