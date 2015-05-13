classdef CPolygon < CPlane3D
    %CPolygon 2D quadrilateral polygon placed in 3D space
    %   Constructor:
    %   pol = CPolygon( R, t, p )
    %       p is 2x4 array with ORDERED points
    
    properties
        p   % 2x4 array with coordinates of points relative to Polygon system
    end
    
    properties (SetAccess = private, Dependent) % (Read-only), Get method
        p3D % 3x4 array with coordinates of points from World 
    end
    
    methods
        % Constructor
        function obj = CPolygon( R, t, p )
            % Build PLANE polygon formed by 4 points
            % The generated internal system of reference (SR) for polygon is:
            % - Origin set at the first point
            % - X direction set to the direction from 1st to 2nd point
            % - Z direction is normal to the polygon plane, with direction
            % defined by traversing the line path (explain better)
            %
            % obj = CPolygon( R, t, p )
            % Build polygon formed by 4 2D points in polygon plane
            % Input:
            %   R - rotation of polygon's SR wrt world
            %   t - translation of polygon's SR wrt world
            %   p - 2x4 array of 2D points
            %
            % obj = CPolygon( p3D )
            % Build polygon formed by 4 3D points expressed in world coordinates
            % Input:
            %   p3D - 3x4 array of 3D points (coplanary!)
            
            if nargin == 0
                R = eye(3);
                t = zeros(3,1);
                p = [];
            elseif nargin == 1
                p3D = R;
                % Compute coordinate system (see help for explanation)
                v12 = p3D(:,2)-p3D(:,1);
                v13 = p3D(:,3)-p3D(:,1);
                Rx = snormalize( v12 );
                Rz = snormalize( cross(v12,v13) );
                Ry = cross(Rz,Rx);
                R  = [Rx,Ry,Rz];
                t  = p3D(:,1);
                % Compute plane points in this new system
                p = R' * (p3D - repmat(t,1,4));
                % Check all points are coplanary (Z set to 0)
                if all(abs(p(end,:)) > 1e-10)
                    warning('Non coplanar?');
                    keyboard
                end
                p(end,:) = [];
            end
            obj = obj@CPlane3D( R, t );
            obj.p  = p;
        end
        
        % Return mask 1 for 2D points inside the polygon
        function in = isInside( obj, pts2D )
            % in = isInside( obj, pts2D )
            % Check if a given 2D point lies inside this object polygon
            % defined by obj.p array
            % Input:
            %   pts2D - 2xN array of points to check
            % Output:
            %   in - 1xN boolean mask (1 for points inside the polygon)
            
            if 0
                aug_p = [ obj.p , obj.p(:,1) ];
                v  = diff( aug_p, 1, 2 );
                vn = [0 -1; 1 0] * v;
                N  = size(pts2D,2);
                cross = zeros( 4, N );
                for i=1:4
                    cross(i,:) = vn(:,i)' * (pts2D - repmat(obj.p(:,i),1,N));
                end
                %             cross = vn' * pts2D;
                in = all( cross >= 0, 1 );
            else
                in = inpolygon(pts2D(1,:),pts2D(2,:),obj.p(1,:),obj.p(2,:));
            end
        end
        
        function c = centroid( this )
            % c = centroid( this )
            % Returns the 3D coordinates for the polygon centroid
            % (useful for plot purposes)
            c = mean( this.p3D, 2 );
        end
                
        % 3D representation
        function p3D = get.p3D(obj)
            p3D = makeinhomogeneous( obj.M * makehomogeneous( obj.p ) );
        end
        function h = plot3( obj, col )
            % h = plot3( obj, col )
            % Plot polygon in 3D space according to its pose
            
            if ~exist('col','var')
                col = 'k';
            end
            
            p3D = obj.p3D;
            p3D(:,end+1) = p3D(:,1); % Close polygon to plot
            h = plot3( p3D(1,:), p3D(2,:), p3D(3,:), '-', 'Color', col, 'Marker','.' );
        end
    end    
end

