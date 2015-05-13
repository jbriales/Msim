classdef CLrf
    % CLrf Basic class for LRF object
    % This base class stores configuration parameters and pose
    % for a Laser RangeFinder.
    %
    %   Constructor:
    %   lrf = CLrf( pose, config )
    %   lrf = CLrf( R, t, N, FOVd, sd, d_range )
    %
    % This class contains a CPose3D and a CConfigLrf object
    %
    % See also CPlane3D, CConfigLrf.
    
    properties
        pose    % The LRF has a reference frame
                % which defines the scan plane (extrinsic configuration)
        config  % The LRF has a set of scan rays (intrinsic configuration)
    end
    
    properties (Dependent)
        v2
        v3 % 3D sampling directions in the world frame (according to current LRF pose)
        scanLines
    end
    
    methods
        % Constructor
        function this = CLrf( pose, config )
                        
            if nargin == 0
                % Assign default values for no input
                pose = CPose3D;
                config = CConfigLrf;
            end
            
            assert( isa(pose,'CPose3D') && isa(config,'CConfigLrf') );
            
            this.pose = pose;
            this.config = config;
        end
               
        function v2 = get.v2( this )
            % v2 = get.v2( this )
            % Get the 2D unitary direction wrt LRF's 2D reference frame for the scan rays
            v2 = [ cos( this.config.theta )
                   sin( this.config.theta ) ];
        end
        function scanLines = get.scanLines( this )
            % scanLines = get.scanLines( this )
            % Get the homogeneous representation for scan rays as lines in P²
            R90 = [0 -1; 1 0];
            scanLines = [ R90 * this.v2
                          zeros(1,this.config.N) ];
        end
        
        function v3 = get.v3( this )
            v3 = this.pose.R * this.make3D( this.v2 );
        end
        
        function xy = rho2xy( this, rho )
            % xy = this.rho2xy( rho )
            % Compute 2D points in the LRF plane
            % corresponding to a vector of measured ranges
            
            assert(numel(rho) == this.config.N);
            
            xy = repmat(rho,2,1) .* this.v2;
        end
        
        % Plotting methods
        function h = plotScan3D( this, xy, color )
            % h = plotScan3D( this, xy, color )
            % Plot the array of sampled points in the current LRF pose
            % This function does not make sense if the input xy
            % does not come from the calling LRF object
            
            if ~exist('color','var')
                color = 'k';
            end
            
            if ~isempty(xy)
                % Transform 2D points in the LRF frame to 3D world points
                pts3D = this.pose + this.make3D( xy );
                h = plot3( pts3D(1,:), pts3D(2,:), pts3D(3,:), ['.',color] );
            else
                h = [];
            end
        end
        
        function h = plot3_ScanRays( this, color )
            % h = plot3_ScanRays( this, color )
            % Plot the scan rays in 3D according to the current LRF pose
            
            if ~exist('color','var')
                color = 'k';
            end
            vectors2D = this.v2;
            vectors3D = this.pose.R(:,1:2) * vectors2D;
            
            tt = repmat( this.t, 1, this.N );
            h = quiver3( tt(1,:), tt(2,:), tt(3,:),...
                         vectors3D(1,:), vectors3D(2,:), vectors3D(3,:),...
                         0.25 * this.frameSize,...
                         'Color', color );
        end
    end
    
    methods(Static)        
        function Xz = make3D( X )
            % X = make3D(x)
            % Complete 3rd dimension of scanned points with a zero
            % (for convention of LRF plane defined by Z=0)
            
            % Add a complete row of zeros at the bottom of the matrix X
            [rows, npts] = size(X);
            Xz = zeros(rows+1, npts);
            Xz(1:rows,:) = X;
        end
    end
end