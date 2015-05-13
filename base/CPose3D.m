classdef CPose3D
    %CPose3D Pose of a 3D object wrt reference (typically World)
    %   Fully defined by an orientation and a translation wrt a reference frame
    %
    %   Constructor:
    %   pose = CPose3D( R, t )
    %   pose = CPose3D( T )
    %   pose = CPose3D( pose )
    %
    %   This class is overloaded with the following operations:
    %   Let p1,p2 be pose objects, T a 4x4 pose matrix, X a 3xN array of 3D points
    %   then
    %       p1 * p2 is equivalent to the composed transformation T1 * T2
    %       p1 * T  = T1 * T
    %       p1 / p2 = T1 * inv(T2)
    %       p1 \ p2 = inv(T1) * T2
    %       p1 / T  = T1 * inv(T)
    %       p1 \ T  = inv(T1) * T
    %       p1 + X  = inhom( T1 * hom(X) )

    properties
        R   % 3x3 rotation matrix defining the orientation as seen from World
        t   % 3x1 translation vector defining the position as seen from World
    end
    
    properties (SetAccess = protected, Dependent) % (Read-only)
        T   % 4x4 pose matrix of the object wrt World
    end
    
    methods
        % Constructor
        function this = CPose3D( in1, t )
            
            % Set default arguments
            if nargin == 0
                R = eye(3);
                t = zeros(3,1);
            elseif nargin == 1
                % If only one input argument is given:
                % CPose3D or transformation matrix case
                pose = in1;
                
                if isa(pose,'CPose3D')
                    % If input object is CPose3D copy it
                    this = pose;
                    return
                end
                
                if CPose3D.isPose(pose)
                    % If input object is a transformation matrix
                    R = pose(1:3,1:3);
                    t = pose(1:3,4);
                end
            else
                R = in1;
            end
            
            % Assign variables to object properties
            this.R  = R;
            this.t  = t;
        end
        
        % Get methods
        function T = get.T( this )
            T = [ this.R this.t ; zeros(1,3) 1 ];
        end
        
        % Display method
        function disp( this )
            % disp( this )
            % Display the 4x4 matrix only, which has all the information
            
            disp( this.T );
        end
               
        % Overloaded operations
        function pose = mtimes( pose1, pose2 )
            % pose = mtimes( pose1, pose2 )
            % Composition of poses, valid for objects and pose matrices
            
            if ~CPose3D.isPose( pose1 ) || ~CPose3D.isPose( pose2 )
                error('Arguments are not valid poses');
            end
            
            % If both arguments are poses
            pose1 = CPose3D( pose1 );
            pose2 = CPose3D( pose2 );
            
            T = pose1.T * pose2.T;
            pose = CPose3D(T);
        end
        
        function pose = mrdivide( pose1, pose2 )
            % pose = mrdivide( pose1, pose2 )
            % Composition of poses, valid for objects and pose matrices
            
            if ~CPose3D.isPose( pose1 ) || ~CPose3D.isPose( pose2 )
                error('Arguments are not valid poses');
            end
            
            % If both arguments are poses
            pose1 = CPose3D( pose1 );
            pose2 = CPose3D( pose2 );
            
            T = pose1.T / pose2.T;
            pose = CPose3D(T);
        end
        
        function pose = mldivide( pose1, pose2 )
            % pose = mrdivide( pose1, pose2 )
            % Composition of poses, valid for objects and pose matrices
            
            if ~CPose3D.isPose( pose1 ) || ~CPose3D.isPose( pose2 )
                error('Arguments are not valid poses');
            end
            
            % If both arguments are poses
            pose1 = CPose3D( pose1 );
            pose2 = CPose3D( pose2 );
            
            T = pose1.T \ pose2.T;
            pose = CPose3D(T);
        end
        
        function points1 = plus( pose12, points2 )
            % points1 = plus( pose12, points2 )
            % Transform points expressed in SR2 to
            % corresponding points in SR1 through pose 1→2 (^1T_2)
            
            assert( isa(pose12,'CPose3D') );
            points1 = makeinhomogeneous( pose12.T * makehomogeneous(points2) );
        end
        
        function h = plotFrame( this, label, color, frameSize )
            % h = plotFrame( this, label, color, frameSize )
            % Plot the frame corresponding to the pose in the reference 3D space
            % Input:
            %   label - name for the pose
            %   color - color(s) to use in the plotted frame
            %   frameSize - size of the lines in the plotted frame
            
            if ~exist('label','var') || isempty(label)
                label = '';
            end
            if ~exist('color','var') || isempty(color)
                color = 'rgb';
            end
            if ~exist('frameSize','var') || isempty(frameSize)
                frameSize = 0.1;
            end
            
            h = plotframe( this.T, frameSize, label, color );
        end
    end
    
    methods(Static, Access = private)
        function bool = isPose( pose )
            % bool = CPose3D.isPose( pose )
            % Check if the given input is a pose or not
            % Valid pose formats are:
            %   - CPose3D objects
            %   - 4x4 transformation matrix
            %   - 3x4 upper submatrix of a transformation matrix
            
            if isa(pose,'CPose3D')
                % Check if it is a CPose3D object
                bool = true;
                return;
            else
                % Check that it is a matrix of correct size
                s = size(pose);
                if ~((s(1)==3||s(1)==4) && s(2)==4)
                    bool = false;
                    return;
                end
                
                % Extract pose parts
                R = pose(1:3,1:3); %#ok<PROP>
                if ~CPose3D.isRotation(R) %#ok<PROP>
                    bool = false;
                    return;
                end
                
                if s(1)==4
                    lowerRow = pose(4,:);
                    if any(lowerRow ~= [0 0 0 1])
                        bool = false;
                        return;
                    end
                end
                
                bool = true;
            end
        end
        
        function bool = isRotation( R )
            % bool = isRotation( R )
            % Check if the input is a rotation matrix (orthonormal)
            
            if abs(det(R) - 1) < 1e3*eps && all(all(abs(R'*R - eye(3)) < 1e3*eps))
                bool = true;
            else
                bool = false;
            end
        end
    end
    
    methods(Static)
        function h = plotReference( label, color, frameSize )
            % h = plotReference( this, label, color, frameSize )
            % Plot the reference frame in the 3D space (World)
            % Input:
            %   label - name for the pose
            %   color - color(s) to use in the plotted frame
            %   frameSize - size of the lines in the plotted frame
            
            if ~exist('label','var')
                label = '';
            end
            if ~exist('color','var') || isempty(color)
                color = 'k';
            end
            if ~exist('frameSize','var') || isempty(frameSize)
                frameSize = 1;
            end
            
            h = plotframe( eye(4), frameSize, label, color );
        end
    end
end