classdef CConfigLrf
    %CConfigLrf Class for intrinsic configuration of a LRF
    % This class accepts the following configuration parameters:
    %   N - number of points in scan
    %   FOVdeg - Field Of View in degrees
    %   vRange - minimum and maximum distances to read
    %   sd - Standard Deviation (in [m]) of measurements
    %
    %   Constructor:
    %   config = CConfigLrf( N, FOVdeg, vRange, sd  )
    %   config = CConfigLrf( config )
    %   config = CConfigLrf( structOfParameters )
    %
    % Other parameters computed from the input ones are:
    %   FOVrad   - Field Of View in radians
    %   minRange - minimum readable distance
    %   maxRange - maximum readable distance
    %   angularResolutionDeg - scanning angular resolution in degrees
    %   angularResolutionRad - scanning angular resolution in radians
    
    properties
        N       % Number of points sampled in a scan
        FOVdeg  % Field Of View in degrees
        vRange  % 1x2 vector with minimum and maximum valid ranges
        sd      % Standard Deviation in range measurements
    end
    
    properties (Dependent)
        theta    % Vector of angles for scan rays in the LRF frame
        FOVrad
        angularResolutionDeg
        angularResolutionRad
        minRange % minimum readable distance
        maxRange % maximum readable distance
    end
    
    methods
        % Constructor
        function this = CConfigLrf( in1, FOVdeg, sd, vRange )
            % CConfigLrf( N, FOVdeg, sd, vRange )
            % CConfigLrf( structOfParameters )
            % CConfigLrf( config )
                        
            if nargin == 0
                % If no given arguments, use default values for Hokuyo UTM
                N = 1081;
                FOVdeg = 270.2;
                sd = 0.03;
                vRange = [0.1 30];
            elseif nargin == 1
                % If one only input argument is given
                if isa(in1,'CConfigLrf')
                    % Copy it if CConfigLrf
                    this = in1;
                    return;
                elseif isstruct(in1)
                    % Or extract fields as arguments (non-safe)
                    S = in1;
                    extractStructFields( S );
                else
                    error('Bad arguments: 1 argument only if CConfigLrf or struct');
                end
            else
                % Else the complete list of input arguments must have been given
                N = in1;
            end
            
            % Assert conditions on input arguments
            assert(N > 0);
            assert(FOVdeg>0 && FOVdeg <=360);
            assert(sd>=0);
            assert(vRange(1)>=0 && vRange(2)>0);
            
            % Set object properties
            this.N = N;
            this.FOVdeg = FOVdeg;
            this.sd = sd;
            this.vRange = vRange;
        end
                
        % Get methods
        function theta = get.theta( this )
            % theta = get.theta( this )
            % Get the angles wrt LRF's 2D reference frame for the scan rays
            theta = linspace( ...
                -deg2rad(this.FOVdeg)/2,...
                +deg2rad(this.FOVdeg)/2,...
                this.N );
        end
        function FOVrad = get.FOVrad(this)
            FOVrad  = deg2rad( this.FOVdeg );
        end
        function angularResolutionDeg = get.angularResolutionDeg(this)
            angularResolutionDeg = this.FOVdeg / (this.N-1) ;
            % Note: The number of steps is one less than points!
        end
        function angularResolutionRad = get.angularResolutionRad(this)
            angularResolutionRad = this.FOVrad / (this.N-1) ;
            % Note: The number of steps is one less than points!
        end
        function minRange = get.minRange(this)
            minRange = this.vRange(1);
        end
        function maxRange = get.maxRange(this)
            maxRange = this.vRange(2);
        end
    end
end