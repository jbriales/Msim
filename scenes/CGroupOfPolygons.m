classdef CGroupOfPolygons < CScannable
    % CGroupOfPolygons An object with several polygons or faces defined
    % from a list of points
    % Useful for mathematical simulation of a scene constituted by planes
    %
    % Constructor:
    % scene = CGroupOfPolygons( mPoints, cListOfIdxs )
    %   mPoints - a 3xN array with 3D coordinates of points
    %   cListOfIdxs - a 1xM cell with lists of idxs for point in each polygon
    %
    % Example of use: A 1x1x1 cubic scene (default if no arguments given)
    % cubeSize = 1;
    % mPoints = cubeSize * [ 0 0 0; eye(3); ones(3,3)-eye(3); 1 1 1 ]';
    % cListOfIdxs = num2cell([1 3 4 5;1 2 4 6;1 2 3 7;2 6 7 8;3 5 7 8;4 5 6 8]',1);
    % scene = CGroupOfPolygons( cubePoints, cListOfIdxsForPolygons );
    
    properties
        pol % 1xN vector of polygon objects contained in the scene
        numPolygons
    end
    
    methods
        % Constructors
        function this = CGroupOfPolygons( mPoints, cListOfIdxs )
            
            if nargin == 0
                error('Default constructor not implemented for CGroupOfPolygons');
            end
            
            this.numPolygons = size(cListOfIdxs,2);
            this.pol = repmat( CPolygon, 1, this.numPolygons );
            for k=1:this.numPolygons
                polIdxs = cListOfIdxs{k};
                points = mPoints(:,polIdxs);
                this.pol(k) = CPolygon( points );
            end
        end
        
        % Complete inherited abstract methods
        function [xy, objectId] = getScanBy( this, lrf, outputType )
            
            % Set default values
            if ~exist('outputType','var')
                outputType = 'asArray';
            end
            
            % Assure valid input arguments
            if ~strcmpi( outputType, 'asArray' )
                error('Non existing output format for getScanBy');
            end
            
            % Sample each polygon in the scene with the given LRF
            [cXy, cIdxs] = deal(cell(1,this.numPolygons));
            for k=1:this.numPolygons
                [cXy{k},cIdxs{k}] = lrf.scanPolygon( this.pol(k) );
            end
            
            % Fuse all samples from different polygons into one single
            % array as for usual LRF
            allIdxs = cell2mat(cIdxs);
            allXy = cell2mat(cXy);
            xy = NaN(2,lrf.config.N);
            xy(:,allIdxs) = allXy;
            objectId = NaN(1,lrf.config.N);
            for k=1:this.numPolygons
                objectId(cIdxs{k}) = k;
            end
        end
        
        % Function
        function h = plot3( this )
            % h = plot3( this )
            % Plot each polygon in the group in 3D
            
            h = zeros(1,this.numPolygons);
            for k=1:this.numPolygons
                h(k) = this.pol(k).plot3( );
            end
        end
        
        function h = plotIds( this )
            % h = plotIds( this )
            % Plot the identity for each polygon in the group
            % as the corresponding index in the list of polygons
            
            h = zeros(1,this.numPolygons);
            for k=1:this.numPolygons
                xyz = this.pol(k).centroid;
                cXyz = num2cell(xyz);
                h(k) = text( cXyz{:}, num2str(k) ); % Use index as identity
            end
        end
    end
    
end