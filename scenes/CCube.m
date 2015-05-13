classdef CCube < CGroupOfPolygons
    %CCube A cube object formed by 6 squares of a given size
    
    properties
        size
    end
    
    methods
        function this = CCube( cubeSize )
            
            % Define 8 points (with origin in (0,0,0)
            % and farthest point in (1,1,1)*cubeSize)
            cubePoints = cubeSize * [ 0 0 0; eye(3); ones(3,3)-eye(3); 1 1 1 ]';
            
            % Define the points contained in each square
            cListOfIdxsForPolygons = num2cell(...
                [1 3 5 4;1 4 6 2;1 2 7 3;2 7 8 6;3 5 8 7;4 6 8 5]',1);

            % Build the cube as a group of polygons
            this = this@CGroupOfPolygons( cubePoints, cListOfIdxsForPolygons );
            
            % Store the property
            this.size = cubeSize;
        end
    end
end

