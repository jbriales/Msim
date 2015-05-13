classdef CScannable
    % CScannable An abstract class to identify objects of classes which
    % can be scanned (in simulation) by a LRF
    %
    % See also: CSimLidar
    
    methods (Abstract)
        [xy, out2] = getScanBy( this, lrf, outputFormat )
        % [xy, objectId] = getScanBy( this, lrf, 'asArray' )
        % [cXy, cIdxs] = getScanBy( this, lrf, 'asCell' )
        % A generic method for simulation of scan of complex scenes
        % Input:
        %   lrf - the scanning object which executes the scan
        %   outputFormat - an optional argument to set the output format,
        %   could be:
        %       'asArray' - the complete sampling array, filled with NaN in
        %       not valid rays
        %       'asCell' - a cell array with an independent sampling array
        %       for each scanned object in the scene.
        %       This option is for legacy from original implementation.
        %
        % Output for 'asArray' format
        %   xy - a 2xN array, with N the number of samples returned by the LRF
        %   objectId - a 1xN mask with the index or label of the object
        %   which provoked the sample
        % Output for 'asCell' format
        %   cXy - a 1xM cell array, each cell an array of sampled points in
        %   the corresponding scene object
        %   cIdxs - a 1xM cell array, each cell an array with the indexes
        %   of corresponding samples
    end
end