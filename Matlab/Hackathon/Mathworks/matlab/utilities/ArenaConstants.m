% Copyright (c) 2016, The MathWorks, Inc.
classdef ArenaConstants < handle
    
    properties (Constant)
        
        TARGET_SIZE = [10 10]
        OBSTACLE_SIZE = [25 25]
        ARENA_LIMITS = [300 300]
        DATA_RELATIVE_PATH = fullfile('..', '..', 'data')
        BACKGROUND_IMAGE = 'BackGround.jpg'
        CURRENT_ARENA_SETUP = 'Sites'
        
    end
    
    methods (Static)
        
        function pos = ConstrainedPosition(type, pos)
            if type == 't' %target
                sz = ArenaConstants.TARGET_SIZE;
            else %obstacle
                sz = ArenaConstants.OBSTACLE_SIZE;
            end
            
            bottomLeftLimits = [1 1] + sz/2;
            topRightLimits = ArenaConstants.ARENA_LIMITS - [1 1] - sz/2;
            
            blCond = pos < bottomLeftLimits;
            pos(blCond) = bottomLeftLimits(blCond);
            
            urCond = pos > topRightLimits;
            pos(urCond) = topRightLimits(urCond);
            pos = round(pos);
        end
       
    end
    
end