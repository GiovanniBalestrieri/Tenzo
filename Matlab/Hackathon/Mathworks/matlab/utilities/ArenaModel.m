% Copyright (c) 2016, The MathWorks, Inc.
classdef ArenaModel < handle
    
    properties (SetAccess = private, GetAccess=public)
        targets_positions = zeros(0,2)
        num_targets = 0
        
        obstacles_positions = zeros(0,2)
        num_obstacles = 0        
    end
    
    events (NotifyAccess = private)
        itemAdded
        itemMoved
        itemDeleted
        modelChanged
        modelDeleted
    end
    
    methods % constructor and destructor
        
        function obj  = ArenaModel()
            
        end
        
        function delete(obj)
            obj.notify('modelDeleted');
        end
        
    end
    
    methods (Access = public)
        
        function addItem(obj, type_char, pos)
            
            pos = ArenaConstants.ConstrainedPosition(type_char, pos);
            
            if type_char == 't' %target
                obj.targets_positions(end+1,:) = pos;
                obj.num_targets = size(obj.targets_positions, 1);
                index = obj.num_targets;
            else %obstacle
                obj.obstacles_positions(end+1,:) = pos;
                obj.num_obstacles = size(obj.obstacles_positions, 1);
                index = obj.num_obstacles;
            end
            
            obj.notify('itemAdded', ArenaModelEventData(type_char, index));
            obj.notify('modelChanged');
            
        end
        
        function moveItem(obj, type_char, index, pos)
            
            pos = ArenaConstants.ConstrainedPosition(type_char, pos);
            
            if type_char == 't' %target
                obj.targets_positions(index, :) = pos;
            else %obstacle
                obj.obstacles_positions(index, :) = pos;
            end
            
            obj.notify('itemMoved', ArenaModelEventData(type_char,index));
            obj.notify('modelChanged');
            
        end
        
        function deleteItem(obj, type_char, index)
            
            if type_char == 't' %target
                obj.targets_positions(index, :) = [];
                obj.num_targets = size(obj.targets_positions, 1);
            else %obstacle
                obj.obstacles_positions(index, :) = [];
                obj.num_obstacles = size(obj.obstacles_positions, 1);
            end
            
            obj.notify('itemDeleted', ArenaModelEventData(type_char, index));
            obj.notify('modelChanged');
            
        end
        
        function S = getData(obj)
            
            S.TargetsPositions = obj.targets_positions;
            S.ObstaclesPositions = ...
                [bsxfun(@plus, obj.obstacles_positions,...
                -ArenaConstants.OBSTACLE_SIZE/2), ...
                repmat(ArenaConstants.OBSTACLE_SIZE, obj.num_obstacles, 1)];
            S.ObstaclesVertices = zeros((obj.num_obstacles+1)*4, 2);
            S.ObstaclesEdges = zeros((obj.num_obstacles+1)*4, 2);
            baseVert = bsxfun(@plus, - ArenaConstants.OBSTACLE_SIZE/2, ...
                [0 0; ...
                ArenaConstants.OBSTACLE_SIZE(1) 0;...
                ArenaConstants.OBSTACLE_SIZE;...
                0 ArenaConstants.OBSTACLE_SIZE(2)]);
            baseEdges = [1 2; 2 3; 3 4; 4 1];
            S.ObstaclesVertices(1:4,:) = [0 0;...
                ArenaConstants.ARENA_LIMITS(1) 0;...
                ArenaConstants.ARENA_LIMITS;...
                0 ArenaConstants.ARENA_LIMITS(2)];
            S.ObstaclesEdges(1:4,:) = baseEdges;
            
            for k=1:obj.num_obstacles
                S.ObstaclesVertices((1:4)+k*4, :) = bsxfun(@plus, ...
                    baseVert, obj.obstacles_positions(k,:));
                S.ObstaclesEdges((1:4)+k*4, :) = baseEdges + 4*k;
            end
            
        end
        
        function setData(obj, S)
            
            for k=obj.num_targets:-1:1
                obj.deleteItem('t', k);
            end
            
            for k=obj.num_obstacles:-1:1
                obj.deleteItem('o', k);
            end
            
            for k=1:size(S.TargetsPositions, 1)
                obj.addItem('t', S.TargetsPositions(k,:));
            end
            
            for k=1:size(S.ObstaclesPositions, 1)
                obj.addItem('o', S.ObstaclesPositions(k,1:2) + ...
                    S.ObstaclesPositions(k,3:4)/2);
            end
            
        end
        
        function newArena(obj, numTargets, numObstacles)
            
            for k=obj.num_obstacles:-1:1
                obj.deleteItem('o', k);
            end
            
            for k=obj.num_targets:-1:1
                obj.deleteItem('t', k);
            end
            
            for k=1:numObstacles
                obj.addItem('o', ArenaConstants.OBSTACLE_SIZE*k/2);
            end
            
            for k=1:numTargets
                obj.addItem('t', ArenaConstants.TARGET_SIZE*k/2);
            end
            
        end
        
    end
    
end