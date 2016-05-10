% Copyright 2016 The MathWorks, Inc.
classdef ArenaView < matlab.mixin.SetGet
    
    properties (Access = public)
        
        model        
        graphic_objects = struct()
        listeners = struct()
        
    end
    
    properties
        
        BeingDeleted = 'off'
        
    end
    
    properties (Dependent)
        
        Parent
        Position
        OuterPosition
        Units
        Visible
        UIContextMenu
        CurrentPoint
        
    end
    
    events (NotifyAccess = private)
        viewDeleted
        graphicItemCreated
    end
    
    methods
        
        function obj  = ArenaView(model, varargin)
            
            obj.model = model;            
            obj.graphic_objects.Targets = ...
                matlab.graphics.primitive.Rectangle.empty;            
            obj.graphic_objects.Obstacles = ...
                matlab.graphics.primitive.Rectangle.empty;
            
            obj.setupAxesAndImage(varargin{:});            
            obj.setupRobot();            
            obj.modelChanged();
            
            obj.listeners.itemAdded = event.listener(obj.model, 'itemAdded', ...
                @(~, eventData) obj.addItem(eventData.Type, eventData.Index));
            
            obj.listeners.itemDeleted = event.listener(obj.model, 'itemDeleted', ...
                @(~, eventData) obj.deleteItem(eventData.Type, eventData.Index));
            
            obj.listeners.itemMoved = event.listener(obj.model, 'itemMoved', ...
                @(~, eventData) obj.moveItem(eventData.Type, eventData.Index));
            
            obj.listeners.modelDeleted = event.listener(obj.model, 'modelDeleted', ...
                @(~, ~) obj.delete());
            
        end
        
        function delete(obj)
            
            obj.BeingDeleted = 'on';
            if isvalid(obj.graphic_objects.axes) || ...
                    strcmpi(obj.graphic_objects.axes.BeingDeleted, 'off')
                delete(obj.graphic_objects.axes);
            end
            notify(obj, 'viewDeleted');
            
        end
        
    end
    
    methods % set
        
        function set.Parent(obj, p)
            obj.graphic_objects.axes.Parent = p;
        end
        
        function set.Position(obj, p)
            obj.graphic_objects.axes.Position = p;
        end
        
        function set.OuterPosition(obj, p)
            obj.graphic_objects.axes.OuterPosition = p;
        end
        
        function set.Units(obj, p)
            obj.graphic_objects.axes.Units = p;
        end
        
        function set.Visible(obj, p)
            obj.graphic_objects.axes.Visible = p;
        end
        
        function set.UIContextMenu(obj, cm)
            
            obj.graphic_objects.image.UIContextMenu = cm;
            
        end
        
        function set.CurrentPoint(obj, cp)
            
            obj.graphic_objects.axes.CurrentPoint = cp;
            
        end

    end
    
    methods % get
        
        function val = get.Parent(obj)
            val = obj.graphic_objects.axes.Parent;
        end
        
        function val = get.Position(obj)
            val = obj.graphic_objects.axes.Position;
        end
        
        function val = get.OuterPosition(obj)
            val = obj.graphic_objects.axes.OuterPosition;
        end
        
        function val = get.Units(obj)
            val = obj.graphic_objects.axes.Units;
        end
        
        function val = get.Visible(obj)
            val = obj.graphic_objects.axes.Visible;
        end
        
        function val = get.UIContextMenu(obj)
            
            val = obj.graphic_objects.image.UIContextMenu;
            
        end
        
        function val = get.CurrentPoint(obj)
            
            val = obj.graphic_objects.axes.CurrentPoint;
            
        end
        
    end
    
    methods (Access=private) % constructor and destructor
        
        function setupAxesAndImage(obj, varargin)
            % setup axe limits
            obj.graphic_objects.axes = axes(...
                'Box','on',...
                'SelectionHighlight', 'off',...
                'YDir', 'normal',...
                'DeleteFcn', @(~,~) delete(obj),...
                'HandleVisibility', 'off', ...
                varargin{:});
            
            % setup background images
            if exist(ArenaConstants.BACKGROUND_IMAGE,'file') % check if exists
                im=imread(ArenaConstants.BACKGROUND_IMAGE); % read image
                obj.graphic_objects.image = image(...
                    'XData',...
                    linspace(0,ArenaConstants.ARENA_LIMITS(1), size(im,1)),...
                    'YData',...
                    linspace(0,ArenaConstants.ARENA_LIMITS(2), size(im,2)),...
                    'CData', im,'Parent',obj.graphic_objects.axes);
                % initialize Image property and displays it
            end
            obj.graphic_objects.axes.Layer = 'top';
            axis(obj.graphic_objects.axes,...
                [0 ArenaConstants.ARENA_LIMITS(1) 0 ...
                ArenaConstants.ARENA_LIMITS(2)]);
            % zoom on arena
            axis(obj.graphic_objects.axes,'square'); % square axis
            grid(obj.graphic_objects.axes); % grid
        end
        
        function setupRobot(obj)
            run('loadRobotParameters');
            
            %setup hgtransform and children, as well as some properties
            obj.graphic_objects.robot = hgtransform(...
                'Parent',obj.graphic_objects.axes); 
            % transform for moving everything easily
            load('RobotImage'); % load image for robot
            robot_width = 24.5; % width of the robot in cm
            robot_length = robot_width/size(CDataRobot, 1)*size(CDataRobot, 2);
            % length deduced
            surface(linspace(0,robot_length,size(CDataRobot,2))-robot_length+5,...
                linspace(0,robot_width,size(CDataRobot,1))-robot_width/2,...
                zeros(size(CDataRobot, 1), size(CDataRobot, 2)),...
                CDataRobot, 'EdgeColor', 'none', 'Parent', obj.graphic_objects.robot,...
                'AlphaData', 0.5); % display robot
            HBVertices = [242 117; 228 36; 142 36; 1 1; 1 234;...
                142 200; 228 200]/size(CDataRobot, 1)*robot_width; % hit box vertices
            HBVertices(:,1) = HBVertices(:,1) - robot_length + 5; % position on x
            HBVertices(:,2) = HBVertices(:,2) - robot_width / 2; % position on y
            patch(HBVertices(:,1), HBVertices(:,2),...
                zeros(size(HBVertices, 1), 1), 'FaceColor', 'none',...
                'Parent', obj.graphic_objects.robot, 'FaceAlpha', 0,...
                'Visible', 'off'); % hit box patch
            obj.graphic_objects.robot.Matrix =  makehgtform('translate',...
                [startPos 0]) * makehgtform('zrotate', theta0);
            uistack(obj.graphic_objects.robot, 'top');
        end
        
        function addItem(obj, type, index)
            
            if type == 't'
                col = [0 0.8 0];
                curvature = 1;
                sz = ArenaConstants.TARGET_SIZE;
                position = obj.model.targets_positions(index,:) - sz/2;
            else
                col = [0.8 0 0];
                curvature = 0;
                sz = ArenaConstants.OBSTACLE_SIZE;
                position = obj.model.obstacles_positions(index,:) - sz/2;
            end
            
            h = rectangle('Position', [position sz],...
                'Curvature', curvature,...
                'FaceColor', col,...
                'Tag', type,...
                'Parent', obj.graphic_objects.axes);
            uistack(h, 'top');
            %uistack(h, 'down');
            
            if type == 't'
                obj.graphic_objects.Targets(index) = h;
            else
                obj.graphic_objects.Obstacles(index) = h;
            end
            
            obj.notify('graphicItemCreated', ArenaModelEventData(type,index));
            
        end
        
        function moveItem(obj, type, index)
            
            if type == 't'
                position = obj.model.targets_positions(index,:) - ...
                    ArenaConstants.TARGET_SIZE/2;
                obj.graphic_objects.Targets(index).Position(1:2) = position;
            else
                position = obj.model.obstacles_positions(index,:) - ...
                    ArenaConstants.OBSTACLE_SIZE/2;
                obj.graphic_objects.Obstacles(index).Position(1:2) = position;
            end
            
        end
        
        function deleteItem(obj, type, index)
            
            if type == 't'
                delete(obj.graphic_objects.Targets(index));
                obj.graphic_objects.Targets(index) = [];
            else
                delete(obj.graphic_objects.Obstacles(index));
                obj.graphic_objects.Obstacles(index) = [];
            end
            
        end
        
        function modelChanged(obj)
            
            for k=1:length(obj.graphic_objects.Targets)
                obj.deleteItem('t', k);
            end
            
            for k=1:length(obj.graphic_objects.Obstacles)
                obj.deleteItem('o', k);
            end
            
            for k=1:obj.model.num_targets
                obj.addItem('t', k);
            end
            
            for k=1:obj.model.num_obstacles
                obj.addItem('o', k);
            end
            
        end
        
    end
    
end