% Copyright 2014 - 2016 The MathWorks, Inc.
classdef TrackViewer < handle
    % TrackViewer  - internal class - display component for the followed track
    properties
        Panel % panel for the axe
        Image % background image
        Axes % axe containing the graphic objects
        
        DisplayPositions
    end
    properties (Access = private)
        Theta = 0 % robot angle
        Track % positions taken by the robot (? x 2)
        ObstaclesVertices % vertices of the obstacles (? x 2)
        ObstaclesEdges % edges (segments) indices of the obstacle (? x 2)
        HBVertices % vertices of the hit box (? x 2) in robot reference frame
        HBVerticesAbs % vertices of the hit box (? x 2) in absolute reference frame
        HBEdges % edges indices of the hit box (? x 2)
        
        % objects and graphic objects
        Targets = matlab.graphics.primitive.Rectangle.empty;% white (not visited)...
        %/ green (visited) circles at targets positions
        TargetsLocated = matlab.graphics.primitive.Patch.empty; % crosses at located targets
        Robot % hgtransform
        VisibilityPatch % visibility polygon
        Obstacles = matlab.graphics.primitive.Surface.empty; % surfaces array of the obstacles
        RayCaster % object for ray casting
        Left % red (detected) / green (not detected) left sensor line
        Right % red (detected) / green (not detected) right sensor line
        MSULine % distance measurement sensor unit line
        MSUText % distance value printed at the end of the line
        HitBox % hit box of the robot
        
        
    end
    
    methods
        
        function obj  = TrackViewer(figH, units, pos)
            % obj  = TrackViewer(figH, units, pos)
            %
            % creates a trackviewer object
            %
            % inputs:
            % - figH: figure object
            % - units: units of the position vector
            % - pos: position vector
            %
            % output:
            % - obj: TrackViewer object
            
            % create the panel
            obj.Panel = uipanel('Parent',figH,'Units',units,'Position',pos);
            % setup most of the object
            obj.createApp;
        end
        
        function hasCrashed = update(obj, track, found, located,pcam,lcam,...
                robotheta, LeftFlag, RightFlag, ArmAngleEstimated, ...
                ArmAngleCommand, DisplayPositions)
            % hasCrashed = TV.update(track, found, located, pcam, lcam,
            % robotheta, LeftFlag, RightFlag, ArmAngle)
            %
            % updates object and return if collision has occured
            %
            % inputs:
            % - track: robot position history (? x 2)
            % - found: indices of the sites found (? x 1 logical)
            % - located: sites located (? x 2)
            % - pcam: camera vision trapez x values (1 x 2)
            % - lcam: camera vision trapez y values (1 x 2)
            % - robotheta: vision angle of the robot (1 x 1)
            % - LeftFlag: flag for detection on left sensor (1 x 1 logical)
            % - RightFlag: flag for detection on right sensor (1 x 1 logical)
            % - ArmAngle: angle of the distance measurement sensor (1 x 1)
            %
            % outputs:
            % - hasCrashed: flag indicating if the robot has collided with
            % an obstacle or the outer arena boundaries (1 x 1 logical)
            
            % Ray casting
            obj.RayCaster.ViewingPosition = [track(end,1) track(end,2)]; % updates viewing position in ray casting object
            obj.RayCaster.CastRays; % cast rays
            
            % Trajectory
            set(obj.Track, 'XData',track(:,1),'YData',track(:,2)); % updates trajectory
            
            % Targets
            if any(found)
                colors = repmat({[0 0.8 0]}, nnz(found), 1);
                [obj.Targets(found).FaceColor] = colors{:};
            end
            
            set(obj.TargetsLocated,'XData',located(:,1),'YData',located(:,2)); % set the target located
            
            % Side sensors
            set(obj.Left, 'Color', [LeftFlag ~LeftFlag 0]); % green if nothing, red if something
            set(obj.Right, 'Color', [RightFlag ~RightFlag 0]); % green if nothing, red if something
            
            % Visibility polygon
            % 1. Trapez base values
            xTrapezRobot = [pcam(1) pcam(2) pcam(2) pcam(1)] ; % trapez x values
            yTrapezRobot = [lcam(1)/2 lcam(2)/2 -lcam(2)/2 -lcam(1)/2] ; % trapez y values
            % 2. Trapez rotated
            xyRobotAbsolute = [cosd(robotheta) -sind(robotheta) ; ...
                sind(robotheta) cosd(robotheta)] * ...
                [xTrapezRobot;yTrapezRobot]; % rotation of robotheta
            % 3. Vision polygon from ray casting
            Vx = flipud(obj.RayCaster.VisionPolygonX); % x polygon values
            Vy = flipud(obj.RayCaster.VisionPolygonY); % y polygon values
            [x, y] = polybool('intersection',xyRobotAbsolute(1,:)'+track(end,1),...
                xyRobotAbsolute(2,:)'+track(end,2), Vx, Vy); % intersect the polygons
            set(obj.VisibilityPatch, 'XData', x(~isnan(x)),...
                'YData', y(~isnan(x))); % set the visbility patch
            
            % distance measurement units detection
            % 1. get barycentric coordinates for all obstacle edges and
            % outer boundaries
            [~, ~, ~, ~, ua, ub] = lineSegmentsIntersection(track(end,1),...
                track(end,2), track(end,1) + cosd(ArmAngleEstimated+robotheta),...
                track(end,2) + sind(ArmAngleEstimated+robotheta),...
                obj.ObstaclesVertices(obj.ObstaclesEdges(:,1),1),...
                obj.ObstaclesVertices(obj.ObstaclesEdges(:,1),2),...
                obj.ObstaclesVertices(obj.ObstaclesEdges(:,2),1),...
                obj.ObstaclesVertices(obj.ObstaclesEdges(:,2),2)); % barycentric coordinates
            % 2. check if barycentric coordinates mean a detection
            isOnRange = (ua >= 10) & (ua <= 80) & (ub >= 0) & (ub <= 1);
            % 3. set length, color for the ray and distance text
            if nnz(isOnRange)==0 % if nothing is detected
                armDist = 80; % distance put to max
                color = [0 1 0]; % color green
                str = ''; % string empty
            else % if something is detected
                armDist = min(ua(isOnRange)); % get minimum distance
                color = [1 0 0]; % color red
                str = num2str(round(armDist)); % string put to distance in cm rounded
            end
            if ArmAngleEstimated ~= ArmAngleCommand
                color = [0 0 0];
            end
            % 4. put the values in the objects
            set(obj.MSULine, 'XData', [10 armDist]*cosd(ArmAngleEstimated),...
                'YData', [10 armDist]*sind(ArmAngleEstimated),...
                'Color', color); % set line color and position
            set(obj.MSUText, 'String', str, 'Position', ...
                armDist*[cosd(ArmAngleEstimated), sind(ArmAngleEstimated)]); % set text
            
            % Update robot position (transform object)
            if isempty(track) || isnan(track(end,1)) % if it is the beginning
                Mx = makehgtform('translate',[0 -30 0]); % Move out of view
            else % if not
                pos = track(end,:); % the position is the last point of the trajectory
                Mx = makehgtform('translate',[pos(1), pos(2), 0]); % make a translation
                obj.Theta = robotheta / 180*pi; % get robot angle in radians
                Rx = makehgtform('zrotate',obj.Theta); % make a rotation
                Mx = Mx*Rx; % combine both transforms
            end
            set(obj.Robot,'Matrix',Mx); % put the transform in the object
            
            % Hit box computations
            % 1. Get the vertices values in absolute reference frame
            obj.HBVerticesAbs = obj.HBVertices*Mx(1:2,1:2)' + ...
                repmat(Mx(1:2,4)', size(obj.HBVertices, 1), 1); % use the tranform
            % 2. Check if it is colliding
            hasCrashed = false; % initialize hasCrashed
            for k=1:length(obj.HBEdges) % for each hit box polygon edge
                [~, isOnSegment] = lineSegmentsIntersection(...
                    obj.HBVerticesAbs(obj.HBEdges(k,1),1),...
                    obj.HBVerticesAbs(obj.HBEdges(k,1),2),...
                    obj.HBVerticesAbs(obj.HBEdges(k,2),1),...
                    obj.HBVerticesAbs(obj.HBEdges(k,2),2),...
                    obj.ObstaclesVertices(obj.ObstaclesEdges(:,1),1),...
                    obj.ObstaclesVertices(obj.ObstaclesEdges(:,1),2),...
                    obj.ObstaclesVertices(obj.ObstaclesEdges(:,2),1),...
                    obj.ObstaclesVertices(obj.ObstaclesEdges(:,2),2)); % compute intersection with obstacle edges
                hasCrashed = hasCrashed || any(isOnSegment); % if any intersection, it has collided
            end
            % 3. Set parameters accordingly
            if hasCrashed % if it is colliding
                color = [1 0 0]; % color red
                alpha = 0.2; % small alpha value
            else %if not
                color = [0 0 0]; % color black
                alpha = 0; % complete transparency
            end
            set(obj.HitBox, 'FaceColor', color, 'FaceAlpha', alpha); % put the values in the objects
            
            %4
            obj.DisplayPositions.XData = [obj.DisplayPositions.XData DisplayPositions(1)];
            obj.DisplayPositions.YData = [obj.DisplayPositions.YData DisplayPositions(2)];
            
            % Refresh graphics
            drawnow expose
            
        end
        
        function setup(obj, SPos, Opos, OVert, OEdges)
            % TV.setup(SPos, Opos, OVert, OEdges)
            %
            % setups most properties related to sites and obstacles
            %
            % inputs:
            % - SPos: Sites positions (? x 2)
            % - Opos: Obstacles positions (? x 4: left bottom widht height)
            % - OVert: Obstacles vertices (? x 2)
            % - OEdges: Obstacles edges indices (? x 2)
            
            obj.ObstaclesVertices = OVert; % assign vertices
            obj.ObstaclesEdges = OEdges; % assign edges
            
            if any(isvalid(obj.Targets))
                obj.Targets.delete;
            end
            if isvalid(obj.TargetsLocated)
                obj.TargetsLocated.delete;
            end
            if isvalid(obj.Obstacles) % if obstacles are defined
                obj.Obstacles.delete; % delete them
            end
            if isvalid(obj.DisplayPositions) % if obstacles are defined
                obj.DisplayPositions.delete; % delete them
            end
            
            Im = flipud(imread('mars_rocks.jpg')); % read and flip image for obstacles
            for k=1:size(Opos, 1) % for each obstacle
                obj.Obstacles = surface(linspace(0,Opos(k,3), size(Im,2)) + Opos(k,1),...
                    linspace(0,Opos(k,4), size(Im, 1)) + Opos(k,2),...
                    zeros(size(Im, 1), size(Im, 2)), ...
                    Im, 'Parent', obj.Axes, 'EdgeColor', 'none'); % create the surface
            end
            
            if ~isempty(SPos)
                h = rectangle('Position', [0 0 10 10], ...
                    'Curvature', 1,...
                    'FaceColor', [1 1 1],...
                    'EdgeColor', [0.8 0 0],...
                    'LineWidth', 1.5,...
                    'Parent', obj.Axes);
                obj.Targets = copyobj(repmat(h, size(SPos,1), 1), ...
                    repmat(obj.Axes, size(SPos,1), 1));
                h.delete;
                for k=1:length(obj.Targets)
                    obj.Targets(k).Position(1:2) = SPos(k,:)-[5 5];
                end
            end
            
            obj.TargetsLocated = patch(NaN, NaN, 1,...
                'Marker', '+', 'MarkerSize', 15, 'MarkerEdgeColor', [0 0 0.8],...
                'Parent', obj.Axes, 'EdgeColor', 'none', 'FaceColor', 'none', ...
                'LineWidth', 2); % Located targets markers
            
            obj.DisplayPositions = line(50,50,'Color','r','LineStyle','none',...
                'LineWidth',2,'Parent', obj.Axes, 'Marker', '.',...
                'MarkerSize', 8 );
            
            obj.RayCaster = RayCasterU(obj.ObstaclesVertices, obj.ObstaclesEdges); % ray casting object
            
            uistack(obj.Robot, 'top'); % ensure the robot is at the top of the ui stack
        end
    end
    
    methods (Access = private)
        
        function createApp(obj)
            % setup most of the object
            
            % setup axe limits
            lengthArena = 300 ; % length of the arena in cm
            obj.Axes = axes('Parent',obj.Panel,'Box','on',...
                'OuterPosition',[0 0 0.98 0.98],...
                'SelectionHighlight', 'off', 'YDir', 'normal');
            
            % setup background images
            imageFilename = 'BackGround.jpg' ; % filename of the bg image
            if exist(imageFilename,'file') % check if exists
                im=imread(imageFilename); % read image
                obj.Image = image('XData',linspace(0,lengthArena,size(im,1)),...
                    'YData', linspace(0,lengthArena,size(im,2)),...
                    'CData', im,'Parent',obj.Axes);
                % initialize Image property and displays it
            end
            obj.Axes.Layer = 'top';
            axis(obj.Axes,[0 lengthArena 0 lengthArena]); % zoom on arena
            axis(obj.Axes,'square'); % square axis
            grid(obj.Axes); % grid
            
            % setup empty graphic objects
            obj.Track = line(NaN,NaN,'Color','b','LineStyle','none',...
                'LineWidth',2,'Parent', obj.Axes, 'Marker', '.',...
                'MarkerSize', 8 ); % object for the trajectory
            obj.VisibilityPatch = patch(NaN,NaN,[0 0 1],...
                'EdgeColor', 'none', 'FaceAlpha',0.5,'Parent',obj.Axes); % Visibility polygon
            obj.DisplayPositions = line(50,50,'Color','r','LineStyle','none',...
                'LineWidth',2,'Parent', obj.Axes, 'Marker', '.',...
                'MarkerSize', 8 );
            
            %setup hgtransform and children, as well as some properties
            obj.Robot = hgtransform('Parent',obj.Axes); % transform for moving everything easily
            load('RobotImage'); % load image for robot
            robot_width = 24.5; % width of the robot in cm
            robot_length = robot_width/size(CDataRobot, 1)*size(CDataRobot, 2); % length deduced
            surface(linspace(0,robot_length,size(CDataRobot,2))-robot_length+5,...
                linspace(0,robot_width,size(CDataRobot,1))-robot_width/2,...
                zeros(size(CDataRobot, 1), size(CDataRobot, 2)),...
                CDataRobot, 'EdgeColor', 'none', 'Parent', obj.Robot); % display robot
            obj.Left = line([0 10]+3.4, [6.25 6.25], 'LineWidth', 2,...
                'Color', [0 1 0], 'Parent', obj.Robot, 'Visible', 'off'); % left sensor line
            obj.Right = line([0 10]+3.4, [-6.25 -6.25], 'LineWidth', 2,...
                'Color', [0 1 0], 'Parent', obj.Robot, 'Visible', 'off'); % right sensor line
            obj.MSULine = line([10, 80], [0, 0], 'LineWidth', 2,...
                'Color', [0 0 0], 'Parent', obj.Robot); % distance measurement captor line
            obj.MSUText = text([0, 0], [0, 0], '', 'Parent', obj.Robot); % distance measured
            obj.HBVertices = [242 117; 228 36; 142 36; 1 1; 1 234;...
                142 200; 228 200]/size(CDataRobot, 1)*robot_width; % hit box vertices
            obj.HBVertices(:,1) = obj.HBVertices(:,1) - robot_length + 5; % position on x
            obj.HBVertices(:,2) = obj.HBVertices(:,2) - robot_width / 2; % position on y
            obj.HitBox = patch(obj.HBVertices(:,1), obj.HBVertices(:,2),...
                zeros(size(obj.HBVertices, 1), 1), 'FaceColor', 'none',...
                'Parent', obj.Robot, 'FaceAlpha', 0); % hit box patch
            numEdges = size(obj.HBVertices, 1); %  number of edges
            obj.HBEdges = [(1:numEdges)' circshift((1:numEdges)', -1, 1)]; % edges indices
        end
    end
    
end
