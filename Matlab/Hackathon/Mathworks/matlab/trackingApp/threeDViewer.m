% Copyright 2014 - 2016 The MathWorks, Inc.
classdef threeDViewer < handle
    % ProgressViewer  - internal class - display component for progress
    % information
    properties
        Panel
    end
    properties (Access = private)
        CurrentPosition
    end
    
    methods 
        function obj = threeDViewer(figH, units, pos)
            obj.Panel = uipanel('Parent',figH,'Units',units,'Position',pos);
            createApp(obj);
            update(obj,[0,0])
        end
        
        function reset(obj)

        end
        
        function update(obj,robotPosition)
            set(obj.CurrentPosition, 'String',...
                sprintf('Position:  x = %1.0f, y = %1.0f',robotPosition(1),robotPosition(2)));
        end
    end
    
    methods (Access = private)
        function createApp(obj)
            obj.CurrentPosition = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',14,...
                'Units','norm', 'Position', [0.05 0.3 0.4 0.3],...
                'HorizontalAlignment','left');
            
            % open vrworld if not open already
            worldfile = 'Robot_Scene_1';
            vr_world = vrworld(worldfile);
            if ~isopen(vr_world)
              open(vr_world);
            end
            ud.vr_world = vr_world;

            % create two canvases in the figure
            c1 = vr.canvas(vr_world, 'Parent', obj.Panel, ...
                      'Units', 'normalized', ...
                      'Position', [0.03 0.03 0.95 0.45]);
            c2 = vr.canvas(vr_world, 'Parent', obj.Panel, ...
                           'Units', 'normalized', ...
                           'Position', [0.03 0.52 0.95 0.45]);
            set(c1, 'Viewpoint', 'Robot Camera');
            set(c2, 'Viewpoint', 'Follow Robot');

            % Associate the figure with the block, and set the figure's UserData.
            %Set_3GFigure(block, obj.Panel);
            %set(obj.Panel, 'UserData', ud, 'HandleVisibility', 'callback');

        end
    end
    
end