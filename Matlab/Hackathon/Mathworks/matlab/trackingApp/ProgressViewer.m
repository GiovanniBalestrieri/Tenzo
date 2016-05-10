% Copyright 2014 - 2016 The MathWorks, Inc.
classdef ProgressViewer < handle
    % ProgressViewer  - internal class - display component for progress
    % information
    properties
        Panel
    end
    properties (Access = private)
        TargetsFound
        TimeElapsed
        TimeLastFound
        CurrentPosition
        CurrentVelocity
        Crashed = false
    end
    
    methods
        function obj = ProgressViewer(figH, units, pos)
            obj.Panel = uipanel('Parent',figH,'Units',units,'Position',pos);
            createApp(obj);
            update(obj,0,0,0,0,[0,0],0, false)
        end
        
        function reset(obj)
            obj.Crashed = false;
        end
        
        function update(obj,numTargets,numFound,timeLastFound,timeElapsed,robotPosition,velocity, hasCrashed)
            obj.Crashed = obj.Crashed || hasCrashed;
            if obj.Crashed
                set(obj.TargetsFound,'String', 'Crashed !');
            else
                set(obj.TargetsFound,'String', sprintf('Sites Found: %d / %d',numFound,numTargets));
            end
            if numTargets == numFound && numTargets > 0
                set(obj.TargetsFound,'ForegroundColor',[0 0.8 0]);
            else
                set(obj.TargetsFound,'ForegroundColor',[0.8 0 0]);
            end
            set(obj.TimeElapsed, 'String',...
                sprintf('Time: %5.1f sec',timeElapsed));
            set(obj.CurrentPosition, 'String',...
                sprintf('Position:  x = %1.0f, y = %1.0f',robotPosition(1),robotPosition(2)));
            set(obj.CurrentVelocity, 'String',...
                sprintf('Speed:  %1.0f cm/s',velocity));
            set(obj.TimeLastFound, 'String',...
                sprintf('Time Last Site Found: %5.1f sec',timeLastFound));
        end
    end
    
    methods (Access = private)
        function createApp(obj)
            obj.TargetsFound = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',18,...
                'ForegroundColor',[0.8 0 0],...
                'Units','norm', 'Position', [0.05 0.4 0.4 0.5],...
                'HorizontalAlignment','left');
            obj.TimeElapsed = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',18,...
                'Units','norm', 'Position', [0.5 0.4 0.45 0.5],...
                'HorizontalAlignment','left');
            obj.CurrentPosition = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',14,...
                'Units','norm', 'Position', [0.05 0.3 0.4 0.3],...
                'HorizontalAlignment','left');
            obj.CurrentVelocity = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',14,...
                'Units','norm', 'Position', [0.05 0.05 0.3 0.3],...
                'HorizontalAlignment','left');
            obj.TimeLastFound = uicontrol('Parent', obj.Panel, 'Style', 'text', ...
                'FontWeight','bold','FontSize',14,...
                'Units','norm', 'Position', [0.5 0.05 0.5 0.3],...
                'HorizontalAlignment','left');
        end
    end
    
end