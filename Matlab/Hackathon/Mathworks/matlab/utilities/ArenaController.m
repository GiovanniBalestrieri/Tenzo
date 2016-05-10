% Copyright (c) 2016, The MathWorks, Inc.
classdef ArenaController < handle
    
    properties
        model
        view
        fig
        
        graphic_objects = struct()
        
        listeners = struct()
        
        selection = gobjects(0)
        
        has_current_file = false
        current_file = ''
        saved_to_file = false
        
        saved_to_workspace = false
    end
    
    methods % constructor and destructor
        
        function obj  = ArenaController(model, view)
            
            obj.model = model;
            obj.view = view;
            obj.fig = view.Parent;
            
            obj.listeners.viewDeleted = event.listener(obj.view, ...
                'viewDeleted', @(~,~) delete(obj));
            obj.listeners.modelChanged = event.listener(obj.model, ...
                'modelChanged', @(~,~) obj.modelChanged());
            obj.listeners.graphicItemCreated = event.listener(obj.view, ...
                'graphicItemCreated', ...
                @(~,data) obj.addItemCallback(data.Type, data.Index));
            
            obj.setupMenus();
            obj.setupToolbar();
            obj.setupContextMenu();
            obj.refreshContextMenus();
            
            obj.fig.WindowButtonMotionFcn = @(~,~) obj.defaultMotion();
            obj.fig.WindowButtonDownFcn = @(~,~) obj.defaultClick();
            obj.fig.WindowButtonUpFcn = @(~,~) obj.defaultUnClick();
            obj.fig.WindowKeyPressFcn = @(~,data) obj.defaultKeyPress(data);
        end
        
        function delete(obj)
            
            structfun(@(x) delete(x), obj.listeners);
            structfun(@(x) delete(x), obj.graphic_objects );
            if isvalid(obj.view) && strcmpi(obj.view.BeingDeleted, 'off')
                delete( [obj.view.graphic_objects.Targets.UIContextMenu] );
                delete( [obj.view.graphic_objects.Obstacles.UIContextMenu] );
            end
            obj.fig.WindowButtonMotionFcn = '';
            obj.fig.WindowButtonDownFcn = '';
            obj.fig.WindowButtonUpFcn = '';
            obj.fig.WindowKeyPressFcn = '';
            
        end
        
    end
    
    methods (Access = private)
        
        function setupMenus(obj)
            obj.setupFileMenu;
            obj.setupEditMenu;
            obj.setupImportMenu;
            obj.setupExportMenu;
        end
        
        function setupFileMenu(obj)
            obj.graphic_objects.fileMenu = uimenu('Parent', obj.fig, ...
                'Label', 'File');
            uimenu('Parent', obj.graphic_objects.fileMenu,...
                'Label', 'New', ...
                'Accelerator', 'n',...
                'Callback', @(~,~) obj.mNewArena());
            uimenu('Parent', obj.graphic_objects.fileMenu,...
                'Label', 'Open',...
                'Accelerator', 'o',...
                'Separator', 'on', ...
                'Callback', @(~,~) obj.mOpen());
            uimenu('Parent', obj.graphic_objects.fileMenu,...
                'Label', 'Save', ...
                'Accelerator', 's',...
                'Callback', @(~,~) obj.mSave);
            uimenu('Parent', obj.graphic_objects.fileMenu,...
                'Label', 'Save As...', ...
                'Callback', @(~,~) obj.mSaveAs);
            uimenu('Parent', obj.graphic_objects.fileMenu,...
                'Label', 'Quit',...
                'Accelerator', 'q', ...
                'Separator', 'on',...
                'Callback', @(~, ~) close(m.fig));
        end
        
        function setupEditMenu(obj)
            obj.graphic_objects.editMenu = uimenu('Parent', obj.fig, ...
                'Label', 'Edit');
            uimenu('Parent', obj.graphic_objects.editMenu,...
                'Label', 'Delete Target/Obstacle  Delete',...
                'Enable', 'off', ...
                'Callback', @(~,~) obj.deleteSelection());
            uimenu('Parent', obj.graphic_objects.editMenu,...
                'Label', 'Add Target',...
                'Separator', 'on',...
                'Callback', @(~,~) obj.model.addItem('t',...
                [randi(ArenaConstants.ARENA_LIMITS(1)-1) ...
                randi(ArenaConstants.ARENA_LIMITS(2)-1)]));
            uimenu('Parent', obj.graphic_objects.editMenu,...
                'Label', 'Add Obstacle',...
                'Callback', @(~,~) obj.model.addItem('o',...
                [randi(ArenaConstants.ARENA_LIMITS(1)-1) ...
                randi(ArenaConstants.ARENA_LIMITS(2)-1)]));
        end
        
        function setupImportMenu(obj)
            obj.graphic_objects.importMenu = uimenu('Parent', obj.fig, ...
                'Label', 'Import');
            uimenu('Parent', obj.graphic_objects.importMenu,...
                'Label', 'From Workspace', ...
                'Callback', @(~,~) obj.mImportFromWorkspace());
            for k=1:2
                if k==1
                    sep = 'on';
                else
                    sep = 'off';
                end
                uimenu('Parent', obj.graphic_objects.importMenu,...
                    'Label', ['Scenario ' num2str(k)], ...
                    'Callback', @(~,~) obj.importScenario(k), ...
                    'Separator', sep);
            end
        end
        
        function setupExportMenu(obj)
            obj.graphic_objects.exportMenu = uimenu('Parent', obj.fig, ...
                'Label', 'Export');
            uimenu('Parent', obj.graphic_objects.exportMenu,...
                'Label', 'To Workspace',...
                'Callback', @(~,~) obj.mSaveToWorkspace());
        end
        
        function setupToolbar(obj)
            obj.graphic_objects.tbar = uitoolbar(obj.fig);
            im = im2double(imread('file_new.png',...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'New Scenario',...
                'ClickedCallback', @(~, ~) obj.mNewArena());
            im = im2double(imread('file_open.png', ...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'Open Scenario',...
                'Separator', 'on',...
                'ClickedCallback', @(~, ~) obj.mOpen);
            im = im2double(imread('file_save.png', ...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'Save Scenario',...
                'ClickedCallback', @(~, ~) obj.mSave);
            im = im2double(imread('Import_16.png',...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'Import Current Scenario',...
                'Separator', 'on',...
                'ClickedCallback', @(~, ~) obj.mImportFromWorkspace());
            im = im2double(imread('Export_16.png',...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'Save to Workspace and Export as Current Scenario',...
                'Separator', 'off',...
                'ClickedCallback', @(~, ~) obj.mSaveToWorkspace());
            im = im2double(imread('SimulinkModel_16.png',...
                'BackGroundColor', [0.94 0.94 0.94]));
            uipushtool('Parent', obj.graphic_objects.tbar,...
                'CData', im, ...
                'ToolTipString', 'Open Model',...
                'Separator', 'on',...
                'ClickedCallback', @(~, ~) obj.mOpenModel());
        end
        
        function setupContextMenu(obj)
            obj.graphic_objects.cm = uicontextmenu(obj.fig);
            uimenu('Parent', obj.graphic_objects.cm,...
                'Label', 'Add Target',...
                'Callback', @(~,~) obj.addItemHere('t'));
            uimenu('Parent', obj.graphic_objects.cm,...
                'Label', 'Add Obstacle',...
                'Callback', @(~,~) obj.addItemHere('o'));
            obj.view.UIContextMenu = obj.graphic_objects.cm;
        end
        
        function addItemCallback(obj, type, Index)
            
            cmenu = uicontextmenu(obj.fig);
            if type == 't'
                label = 'Delete Target   Delete';
                vect = obj.view.graphic_objects.Targets;
            else
                label = 'Delete Obstacle   Delete';
                vect = obj.view.graphic_objects.Obstacles;
            end            
            uimenu(cmenu, ...
                'Label', label, ...
                'Callback', @(~,~) obj.deleteSelection());
            vect(Index).UIContextMenu = cmenu;
            
        end
        
        function refreshContextMenus(obj)
            
            for k=1:obj.model.num_targets
                obj.addItemCallback('t', k);
            end
            
            for k=1:obj.model.num_obstacles
                obj.addItemCallback('o', k);
            end
            
        end
        
    end
    
    methods (Access = private)
        
        function mNewArena(obj)
            
            answer = inputdlg({'Number of Targets:',...
                'Number of Obstacles:'}, ...
                'New', 1, {'1', '0'});
            
            if ~isempty(answer)
                
                numTargets = str2double(answer{1});
                numObstacles = str2double(answer{2});
                
                obj.model.newArena(numTargets, numObstacles);
                
            end
            
        end
        
        function mSave(obj)
            
            if ~obj.has_current_file
                obj.mSaveAs();
            else
                obj.saveFile(obj.current_file);
                obj.saved_to_file  = true;
                obj.updateFigureName();
            end
            
        end
        
        function mSaveAs(obj)
            
            file_dir = fileparts(mfilename('fullpath'));
            file_dir = fullfile(file_dir,...
                ArenaConstants.DATA_RELATIVE_PATH, 'Custom');
            [filename, pathname] = uiputfile(file_dir, ...
                'Save Scenario Configuration');
            if ~isnumeric(filename) && ~isnumeric(pathname)
                obj.saveFile(fullfile(pathname, filename));
                obj.has_current_file = true;
                obj.current_file = filename;
                obj.saved_to_file = true;
                obj.updateFigureName();
            end
            
        end
        
        function mOpen(obj)
            
            file_dir = fileparts(mfilename('fullpath'));
            file_dir = fullfile(file_dir,...
                ArenaConstants.DATA_RELATIVE_PATH, 'Custom');
            [filename, pathname] = uigetfile('*.m', ...
                'Open Scenario Configuration', file_dir);
            if ~isnumeric(filename) && ~isnumeric(pathname)
                obj.openFile(fullfile(pathname,filename));
                obj.has_current_file = true;
                obj.current_file = filename;
                obj.saved_to_file = true;
                obj.updateFigureName();
            end
            
        end
        
        function mImportFromWorkspace(obj)
            
            file_dir = fileparts(mfilename('fullpath'));
            filename = fullfile(file_dir,...
                ArenaConstants.DATA_RELATIVE_PATH, ...
                ArenaConstants.CURRENT_ARENA_SETUP);
            obj.openFile(filename);
            obj.saved_to_workspace = true;
            obj.saved_to_file = false;
            obj.has_current_file = false;
            obj.current_file = '';
            obj.updateFigureName();
            
        end
        
        function mSaveToWorkspace(obj)
            
            filename = fileparts(mfilename('fullpath'));
            filename = fullfile(filename, ...
                ArenaConstants.DATA_RELATIVE_PATH, ...
                ArenaConstants.CURRENT_ARENA_SETUP);
            obj.saveFile(filename);
            
            S = obj.model.getData();
            
            % Export in base workspace
            assignin('base','ObstaclesPositions', S.ObstaclesPositions);
            assignin('base','ObstaclesVertices', S.ObstaclesVertices);
            assignin('base','ObstaclesEdges', S.ObstaclesEdges);
            assignin('base','SitesPositions', S.TargetsPositions);
            
            obj.saved_to_workspace = true;
            obj.updateFigureName();
            
        end
        
        function mOpenModel(obj)
            
            obj.fig.Pointer = 'watch';
            drawnow;
            open_system('SimulationModel')
            obj.fig.Pointer = 'arrow';
            
        end
        
        function saveFile(obj, filename)
            
            S = obj.model.getData();
            
            % Open new file for writing
            [pathstr, name, ~] = fileparts(filename);
            fid = fopen([fullfile(pathstr, name) '.m'],'wt');
            
            % Set file header (to write that file is autogenerated and print the
            % date)
            fprintf(fid,'%% File Autogenerate by function %s\n',mfilename);
            fprintf(fid,'%% Date %s\n',datestr(now,'dddd dd mmmm yyyy HH:MM:SS'));
            
            % Write vector SitesPositions with new values
            fprintf(fid,'SitesPositions = single([...\n');
            fprintf(fid,'\t%4.0f,%4.0f;...\n',S.TargetsPositions');
            fprintf(fid,'\t]);\n\n');
            
            % Write vector ObstaclesPositions with new values
            fprintf(fid,'ObstaclesPositions = single([...\n');
            fprintf(fid,'\t%4.0f,%4.0f,%4.0f,%4.0f;...\n',S.ObstaclesPositions');
            fprintf(fid,'\t]);\n\n');
            
            % Write vector ObstaclesVertices with new values
            fprintf(fid,'ObstaclesVertices = single([...\n');
            fprintf(fid,'\t%4.0f,%4.0f;...\n',S.ObstaclesVertices');
            fprintf(fid,'\t]);\n\n');
            
            % Write vector ObstaclesEdges with new values
            fprintf(fid,'ObstaclesEdges = single([...\n');
            fprintf(fid,'\t%4.0f,%4.0f;...\n',S.ObstaclesEdges');
            fprintf(fid,'\t]);\n\n');
            
            % Close file
            fclose(fid);
            
        end
        
        function openFile(obj, filename)
            
            clear(filename);
            run(filename);            
            S.ObstaclesPositions = ObstaclesPositions;
            S.ObstaclesVertices = ObstaclesVertices;
            S.ObstaclesEdges = ObstaclesEdges;
            S.TargetsPositions = SitesPositions;
            obj.model.setData(S);
            
        end
        
        function addItemHere(obj, type)
            
            obj.model.addItem(type, obj.view.CurrentPoint(1,1:2));
            
        end
        
        function importScenario(obj, num)
            
            file_dir = fileparts(mfilename('fullpath'));
            filename = fullfile(file_dir,...
                ArenaConstants.DATA_RELATIVE_PATH, ...
                'Scenarios',...
                ['Scenario' num2str(num) '.m']);
            obj.openFile(filename);
            obj.saved_to_workspace = true;
            obj.saved_to_file = false;
            obj.has_current_file = false;
            obj.current_file = '';
            obj.updateFigureName();
            
        end
        
        function defaultMotion(obj)
            
            cp = obj.view.CurrentPoint(1,1:2);
            Tpos = obj.model.targets_positions;
            overT = ~isempty(Tpos) && any( all( bsxfun( ...
                @ge, cp, bsxfun(@plus,Tpos,-ArenaConstants.TARGET_SIZE/2)),2 ) & ...
                all( bsxfun(...
                @le, cp, bsxfun(@plus,Tpos,+ArenaConstants.TARGET_SIZE/2)),2 ) );
            Opos = obj.model.obstacles_positions;
            overO = ~isempty(Opos) && any( all( bsxfun( ...
                @ge, cp, bsxfun(@plus,Opos,-ArenaConstants.OBSTACLE_SIZE/2)),2 ) & ...
                all( bsxfun(...
                @le, cp, bsxfun(@plus,Opos,+ArenaConstants.OBSTACLE_SIZE/2)),2) );
            if  overO || overT
                obj.fig.Pointer = 'fleur';
            else
                obj.fig.Pointer = 'arrow';
            end
            
        end
        
        function defaultClick(obj)
            
            if isvalid(obj.selection)
                obj.selection.Selected = 'off';
            end
            
            sel = obj.fig.CurrentObject;
            
            if isa(sel, 'matlab.graphics.primitive.Rectangle')
                
                obj.setSelection(sel);
                
                sel.Selected = 'on';
                
                pos = sel.Position(1:2) + sel.Position(3:4)/2;
                
                if sel.Tag == 't'
                    index = find(all(bsxfun(...
                        @eq, pos, obj.model.targets_positions),2));
                else
                    index = find(all(bsxfun(...
                        @eq, pos, obj.model.obstacles_positions),2));
                end
                
                startCP = obj.view.CurrentPoint(1,1:2);
                
                obj.fig.WindowButtonMotionFcn = @(~,~) obj.clickedMotion(...
                    sel.Tag, index, pos - startCP);
                
            else
                
                obj.emptySelection();
                
            end
            
        end
        
        function clickedMotion(obj, type, index, delta)
            
            cp = obj.view.CurrentPoint(1,1:2);
            obj.model.moveItem(type, index, cp + delta);
            
        end
        
        function defaultUnClick(obj)
            
            obj.fig.WindowButtonMotionFcn = @(~,~) obj.defaultMotion();
            
        end
        
        function deleteSelection(obj)
            
            if ~isempty(obj.selection)
                
                sel = obj.selection;
                
                pos = sel.Position(1:2) + sel.Position(3:4)/2;
                
                if sel.Tag == 't'
                    index = find(all(bsxfun(...
                        @eq, pos, obj.model.targets_positions),2));
                else
                    index = find(all(bsxfun(...
                        @eq, pos, obj.model.obstacles_positions),2));
                end
                
                obj.model.deleteItem(sel.Tag, index);
                
                obj.emptySelection();
                
            end
            
        end
        
        function emptySelection(obj)
            
            obj.selection = gobjects(0);
            obj.graphic_objects.editMenu.Children(3).Enable = 'off';
            
        end
        
        function setSelection(obj, sel)
            
            obj.selection = sel;
            obj.graphic_objects.editMenu.Children(3).Enable = 'on';
            uistack(obj.selection, 'top');
            %uistack(obj.selection, 'down');
            
        end
        
        function defaultKeyPress(obj, data)
            
            if isempty(data.Modifier) && strcmpi(data.Key, 'delete')                
                obj.deleteSelection();                
            end
            
        end
        
        function modelChanged(obj)
            
            obj.saved_to_file = false;
            obj.saved_to_workspace = false;
            obj.updateFigureName();
            
        end
        
        function updateFigureName(obj)
            
            if obj.saved_to_workspace
                mod1 = '';
            else
                mod1 = '*';
            end
            if obj.has_current_file
                if obj.saved_to_file
                    mod2 = '';
                else
                    mod2 = '*';
                end
                cf = [' | ' obj.current_file mod2];
            else
                cf = '';
            end
            
            obj.fig.Name = ['Arena Setup' mod1 cf];
            
        end
        
    end
    
end