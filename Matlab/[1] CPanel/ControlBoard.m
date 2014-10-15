function ControlBoard()

global xbee;
global tenzo;
    %# create tabbed GUI
    hFig = figure('Menubar','none');
    s = warning('off', 'MATLAB:uitabgroup:OldVersion');
    hTabGroup = uitabgroup('Parent',hFig);
    warning(s);
    hTabs(1) = uitab('Parent',hTabGroup, 'Title','Home');
    hTabs(2) = uitab('Parent',hTabGroup, 'Title','Motors');
    hTabs(3) = uitab('Parent',hTabGroup, 'Title','Sensors');
    hTabs(4) = uitab('Parent',hTabGroup, 'Title','Control');
    set(hTabGroup, 'SelectedTab',hTabs(1));
    
%     if get(hTabGroup,'SelectedTab') == hTabs(1)
%         disp('Tab1 selected');
%     end
%     if get(hTabGroup,'SelectedTab') == hTabs(3)
%         disp('Tab3 selected');
%     end
    
    %# Home UI components
    uicontrol('Style','text', 'String','Tenzo Control Panel', ...
        'Position', [55 310 420 50],...
        'Parent',hTabs(1), 'FontSize',20,'FontWeight','bold');
    
    uicontrol('Style','text', 'String','v. 1.00', ...
        'Position', [55 268 420 50],...
        'Parent',hTabs(1), 'FontSize',10,'FontWeight','normal');
    
    takeOffBtn = uicontrol('Style','pushbutton', 'String','Take Off', ...
        'Position', [350 210 120 30],...
        'Parent',hTabs(1), 'Callback',@takeOffCallback);
    
    hoverBtn = uicontrol('Style','pushbutton', 'String','Hover', ...
        'Position', [70 210 120 30],...
        'Parent',hTabs(1), 'Callback',@hoverCallback);
    
    landBtn = uicontrol('Style','pushbutton', 'String','Land', ...
        'Position', [210 210 120 30],...
        'Parent',hTabs(1), 'Callback',@landCallback);
    
    conTxt = uicontrol('Style','text', 'String','Offline','ForegroundColor',[.99 .183 0.09], ...
        'Position', [70 20 100 30],...
        'Parent',hTabs(1), 'FontSize',13,'FontWeight','bold');
    
    connect = uicontrol('Style','togglebutton', 'String','Connect', ...
        'Position', [400 20 120 30],'BackgroundColor',[.21 .96 .07],...
        'Parent',hTabs(1), 'Callback',@connection);        
    
    %# Motors Ui Components
    
    uicontrol('Style','text', 'String','Motor Status', ...
        'Position', [0 308 170 50],...
        'Parent', hTabs(2), 'FontSize',15,'FontWeight','bold');
    
    upBtn = uicontrol('Style','pushbutton', 'String','UP', ...
        'Position', [430 70 70 30],...
        'Parent',hTabs(2), 'Callback',@upCallback);
    
    downBtn = uicontrol('Style','pushbutton', 'String','DW', ...
        'Position', [430 20 70 30],...
        'Parent',hTabs(2), 'Callback',@downCallback);
    
%     uicontrol('Style','text', 'String','v. 1.00', ...
%         'Position', [55 168 420 50],...
%         'Parent',hTabs(2), 'FontSize',10,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 255 310 50 50 ]);
    
    m1Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [270 327 20 20],...
        'Parent',hTabs(2), 'FontSize',15,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 55 170 50 50 ]);
    
    m2Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [70 187 20 20],...
        'Parent',hTabs(2), 'FontSize',15,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 255 30 50 50 ]);
    
    m3Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [270 47 20 20],...
        'Parent',hTabs(2), 'FontSize',15,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 440 175 50 50 ]);
    
    m4Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [455 187 20 20],...
        'Parent',hTabs(2), 'FontSize',15,'FontWeight','normal');
    
    % Sensors UI Components
    hAx = axes('Parent',hTabs(3));
    
    uicontrol('Style','popupmenu','Position', [370 360 150 30],... 
        'String','Select|Gyro|Accelerometer|Angles (est)', ...
        'Parent',hTabs(3), 'Callback',@popupCallback);
    hLine = plot(NaN, NaN, 'Parent',hAx, 'Color','r','LineWidth',2);

    %# Control UI Components
    
    %% button callbacks
    
    
    %# drop-down menu callback
    function popupCallback(src,~)
        %# update plot color
        val = get(src,'Value');
        
        % Gyro
        if val == 2
            try
                while (get(xbee, 'BytesAvailable')~=0 && tenzo == true)
                    % read until terminator
                    sentence = fscanf( xbee, '%s'); % this reads in as a string (until a terminater is reached)
                    if (strcmp(sentence(1,1),'R'))
                        %decodes "sentence" seperated (delimted) by commaseck Unit')
                        [Roll, theta, Pitch, pitch, Yaw, yaw, KalmanRoll, kr, KalmanPitch, kp, OmegaX, wx, OmegaY, wy, OmegaZ, wz, AccX, ax, AccY, ay, AccZ, az] = strread(sentence,'%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f',1,'delimiter',',');
                    end
                end
            end
        end
        if val == 3
            
        end
        
        if val == 4
            
        end
        clr = {'r' 'g' 'b'};
        set(hLine, 'Color',clr{val})

        %# swithc to plot tab
        set(hTabGroup, 'SelectedTab',hTabs(3));
        drawnow
    end

    function connection(~,~)
        % Checks whether the user wants to connect and establishes the
        % Arduino connection
        if get(connect,'Value') == 1
            disp('Connecting...');
            %% Check to see if there are existing serial objects 
            % (instrfind) whos 'Port' property is set to 'COM3'
            
            oldSerial = instrfind('Port', 'COM3'); 
            % can also use instrfind() with no input arguments to find 
            % ALL existing serial objects

            % if the set of such objects is not(~) empty
            if (~isempty(oldSerial))  
                disp('WARNING:  COM3 in use.  Closing.')
                delete(oldSerial)
            end
            
            %%  Setting up serial communication
            % XBee expects the end of commands to be delineated by a carriage return.
            
            xbee = serial('COM3','baudrate',57600,'terminator','CR','tag','Quad');

            % Max wait time
            set(xbee, 'TimeOut', 5);  
            % One message long buffer
            set(xbee, 'InputBufferSize',  390 )
            % Open the serial
            fopen(xbee);    
            
            %% Testing Wireless communication

            fprintf(xbee,'T');
            ack = fscanf( xbee, '%s')
            if (strcmp(deblank(ack), 'K') == 1)
                yes = 'Y';
                fwrite(xbee,yes);
                set(connect,'BackgroundColor',[.99 .183 0.09],'String','Disconnect');
                set(conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');
                tenzo = true;
                disp ('Connection established. Rock & Roll!');
            else
                no = 'N';
                fwrite(xbee,no);
                set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
                tenzo = false;
                disp ('Communication problem. Check Hardware and retry.');
            end
        end
        if get(connect,'Value') == 0
            disp('A');  
            set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
            set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
            fclose(xbee);
            delete(xbee);
            tenzo = false;
            disp('Connection closed...');
        end
        if tenzo == true
            while(true)
                fprintf(xbee,'M') ; 
            end
        end
        %% Change ui components status
        
    end
    
%     function loadButtonCallback(src,evt)
%         %# load data
%         [fName,pName] = uigetfile('*.mat', 'Load data');
%         if pName == 0, return; end
%         data = load(fullfile(pName,fName), '-mat', 'X');
% 
%         %# plot
%         set(hLine, 'XData',data.X(:,1), 'YData',data.X(:,2));
% 
%         %# swithc to plot tab
%         set(hTabGroup, 'SelectedTab',hTabs(3));
%         drawnow
%     end
% 
%     %# drop-down menu callback
%     function popupCallback(src,evt)
%         %# update plot color
%         val = get(src,'Value');
%         clr = {'r' 'g' 'b'};
%         set(hLine, 'Color',clr{val})
% 
%         %# swithc to plot tab
%         set(hTabGroup, 'SelectedTab',hTabs(3));
%         drawnow
%     end
end