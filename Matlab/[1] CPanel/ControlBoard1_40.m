function ControlBoard()

global version;
global xbee;
global tenzo;
global Roll;
global theta;
global thetaOld;
global Pitch;
global pitch;
global Yaw;
global yaw;
global OmegaX;
global wx;
global OmegaY;
global wy ;
global OmegaZ;
global wz; 
global AccX;
global ax; 
global AccY;
global ay;
global AccZ;
global az;
% Motor speed received
global Motor;
global omegaR;
% Motor speed value
global omega;
global alpha;

% Boolean vars
global accelero;
global magneto;
global gyrosco;

% pidState
global pidStrategy;
global pidModeStrategy;


% Timers
global timerXbee;
global gyroTimer;
gyroTimer = timer('ExecutionMode','FixedRate','Period',0.3,'TimerFcn',{@graphGyro});

global angleTimer;
angleTimer = timer('ExecutionMode','FixedRate','Period',0.3,'TimerFcn',{@graphAngles});
                          
global accTimer;
accTimer = timer('ExecutionMode','FixedRate','Period',0.3,'TimerFcn',{@graphAcc});
          
% Complementary and Kalman Filert values
global KalmanRoll;
global kr;
global KalmanPitch;
global kp;
version = 1.40;

%% Plot variables

buf_len = 100;
index = 1:buf_len;

% create variables for the Xaxis
gxdata = zeros(buf_len,1);
gydata = zeros(buf_len,1);
gzdata = zeros(buf_len,1);

gxFdata = zeros(buf_len,1);
gyFdata = zeros(buf_len,1);
gzFdata = zeros(buf_len,1);
axdata = zeros(buf_len,1);
aydata = zeros(buf_len,1);
azdata = zeros(buf_len,1);
Tdata = zeros(buf_len,1);
Pdata = zeros(buf_len,1);
Ydata = zeros(buf_len,1);

% Kalman & Complementary Filter errors variables for the Axis
EKXdata = zeros(buf_len,1);
EKYdata = zeros(buf_len,1);

wx = 0;
gxFilt = 0;
gyFilt = 0;
gzFilt = 0;
TFilt = 0;
PFilt = 0;
YFilt = 0;
alpha = 0.7;

% initial motor speed
omega = 0;

% Boolean
magneto = false;
gyrosco = false;
accelero = false;

pidStrategy ='U';
pidModeStrategy = 'U';

    %% create tabbed GUI
    hFig = figure('Menubar','none');
    s = warning('off', 'MATLAB:uitabgroup:OldVersion');
    hTabGroup = uitabgroup('Parent',hFig);
    warning(s);
    hTabs(1) = uitab('Parent',hTabGroup, 'Title','Home');
    hTabs(2) = uitab('Parent',hTabGroup, 'Title','Motors');
    hTabs(3) = uitab('Parent',hTabGroup, 'Title','Sensors');
    hTabs(4) = uitab('Parent',hTabGroup, 'Title','Control');
    set(hTabGroup, 'SelectedTab',hTabs(1));
    Listener = addlistener(hTabGroup,'SelectedTab','PostSet',@tabGroupCallBack);
    
    if get(hTabGroup,'SelectedTab') == hTabs(1)
        disp('Tab1 selected');
    end
%     if get(hTabGroup,'SelectedTab') == hTabs(3)
%         disp('Tab3 selected');
%     end

    function tabGroupCallBack(~,~)
        %# Set visibility's slider to OFF
        val = get(hTabGroup,'SelectedTab');
        if val ~= hTabs(4)
        %disp('Tab1 selected');
        set(pidKiSlider,'Visible','off');
        set(pidKpSlider,'Visible','off');
        set(pidKdSlider,'Visible','off');
        disp(pidStrategy);
        end
        if val == hTabs(4) && get(pidRad,'Value') == 1
        %disp('Tab1 selected');
        set(pidKiSlider,'Visible','on');
        set(pidKpSlider,'Visible','on');
        set(pidKdSlider,'Visible','on');
        end
    end
    
    %# Home UI components
    uicontrol('Style','text', 'String','Tenzo Control Panel', ...
        'Position', [55 310 420 50],...
        'Parent',hTabs(1), 'FontSize',20,'FontWeight','bold');
    
    uicontrol('Style','text', 'String',version, ...
        'Position', [55 268 420 50],...
        'Parent',hTabs(1), 'FontSize',10,'FontWeight','normal');
    
    takeOffBtn = uicontrol('Style','pushbutton', 'String','Take Off', ...
        'Position', [70 210 120 30],...
        'Parent',hTabs(1), 'Callback',@takeOffCallback);
    
    hoverBtn = uicontrol('Style','pushbutton', 'String','Hover', ...
        'Position', [210 210 120 30],...
        'Parent',hTabs(1), 'Callback',@hoverCallback);
    
    landBtn = uicontrol('Style','pushbutton', 'String','Land', ...
        'Position', [350 210 120 30],...
        'Parent',hTabs(1), 'Callback',@landCallback);
    
    conTxt = uicontrol('Style','text', 'String','Offline','ForegroundColor',[.99 .183 0.09], ...
        'Position', [70 20 100 30],...
        'Parent',hTabs(1), 'FontSize',13,'FontWeight','bold');
    
    connect = uicontrol('Style','togglebutton', 'String','Connect', ...
        'Position', [400 20 120 30],'BackgroundColor',[.21 .96 .07],...
        'Parent',hTabs(1), 'Callback',@connection);        
    
    %# Control Ui Components
    
    pidPopup = uicontrol('Style','popupmenu','Position', [370 325 150 30],... 
        'String','Select|Roll|Pitch|Yaw|Altitude','visible','off', ...
        'Parent',hTabs(4), 'Callback',@pidPopupCallback);
    
    pidModePopup = uicontrol('Style','popupmenu','Position', [370 280 150 30],... 
        'String','Select|Conservative|Aggressive','visible','off', ...
        'Parent',hTabs(4), 'Callback',@pidModePopupCallback);
    
    welcomeControl = uicontrol('Style','text', 'String','Control Strategies', ...
        'Position', [260 157 150 50],...
        'Parent', hTabs(4), 'FontSize',15,'FontWeight','bold');
    
    workInProgress = uicontrol('Style','text','Visible','off',...
        'String','Work in Progress','Position', [260 157 150 50],...
        'Parent', hTabs(4), 'FontSize',15,'FontWeight','bold');
    
    % Create the button group.
    controlGroup = uibuttongroup('Parent', hTabs(4),'visible','off','Position',[0 0 .2 1]);
    
    uicontrol('Style','text', 'String','Select Control', ...
        'Position', [2 360 80 20],...
        'Parent', hTabs(4), 'FontSize',9);
    
    % Create three radio buttons in the button group.
    pidRad = uicontrol('Style','Radio','String','PID',...
        'pos',[10 250 80 30],'parent',controlGroup,'HandleVisibility','off');
    lqrRad = uicontrol('Style','Radio','String','LQR',...
        'pos',[10 150 80 30],'parent',controlGroup,'HandleVisibility','off');
    HInfRad = uicontrol('Style','Radio','String','H-Inifinite',...
        'pos',[10 50 80 30],'parent',controlGroup,'HandleVisibility','off');
    % Initialize some button group properties. 
    set(controlGroup,'SelectionChangeFcn',@selcbkControl);
    set(controlGroup,'SelectedObject',[]);  % No selection
    set(controlGroup,'Visible','on');
    
    % Pid Aggressive threshold value 
    
    frameThreshold = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 222 323 30 30 ]);
    
    thresholdPIDVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [226 325 20 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    thresholdPIDTxt = uicontrol('Style','text', 'String','Threshold', ...
        'Position', [135 325 70 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',11,'FontWeight','normal');
    
    sendThresholdBtn = uicontrol('Style','pushbutton', 'String','Send', ...
        'Position', [140 290 70 30],'Visible','off', ...
        'Parent',hTabs(4), 'Callback',@resetCallback);
    
    % Pid Txt & Vals    
    
    framePidKpVal = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 480 215 50 30 ]);
    
    pidKpVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 218 40 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    framePidKdVal = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 480 140 50 30 ]);
    
    pidKdVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 143 40 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    framePidKiVal = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 480 53 50 30 ]);
    
    pidKiVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 56 40 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    pidKpSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',2,'Callback',@(s,e) disp('KpSlider'),...
    'SliderStep',[0.01 0.10],'Position', [140 185 350 20]);
    KpListener = addlistener(pidKpSlider,'Value','PostSet',@pidKpSliderCallBack);
    
    pidKdSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',2,'Callback',@(s,e) disp('KdSlider'),...
    'SliderStep',[0.01 0.10],'Position', [140 110 350 20]);
    KdListener = addlistener(pidKdSlider,'Value','PostSet',@pidKdSliderCallBack);
    
    pidKiSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',2,'Callback',@(s,e) disp('KiSlider'),...
    'SliderStep',[0.01 0.10],'Position',[140 30 350 20]);
    KiListener = addlistener(pidKiSlider,'Value','PostSet',@pidKiSliderCallBack);
    
    pidKpTxt = uicontrol('Style','text','Visible','off',...
        'String','Proportional: Kp','Position', [140 190 150 50],...
        'Parent', hTabs(4), 'FontSize',11);
    
    pidKdTxt = uicontrol('Style','text','Visible','off',...
        'String','Derivative: Kd','Position', [140 115 150 50],...
        'Parent', hTabs(4), 'FontSize',11);
    
    pidKiTxt = uicontrol('Style','text','Visible','off',...
        'String','Integral: Ki','Position', [140 37 150 50],...
        'Parent', hTabs(4), 'FontSize',11);    
    
    %# Motors Ui Components
    
     uicontrol('Style','text', 'String','Motor Status', ...
        'Position', [260 157 150 50],...
        'Parent', hTabs(2), 'FontSize',15,'FontWeight','bold');
    
    % Create the button group.
    h = uibuttongroup('Parent', hTabs(2),'visible','off','Position',[0 0 .2 1]);
    
    uicontrol('Style','text', 'String','Select Mode', ...
        'Position', [2 360 80 20],...
        'Parent', hTabs(2), 'FontSize',9);
    
    % Create three radio buttons in the button group.
    u0 = uicontrol('Style','Radio','String','Test',...
        'pos',[10 250 80 30],'parent',h,'HandleVisibility','off');
    u1 = uicontrol('Style','Radio','String','Manual',...
        'pos',[10 150 80 30],'parent',h,'HandleVisibility','off');
    u2 = uicontrol('Style','Radio','String','Safe',...
        'pos',[10 50 80 30],'parent',h,'HandleVisibility','off');
    % Initialize some button group properties. 
    set(h,'SelectionChangeFcn',@selcbk);
    set(h,'SelectedObject',[]);  % No selection
    set(h,'Visible','on');
    
    upBtn = uicontrol('Style','pushbutton', 'String','UP', ...
        'Position', [400 70 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@upCallback);
    
    downBtn = uicontrol('Style','pushbutton', 'String','DW', ...
        'Position', [400 20 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@downCallback);
    
    up1Btn = uicontrol('Style','pushbutton', 'String','UP1', ...
        'Position', [480 70 70 30],'Visible','off', ...
        'Parent',hTabs(2),'BackgroundColor',[.21 .96 .07],...
        'Callback',@up1Callback);
    
    down1Btn = uicontrol('Style','pushbutton', 'String','DW1', ...
        'Position', [480 20 70 30],'Visible','off', ...
        'Parent',hTabs(2),'BackgroundColor',[.21 .96 .07],...
        'Callback',@down1Callback);
    
    
    initializeBtn = uicontrol('Style','pushbutton', 'String','Initialize', ...
        'Position', [130 20 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@initializeCallback);
    
    resetBtn = uicontrol('Style','pushbutton', 'String','Panic', ...
        'Position', [130 70 70 30],'Visible','off','BackgroundColor',[.99 .183 0.09], ...
        'Parent',hTabs(2), 'Callback',@resetCallback);
    
    testM1 = uicontrol('Style','pushbutton', 'String','Test', ...
        'Position', [295 277 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@testM1Callback);
    
    testM2 = uicontrol('Style','pushbutton', 'String','Test', ...
        'Position', [130 142 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@testM2Callback);
    
    testM3 = uicontrol('Style','pushbutton', 'String','Test', ...
        'Position', [295 85 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@testM3Callback);
    
    testM4 = uicontrol('Style','pushbutton', 'String','Test', ...
        'Position', [470 142 70 30],'Visible','off', ...
        'Parent',hTabs(2), 'Callback',@testM4Callback);
    
%     uicontrol('Style','text', 'String','v. 1.00', ...
%         'Position', [55 168 420 50],...
%         'Parent',hTabs(2), 'FontSize',10,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 305 310 50 50 ]);
    
    m1Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [320 327 20 25],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 140 175 50 50 ]);
    
    m2Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [155 192 20 25],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 305 30 50 50 ]);
    
    m3Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [320 47 20 25],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 480 175 50 50 ]);
    
    m4Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [495 192 20 25],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    % Sensors UI Components
    
    uicontrol('Style','popupmenu','Position', [370 360 150 30],... 
        'String','Select|Gyro|Accelerometer|Angles (est)', ...
        'Parent',hTabs(3), 'Callback',@popupCallback);
    
    
    %hAx = axes('Parent',hTabs(3));
    %hLine = plot(NaN, NaN, 'Parent',hAx, 'Color','r','LineWidth',2);

    %# Control UI Components
    
    %% button callbacks
    function upCallback(src,eventData)
       fprintf(xbee,'w');
    end

    function up1Callback(src,eventData)
           fprintf(xbee,'E');
    end

    function downCallback(src,eventData)
       fprintf(xbee,'s');
    end

    function down1Callback(src,eventData)
       fprintf(xbee,'D');
    end

    function initializeCallback(src,eventData)
       fprintf(xbee,'i');
    end

    function resetCallback(src,eventData)
       fprintf(xbee,'r');
    end

    function testM1Callback(src,eventData)
       fprintf(xbee,'1');
    end
    
    function testM2Callback(src,eventData)
       fprintf(xbee,'2'); 
    end

    function testM3Callback(src,eventData)
       fprintf(xbee,'3'); 
    end

    function testM4Callback(src,eventData)
       fprintf(xbee,'4'); 
    end

    function pidKpSliderCallBack(src,eventData)
       set(pidKpVal,'String',get(pidKpSlider,'Value')); 
       if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
           % Send 'X,opt1,opt2,opt3,val,X'
       strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
           num2str(get(pidKpSlider,'Value')),',X']
       fprintf(xbee,'%s',strindToSend,'sync'); 
       end
    end

    function pidKdSliderCallBack(src,eventData)
       set(pidKdVal,'String',get(pidKdSlider,'Value'));
       if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
       strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',1,', ...
           num2str(get(pidKpSlider,'Value')),',X']
       fprintf(xbee,'%s',strindToSend,'sync'); 
       end
    end

    function pidKiSliderCallBack(src,eventData)
       set(pidKiVal,'String',get(pidKiSlider,'Value'));
       if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
       strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',2,', ...
           num2str(get(pidKpSlider,'Value')),',X']
       fprintf(xbee,'%s',strindToSend,'sync'); 
       end
    end

    function selcbk(source,eventdata)
        disp(['You are in ',get(get(source,'SelectedObject'),'String'),' mode ']);
        
        % If Manual mode is seleceted toggle visibility of up/dw
        if get(u1,'Value') == 1
            set(upBtn,'Visible','on');
            set(downBtn,'Visible','on');
            set(up1Btn,'Visible','on');
            set(down1Btn,'Visible','on');
            set(initializeBtn,'Visible','on');
            set(resetBtn,'Visible','on');
        else
            set(upBtn,'Visible','off');
            set(downBtn,'Visible','off');
            set(up1Btn,'Visible','off');
            set(down1Btn,'Visible','off');
            set(initializeBtn,'Visible','off');
            set(resetBtn,'Visible','off');
        end
        
        % If Test is selected toggle visibility btns
        if get(u0,'Value') == 1
            set(testM1,'Visible','on');
            set(testM2,'Visible','on');
            set(testM3,'Visible','on');
            set(testM4,'Visible','on');
        else
            set(testM1,'Visible','off');
            set(testM2,'Visible','off');
            set(testM3,'Visible','off');
            set(testM4,'Visible','off');
        end
    end

function selcbkControl(source,eventdata)
        disp(['You are in ',get(get(source,'SelectedObject'),'String'),' mode ']);
        
        % If Manual mode is seleceted toggle visibility of up/dw
        if get(pidRad,'Value') == 1
            set(welcomeControl,'Visible','off');
            set(workInProgress,'Visible','off');
            set(frameThreshold,'Visible','on');
            set(thresholdPIDVal,'Visible','on');
            set(thresholdPIDTxt,'Visible','on');
            set(sendThresholdBtn,'Visible','on'); 
            set(pidModePopup,'Visible','on');
            set(framePidKpVal,'Visible','on');
            set(pidKpVal,'Visible','on');
            set(pidKpTxt,'Visible','on');
            set(pidKpSlider,'Visible','on');
            set(framePidKdVal,'Visible','on');
            set(pidKdVal,'Visible','on');
            set(pidKdTxt,'Visible','on');
            set(pidKdSlider,'Visible','on');
            set(framePidKiVal,'Visible','on');
            set(pidKiVal,'Visible','on');
            set(pidKiTxt,'Visible','on');
            set(pidKiSlider,'Visible','on');
            set(pidPopup,'Visible','on');             
        else                        
            set(frameThreshold,'Visible','off');
            set(thresholdPIDTxt,'Visible','off');
            set(thresholdPIDVal,'Visible','off');
            set(sendThresholdBtn,'Visible','off');
            set(welcomeControl,'Visible','off');
            set(workInProgress,'Visible','off'); 
            set(pidModePopup,'Visible','off');
            set(framePidKpVal,'Visible','off');
            set(pidKpVal,'Visible','off');
            set(pidKpTxt,'Visible','off');
            set(pidKpSlider,'Visible','off');
            set(framePidKdVal,'Visible','off');
            set(pidKdVal,'Visible','off');
            set(pidKdTxt,'Visible','off');
            set(pidKdSlider,'Visible','off');
            set(framePidKiVal,'Visible','off');
            set(pidKiVal,'Visible','off');
            set(pidKiTxt,'Visible','off');
            set(pidKiSlider,'Visible','off');
            set(pidPopup,'Visible','off');        
        end
        
        % If Test is selected toggle visibility btns
        if get(lqrRad,'Value') == 1            
            set(workInProgress,'Visible','on');
            set(frameThreshold,'Visible','off');
            set(thresholdPIDVal,'Visible','off');
            set(thresholdPIDTxt,'Visible','off');
            set(sendThresholdBtn,'Visible','off');
        else
        end
        
        if get(HInfRad,'Value') == 1            
            set(workInProgress,'Visible','on');
            set(frameThreshold,'Visible','off');
            set(thresholdPIDVal,'Visible','off');
            set(thresholdPIDTxt,'Visible','off');
            set(sendThresholdBtn,'Visible','off');
        else
            
        end
    end
    
    %# drop-down menu callback
    function popupCallback(src,~)
        %# update plot color
        val = get(src,'Value');
        
        % Gyro
        if val == 2
            magneto = false;
            accelero = false;
            gyrosco = true;
            start(gyroTimer);
            stop(angleTimer);
            stop(accTimer);
        end
        
        % Accelerometer
        if val == 3
            magneto = false;
            accelero = true;
            gyrosco = false;
            stop(gyroTimer);
            stop(angleTimer);
            start(accTimer);                    
        end
        
        % Magnetometer
        if val == 4
            magneto = true;
            accelero = false;
            gyrosco = false;
           stop(gyroTimer);
           start(angleTimer);
           stop(accTimer); 
        end
    end
    
    %# drop-down pid menu callback
    function pidPopupCallback(src,~)
        %# update plot color
        val = get(src,'Value');
        
        % Roll Pid Selected
        if val == 1
           pidStrategy = 'U';
           %disp('Unset');
        end
        
        % Roll Pid Selected
        if val == 2
           pidStrategy = '0';
           %disp('Rol');
        end
        
        % Pitch Pid Selected
        if val == 3
           pidStrategy = '1';  
           %disp('Pit');            
        end
        
        % Yaw Pid Selected
        if val == 4
           pidStrategy = '2';
           %disp('Yaw');
        end
        
        % Altitude Pid Selected
        if val == 5
           pidStrategy = '3';
           %disp('Alt');
        end
    end
    
    %# drop-down pid Mode menu callback
    function pidModePopupCallback(src,~)
        %# update plot color
        val = get(src,'Value');
        % Roll Pid Selected
        if val == 1
           pidModeStrategy = 'U';
           %disp('Unset');
        end
        
        % Conservative Mode Selected
        if val == 2
           pidModeStrategy = '0';
           %disp('Con');
        end
        
        % Aggressive Mode Selected
        if val == 3
           pidModeStrategy = '1'; 
           %disp('Agg');
        end
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

            xbee = serial('COM3','baudrate',9600,'terminator','CR','tag','Quad');

            % Max wait time
            set(xbee, 'TimeOut', 5);  
            % One message long buffer
            set(xbee, 'InputBufferSize',  390 )
            % Open the serial
            fopen(xbee);    

            %% Testing Wireless communication
            timerXbee = timer('ExecutionMode','FixedRate','Period',0.1,'TimerFcn',{@storeDataFromSerial});
            fprintf(xbee,'T');
                disp('yr  t')
            ack = fscanf( xbee, '%s');
            disp(ack);
            if (strcmp(deblank(ack), 'K') == 1)
                yes = 'Y';
                fwrite(xbee,yes);
                set(connect,'BackgroundColor',[.99 .183 0.09],'String','Disconnect');
                set(conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');
                tenzo = true;
                start(timerXbee);
                fprintf(xbee,'M') ; 
                disp ('Connection established. Rock & Roll!');
            else
                no = 'N';
                fwrite(xbee,no);
                set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
                tenzo = false;
                disp ('Communication problem. Check Hardware and retry.');
                warndlg('Communication busy. Press OK and reconnect','!! Warning !!')
            end
        end
        if get(connect,'Value') == 0
            set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
            set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
            fprintf(xbee,'S') ; 
            fclose(xbee);
            delete(xbee);
            tenzo = false;
            stop(timerXbee);
            delete(timerXbee);
            disp('Connection closed...');
        end
        %% Change ui components status
        
    end

    function graphAngles(obj,event,handles)
        % To debug uncomment the following line
        %disp('Angles');
       %Plot the X magnitude
        h1 = subplot(3,1,1,'Parent',hTabs(3));
        %set(hAx,'title','X angular velocity in deg/s');
        plot(h1,index,Tdata,'r','LineWidth',2);
        hold on;
        plot(h1,index,EKXdata,'b-','LineWidth',1);
        hold off;
%             xlabel('Time')
%             ylabel('Wx');
%             axis([1 buf_len -80 80]);
%             %hold on;
        h2 = subplot(3,1,2,'Parent',hTabs(3));
%             title('Y angular velocity in deg/s');
        plot(h2,index,Pdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
        hold on;
        plot(h2,index,EKYdata,'b-','LineWidth',1);
        hold off;
%             xlabel('Time');
%             ylabel('Wy Acc');
%             axis([1 buf_len -80 80]);
        h3 = subplot(3,1,3,'Parent',hTabs(3));
%             title('Z angular velocity in deg/s');
%             %hold on;
        plot(h3,index,Ydata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%       axis([1 buf_len -80 80]);
        hold on;
        plot(h2,index,EKYdata,'b-','LineWidth',1);
        hold off;
%             xlabel('Time');
%             ylabel('Wz Acc'); 
    end

    function graphGyro(obj,event,handles)
        % To debug uncomment the following line
        %disp('Gyro');
        %Plot the X magnitude
        h1 = subplot(3,1,1,'Parent',hTabs(3));
        %set(hAx,'title','X angular velocity in deg/s');
        plot(h1,index,gxFdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             xlabel('Time');
%             ylabel('Wx');
%             axis([1 buf_len -80 80]);
%             %hold on;
        h2 = subplot(3,1,2,'Parent',hTabs(3));
%             title('Y angular velocity in deg/s');
        plot(h2,index,gyFdata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             xlabel('Time');
%             ylabel('Wy Acc');
%             axis([1 buf_len -80 80]);
        h3 = subplot(3,1,3,'Parent',hTabs(3));
%             title('Z angular velocity in deg/s');
%             %hold on;
        plot(h3,index,gzFdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             axis([1 buf_len -80 80]);
%             xlabel('Time');
%             ylabel('Wz Acc');                
    end

    function graphAcc(obj,event,handles)
        % To debug uncomment the following line
        %disp('Acc');
        %Plot the X magnitude
        h1 = subplot(3,1,1,'Parent',hTabs(3));
        %set(hAx,'title','X angular velocity in deg/s');
        plot(h1,index,axdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             xlabel('Time');
%             ylabel('Wx');
%             axis([1 buf_len -80 80]);
%             %hold on;
        h2 = subplot(3,1,2,'Parent',hTabs(3));
%             title('Y angular velocity in deg/s');
        plot(h2,index,aydata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             xlabel('Time');
%             ylabel('Wy Acc');
%             axis([1 buf_len -80 80]);
        h3 = subplot(3,1,3,'Parent',hTabs(3));
%             title('Z angular velocity in deg/s');
%             %hold on;
        plot(h3,index,azdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
%             axis([1 buf_len -80 80]);
%             xlabel('Time');
%             ylabel('Wz Acc');
    end

    function storeDataFromSerial(obj,event,handles)
       try
            while (get(xbee, 'BytesAvailable')~=0 && tenzo == true)
                % read until terminator
                sentence = fscanf( xbee, '%s'); % this reads in as a string (until a terminater is reached)
                if (strcmp(sentence(1,1),'R'))
                    %decodes "sentence" seperated (delimted) by commaseck Unit')
                    [Roll, theta, Pitch, pitch, Yaw, yaw, KalmanRoll, kr, KalmanPitch, kp, OmegaX, wx, OmegaY, wy, OmegaZ, wz, AccX, ax, AccY, ay, AccZ, az, Motor, omegaR] = strread(sentence,'%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f',1,'delimiter',',');

                    % get motors speed value
                    omega = omegaR;
                    set(m1Txt,'String',omega); 
                    set(m2Txt,'String',omega); 
                    set(m3Txt,'String',omega); 
                    set(m4Txt,'String',omega); 
                    
%                     % Gets gyro data                    
%                     gxdata = [ gxdata(2:end) ; wx ];
%                     gydata = [ gydata(2:end) ; wy ];
%                     gzdata = [ gzdata(2:end) ; wz ];
                    
                    if gyrosco == true
                        % Filters gyro data and stores them
                        gxFilt = (1 - alpha)*gxFilt + alpha*wx;
                        gyFilt = (1 - alpha)*gyFilt + alpha*wy;
                        gzFilt = (1 - alpha)*gzFilt + alpha*wz;

                        gxFdata = [ gxFdata(2:end) ; gxFilt ];
                        gyFdata = [ gyFdata(2:end) ; gyFilt ];
                        gzFdata = [ gzFdata(2:end) ; gzFilt ];
                    end
                    
                    if magneto == true
                        % Gets Magnetometer and Estimated angles
                        if (theta>90)
                            theta = theta - 360;
                        end
                        if (pitch > 90)
                            pitch =  pitch - 360;
                        end 

                        % Apply noise filtering
                        % Uncomment for noise filtering
    %                     TFilt = (1 - alpha)*TFilt + alpha*theta;
    %                     PFilt = (1 - alpha)*PFilt + alpha*pitch;
    %                     YFilt = (1 - alpha)*YFilt + alpha*yaw;

    %                     Tdata = [ Tdata(2:end) ; TFilt ];
    %                     Pdata = [ Pdata(2:end) ; PFilt ];
    %                     Ydata = [ Ydata(2:end) ; YFilt ]; 

                        Tdata = [ Tdata(2:end) ; theta ];
                        Pdata = [ Pdata(2:end) ; pitch ];
                        Ydata = [ Ydata(2:end) ; yaw ]; 

                        TFilt = (1 - alpha)*TFilt + alpha*kr;
                        PFilt = (1 - alpha)*PFilt + alpha*kp;

                        EKXdata = [ EKXdata(2:end) ; TFilt ];
                        EKYdata = [ EKYdata(2:end) ; PFilt ];
                    end
                    
                    if accelero == true
                        % Gets Accelerometer data
                        axdata = [ axdata(2:end) ; ax ];
                        aydata = [ aydata(2:end) ; ay ];
                        azdata = [ azdata(2:end) ; az ];  
                    end
                end
            end
       end
    end  
end