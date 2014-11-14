function ControlBoard()

clear all;
clc;

global xbee;
global portWin;
global portUnix;
global baudrate;
global terminator;
global inputBuffSize;
global outputBuffSize;
global tag;
global tenzo;
global matlabAdd;
global arduinoAdd;
global versionProtocol;
global version;
global headerLength;
global cmdLength;
global iHoverID;
global landID;
global takeOffID;
global motorsID;
global accID;
global gyroID;
global magnID;
global estID;
global sonicID;
global gpsID;
global baroID;
global rollConsID;
global pitchConsID;
global yawConsID;
global altitudeConsID;
global rollAggID;
global pitchAggID;
global yawAggID;
global altitudeAggID;
global rCRPID;
global rCPPID;
global rCYPID;
global rCAPID;
global rARPID;
global rAPPID;
global rAYPID;
global rAAPID;

global bufferSend;
global landingSpeed;

% Serial Acks
global takeOffAck;
global landAck;
global hoverAck;
global accReceived;
global accRequested;
global gyroRequested;
global anglesRequested;
global gyroReceived;
global magnReceived;
global estReceived;
global cmdtype;

% Sensors vars
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
global defaultAlt;
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
global filterMagn;
global filterAcc;
global filterEst;
global filterGyro;

% pidState
global pidStrategy;
global pidModeStrategy;
global pidRead;

% Timers
global timerXbee;
global gyroTimer;

%% Pid Tuning
global aggAltKp;
global aggAltKd;
global aggAltKi;
global consAltKp;
global consAltKd;
global consAltKi;
global aggRollKp;
global aggRollKd;
global aggRollKi;
global consRollKp;
global consRollKd;
global consRollKi;
global aggPitchKp;
global aggPitchKd;
global aggPitchKi;
global consPitchKp;
global consPitchKd;
global consPitchKi;
global aggYawKp;
global aggYawKd;
global aggYawKi;
global consYawKp;
global consYawKd;
global consYawKi;

%% Serial protocol

versionProtocol = 1;
% command Map
motorsID = 1;
accID = 2;
gyroID = 3;
magnID = 4;
estID = 5;
sonicID = 6;
gpsID = 7;
baroID = 8;
rollConsID = 9;
pitchConsID = 10;
yawConsID = 11;
altitudeConsID = 12;
rollAggID = 13;
pitchAggID = 14;
yawAggID = 15;
altitudeAggID = 16;
takeOffID = 17;
iHoverID = 18;
landID = 19;
rCRPID = 21;
rCPPID = 22;
rCYPID = 23;
rCAPID = 24;
rARPID = 25;
rAPPID = 26;
rAYPID = 27;
rAAPID = 28;

cmdLength = 17;
headerLength = 13;
arduinoAdd = 1;
matlabAdd = 2;
portWin = 'Com3';
portUnix = '\dev\ttyS0';
baudrate = 19200;
% buffer size should be the same as the one specified on the Arduino side
inputBuffSize = 47+1;
outputBuffSize = 31;
terminator = 'CR';
tag = 'Quad';

gyroTimer = timer('ExecutionMode','FixedRate','Period',1.5,'TimerFcn',{@graphGyro});

global angleTimer;
angleTimer = timer('ExecutionMode','FixedRate','Period',1.5,'TimerFcn',{@graphAngles});
                          
global accTimer;
accTimer = timer('ExecutionMode','FixedRate','Period',1.5,'TimerFcn',{@graphAcc});
          
% Complementary and Kalman Filert values
global KalmanRoll;
global kr;
global KalmanPitch;
global kp;
version = 1.45;
landingSpeed = 2;

% Serial Sensors Acks initialization
accReceived = false;
gyroReceived = false;
magnReceived = false;
estReceived = false;

doubleAnglePlot = false;

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
Rdata = zeros(buf_len,1);
Pdata = zeros(buf_len,1);
Ydata = zeros(buf_len,1);

% Kalman & Complementary Filter errors variables for the Axis
EKXdata = zeros(buf_len,1);
EKYdata = zeros(buf_len,1);
EKZdata = zeros(buf_len,1);

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
filterEst = false;
filterMagn = false;
filterAcc = false;
filterGyro = false;

pidStrategy ='U';
pidModeStrategy = 'U';
pidRead = 0;

%% variables declaration

takeOffAck = 0;
hoverAck = 0;
landAck = 1;
defaultAlt = 1;
% used to store long/short num to arrays
weights2 = 2.^([7:-1:0]);

bufferSend = zeros(1, outputBuffSize);
  
disp('Welcome to the CPanel');
% Delete all serial connections
delete(instrfindall)

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
    
    hoverBtn = uicontrol('Style','pushbutton', 'String','iHover', ...
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
    
    referencePIDVal = uicontrol('Style','edit', 'String','0', ...
        'Position', [226 325 20 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    referencePIDTxt = uicontrol('Style','text', 'String','Reference', ...
        'Position', [135 325 70 25],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',11,'FontWeight','normal');
    
    sendPidValsBtn = uicontrol('Style','pushbutton', 'String','Send', ...
        'Position', [140 290 70 30],'Visible','off', ...
        'Parent',hTabs(4), 'Callback',@sendPidCallback);
    
    readPidValsBtn = uicontrol('Style','pushbutton', 'String','Read', ...
        'Position', [140 250 70 30],'Visible','off', ...
        'Parent',hTabs(4), 'Callback',@readPidCallback);
    
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

    function landCallback(src,eventData)
        if tenzo == true
            if takeOffAck == 1
                % Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(landID);
                %cmd(2,4) = uint8(defaultAlt);
                bits = reshape(bitget(landingSpeed,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
                % wait for feedback from Tenzo and change state of btn
            else
               warndlg('Tenzo is not in hovering mode. First Take Off then try again. ','!! Warning !!') 
               % you can start take off protocol automatically
            end  
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function takeOffCallback(src,eventData)
        if tenzo == true
            if takeOffAck == 0
                % Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(takeOffID);
                %cmd(2,4) = uint8(defaultAlt);
                bits = reshape(bitget(defaultAlt,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
                % wait for feedback from Tenzo and change state of btn
            else
               warndlg('Tenzo already out in space. Connection blocked. Protocol 1','!! DANGER !!') 
               % you can start take off protocol automatically
            end  
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function hoverCallback(src,eventData)
        if tenzo == true
            if takeOffAck == 1
                if hoverAck == 0
                    %warndlg('Enabling PID. Safe flight.','Report') 
                    % Initialize the cmd array
                    cmd = zeros(8,4,'uint8');
                    cmd(1,1) = uint8(iHoverID);
                    % Sends 1 to activate PID
                    bits = reshape(bitget(1,32:-1:1),8,[]);
                    cmd(2,:) = weights2*bits;
                    sendMess(cmd);
                    % wait for feedback from Tenzo and change state of btn
                else
                   warndlg('Desactivating PID','!! Warning !!') 
                   % Initialize the cmd array
                   cmd = zeros(8,4,'uint8');
                   cmd(1,1) = uint8(iHoverID);
                    % Sends 0 to disable PID
                   bits = reshape(bitget(0,32:-1:1),8,[]);
                   cmd(2,:) = weights2*bits;
                   sendMess(cmd); 
                   % you can start take off protocol automatically
                end
            else
               warndlg('Tenzo is not flying. First Take Off then try again. ','!! Warning !!') 
               % you can start take off protocol automatically
            end  
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
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

%    function pidKpSliderCallBack(src,eventData)
%        set(pidKpVal,'String',get(pidKpSlider,'Value')); 
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%            % Send 'X,opt1,opt2,opt3,val,X'
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
%            num2str(get(pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
%     end
% 
%     function pidKdSliderCallBack(src,eventData)
%        set(pidKdVal,'String',get(pidKdSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',1,', ...
%            num2str(get(pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
%     end
% 
%     function pidKiSliderCallBack(src,eventData)
%        set(pidKiVal,'String',get(pidKiSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',2,', ...
%            num2str(get(pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
%     end

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
            set(referencePIDVal,'Visible','on');
            set(referencePIDTxt,'Visible','on');
            set(sendPidValsBtn,'Visible','on'); 
            set(readPidValsBtn,'Visible','on'); 
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
            set(referencePIDTxt,'Visible','off');
            set(referencePIDVal,'Visible','off');
            set(sendPidValsBtn,'Visible','off');
            set(readPidValsBtn,'Visible','off');
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
            set(referencePIDVal,'Visible','off');
            set(referencePIDTxt,'Visible','off');
            set(sendPidValsBtn,'Visible','off');
        else
        end
        
        if get(HInfRad,'Value') == 1            
            set(workInProgress,'Visible','on');
            set(frameThreshold,'Visible','off');
            set(referencePIDVal,'Visible','off');
            set(referencePIDTxt,'Visible','off');
            set(sendPidValsBtn,'Visible','off');
        else
            
        end
    end
    
    %# drop-down menu callback
    function popupCallback(src,~)
        %# update plot color
        val = get(src,'Value');
        
        if val == 1
            magneto = false;
            accelero = false;
            gyrosco = false;
            stop(gyroTimer);
            stop(angleTimer);
            stop(accTimer);
        end
        
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
        
        % Selected
        if val == 1
           pidStrategy = 'U';
           %disp('Unset');
        end
        
        % Roll Pid Selected
        if val == 2
           pidStrategy = '0';
           disp('Rol');
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
           disp('Con');
        end
        
        % Aggressive Mode Selected
        if val == 3
           pidModeStrategy = '1'; 
           %disp('Agg');
        end
    end
    
    function connection(~,~)        
        if get(connect,'Value') == 1
            disp('Connecting...');

            %% Check to see if there are existing serial objects 
            % (instrfind) whos 'Port' property is set to 'COM3'

            oldSerial = instrfind('Port', portWin); 
            % can also use instrfind() with no input arguments to find 
            % ALL existing serial objects

            % if the set of such objects is not(~) empty
            if (~isempty(oldSerial))  
                disp('WARNING:  port in use.  Closing.')
                delete(oldSerial)
            end

            %%  Setting up serial communication
            % XBee expects the end of commands to be delineated by a carriage return.

            xbee = serial(portWin,'baudrate',baudrate,'terminator',terminator,'tag',tag);

            % Max wait time
            set(xbee, 'TimeOut', 10);  
            % One message long buffer
            set(xbee, 'InputBufferSize',inputBuffSize)
            % Open the serial
            fopen(xbee);    

            %% Testing Wireless communication
            timerXbee = timer('ExecutionMode','FixedRate','Period',0.1,'TimerFcn',{@storeDataFromSerial});
            disp ('sending 16 to Arduino');
            fwrite(xbee,16); 
            disp('Ack Requested.');
            ack = fread(xbee);
            disp('Receiving: ');
            disp(ack);
            if (ack == 17)
                disp ('sending 18 to Arduino');  
                fwrite(xbee,18);     
                tenzo = true; 
                set(connect,'BackgroundColor',[.99 .183 0.09],'String','Disconnect');
                set(conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');    
                disp ('Connection established. Rock & Roll!'); 
                start(timerXbee);  
                disp ('Connection established. Rock & Roll!');
            else
                disp ('Sending 19 to Arduino.');
                % Received something else
                disp ('sending 19 to Arduino');  
                fwrite(xbee,19);
                tenzo = false;
                set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
                disp ('Communication problem. Check Hardware and retry.');
                warndlg('Communication busy. Press OK and reconnect','!! Warning !!')
            end
        end

        if get(connect,'Value') == 0
            disp ('sending 20 to Arduino');  
            fwrite(xbee,20);
            % Waiting for ack
            ack = fread(xbee);
            disp('Receiving: ');
            disp(ack);
            if (ack == 21)
                tenzo = false;
                stop(timerXbee);
                delete(timerXbee);
                set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');  
                disp('Connection closed...');        
                fclose(xbee);
                delete(xbee);
                clear xbee;
            else
                
            end
        end 
    end

    function sendMess(obj)
        % Build the message Header + Command (only one command at the time)

        %Initialize header as an array full of 'f'
        %hr = repmat('f',8,4);
        hr = zeros(8,4,'uint8');
        %% Build Header

        % source address
        hr(1,1) = matlabAdd;
        % destination address
        hr(2,1) = arduinoAdd;
        % version  
        bits = reshape(bitget(versionProtocol,32:-1:1),8,[]);
        hr(3,:) = weights2*bits;
        % number of commands
        hr(4,1) = uint8(1);
        % header length
        hr(5,1) = uint8(headerLength);
        % command length
        hr(6,1) = uint8(cmdLength);
        % total length
        bits = reshape(bitget(32,32:-1:1),8,[]);
        tmp = weights2*bits;
        hr(7,1:2) = tmp(1,3:4);
        % Crc
        hr(8,1) = uint8(0);
        hr(8,2) = uint8(12); % TODO

        % header
        bufferSend(1) = hr(1,1);
        bufferSend(2) = hr(2,1);
        bufferSend(3) = hr(3,1);
        bufferSend(4) = hr(3,2);
        bufferSend(5) = hr(3,3);
        bufferSend(6) = hr(3,4);
        bufferSend(7) = hr(4,1);
        bufferSend(8) = hr(5,1);
        bufferSend(9) = hr(6,1);
        bufferSend(10) = hr(7,1);
        bufferSend(11) = hr(7,2);
        bufferSend(12) = hr(8,1);
        bufferSend(13) = hr(8,2);
        bufferSend(14) = obj(1,1);
        % cmd 1
        bufferSend(15) = obj(2,1);
        bufferSend(16) = obj(2,2);
        bufferSend(17) = obj(2,3);
        bufferSend(18) = obj(2,4);
        % cmd 1
        bufferSend(19) = obj(3,1);
        bufferSend(20) = obj(3,2);
        bufferSend(21) = obj(3,3);
        bufferSend(22) = obj(3,4);
        % cmd 1
        bufferSend(23) = obj(4,1);
        bufferSend(24) = obj(4,2);
        bufferSend(25) = obj(4,3);
        bufferSend(26) = obj(4,4);
        % cmd 1
        bufferSend(27) = obj(5,1);
        bufferSend(28) = obj(5,2);
        bufferSend(29) = obj(5,3);
        bufferSend(30) = obj(5,4);

        disp(bufferSend);
        fwrite(xbee,bufferSend,'uint8');    
        %xbee.ValuesSent
        
        %% Command

        %s = struct('H',value1,'C1',valueN)
    end

    function graphAngles(obj,event,handles)
        % To debug uncomment the following line
        %disp('Angles');
        anglesRequested = true;
        
        %% Requests data only if previous ones have been received and plotted
        
        if estReceived || anglesRequested
            %Initialize the cmd array
            cmd = zeros(8,4,'uint8');
            % You can send 
            % cmd(1,1) = uint8(magnID); OR
            cmd(1,1) = uint8(estID);
            % Sends 1 to activate PID
            bits = reshape(bitget(0,32:-1:1),8,[]);
            cmd(2,:) = weights2*bits;
            sendMess(cmd);
        else
            disp('Not received yet Gyro');
        end
    end

    function sendPidCallback(obj,event)
        if tenzo == true
            if takeOffAck == 1
                if hoverAck == 1
                    if pidRead == 1
                       if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
                           if strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'0')
                               % R C
                               cmdtype = rollConsID;
                           elseif strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'1')
                               % R A        
                               cmdtype = rollAggID;
                           elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'0')
                               % P C
                               cmdtype = pitchConsID;
                           elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'1')
                               % P A                                 
                               cmdtype = pitchAggID;
                           elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'0')
                               % Y C
                               cmdtype = yawConsID;
                           elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'1')
                               % Y A                                 
                               cmdtype = yawAggID;
                           elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'0')
                               % A C
                               cmdtype = altitudeConsID;
                           elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'1')
                               % A A  (american Airlines -> Allin                        
                               cmdtype = altitudeAggID;
                           end
                           % Send 'X,opt1,opt2,opt3,val,X'
%                            strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
%                            num2str(get(pidKpSlider,'Value')),',X']
%                            fprintf(xbee,'%s',strindToSend,'sync');  

                            %Initialize the cmd array
                            cmd = zeros(8,4,'uint8');
                            % You can send 
                            % cmd(1,1) = uint8(magnID); OR
                            cmd(1,1) = uint8(cmdtype);
                            % Sends 1 to activate PID
                            disp('ref value:');
                            disp(get(referencePIDTxt,'String'));
                            %bits = reshape(bitget(get(referencePIDTxt,'String'),32:-1:1),8,[]);
%                             cmd(2,:) = weights2*bits;
                            disp('Kp Slider value:');
                            disp(get(pidKpSlider,'Value')*100);
%                             bits = reshape(bitget(get(pidKpSlider,'Value')*100,32:-1:1),8,[]);
%                             cmd(3,:) = weights2*bits;
%                             bits = reshape(bitget(get(pidKdSlider,'Value')*100,32:-1:1),8,[]);
%                             cmd(4,:) = weights2*bits;
%                             bits = reshape(bitget(get(pidKiSlider,'Value')*100,32:-1:1),8,[]);
%                             cmd(5,:) = weights2*bits;
                            disp('sending');
                            sendMess(cmd);
                       else
                           warndlg('Please select correct mode from Popo menus','!! Warning !!')
                       end  
                   else
                       warndlg('Read actual values first','!! Warning !!')
                   end
                else
                    warndlg('Pid not active, Activate iHover function','!! Warning !!')
                end
            else
                warndlg('Tenzo is not flying. First Take Off then try again. ','!! Warning !!')
            end
        else
            warndlg('Please connect first ','!! Warning !!')     
        end
    end


    function readPidCallback(obj,event)
        if tenzo == true
            if takeOffAck == 1
                if hoverAck == 1
                   if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
                       pidRead = true;
                       if strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'0')
                           % R C
                           cmdtype = rCRPID;
                           disp('dovrebbe eessre 21');
                       elseif strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'1')
                           % R A        
                           cmdtype = rCPPID;
                       elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'0')
                           % P C
                           cmdtype = rCYPID;
                       elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'1')
                           % P A                                 
                           cmdtype = rCAPID;
                       elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'0')
                           % Y C
                           cmdtype = rAPPID;
                       elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'1')
                           % Y A                                 
                           cmdtype = rARPID;
                       elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'0')
                           % A C
                           cmdtype = rAYPID;
                       elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'1')
                           % A A  (american Airlines -> Allin                        
                           cmdtype = rAAPID;
                       end
                       % Send 'X,opt1,opt2,opt3,val,X'
%                            strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
%                            num2str(get(pidKpSlider,'Value')),',X']
%                            fprintf(xbee,'%s',strindToSend,'sync');  
                        disp('cmdtype');
                        disp(cmdtype);
                        disp('cmdtype');
                        %Initialize the cmd array
                        cmd = zeros(8,4,'uint8');
                        cmd(1,1) = uint8(cmdtype);
                        sendMess(cmd);
                   else
                       warndlg('Please select correct mode from Popo menus','!! Warning !!')
                   end  
                else
                    warndlg('Pid not active, Activate iHover function','!! Warning !!')
                end
            else
                warndlg('Tenzo is not flying. First Take Off then try again. ','!! Warning !!')
            end
        else
            warndlg('Please connect first ','!! Warning !!')     
        end
    end

    function graphGyro(obj,event,handles)
        % To debug uncomment the following line
        %disp('Gyro');
        gyroRequested = true;
        %% Requests data only if previous ones have been received and plotted
        
        if gyroReceived || gyroRequested
            % Initialize the cmd array
            cmd = zeros(8,4,'uint8');
            cmd(1,1) = uint8(gyroID);
            % Sends 1 to activate PID
            bits = reshape(bitget(0,32:-1:1),8,[]);
            cmd(2,:) = weights2*bits;
            sendMess(cmd);
        else
            disp('Not received yet Gyro');
        end
            
    end

    function graphAcc(obj,event,handles)
        % To debug uncomment the following line
        %disp('Acc');
        accRequested = true;
        %% Requests data only if previous ones have been received and plotted
        
        if accReceived || accRequested
            % Initialize the cmd array
            cmd = zeros(8,4,'uint8');
            cmd(1,1) = uint8(accID);
            % Sends 1 to activate PID
            bits = reshape(bitget(0,32:-1:1),8,[]);
            cmd(2,:) = weights2*bits;
            accReceived = false;
            sendMess(cmd);
        else
            disp('Not received yet');
        end
    end

    function storeDataFromSerial(obj,event,handles)
       
       while (get(xbee, 'BytesAvailable')~=0 && tenzo == true)
            % read until terminator
            [mess,count] = fread(xbee);
            disp('Reading incoming buffer. Dimensions:');
            %% Debug stuff

            disp(count);
            disp(mess);
            %% Parsing the message

            if (count >= (inputBuffSize) && mess(2) == matlabAdd && mess(1) == arduinoAdd)
                % Mess sent from Arduino to MATLAB
                % Assemble long int sent bytewise
                versionArd = typecast([uint8(mess(3)), uint8(mess(4)),uint8(mess(5)), uint8(mess(6))], 'int32');
                if versionProtocol ~= versionArd
                    warndlg('Your version is different from the one in Tenzo. Sync your repository.','!! Warning !!') 
                end
                numCmd = mess(7)
                %Arduino  tells the receiver where commands start
                readFrom = mess(8);
%                     disp('ReadFrom');
%                     disp(readFrom);
                sizeOfEachCmd = mess(9);
                totMessLength = typecast([uint8(mess(10)), uint8(mess(11))], 'int16');
                disp('Total message length (header value):');
                disp(totMessLength);
                % TODO CRC check for message's integrity
                crcvalue = typecast([uint8(mess(12)), uint8(mess(13))], 'int16');
                %disp('CRC value');
                %disp(crcvalue);

                %% Read Commands
                for i = 0:(numCmd-1)
                    typei = (readFrom+1)+i*sizeOfEachCmd;
                    type = mess(typei) 
                    disp('Message #:');
                    disp(i+1);

                    if type == 0 
                        % Motors
                        disp('Unset');
                        break;
                    end
                    if type == 1 
                        % Motors
                        disp('Motors');
                        speed1 = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('speed:');
                        disp(speed1);                       
                    end
                    if type == accID 
                        disp('Accelerations');
                        % Acc
                        accXr = typecast([int8(mess(typei + 1)), int8(mess(typei + 2)),int8(mess(typei + 3)), int8(mess(typei + 4))], 'int32');
                        disp('accX:');
                        disp(accXr);

                        accYr = typecast([int8(mess(typei + 5)), int8(mess(typei + 6)),int8(mess(typei + 7)), int8(mess(typei + 8))], 'int32');
                        disp('accY:');
                        disp(double(accYr));                            

                        accZr = typecast([int8(mess(typei + 9)), int8(mess(typei + 10)),int8(mess(typei + 11)), int8(mess(typei + 12))], 'int32');
                        disp('accZ:');
                        disp(accZr);
                        
                        if accelero == true
                            % Gets Accelerometer data
                            axdata = [ axdata(2:end) ; accXr ];
                            aydata = [ aydata(2:end) ; accYr ];
                            azdata = [ azdata(2:end) ; accZr ];  
                        end               
                        %Plot the X magnitude
                        h1 = subplot(3,1,1,'Parent',hTabs(3));
                        %set(hAx,'title','X angular velocity in deg/s');
                        if filterAcc
                            plot(h1,index,axdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        else
                            plot(h1,index,axdata,'r','LineWidth',2);
                        end
                        %xlabel('Time');
                        %ylabel('Wx');
                        %axis([1 buf_len -80 80]);
                        %hold on;
                        h2 = subplot(3,1,2,'Parent',hTabs(3));
                        %title('Y angular velocity in deg/s');
                        if filterAcc
                            plot(h2,index,aydata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        else
                            plot(h2,index,aydata,'b','LineWidth',2);
                        end
                        %xlabel('Time');
                        %ylabel('Wy Acc');
                        %axis([1 buf_len -80 80]);
                        h3 = subplot(3,1,3,'Parent',hTabs(3));
                        %title('Z angular velocity in deg/s');
                        %hold on;
                        if filterAcc
                            plot(h3,index,azdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        else
                            plot(h3,index,azdata,'g','LineWidth',2);
                        end
                        %axis([1 buf_len -80 80]);
                        %xlabel('Time');
                        %ylabel('Wz Acc');
                        
                        % Toggle ack 
                        accReceived = true;
                    end
                    if type == gyroID
                        % Gyro
                        disp('Gyro');
                        wXr = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('wX:');
                        disp(wXr/100);

                        wYr = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('wY:');
                        disp(wYr/100);                             

                        wZr = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('wZ:');
                        disp(wZr/100);
                        
                        if gyrosco == true
                            if filterGyro
                                % Filters gyro data and stores them
                                gxFilt = (1 - alpha)*gxFilt + alpha*wXr/100;
                                gyFilt = (1 - alpha)*gyFilt + alpha*wYr/100;
                                gzFilt = (1 - alpha)*gzFilt + alpha*wZr/100;
                            else
                                gxFilt = wXr/100;
                                gyFilt = wYr/100;
                                gzFilt = wZr/100;
                            end
                                gxFdata = [ gxFdata(2:end) ; gxFilt ];
                                gyFdata = [ gyFdata(2:end) ; gyFilt ];
                                gzFdata = [ gzFdata(2:end) ; gzFilt ];                            
                        end
                        
                        %Plot the X magnitude
                        h1 = subplot(3,1,1,'Parent',hTabs(3));
                        %set(hAx,'title','X angular velocity in deg/s');
                        plot(h1,index,gxFdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        %xlabel('Time');
                        %ylabel('Wx');
                        %axis([1 buf_len -80 80]);
                        %hold on;
                        h2 = subplot(3,1,2,'Parent',hTabs(3));
                        %title('Y angular velocity in deg/s');
                        plot(h2,index,gyFdata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        %xlabel('Time');
                        %ylabel('Wy Acc');
                        %axis([1 buf_len -80 80]);
                        h3 = subplot(3,1,3,'Parent',hTabs(3));
                        %title('Z angular velocity in deg/s');
                        %hold on;
                        plot(h3,index,gzFdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        %axis([1 buf_len -80 80]);
                        %xlabel('Time');
                        %ylabel('Wz Acc');    
                        
                        % Toggle ack 
                        gyroReceived = true;
                    end
                    if type == magnID 
                        % Magn
                        disp('Magn');;
                        rollM = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('rollMagn:');
                        disp(rollM);

                        pitchM = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('pitchMagn:');
                        disp(pitchM);                             

                        bearingM = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('yawMagn:');
                        disp(bearingM);
                        
                        if magneto == true
                            % Gets Magnetometer and Estimated angles
                            if (rollM>90)
                                rollM = rollM - 360;
                            end
                            if (pitchM > 90)
                                pitchM =  pitchM - 360;
                            end 
                            
                            if filterMagn
                                % Apply noise filtering
                                TFilt = (1 - alpha)*TFilt + alpha*rollM;
                                PFilt = (1 - alpha)*PFilt + alpha*pitchM;
                                YFilt = (1 - alpha)*YFilt + alpha*bearingM/100;

                                Rdata = [ Rdata(2:end) ; TFilt ];
                                Pdata = [ Pdata(2:end) ; PFilt ];
                                Ydata = [ Ydata(2:end) ; YFilt ]; 
                            else
                                Rdata = [ Rdata(2:end) ; rollM ];
                                Pdata = [ Pdata(2:end) ; pitchM ];
                                Ydata = [ Ydata(2:end) ; bearingM/100 ]; 
                            end
                        else
                            disp('Warning! Received magneto data but not requested');
                        end
                        
                        % Toggle ack 
                        magnReceived = true;
                    end
                    if type == estID
                        % Est
                        disp(' Kalman Est');
                        disp('Magn');;
                        rollE = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('roll K:');
                        disp(rollE);

                        pitchE = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('pitch K:');
                        disp(pitchE);                             

                        bearingE = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('yaw K:');
                        disp(single(bearingE/100));
                        
                            
                        if filterEst
                            REstFilt = (1 - alpha)*REstFilt + alpha*rollE;
                            PEstFilt = (1 - alpha)*PEstFilt + alpha*pitchE;
                            YEstFilt = (1 - alpha)*YEstFilt + alpha*bearingE/100;
                        else
                            REstFilt = rollE;
                            PEstFilt = pitchE;
                            YEstFilt = bearingE/100;
                        end
                        EKXdata = [ EKXdata(2:end) ; REstFilt ];
                        EKYdata = [ EKYdata(2:end) ; PEstFilt ];
                        EKZdata= [ EKYdata(2:end) ; YEstFilt ];
                        
                        %% Plot Angles

                        
                        %Plot the X magnitude
                        h1 = subplot(3,1,1,'Parent',hTabs(3));
                        %set(hAx,'title','X angular velocity in deg/s');
                        plot(h1,index,EKXdata,'b-','LineWidth',2);
                        if doubleAnglePlot
                            hold on;
                            plot(h1,index,Rdata,'r','LineWidth',1);
                            hold off;
                        end
                        %xlabel('Time')
                        %ylabel('Wx');
                        %axis([1 buf_len -80 80]);
                        %hold on;
                        h2 = subplot(3,1,2,'Parent',hTabs(3));
                        %title('Y angular velocity in deg/s');
                        plot(h2,index,EKYdata,'b-','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        if doubleAnglePlot
                            hold on;
                            plot(h2,index,Pdata,'r','LineWidth',1);
                            hold off;
                        end
                        %xlabel('Time');
                        %ylabel('Wy Acc');
                        %axis([1 buf_len -80 80]);
                        h3 = subplot(3,1,3,'Parent',hTabs(3));
                        %title('Z angular velocity in deg/s');
                        %hold on;
                        plot(h3,index,EKZdata,'b-','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                        %axis([1 buf_len -80 80]);
                        if doubleAnglePlot
                            hold on;
                            plot(h3,index,Ydata,'r','LineWidth',2); 
                            hold off;
                        end
                        %xlabel('Time');
                        %ylabel('Wz Acc'); 
                        
                        % Toggle ack 
                        estReceived = true;
                    end
                    if type == 6
                        % Sonic
                        disp('Sonic');                        
                    end
                    if type == 7 
                        % Gps
                        disp('Gps');
                    end
                    if type == 8 
                        % Barometer
                        disp('Barometer');
                    end                    
                    if type == 9 
                        % Pid Roll CONS
                        disp('Pid Roll Cons');
                        consRollKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('consRollKp:');
                        consRollKp = consRollKpTemp/100;
                        disp(consRollKp);

                        consRollKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('consRollKd:');
                        consRollKd = consRollKdTemp/100;
                        disp(consRollKd);                             

                        consRollKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('consRollKi:');
                        consRollKi = consRollKiTemp/100;
                        disp(consRollKi);    

                        setpointRollTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('threshold:');
                        disp(setpointRollTemp);   
                        
                        if strcmp(pidModeStrategy,'0')
                            set(pidKpSlider,'Value',consRollKp);
                            set(pidKpSlider,'Value',consRollKd);
                            set(pidKpSlider,'Value',consRollKi);
                            set(referencePIDTxt,'String',setpointRollTemp);
                        end
                    end                   
                    if type == 13 
                        % Pid Roll AGG
                        disp('Pid Roll AGG');
                        aggRollKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('aggRollKp:');
                        aggRollKp = aggRollKpTemp/100;
                        disp(aggRollKpTemp/100);

                        aggRollKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('aggRollKd:');
                        aggRollKd = aggRollKdTemp/100;
                        disp(aggRollKd);                             

                        aggRollKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('aggRollKi:');
                        aggRollKi = aggRollKiTemp/100;
                        disp(aggRollKi);    

                        setpointRollTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('threshold:');
                        disp(setpointRollTemp);   
                       
                        if strcmp(pidModeStrategy,'1') 
                            set(pidKpSlider,'Value',aggRollKp);
                            set(pidKpSlider,'Value',aggRollKd);
                            set(pidKpSlider,'Value',aggRollKi);
                            set(referencePIDTxt,'String',setpointRollTemp);
                        end
                    end
                    if type == 10 
                        % Pid Pitch CONS
                        disp('Pid Pitch Cons');
                        consPitchKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        consPitchKp = consPitchKpTemp/100;
                        disp(consPitchKp);

                        consPitchKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        consPitchKd = consPitchKdTemp/100;
                        disp(consPitchKd);                             

                        consPitchKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        consPitchKi = consPitchKiTemp/100;
                        disp(consPitchKi);    

                        setpointPitchTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('threshold:');
                        disp(setpointPitchTemp); 
                        
                        if strcmp(pidModeStrategy,'0')
                            set(pidKpSlider,'Value',consPitchKp);
                            set(pidKpSlider,'Value',consPitchKd);
                            set(pidKpSlider,'Value',consPitchKi);
                            set(referencePIDTxt,'String',setpointPitchTemp);
                        end
                    end                   
                    if type == 14 
                        % Pid Pitch AGG
                        disp('Pid Pitch Agg');
                        aggPitchKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        aggPitchKp = aggPitchKpTemp/100;
                        disp(aggPitchKp);

                        aggPitchKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        aggPitchKd = aggPitchKdTemp/100;
                        disp(aggPitchKd);                             

                        aggPitchKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        aggPitchKi = aggPitchKiTemp/100;
                        disp(aggPitchKi);    

                        setpointPitchTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('threshold:');
                        disp(setpointPitchTemp);   
                        
                        if strcmp(pidModeStrategy,'1') 
                            set(pidKpSlider,'Value',aggPitchKp);
                            set(pidKdSlider,'Value',aggPitchKd);
                            set(pidKiSlider,'Value',aggPitchKi);
                            set(referencePIDTxt,'String',setpointPitchTemp);
                        end
                    end
                    if type == 11 
                        % Pid Yaw CONS
                        disp('Pid Yaw Cons');
                        consYawKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        consYawKp = consYawKpTemp/100;
                        disp(consYawKp);

                        consYawKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        consYawKd = consYawKdTemp/100;
                        disp(consYawKd);                             

                        consYawKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        consYawKi = consYawKiTemp/100;
                        disp(consYawKi);    

                        setpointYawTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('threshold:');
                        disp(setpointYawTemp);  
                        
                        if strcmp(pidModeStrategy,'0')
                            set(pidKpSlider,'Value',consYawKp);
                            set(pidKpSlider,'Value',consYawKd);
                            set(pidKpSlider,'Value',consYawKi);
                            set(referencePIDTxt,'String',setpointYawTemp);
                        end
                    end                   
                    if type == 15 
                        % Pid Yaw AGG
                        disp('Pid Yaw AGG');
                        aggYawKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        aggYawKp = aggYawKpTemp/100;
                        disp(aggYawKp);

                        aggYawKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        aggYawKd = aggYawKdTemp/100;
                        disp(aggYawKd);                             

                        aggYawKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        aggYawKi = aggYawKiTemp/100;
                        disp(aggYawKiTemp/100);    

                        setpointYawTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('val4:');
                        disp(setpointYawTemp);   
                        
                        if strcmp(pidModeStrategy,'1') 
                            set(pidKpSlider,'Value',aggYawKp);
                            set(pidKdSlider,'Value',aggYawKd);
                            set(pidKiSlider,'Value',aggYawKi);
                            set(referencePIDTxt,'String',setpointYawTemp);
                        end
                    end
                    if type == 12
                        % Pid Alt CONS
                        disp('Pid Alt Cons');
                        consAltKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        consAltKp = consAltKpTemp/100;
                        disp(consAltKp);

                        consAltKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        consAltKd = consAltKdTemp/100;
                        disp(consAltKd);                             

                        consAltKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        consAltKi = consAltKiTemp/100;
                        disp(consAltKi);    

                        setpointAltTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('val4:');
                        disp(setpointAltTemp);   
                        
                        if strcmp(pidModeStrategy,'0')
                            set(pidKpSlider,'Value',consAltKp);
                            set(pidKpSlider,'Value',consAltKd);
                            set(pidKpSlider,'Value',consAltKi);
                            set(referencePIDTxt,'String',setpointAltTemp);
                        end
                    end                   
                    if type == 16 
                        % Pid Alt AGG
                        disp('Pid Alt AGG');
                        aggAltKpTemp = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('val1:');
                        aggAltKp = aggAltKpTemp/100;
                        disp(aggAltKp);

                        aggAltKdTemp = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                        disp('val2:');
                        aggAltKd = aggAltKdTemp/100;
                        disp(aggAltKd);                             

                        aggAltKiTemp = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                        disp('val3:');
                        aggAltKi = aggAltKiTemp/100;
                        disp(aggAltKi);    

                        setpointAltTemp = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                        disp('val4:');
                        disp(setpointAltTemp); 
                        
                        if strcmp(pidModeStrategy,'1') 
                            set(pidKpSlider,'Value',aggAltKp);
                            set(pidKdSlider,'Value',aggAltKd);
                            set(pidKiSlider,'Value',aggAltKi);
                            set(referencePIDTxt,'String',setpointAltTemp);
                        end
                    end
                    if type == takeOffID
                        % Take Off Ack
                        takeOffAck = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('Take Off Ack');
                        disp(takeOffAck);
                        if takeOffAck == 1
                            set(takeOffBtn,'String','Flying');
                            landAck = 0;
                            disp('Changed landAck:');
                            disp(landAck);
                            set(landBtn,'String','Land');
                        else                            
                            set(takeOffBtn,'String','Take Off');
                        end
                    end                    
                    if type == iHoverID
                        % Hovering Ack
                        hoverAck = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('Hovering Ack');
                        disp(hoverAck);
                        if hoverAck == 1
                            set(hoverBtn,'String','NoPid');
                            disp('Pid enabled');
                        else                            
                            set(hoverBtn,'String','iHoverPid');
                            disp('Unsafe hovering OR Landed');
                        end
                    end                    
                    if type == landID
                        % Landed Ack
                        landAck = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                        disp('Landed Ack');
                        disp(landAck);
                        if landAck == 1                           
                            set(landBtn,'String','Landed'); 
                            set(takeOffBtn,'String','Take Off');
                            takeOffAck = 0;
                        end
                        break;
                    end
            end
%             while (get(xbee, 'BytesAvailable')~=0 && tenzo == true)
%                 % read until terminator
%                 sentence = fscanf( xbee, '%s') % this reads in as a string (until a terminater is reached)
%                 if (strcmp(sentence(1,1),'R'))
%                     %decodes "sentence" seperated (delimted) by commaseck Unit')
%                     [Roll, theta, Pitch, pitch, Yaw, yaw, KalmanRoll, kr, KalmanPitch, kp, OmegaX, wx, OmegaY, wy, OmegaZ, wz, AccX, ax, AccY, ay, AccZ, az, Motor, omegaR] = strread(sentence,'%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f',1,'delimiter',',');
% 
%                     % get motors speed value
%                     omega = omegaR;
%                     set(m1Txt,'String',omega); 
%                     set(m2Txt,'String',omega); 
%                     set(m3Txt,'String',omega); 
%                     set(m4Txt,'String',omega); 
%                     
% %                     % Gets gyro data                    
% %                     gxdata = [ gxdata(2:end) ; wx ];
% %                     gydata = [ gydata(2:end) ; wy ];
% %                     gzdata = [ gzdata(2:end) ; wz ];
%                     
%                     if gyrosco == true
%                         % Filters gyro data and stores them
%                         gxFilt = (1 - alpha)*gxFilt + alpha*wx;
%                         gyFilt = (1 - alpha)*gyFilt + alpha*wy;
%                         gzFilt = (1 - alpha)*gzFilt + alpha*wz;
% 
%                         gxFdata = [ gxFdata(2:end) ; gxFilt ];
%                         gyFdata = [ gyFdata(2:end) ; gyFilt ];
%                         gzFdata = [ gzFdata(2:end) ; gzFilt ];
%                     end
%                     
%                     if magneto == true
%                         % Gets Magnetometer and Estimated angles
%                         if (theta>90)
%                             theta = theta - 360;
%                         end
%                         if (pitch > 90)
%                             pitch =  pitch - 360;
%                         end 
% 
%                         % Apply noise filtering
%                         % Uncomment for noise filtering
%     %                     TFilt = (1 - alpha)*TFilt + alpha*theta;
%     %                     PFilt = (1 - alpha)*PFilt + alpha*pitch;
%     %                     YFilt = (1 - alpha)*YFilt + alpha*yaw;
% 
%     %                     Rdata = [ Rdata(2:end) ; TFilt ];
%     %                     Pdata = [ Pdata(2:end) ; PFilt ];
%     %                     Ydata = [ Ydata(2:end) ; YFilt ]; 
% 
%                         Rdata = [ Rdata(2:end) ; theta ];
%                         Pdata = [ Pdata(2:end) ; pitch ];
%                         Ydata = [ Ydata(2:end) ; yaw ]; 
% 
%                         TFilt = (1 - alpha)*TFilt + alpha*kr;
%                         PFilt = (1 - alpha)*PFilt + alpha*kp;
% 
%                         EKXdata = [ EKXdata(2:end) ; TFilt ];
%                         EKYdata = [ EKYdata(2:end) ; PFilt ];
%                     end
%                     
%                     if accelero == true
%                         % Gets Accelerometer data
%                         axdata = [ axdata(2:end) ; ax ];
%                         aydata = [ aydata(2:end) ; ay ];
%                         azdata = [ azdata(2:end) ; az ];  
%                     end
%                 end
%             end
            end %Message delivered from Arduino
       end
    end  
end