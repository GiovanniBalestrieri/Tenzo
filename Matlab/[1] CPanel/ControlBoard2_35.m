function ControlBoard()
delete(timerfindall);
clear all;
clc;

global xbee;
global portWin;
global portUnix;
global xbeeBR;
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
global tenzoStateID;
global connID;

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
global conAck;

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

% Pid Tuning
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

% OMEga

global consRollKpW;
global consRollKdW;
global consRollKiW
global consPitchKpW;
global consPitchKdW;
global consPitchKiW;
global consYawKpW;
global consYawKdW;
global consYawKiW;



% fine

global rt;
global firing;
global recording;
global plotting;
global asked;
global axdata;
global aydata;
global azdata;
global axdataDa;
global aydataDa;
global azdataDa;
global AccXDa;
global AccYDa;
global AccZDa;
global time;
global serialFlag;
global samplesNumMax;
global timerArduino;
global timerConnect;
global requestPending;
global serial1;
global serial0;
global serial2;
global tenzoConnectionRequested;
global accTag;
global timerTag;
global gyroTag;
global footerTag;
global estTag; 
global throttleTag;

% pid tag
global yawConsTagW;
global yawConsTag;
global rollConsTagW;
global rollConsTag;
global pitchConsTagW;
global pitchConsTag;
global yawAggTag;
global pitchAggTag;
global rollAggTag;
global accFilterTag;
global pidToggleTag;
global pidEnableTag;
global pidDisableTag;
global stringPidToSend;
global takeOffAckTag;
global landAckTag;

stringPidToSend = '';

global timerSamples;
% Version

version = 2.35;

%% Serial Protocol 2 - Bluetooth

accTag = 'a';
gyroTag = 'o';
timerTag = 't';
footerTag = 'z';
throttleTag = 'm';
estTag = 'e';
pidToggleTag = 'c';
pidEnableTag = 'e';
pidDisableTag = 'd';
accFilterTag = 'f';
takeOffAckTag = 'i';
landAckTag = 'L';

% dynamic pid 
rollConsTag = 'rc';
rollAggTag = 'ra';
rollConsTagW = 'rw';
pitchConsTag = 'pc';
pitchAggTag = 'pa';
pitchConsTagW = 'pw';
yawConsTag = 'yc';
yawAggTag = 'ya';
yawConsTagW = 'yw';


%% Data Acquisition vars

samplesNumMax = 1000;
acceleration.s = 0;
ax=0;
ay=0;
az=0;
t=0;
serialFlag=0;
prevTimer = 0;
deltaT = 0;
contSamples = 0;
    

buffLenDa = 500;
longBuffLen = 1000;
indexDa=1:buffLenDa;
asked = false;
rt = false;
plotting = false;
receiving = false;
requestPending = false;
firing = false;

AccX = zeros(buffLenDa,1);
AccY = zeros(buffLenDa,1);
AccZ = zeros(buffLenDa,1);
axdataDa = zeros(buffLenDa,1);
aydataDa = zeros(buffLenDa,1);
azdataDa = zeros(buffLenDa,1);
time  = zeros(buffLenDa,1);

% Serial protocol
serial1 = false;
serial0 = false;
serial2 = true;

versionProtocol = 5;
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
enableMotorsID = 20;
rCRPID = 21;
rCPPID = 22;
rCYPID = 23;
rCAPID = 24;
rARPID = 25;
rAPPID = 26;
rAYPID = 27;
rAAPID = 28;
tenzoStateID = 30;
connID = 31;
accValuesID = 32;

cmdLength = 17;
headerLength = 13;
arduinoAdd = 1;
matlabAdd = 2;
portWin = 'Com3';
portUnix = '/dev/rfcomm0';
xbeeBR = 115200;
% buffer size should be the same as the one specified on the Arduino side
inputBuffSize = 47+1;
outputBuffSize = 31;
terminator = 'CR';
tag = 'Quad';

gyroTimer = timer('ExecutionMode','FixedRate','Period',0.01,'TimerFcn',{@graphGyro});

global angleTimer;
angleTimer = timer('ExecutionMode','FixedRate','Period',0.02,'TimerFcn',{@graphAngles});
                          
global accTimer;
accTimer = timer('ExecutionMode','FixedRate','Period',0.01,'TimerFcn',{@graphAcc});
          
% Complementary and Kalman Filert values
global KalmanRoll;
global kr;
global KalmanPitch;
global kp;
landingSpeed = 2;

% Serial Sensors Acks initialization
accReceived = false;
gyroReceived = false;
magnReceived = false;
estReceived = false;

% Data aquisiton boolean vars
rt = false;
recording = false;
firing = false;
plotting = false;

doubleAnglePlot = false;

% Data acquisition variables

contSamples = 0;
ax = 0;
ay = 0;
az = 0;
time = 0;

%% Vocal settings

% Disable vocal acks: speakCmd = false
% Set default voice: voice = it
% Set verbosity level: vocalVerb: 1 = max -> n = min;
speakCmd = false;
uk = 'Microsoft Hazel Desktop - English (Great Britain)';
it = 'Microsoft Elsa Desktop - Italian (Italy)';
us = 'Microsoft Zira Desktop - English (United States)';
voice = uk;
vocalVerb = 2;
% Plot variables

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
rt = false;

pidStrategy ='U';
pidModeStrategy = 'U';
pidRead = 0;

% variables declaration

takeOffAck = 0;
hoverAck = -7;
landAck = 1;
defaultAlt = 1;
% used to store long/short num to arrays
weights2 = 2.^([7:-1:0]);

bufferSend = zeros(1, outputBuffSize);
  
disp('Welcome to the CPanel');
% Delete all serial connections
delete(instrfindall)

    %% create tabbed GUI
    handles.hFig = figure('Menubar','none');
    s = warning('off', 'MATLAB:uitabgroup:OldVersion');
    
    hTabGroup = uitabgroup('Parent',handles.hFig);
    warning(s);
    hTabs(1) = uitab('Parent',hTabGroup, 'Title','Home');
    hTabs(2) = uitab('Parent',hTabGroup, 'Title','Motors');
    hTabs(3) = uitab('Parent',hTabGroup, 'Title','Sensors');
    hTabs(4) = uitab('Parent',hTabGroup, 'Title','Control');
    hTabs(5) = uitab('Parent',hTabGroup, 'Title','Acquisition');
    set(hTabGroup, 'SelectedTab',hTabs(1));
    Listener = addlistener(hTabGroup,'SelectedTab','PostSet',@tabGroupCallBack);
    
%     if get(hTabGroup,'SelectedTab') == hTabs(1)
%         disp('Tab1 selected');
%     end
%     if get(hTabGroup,'SelectedTab') == hTabs(2)
%         disp('Motors selected');
%     end
%     if get(hTabGroup,'SelectedTab') == hTabs(3)
%         disp('Tab1 selected');
%     end


    function tabGroupCallBack(~,~)
        %# Set visibility's slider to OFF
        val = get(hTabGroup,'SelectedTab');
        if val ~= hTabs(4)
        %disp('Tab1 selected');
            set(handles.pidKiSlider,'Visible','off');
            set(handles.pidKpSlider,'Visible','off');
            set(handles.pidKdSlider,'Visible','off');
            disp(pidStrategy);
        end
        if val == hTabs(4) 
            if get(pidRad,'Value') == 1
                %disp('Tab1 selected');
                set(handles.pidKiSlider,'Visible','on');
                set(handles.pidKpSlider,'Visible','on');
                set(handles.pidKdSlider,'Visible','on');
            end
        end
        if val == hTabs(5)
            disp('Usb cable required!');
            warndlg('Usb Cable required to acquire data','Attention');
        end
    end

    % Data Acquisition panel

    if (~exist('handles.plot','var'))
        handles.record = uicontrol('Style','togglebutton', 'String','Record', ...
            'Position', [110 90 120 30],...
            'Parent',hTabs(5),'Callback',@recordCallback);
    end
    
    if (~exist('handles.plot','var'))
        handles.realTime = uicontrol('Style','togglebutton', 'String','Real time', ...
            'Position', [310 90 120 30],...
            'Parent',hTabs(5),'Callback',@rtCallback);
    end
    
    if (~exist('handles.start','var'))
        handles.start = uicontrol('Style','pushbutton', 'String','Start', ...
            'Position', [110 20 120 30],...
            'Parent',hTabs(5), 'Callback',@startCallback);
    end
    
    if (~exist('handles.stop','var'))
        handles.stop = uicontrol('Style','pushbutton', 'String','Stop', ...
            'Position', [310 20 120 30],...
            'Parent',hTabs(5), 'Callback',@stopCallback);
    end
    
    %% Home UI components
    uicontrol('Style','text', 'String','Tenzo Control Panel', ...
        'Position', [55 310 420 50],...
        'Parent',hTabs(1), 'FontSize',20,'FontWeight','bold');
    
    uicontrol('Style','text', 'String',version, ...
        'Position', [55 268 420 50],...
        'Parent',hTabs(1), 'FontSize',10,'FontWeight','normal');
    
    handles.takeOffBtn = uicontrol('Style','pushbutton', 'String','Take Off', ...
        'Position', [70 210 120 30],...
        'Parent',hTabs(1), 'Callback',@takeOffCallback);
    
    handles.hoverBtn = uicontrol('Style','pushbutton', 'String','iHover', ...
        'Position', [210 210 120 30],...
        'Parent',hTabs(1), 'Callback',@hoverCallback);
    
    handles.landBtn = uicontrol('Style','pushbutton', 'String','Land', ...
        'Position', [350 210 120 30],...
        'Parent',hTabs(1), 'Callback',@landCallback);
    
    handles.conTxt = uicontrol('Style','text', 'String','Offline','ForegroundColor',[.99 .183 0.09], ...
        'Position', [70 20 100 30],...
        'Parent',hTabs(1), 'FontSize',13,'FontWeight','bold');
    
    handles.connect = uicontrol('Style','togglebutton', 'String','Connect', ...
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
    
    %% Create the button group.
    
    controlGroup = uibuttongroup('Parent', hTabs(4),'visible','off','Position',[0 0 .2 1]);
    
    uicontrol('Style','text', 'String','Select Control', ...
        'Position', [2 340 80 30],...
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
        'Parent',hTabs(4), 'Position',[ 221 320 50 50 ]);
    
    handles.referencePIDVal = uicontrol('Style','edit', 'String','0', ...
        'Position', [226 325 40 40],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',13,'FontWeight','normal');
    
    referencePIDTxt = uicontrol('Style','text', 'String','Reference', ...
        'Position', [135 325 80 40],'Visible','off',...
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
    
    handles.pidKpVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 220 40 15],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',10,'FontWeight','normal');
    
    framePidKdVal = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 480 140 50 30 ]);
    
    handles.pidKdVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 146 40 15],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',10,'FontWeight','normal');
    
    framePidKiVal = uicontrol('Style','frame','Visible','off', ...
        'Parent',hTabs(4), 'Position',[ 480 53 50 30 ]);
    
    handles.pidKiVal = uicontrol('Style','text', 'String','AS', ...
        'Position', [484 58 40 15],'Visible','off',...
        'Parent',hTabs(4), 'FontSize',10,'FontWeight','normal');
    
    handles.pidKpSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',1,'Callback',@(s,e) disp('KpSlider'),...
    'SliderStep',[0.01 0.05],'Position', [140 185 350 20]);
    KpListener = addlistener(handles.pidKpSlider,'Value','PostSet',@pidKpSliderCallBack);
    
    handles.pidKdSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',0.5,'Callback',@(s,e) disp('KdSlider'),...
    'SliderStep',[0.01 0.05],'Position', [140 110 350 20]);
    KdListener = addlistener(handles.pidKdSlider,'Value','PostSet',@pidKdSliderCallBack);
    
    handles.pidKiSlider = uicontrol('Style','slider','Visible','off',...
    'min',0,'max',0.5,'Callback',@(s,e) disp('KiSlider'),...
    'SliderStep',[0.01 0.05],'Position',[140 30 350 20]);
    KiListener = addlistener(handles.pidKiSlider,'Value','PostSet',@pidKiSliderCallBack);
    
    handles.pidKpTxt = uicontrol('Style','text','Visible','off',...
        'String','Proportional: Kp','Position', [140 190 150 50],...
        'Parent', hTabs(4), 'FontSize',11);
    
    handles.pidKdTxt = uicontrol('Style','text','Visible','off',...
        'String','Derivative: Kd','Position', [140 115 150 50],...
        'Parent', hTabs(4), 'FontSize',11);
    
    handles.pidKiTxt = uicontrol('Style','text','Visible','off',...
        'String','Integral: Ki','Position', [140 37 150 50],...
        'Parent', hTabs(4), 'FontSize',11);    
    
    function pidKpSliderCallBack(src,eventData)
       set(handles.pidKpVal,'String',get(handles.pidKpSlider,'Value')); 
       if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
           
       end
    end

    function pidKdSliderCallBack(src,eventData)
       set(handles.pidKdVal,'String',get(handles.pidKdSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',1,', ...
%            num2str(get(handles.pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%       end
    end

    function pidKiSliderCallBack(src,eventData)
       set(handles.pidKiVal,'String',get(handles.pidKiSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',2,', ...
%            num2str(get(handles.pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
    end
    
    %% Motors Ui Components
    
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
    
     %% Deprecated 
%     connectMotors = uicontrol('Style','togglebutton', 'String','Communicate', ...
%         'Position', [400 20 120 30],'BackgroundColor',[.21 .96 .07],...
%         'Parent',hTabs(2), 'Callback',@connectionMotors);   
    
    %%
    
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
        'Position', [316 311 30 35],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 140 175 50 50 ]);
    
    m2Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [154 178 30 35],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 305 30 50 50 ]);
    
    m3Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [316 34 30 35],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    uicontrol('Style','frame', ...
        'Parent',hTabs(2), 'Position',[ 480 175 50 50 ]);
    
    m4Txt = uicontrol('Style','text', 'String','0', ...
        'Position', [493 178 30 35],...
        'Parent',hTabs(2), 'FontSize',13,'FontWeight','normal');
    
    %% Sensors Tab UI Components
    
    uicontrol('Style','popupmenu','Position', [370 360 150 30],... 
        'String','Select|Gyro|Accelerometer|Angles (est)', ...
        'Parent',hTabs(3), 'Callback',@popupCallback);
    
    guidata(handles.hFig,handles);
    
    % Start Acquisition
    if (~exist('handles.start','var'))
        handles.sensorStart = uicontrol('Style','pushbutton', 'String','Rec', ...
            'Position', [20 320 30 30],...
            'Parent',hTabs(3), 'Callback',@startSensorCallback);
    end
    
    %Stop acquisition
    if (~exist('handles.stop','var'))
        handles.stopSensor = uicontrol('Style','pushbutton', 'String','Stp', ...
            'Position', [20 270 30 30],...
            'Parent',hTabs(3), 'Callback',@stopSensorCallback);
    end
    
     % Show Filter value
    if (~exist('handles.filterVal','var'))
        handles.filterVal = uicontrol('Style','text', 'String','Fac', ...
            'Position', [20 230 30 20],...
            'Parent',hTabs(3), 'FontSize',10,'FontWeight','normal');
    end
    
    %Change Filter Acc ++
    if (~exist('handles.filterAccIncrease','var'))
        handles.filterAccIncrease = uicontrol('Style','pushbutton', 'String','FAc', ...
            'Position', [20 190 30 30],...
            'Parent',hTabs(3), 'Callback',@filterAccIncCallback);
    end
    
    
    %Change Filter Acc --
    if (~exist('handles.filterAccDecrease','var'))
        handles.filterAccDecrease = uicontrol('Style','pushbutton', 'String','FAd', ...
            'Position', [20 140 30 30],...
            'Parent',hTabs(3), 'Callback',@filterAccDecCallback);
    end
    
    % Show throttle speed
    if (~exist('handles.throttleVal','var'))
        handles.throttleVal = uicontrol('Style','text', 'String','throttle', ...
            'Position', [20 100 30 20],...
            'Parent',hTabs(3), 'FontSize',10,'FontWeight','normal');
    end
    
    %Increase throttle speed
    if (~exist('handles.upMotors','var'))
        handles.upMotors = uicontrol('Style','pushbutton', 'String','UP', ...
            'Position', [20 60 30 30],...
            'Parent',hTabs(3), 'Callback',@upMotorsCallback);
    end
    
    % Decrease throttle speed
    if (~exist('handles.downMotors','var'))
        handles.downMotors = uicontrol('Style','pushbutton', 'String','DW', ...
            'Position', [20 20 30 30],...
            'Parent',hTabs(3), 'Callback',@downMotorsCallback);
    end
    
    %% Sensor Tab Callbacks
    
    
    function stopSensorCallback(obj,event,handles)
        asked = ~asked;
        disp('Saving records to file');
        timerSamples = 0;
    end
    
    function startSensorCallback(obj,event,h)
        disp('Recording ...');
        asked = ~asked;
        asked
        
        if asked == true
            %timerSamples = 0
          
            if serialFlag == 0 
                %timerArduino = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',{@storeSerial});    
                %timerConnect = timer('ExecutionMode','fixedRate','Period',0.01,'TimerFcn',{@requestData});%,'StopFcn',{@stoppedCon});    
                %start(timerConnect);
                %[acceleration.s] = setupSerial();
                
            else
                disp('serial flag = 1 and asked = true');
            end
        else 
            if serialFlag == 1 
                receiving = false;
                recording = false;
                set(handles.start,'String','Stp');
                serialFlag = 0;
            else 
                disp('serial1 0');
            end
        end
        %disp(abs(az));
        disp('[verbose] serialFlag:');
        serialFlag;
    end

    function upMotorsCallback(obj,event,h)
        disp('UP ...');
        cmd = 'q';
        sendNMess(cmd);
    end

    function filterAccIncCallback(obj,event,h)
        disp('alphaA ++');
        cmd = 'w';
        sendNMess(cmd);
    end

    function filterAccDecCallback(obj,event,h)
        disp('alphaA --');
        cmd = 's';
        sendNMess(cmd);
    end

    function downMotorsCallback(obj,event,h)
        disp('DOWN ...');
        cmd = 'a';
        sendNMess(cmd);
    end
    
    %% Data acquisition panel Callback
    
    function startCallback(obj,event,h)
        disp('start pressed');
        asked = ~asked;
        delete(timerfindall);
        if asked == true
            if serialFlag == 0 
                % Activate timers TODO
                [acceleration.s] = setupSerial();
            end
        else 
            if serialFlag == 1 
                receiving = false;
                recording = false;
                set(handles.start,'String','Start');
                serialFlag = 0;
            end
        end
        %disp(abs(az));
        serialFlag;
    end
    
    function [s] = setupSerial()
        %comPortWin = 'COM4';
        comPort = portUnix;
        oldSerial = instrfind('Port', comPort); 
        % if the set of such objects is not(~) empty
        if (~isempty(oldSerial))  
            disp('WARNING:  Port in use.  Closing.')
            delete(oldSerial)
        end
        s = serial(comPort);

        % Max wait time
        set(s, 'TimeOut', 5); 
        % set(s,'terminator','CR');
        set(s,'BaudRate',xbeeBR);
        fopen(s);

        disp('Sending Request.');
        acceleration.s = s;
        timerArduino = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',{@storeSerial});    
        timerConnect = timer('ExecutionMode','fixedRate','Period',5,'TimerFcn',{@connectSerial});%,'StopFcn',{@stoppedCon});    
        try 
            start(timerConnect);
        catch
            disp '******** InstrumentSubscription ERROR *********'
            disp (exception.message);
            disp '***********************************************'
        end
    end


    function connectSerial(obj,event,h)
        if serialFlag == 0 && requestPending == false
            if isvalid(acceleration.s) == 1
                fwrite(acceleration.s,16); 
            end
            if (strcmp(get(timerArduino,'Running'),'off'))
                try
                    start(timerArduino);    
                catch
                    disp '******** InstrumentSubscription ERROR *********'
                    disp (exception.message);
                    disp '***********************************************'
                end
            end
        end
    end

    function stoppedCon(obj,event,h)       
        serialFlag = 1;
        requestPending = false;
        stop(timerArduino);   
        stop(timerConnect);      
        set(handles.start,'String','Stop');
        disp('Connection established');
    end

    function storeSerial(obj,event,handles)
        while (get(acceleration.s, 'BytesAvailable')==1)
            [mess,cont] = fread(acceleration.s);
            %disp('Received bytes:');
            %disp(cont);
            %disp(mess);
            if (mess == 17)             
                %display(['Collecting data']);
                fwrite(acceleration.s,18); 
                requestPending = true;   
            elseif (mess == 19)  
                stoppedCon();
            end
        end
    end

    function stopCallback(obj,event,handles)
        if (~exist('serialFlag','var')) 
            disp('disconnect');
        end 
    end 

     function recordCallback(obj,event,handles)
     end
     
     function recordCallback1(obj,event,handles)
     end
     
     function rtCallback(obj,event,h)
     end
    
    
    %% Motors Up btn callbacks
    function upCallback(src,eventData)
       %fprintf(xbee,'w');
       if tenzo == true
         % Initialize the cmd array
         cmd = zeros(8,4,'uint8');
         cmd(1,1) = uint8(motorsID);
         % Sends 1 to activate PID
         bits = reshape(bitget(5 + 100,32:-1:1),8,[]);
         cmd(2,:) = weights2*bits;
         sendMess(cmd);
       end
    end

%% Deprecated 

    function connectionMotors(src,eventData) 
       %fprintf(xbee,'w');
       if tenzo == true           
            if get(connectMotors,'Value') == 1
                 % Initialize the cmd array
                 cmd = zeros(8,4,'uint8');
                 cmd(1,1) = uint8(enableMotorsID);
                 % Sends 1 to activate PID
                 bits = reshape(bitget(1,32:-1:1),8,[]);
                 cmd(2,:) = weights2*bits;
                 sendMess(cmd);
            elseif get(connectMotors,'Value') == 0
                % Initialize the cmd array
                 cmd = zeros(8,4,'uint8');
                 cmd(1,1) = uint8(enableMotorsID);
                 % Sends 1 to activate PID
                 bits = reshape(bitget(0,32:-1:1),8,[]);
                 cmd(2,:) = weights2*bits;
                 sendMess(cmd);
            end
       else
           warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!');
       end
    end

%%
 
    function up1Callback(src,eventData)
       %fprintf(xbee,'E');
       if tenzo == true
         % Initialize the cmd array
         cmd = zeros(8,4,'uint8');
         cmd(1,1) = uint8(motorsID);
         bits = reshape(bitget((100 + 2),32:-1:1),8,[]);
         cmd(2,:) = weights2*bits;
         sendMess(cmd);
       end
    end

    function downCallback(src,eventData)
       %fprintf(xbee,'s');
       if tenzo == true
         % Initialize the cmd array
         cmd = zeros(8,4,'uint8');
         cmd(1,1) = uint8(motorsID);
         bits = reshape(bitget((100 - 5),32:-1:1),8,[]);
         cmd(2,:) = weights2*bits;
         sendMess(cmd);
       end
    end

    function down1Callback(src,eventData)
       %fprintf(xbee,'D');
       if tenzo == true
         % Initialize the cmd array
         cmd = zeros(8,4,'uint8');
         cmd(1,1) = uint8(motorsID);
         bits = reshape(bitget((100 - 2),32:-1:1),8,[]);
         cmd(2,:) = weights2*bits;
         sendMess(cmd);
       end
    end

    function initializeCallback(src,eventData)
       %fprintf(xbee,'i');
       if tenzo == true
            if takeOffAck == 0
%                 % Initialize the cmd array
%                 cmd = zeros(8,4,'uint8');
%                 cmd(1,1) = uint8(takeOffID);
%                 %cmd(2,4) = uint8(defaultAlt);
%                 bits = reshape(bitget(defaultAlt,32:-1:1),8,[]);
%                 cmd(2,:) = weights2*bits;
                cmd = 'i';
                sendMess(cmd);
                if speakCmd && vocalVerb>=1 
                        %tts('Decollo programmato',voice);
                        tts('Starting take off protocol.',voice);
                end
            else
               warndlg('Tenzo already out in space. Connection blocked. Protocol 1','!! DANGER !!') 
               % you can start take off protocol automatically
            end  
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function resetCallback(src,eventData)
       if tenzo == true
            if takeOffAck == 1
                % Initialize the cmd array
%                 cmd = zeros(8,4,'uint8');
%                 cmd(1,1) = uint8(landID);
%                 %cmd(2,4) = uint8(defaultAlt);
%                 bits = reshape(bitget(landingSpeed,32:-1:1),8,[]);
%                 cmd(2,:) = weights2*bits;
                cmd = 'r';
                sendMess(cmd);
                % wait for feedback from Tenzo and change state of btn
                if speakCmd && vocalVerb>=1 
                        %tts('atterraggio programmato',voice);
                        tts('Starting land protocol.',voice);
                end
            else
               warndlg('Tenzo is not in hovering mode. First Take Off then try again. ','!! Warning !!') 
               % you can start take off protocol automatically
            end  
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function landCallback(src,eventData)
        if tenzo == true
            if takeOffAck == 1
                % Initialize the cmd array
%                 cmd = zeros(8,4,'uint8');
%                 cmd(1,1) = uint8(landID);
%                 %cmd(2,4) = uint8(defaultAlt);
%                 bits = reshape(bitget(landingSpeed,32:-1:1),8,[]);
%                 cmd(2,:) = weights2*bits;
                cmd = 'L';
                sendMess(cmd);
                % wait for feedback from Tenzo and change state of btn
                if speakCmd && vocalVerb>=1 
                        %tts('atterraggio programmato',voice);
                        tts('Starting land protocol.',voice);
                end
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
%                 % Initialize the cmd array
%                 cmd = zeros(8,4,'uint8');
%                 cmd(1,1) = uint8(takeOffID);
%                 %cmd(2,4) = uint8(defaultAlt);
%                 bits = reshape(bitget(defaultAlt,32:-1:1),8,[]);
%                 cmd(2,:) = weights2*bits;
                cmd = 'i';
                sendNMess(cmd);
                if speakCmd && vocalVerb>=1 
                        %tts('Decollo programmato',voice);
                        tts('Starting take off protocol.',voice);
                end
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
                    if speakCmd && vocalVerb>=2 
                            %tts('Abilitazione controllore pid',voice);
                            tts('Enabling pid.',voice);
                    end
                    disp('Enabling PID'); 
                    warndlg('Enabling PID. Safe flight.','Report') 
                    % Initialize the cmd array
%                     cmd = zeros(8,4,'uint8');
%                     cmd(1,1) = uint8(iHoverID);
%                     % Sends 1 to activate PID
%                     bits = reshape(bitget(1,32:-1:1),8,[]);
%                     cmd(2,:) = weights2*bits;
                    cmd = 'p';
                    sendNMess(cmd);
                    % wait for feedback from Tenzo and change state of btn
                else
                    if speakCmd && vocalVerb>=2 
                            %tts('Attenzione. Disabilitazione controllore pid.',voice);
                            tts('Warning. Disabling PID.',voice);
                    end
                    disp('Desactivating PID');
                    %warndlg('Desactivating PID','!! Warning !!') 
                   % Initialize the cmd array
%                    cmd = zeros(8,4,'uint8');
%                    cmd(1,1) = uint8(iHoverID);
%                     % Sends 0 to disable PID
%                    bits = reshape(bitget(0,32:-1:1),8,[]);
%                    cmd(2,:) = weights2*bits;
                   cmd = 'p';
                   sendNMess(cmd); 
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
       if tenzo == true
        % fprintf(xbee,'1');
         % Initialize the cmd array
        cmd = zeros(8,4,'uint8');
        cmd(1,1) = uint8(motorsID);
        % Sends 1 to activate PID
        bits = reshape(bitget(10 + 100,32:-1:1),8,[]);
        cmd(2,:) = weights2*bits;
        sendMess(cmd);
       end
    end
    
    function testM2Callback(src,eventData)
       if tenzo == true
        % fprintf(xbee,'2');
         % Initialize the cmd array
        cmd = zeros(8,4,'uint8');
        cmd(1,1) = uint8(motorsID);
        % Sends 1 to activate PID
        bits = reshape(bitget(10 + 100,32:-1:1),8,[]);
        cmd(3,:) = weights2*bits;
        sendMess(cmd);
       end
    end

    function testM3Callback(src,eventData)
       if tenzo == true
        % fprintf(xbee,'3');
         % Initialize the cmd array
        cmd = zeros(8,4,'uint8');
        cmd(1,1) = uint8(motorsID);
        % Sends 1 to activate PID
        bits = reshape(bitget(10 + 100,32:-1:1),8,[]);
        cmd(4,:) = weights2*bits;
        sendMess(cmd);
       end
    end

    function testM4Callback(src,eventData)
       if tenzo == true
        % fprintf(xbee,'4');
         % Initialize the cmd array
        cmd = zeros(8,4,'uint8');
        cmd(1,1) = uint8(motorsID);
        % Sends 1 to activate PID
        bits = reshape(bitget(10 +100,32:-1:1),8,[]);
        cmd(5,:) = weights2*bits;
        sendMess(cmd);
       end 
    end

    

%    function pidKpSliderCallBack(src,eventData)
%        set(handles.pidKpVal,'String',get(handles.pidKpSlider,'Value')); 
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%            % Send 'X,opt1,opt2,opt3,val,X'
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
%            num2str(get(handles.pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
%     end
% 
%     function pidKdSliderCallBack(src,eventData)
%        set(handles.pidKdVal,'String',get(handles.pidKdSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',1,', ...
%            num2str(get(handles.pidKpSlider,'Value')),',X']
%        fprintf(xbee,'%s',strindToSend,'sync'); 
%        end
%     end
% 
%     function pidKiSliderCallBack(src,eventData)
%        set(handles.pidKiVal,'String',get(handles.pidKiSlider,'Value'));
%        if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
%        strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',2,', ...
%            num2str(get(handles.pidKpSlider,'Value')),',X']
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
            set(handles.referencePIDVal,'Visible','on');
            set(referencePIDTxt,'Visible','on');
            set(sendPidValsBtn,'Visible','on'); 
            set(readPidValsBtn,'Visible','on'); 
            set(pidModePopup,'Visible','on');
            set(framePidKpVal,'Visible','on');
            set(handles.pidKpVal,'Visible','on');
            set(handles.pidKpTxt,'Visible','on');
            set(handles.pidKpSlider,'Visible','on');
            set(framePidKdVal,'Visible','on');
            set(handles.pidKdVal,'Visible','on');
            set(handles.pidKdTxt,'Visible','on');
            set(handles.pidKdSlider,'Visible','on');
            set(framePidKiVal,'Visible','on');
            set(handles.pidKiVal,'Visible','on');
            set(handles.pidKiTxt,'Visible','on');
            set(handles.pidKiSlider,'Visible','on');
            set(pidPopup,'Visible','on');             
        else                        
            set(frameThreshold,'Visible','off');
            set(referencePIDTxt,'Visible','off');
            set(handles.referencePIDVal,'Visible','off');
            set(sendPidValsBtn,'Visible','off');
            set(readPidValsBtn,'Visible','off');
            set(welcomeControl,'Visible','off');
            set(workInProgress,'Visible','off'); 
            set(pidModePopup,'Visible','off');
            set(framePidKpVal,'Visible','off');
            set(handles.pidKpVal,'Visible','off');
            set(handles.pidKpTxt,'Visible','off');
            set(handles.pidKpSlider,'Visible','off');
            set(framePidKdVal,'Visible','off');
            set(handles.pidKdVal,'Visible','off');
            set(handles.pidKdTxt,'Visible','off');
            set(handles.pidKdSlider,'Visible','off');
            set(framePidKiVal,'Visible','off');
            set(handles.pidKiVal,'Visible','off');
            set(handles.pidKiTxt,'Visible','off');
            set(handles.pidKiSlider,'Visible','off');
            set(pidPopup,'Visible','off');        
        end
        
        % If Test is selected toggle visibility btns
        if get(lqrRad,'Value') == 1            
            set(workInProgress,'Visible','on');
            set(frameThreshold,'Visible','off');
            set(handles.referencePIDVal,'Visible','off');
            set(referencePIDTxt,'Visible','off');
            set(sendPidValsBtn,'Visible','off');
        else
        end
        
        if get(HInfRad,'Value') == 1            
            set(workInProgress,'Visible','on');
            set(frameThreshold,'Visible','off');
            set(handles.referencePIDVal,'Visible','off');
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
            try
                start(gyroTimer);
                stop(angleTimer);
                stop(accTimer);
            catch
                disp '******** InstrumentSubscription ERROR *********'
                disp (exception.message);
                disp '***********************************************'
            end
                
        end
        
        % Accelerometer
        if val == 3
            magneto = false;
            accelero = true;
            gyrosco = false;
            try
                stop(gyroTimer);
                stop(angleTimer);
                start(accTimer);                    
            catch
                disp '******** InstrumentSubscription ERROR *********'
                disp (exception.message);
                disp '***********************************************'
            end
        end
        
        % Magnetometer
        if val == 4
            magneto = true;
            accelero = false;
            gyrosco = false;
            try
               stop(gyroTimer);
               start(angleTimer);
               stop(accTimer); 
            catch
                disp '******** InstrumentSubscription ERROR *********'
                disp (exception.message);
                disp '***********************************************'
            end
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
    
    %% Handles bluetooth connection
    % #connection
    function connection(obj,event,handles) 
        handles = guidata(gcf);
        if get(handles.connect,'Value') == 1
            disp('Connecting...');
            
            delete(instrfindall);
            % Check to see if there are existing serial objects 
            % (instrfind) whos 'Port' property is set to 'COM3'

            oldSerial = instrfind('Port', portUnix); 
            % can also use instrfind() with no input arguments to find 
            % ALL existing serial objects

            % if the set of such objects is not(~) empty
            if (~isempty(oldSerial))  
                disp('WARNING:  port in use.  Closing.')
                delete(oldSerial)
            end

            %  Setting up serial communication
            % XBee expects the end of commands to be delineated by a carriage return.

            xbee = serial(portUnix,'baudrate',xbeeBR,'tag',tag);
            %xbee = serial(portWin,'baudrate',xbeeBR,'terminator',terminator,'tag',tag);

            % Max wait time
            set(xbee, 'TimeOut', 10);  
            % One message long buffer
            set(xbee, 'InputBufferSize',inputBuffSize)
            % Open the serial
            fopen(xbee);    

            % Testing Wireless communication
            timerXbee = timer('ExecutionMode','FixedRate','Period',0.1,'TimerFcn',{@storeDataFromSerial});
            try
                start(timerXbee);  
            catch
                disp '******** InstrumentSubscription ERROR *********'
                disp (exception.message);
                disp '***********************************************'
            end
            
            % variable tenzo defines the connection status
            tenzo = false;
            
            if (serial2)
                cmd = 'c';
                sendNMess(cmd);
                tenzoConnectionRequested = true;
            elseif (serial1 || serial0)
                %BYTE-WISE Comm
                % Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(connID);
                % %cmd(2,4) = uint8(defaultAlt);
                bits = reshape(bitget(1,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
            end            
        end

        if get(handles.connect,'Value') == 0
            disp ('Disconnecting...');             
            tenzo = false;
            
            
            if (serial1 || serial0)
                % Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(connID);
                bits = reshape(bitget(0,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
            elseif (serial2)
                cmd = 'X';
                sendNMess(cmd);
            end
            
            set(handles.connect,'String','Connect');
        end 
    end

    %% Send topics
    function sendNMess(obj)
        fprintf(xbee,obj);        
        %disp('Tot bytes sent');
        %xbee.ValuesSent
    end

    %% Send topics
    function sendMess(obj)
        % Build the message Header + Command (only one command at the time)

        %Initialize header as an array full of 'f'
        %hr = repmat('f',8,4);
        hr = zeros(8,4,'uint8');
        % Build Header

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
        % cmd 2
        bufferSend(19) = obj(3,1);
        bufferSend(20) = obj(3,2);
        bufferSend(21) = obj(3,3);
        bufferSend(22) = obj(3,4);
        % cmd 3
        bufferSend(23) = obj(4,1);
        bufferSend(24) = obj(4,2);
        bufferSend(25) = obj(4,3);
        bufferSend(26) = obj(4,4);
        % cmd 4
        bufferSend(27) = obj(5,1);
        bufferSend(28) = obj(5,2);
        bufferSend(29) = obj(5,3);
        bufferSend(30) = obj(5,4);

        disp(bufferSend);
        fwrite(xbee,bufferSend,'uint8');    
        disp('Tot bytes sent');
        xbee.ValuesSent
        
        % Command

        %s = struct('H',value1,'C1',valueN)
    end

    %% Plot Theta phi psi
    function graphAngles(obj,event,~)
        % To debug uncomment the following line
        %disp('Angles');
        anglesRequested = true;
        
        % Requests data only if previous ones have been received and plotted
        
        if estReceived || anglesRequested
            if (serial1 || serial0)
                %Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                % You can send 
                % cmd(1,1) = uint8(magnID); OR
                cmd(1,1) = uint8(estID);
                % Sends 1 to activate PID
                bits = reshape(bitget(0,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
            elseif (serial2)
               cmd = 'e';
               sendNMess(cmd);
            end
        else
            disp('Not received yet Gyro');
        end
    end

    %% Handles pid output topics
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
%                            num2str(get(handles.pidKpSlider,'Value')),',X']
%                            fprintf(xbee,'%s',strindToSend,'sync');  

                            %Initialize the cmd array
                            cmd = zeros(8,4,'uint8');
                            % You can send 
                            % cmd(1,1) = uint8(magnID); OR
                            cmd(1,1) = uint8(cmdtype);
                            % Sends 1 to activate PID
                            disp('ref value:');
                            disp(get(handles.referencePIDVal,'String'));
                            disp('Kp val');
                            disp(get(handles.pidKpSlider,'Value'));
                            disp('Rounded Kp val');
                            disp(double(round(get(handles.pidKpSlider,'Value')*1000)));
                            disp('Rounded Kd val');
                            disp(double(round(get(handles.pidKdSlider,'Value')*1000)));
                            disp('Rounded Ki val');
                            disp(double(round(get(handles.pidKiSlider,'Value')*1000)));
                            if get(handles.pidKpSlider,'Value')<=0.5
                                bits = reshape(bitget(double(round(get(handles.pidKpSlider,'Value')*1000)),32:-1:1),8,[]);
                            else
                                bits = reshape(bitget(double(round(0.5*1000)),32:-1:1),8,[]);
                            end
                            cmd(2,:) = weights2*bits;
                            if get(handles.pidKdSlider,'Value')<=0.5
                                bits = reshape(bitget(double(round(get(handles.pidKdSlider,'Value')*1000)),32:-1:1),8,[]);
                            else
                                bits = reshape(bitget(double(round(0.5*1000)),32:-1:1),8,[]);
                            end
                            %bits = reshape(bitget(double(round(get(handles.pidKdSlider,'Value')*1000)),32:-1:1,'int32'),8,[]);
                            cmd(3,:) = weights2*bits;
                            if get(handles.pidKiSlider,'Value')<=0.5
                                bits = reshape(bitget(double(round(get(handles.pidKiSlider,'Value')*1000)),32:-1:1),8,[]);
                            else
                                bits = reshape(bitget(double(round(0.5*1000)),32:-1:1),8,[]);
                            end
                            %bits = reshape(bitget(double(round(get(handles.pidKiSlider,'Value')*1000)),32:-1:1,'int32'),8,[]);
                            cmd(4,:) = weights2*bits;
                            bits = reshape(bitget(str2double(get(handles.referencePIDVal,'String')),32:-1:1,'int32'),8,[]);
                            cmd(5,:) = weights2*bits;
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


    %% Handles Pid input topics
    function readPidCallback(obj,event)
        if tenzo == true
            if takeOffAck == 1
                %if hoverAck == 1
                   if ~strcmp(pidStrategy,'U') && ~strcmp(pidModeStrategy,'U')
                       pidRead = true;
                       if strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'0')
                           % R C
                           cmdtype = rCRPID;
                           disp('dovrebbe eessre 21');
                       elseif strcmp(pidStrategy,'0') && strcmp(pidModeStrategy,'1')
                           % R A        
                           cmdtype = rARPID;
                       elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'0')
                           % P C
                           cmdtype = rCPPID;
                       elseif strcmp(pidStrategy,'1') && strcmp(pidModeStrategy,'1')
                           % P A                                 
                           cmdtype = rAPPID;
                       elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'0')
                           % Y C
                           cmdtype = rCYPID;
                       elseif strcmp(pidStrategy,'2') && strcmp(pidModeStrategy,'1')
                           % Y A                                 
                           cmdtype = rAYPID;
                       elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'0')
                           % A C
                           cmdtype = rCAPID;
                       elseif strcmp(pidStrategy,'3') && strcmp(pidModeStrategy,'1')
                           % A A  (american Airlines -> Allin                        
                           cmdtype = rAAPID;
                       end
                       % Send 'X,opt1,opt2,opt3,val,X'
%                            strindToSend = ['X,',pidStrategy,',',pidModeStrategy,',0,',...
%                            num2str(get(handles.pidKpSlider,'Value')),',X']
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
                %else
                 %   warndlg('Pid not active, Activate iHover function','!! Warning !!')
                %end
            else
                warndlg('Tenzo is not flying. First Take Off then try again. ','!! Warning !!')
            end
        else
            warndlg('Please connect first ','!! Warning !!')     
        end
    end

    %% Plot angular velocities
    % #Gyro
    function graphGyro(obj,event,handles)
        % To debug uncomment the following line
        % disp('Gyro');
        gyroRequested = true;
        % Requests data only if previous ones have been received and plotted
        
        if gyroReceived || gyroRequested
            % Initialize the cmd array
            if (serial1 || serial0)
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(gyroID);
                bits = reshape(bitget(0,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                sendMess(cmd);
            elseif (serial2)
                cmd = 'o';
                sendNMess(cmd);
            end
        else
            disp('Not received yet Gyro');
        end
            
    end

    %% Plot accelerations
    % #Acceleration
    function graphAcc(obj,event,handles)
        % To debug uncomment the following line
        %disp('Acc');
        accRequested = true;
        % Requests data only if previous ones have been received and plotted
        
        if accReceived || accRequested
            if (serial1 || serial0)
                % Initialize the cmd array
                cmd = zeros(8,4,'uint8');
                cmd(1,1) = uint8(accID);
                bits = reshape(bitget(0,32:-1:1),8,[]);
                cmd(2,:) = weights2*bits;
                accReceived = false;
                sendMess(cmd);
            elseif (serial2)
                cmd = 'l';
                sendNMess(cmd);
            end
        else
            disp('Not received yet');
        end
    end

    function sendStates()
        cmd = zeros(8,4,'uint8');
        cmd(1,1) = uint8(tenzoStateID);
        bits = reshape(bitget(double(takeOffAck),32:-1:1),8,[]);
        cmd(2,:) = weights2*bits;
        bits = reshape(bitget(double(hoverAck),32:-1:1),8,[]);
        cmd(3,:) = weights2*bits;
        bits = reshape(bitget(double(landAck),32:-1:1),8,[]);
        cmd(4,:) = weights2*bits;
        sendMess(cmd); 
    end

    function storeDataFromSerial(obj,event,handles)
        handles = guidata(gcf);
        while (get(xbee, 'BytesAvailable')~=0)
            if (serial2)
                serialProtocol2();
            elseif (serial1)
                serialProtocol1();
            elseif (serial0)
                serialProtocol0();
            end
        end
    end 
    
    %% Serial Protocol 2.0 Bluetooth
    % #bluetooth #record #plot
    function serialProtocol2()
        [mess,count] = fscanf(xbee);
        %disp('Reading incoming buffer. Dimensions:');
        % Debug stuff
        %disp(count);
        %disp(mess);    
        mess = deblank(mess)
        
        if (tenzo == false)
            if (strcmp(mess,'K') && tenzoConnectionRequested)
                tenzo = true;
                set(handles.connect,'String','Disconnect');
                set(handles.conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');    
                disp ('Connection established. Rock & Roll!'); 
                if speakCmd && vocalVerb>=1 
                        %tts('Connessione eseguita',voice);
                        tts('Connection Established',voice);
                end
                tenzoConnectionRequested = false;
            elseif (~strcmp(mess,'K') &&  tenzoConnectionRequested)
                tenzo = false;
                set(handles.connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(handles.conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
                if speakCmd && vocalVerb>=1 
                    %    tts('Problema di connessione',voice);
                        tts('Connection problem',voice);
                end
                disp ('Communication problem. Check Hardware and retry.');            
            elseif (strcmp(mess,'X'))
                tenzo = false;
                %delete(timerXbee);
                %set(connect,'String','Connect');
                set(handles.connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(handles.conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');  
                disp('Connection closed...');
                if speakCmd && vocalVerb>=1 
                    tts('Connection closed',voice);
                end
                stop(timerXbee);
                fclose(xbee);
            end
        elseif (tenzo)
            % Communication established
            footer = mess(size(mess,2));size(mess,2)
            % if message is correct
            if footer == footerTag
            tag = mess(1);
                if tag == accTag
                    %disp('Accelerations');
                    % Acc time serial
                    [R,accXr,accYr,accZr,t] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                     if accelero == true
                        % Gets Accelerometer data
                        axdata = [ axdata(2:end) ; double(accXr) ];
                        aydata = [ aydata(2:end) ; double(accYr) ];
                        azdata = [ azdata(2:end) ; double(accZr) ];  
                     end
                     
                     %Plot the X magnitude
                    h1 = subplot(3,1,1,'Parent',hTabs(3));
                    if filterAcc
                        plot(h1,index,axdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    else
                        plot(h1,index,axdata,'r','LineWidth',2);
                    end
                    grid on;
                    
                    h2 = subplot(3,1,2,'Parent',hTabs(3));
                    if filterAcc
                        plot(h2,index,aydata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    else
                        plot(h2,index,aydata,'b','LineWidth',2);
                    end
                    grid on;
                    
                    h3 = subplot(3,1,3,'Parent',hTabs(3));
                    if filterAcc
                        plot(h3,index,azdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    else
                        plot(h3,index,azdata,'g','LineWidth',2);
                    end
                    grid on;
                    accReceived = true;
                    if asked
                        % Write to file
                        dlmwrite('accx.dat',[accXr accYr accZr],'-append', 'delimiter', ',');
                    end
                
                elseif tag == gyroTag
                    % Gyro                    
                    [R,wXr,wYr,wZr,t] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    if gyrosco == true
                        if filterGyro
                            % Filters gyro data and stores them
                            gxFilt = (1 - alpha)*gxFilt + alpha*wXr;
                            gyFilt = (1 - alpha)*gyFilt + alpha*wYr;
                            gzFilt = (1 - alpha)*gzFilt + alpha*wZr;
                        else
                            gxFilt = wXr;
                            gyFilt = wYr;
                            gzFilt = wZr;
                        end
                            gxFdata = [ gxFdata(2:end) ; gxFilt ];
                            gyFdata = [ gyFdata(2:end) ; gyFilt ];
                            gzFdata = [ gzFdata(2:end) ; gzFilt ];                            
                    end

                    h1 = subplot(3,1,1,'Parent',hTabs(3));
                    plot(h1,index,gxFdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    grid on;
                    
                    h2 = subplot(3,1,2,'Parent',hTabs(3));
                    plot(h2,index,gyFdata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    grid on;
                   
                    h3 = subplot(3,1,3,'Parent',hTabs(3));
                    plot(h3,index,gzFdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    
                    grid on;
                    
                    if asked
                        % Save to File
                        if filterGyro
                            dlmwrite('gyrox.dat',[wXr wYr wZr],'-append', 'delimiter', ',');
                        else
                            dlmwrite('gyrox.dat',[gxFilt gyFilt gzFilt],'-append', 'delimiter', ',');
                        end
                    end
                    gyroReceived = true;
                elseif tag == estTag
                     % Magn
                    %disp('Est');
                    [R,rollM,pitchM,bearingM,t] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
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
                            TFilt = (1 - alpha)*TFilt + alpha*rollM
                            PFilt = (1 - alpha)*PFilt + alpha*pitchM;
                            YFilt = (1 - alpha)*YFilt + alpha*bearingM;

                            Rdata = [ Rdata(2:end) ; TFilt ];
                            Pdata = [ Pdata(2:end) ; PFilt ];
                            Ydata = [ Ydata(2:end) ; YFilt ]; 
                        else
                            Rdata = [ Rdata(2:end) ; rollM ];
                            Pdata = [ Pdata(2:end) ; pitchM ];
                            Ydata = [ Ydata(2:end) ; bearingM]; 
                        end
                    else
                        disp('Warning! Received magneto data but not requested');
                    end

                    %Plot the X magnitude
                    h1 = subplot(3,1,1,'Parent',hTabs(3));
                    plot(h1,index,Rdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    grid on;
                    
                    h2 = subplot(3,1,2,'Parent',hTabs(3));
                    plot(h2,index,Pdata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    grid on;
                    
                    h3 = subplot(3,1,3,'Parent',hTabs(3));
                    grid on;
                    plot(h3,index,Ydata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                    
                    if asked
                        % Write to file
                        if filterMagn
                            dlmwrite('angx.dat',[TFilt PFilt YFilt],'-append', 'delimiter', ',');
                        else
                            dlmwrite('angx.dat',[rollM pitchM bearingM],'-append', 'delimiter', ',');
                        end
                    end
                    magnReceived = true;
                elseif tag == throttleTag
                    % TODO
                    [R,throttleActualValue,N] = strread(mess,'%s%f%s',1,'delimiter',',');
                    set(handles.throttleVal,'String',throttleActualValue);
                elseif tag == rollConsTag(1) && mess(2) == rollConsTag(2)
                    % Pid Roll CONS
                    disp('Pid Roll Cons');
                    [R,consRollKp,consRollKi,consRollKd,setpointRollTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    % TODO get setpoint  

                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consRollKp);
                        set(handles.pidKdSlider,'Value',consRollKd);
                        set(handles.pidKiSlider,'Value',consRollKi);
                        set(handles.referencePIDVal,'String',setpointRollTemp);
                    end   
                 elseif tag == rollAggTag(1) && mess(2) == rollAggTag(2)
                    % Pid Roll CONS
                    disp('Pid Roll Agg');
                    [R,aggRollKp,aggRollKi,aggRollKd,setpointRollTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',aggRollKp);
                        set(handles.pidKdSlider,'Value',aggRollKd);
                        set(handles.pidKiSlider,'Value',aggRollKi);
                        set(handles.referencePIDVal,'String',setpointRollTemp);
                    end   
                  elseif tag == pitchConsTag(1) && mess(2) == pitchConsTag(2)
                    % Pid Pitch CONS
                    disp('Pid Pitch Cons');
                    
                    [R,consPitchKp,consPitchKi,consPitchKd,setpointPitchTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consPitchKp);
                        set(handles.pidKdSlider,'Value',consPitchKd);
                        set(handles.pidKiSlider,'Value',consPitchKi);
                        set(handles.referencePIDVal,'String',setpointPitchTemp);
                    end   
                 elseif tag == pitchAggTag(1) && mess(2) == pitchAggTag(2)
                    % Pid Pitch Agg
                    disp('Pid Pitch Agg');
                    
                    [R,aggPitchKp,aggPitchKi,aggPitchKd,setpointPitchTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',aggPitchKp);
                        set(handles.pidKdSlider,'Value',aggPitchKd);
                        set(handles.pidKiSlider,'Value',aggPitchKi);
                        set(handles.referencePIDVal,'String',setpointPitchTemp);
                    end  
                  elseif tag == yawConsTag(1) && mess(2) == yawConsTag(2)
                    % Pid yaw CONS
                    disp('Pid yaw Cons');
    
                    [R,consYawKp,consYawKi,consYawKd,setpointYawTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consYawKp);
                        set(handles.pidKdSlider,'Value',consYawKd);
                        set(handles.pidKiSlider,'Value',consYawKi);
                        set(handles.referencePIDVal,'String',setpointYawTemp);
                    end   
                 elseif tag == yawAggTag(1) && mess(2) == yawAggTag(2)
                    % Pid Yaw Agg
                    disp('Pid Yaw Agg');

                    [R,aggYawKp,aggYawKi,aggYawKd,setpointYawTemp,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',aggYawKp);
                        set(handles.pidKdSlider,'Value',aggYawKd);
                        set(handles.pidKiSlider,'Value',aggYawKi);
                        set(handles.referencePIDVal,'String',setpointYawTemp);
                    end 
                elseif tag == rollConsTagW(1) && mess(2) == rollConsTagW(2)
                    % Pid Roll CONS
                    disp('Pid Roll W Cons');
                    [R,consRollKpW,consRollKiW,consRollKdW,setpointRollTempW,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    % TODO get setpoint  

                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consRollKpW);
                        set(handles.pidKdSlider,'Value',consRollKdW);
                        set(handles.pidKiSlider,'Value',consRollKiW);
                        set(handles.referencePIDVal,'String',setpointRollTempW);
                    end
                elseif tag == pitchConsTagW(1) && mess(2) == pitchConsTagW(2)
                    % Pid Pitch CONS
                    disp('Pid Pitch W Cons');
                    
                    [R,consPitchKpW,consPitchKiW,consPitchKdW,setpointPitchTempW,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consPitchKpW);
                        set(handles.pidKdSlider,'Value',consPitchKdW);
                        set(handles.pidKiSlider,'Value',consPitchKiW);
                        set(handles.referencePIDVal,'String',setpointPitchTempW);
                    end
                elseif tag == yawConsTagW(1) && mess(2) == yawConsTagW(2)
                    % Pid yaw CONS
                    disp('Pid yaw Cons');    
                    [R,consYawKpW,consYawKiW,consYawKdW,setpointYawTempW,N] = strread(mess,'%s%f%f%f%s',1,'delimiter',',');
                    
                    if strcmp(pidModeStrategy,'0')
                        set(handles.pidKpSlider,'Value',consYawKpW);
                        set(handles.pidKdSlider,'Value',consYawKdW);
                        set(handles.pidKiSlider,'Value',consYawKiW);
                        set(handles.referencePIDVal,'String',setpointYawTempW);
                    end 
                 elseif tag == accFilterTag
                    % Pid yaw CONS
                    disp('Changing Filter parameter');
                    [R,filterParam,N] = strread(mess,'%s%f%s',1,'delimiter',',');
                    set(handles.filterVal,'Value',filterParam);
                    
                elseif tag == pidToggleTag 
                    if mess(1) == pidEnableTag
                        disp('Pid enabled');
                        set(handles.hoverBtn,'String','iHoverPid');
                    elseif mess(1) == pidDisableTag
                        disp('Pid disabled');
                        set(handles.hoverBtn,'String','NoPid');                            
                    end
                elseif tag == takeOffAckTag 
                        set(handles.takeOffBtn,'String','Flying');
                        takeOffAck = 1;
                        landAck = 0;
                        disp('Changed landAck:');
                        disp(landAck);
                        set(handles.landBtn,'String','Land');
                elseif tag == landAckTag                   
                        set(handles.landBtn,'String','Landed'); 
                        set(handles.takeOffBtn,'String','Take Off');
                        takeOffAck = 0; 
                end
            end            
        end        
    end
end