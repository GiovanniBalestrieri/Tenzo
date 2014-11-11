%% Robust serial communication

function ControlBoard()

clear all;

global xbee;
global portWin;
global portUnix;
global baudrate;
global terminator;
global buffSize;
global tag;
global tenzo;
global timerXbee;
global matlabAdd;
global arduinoAdd;
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
global bufferSend;

%% Serial protocol

version = 1;
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

cmdLength = 17;
headerLength = 13;
arduinoAdd = 1;
matlabAdd = 2;
portWin = 'Com3';
portUnix = '\dev\ttyS0';
baudrate = 19200;
% buffer size should be the same as the one specified on the Arduino side
buffSize = 35;
terminator = 'CR';
tag = 'Quad';

%% variables declaration

defaultAlt = 1;
% used to store long/short num to arrays
weights2 = 2.^([7:-1:0]);

bufferSend = zeros(1,buffSize-1);

disp('Welcome to the Home Panel');
% Delete all serial connections
delete(instrfindall)

%% Creating Basic GUI
    hFig = figure('Menubar','none');
    s = warning('off', 'MATLAB:uitabgroup:OldVersion');
    hTabGroup = uitabgroup('Parent',hFig);
    warning(s);
    hTabs(1) = uitab('Parent',hTabGroup, 'Title','Home');
    hTabs(2) = uitab('Parent',hTabGroup, 'Title','Room');
    set(hTabGroup, 'SelectedTab',hTabs(1));
    Listener = addlistener(hTabGroup,'SelectedTab','PostSet',@tabGroupCallBack);

%# Home UI components
    uicontrol('Style','text', 'String','Control Panel', ...
        'Position', [55 310 420 50],...
        'Parent',hTabs(1), 'FontSize',20,'FontWeight','bold');
    
    uicontrol('Style','text', 'String',version, ...
        'Position', [55 268 420 50],...
        'Parent',hTabs(1), 'FontSize',10,'FontWeight','normal');
    
    Btn1 = uicontrol('Style','pushbutton', 'String','W', ...
        'Position', [70 210 120 30],...
        'Parent',hTabs(1), 'Callback',@Callback1);
    
    Btn2 = uicontrol('Style','pushbutton', 'String','L', ...
        'Position', [210 210 120 30],...
        'Parent',hTabs(1), 'Callback',@Callback2);
    
    Btn3 = uicontrol('Style','pushbutton', 'String','F', ...
        'Position', [350 210 120 30],...
        'Parent',hTabs(1), 'Callback',@Callback3);
    
    conTxt = uicontrol('Style','text', 'String','Offline','ForegroundColor',[.99 .183 0.09], ...
        'Position', [70 20 100 30],...
        'Parent',hTabs(1), 'FontSize',13,'FontWeight','bold');
    
    connect = uicontrol('Style','togglebutton', 'String','Connect', ...
        'Position', [400 20 120 30],'BackgroundColor',[.21 .96 .07],...
        'Parent',hTabs(1), 'Callback',@connection);
    
    %% Callbacks
    
    function Callback1(src,eventData)
        if tenzo == true
            fwrite(xbee,23); 
            % wait for feedback from Tenzo and change state of btn
        else
            warndlg('Connection Required. Establishing serial communication','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function Callback2(src,eventData)
        if tenzo == true
            % sends 22 (dec) = 0x16
            fwrite(xbee,hex2dec('16')); 
            % wait for feedback from Tenzo and change state of btn
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    function Callback3(src,eventData)
        if tenzo == true
            % Initialize the cmd array
            cmd = zeros(8,4,'uint8');
            cmd(1,1) = uint8(iHoverID);
            %cmd(2,4) = uint8(defaultAlt);
            bits = reshape(bitget(defaultAlt,32:-1:1),8,[]);
            cmd(2,:) = weights2*bits;
            bits = reshape(bitget(33,32:-1:1),8,[]);
            cmd(3,:) = weights2*bits;
            bits = reshape(bitget(44,32:-1:1),8,[]);
            cmd(4,:) = weights2*bits;
            bits = reshape(bitget(55,32:-1:1),8,[]);
            cmd(5,:) = weights2*bits;
            disp('sending');
            sendMess(cmd);
            %fwrite(xbee,hex2dec('16')); 
            % wait for feedback from Tenzo and change state of btn
        else
            warndlg('Connection Required. Establish serial communication first and retry','!! Warning !!') 
            % you can start the connection automatically
        end
    end

    % Could be useful
    function tabGroupCallBack(~,~)
        val = get(hTabGroup,'SelectedTab');
        if val == hTabs(1)
        disp('Home selected');
        end
        if val == hTabs(2)
        disp('Room selected');
        end
    end

%% Connection
function connection(~,~)
        % Checks whether the user wants to connect and establishes the
        % Arduino connection
        
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
            set(xbee, 'TimeOut', 5);  
            % One message long buffer
            set(xbee, 'InputBufferSize',buffSize)
            % Open the serial
            fopen(xbee);    

            %% Testing Wireless communication
            timerXbee = timer('ExecutionMode','FixedRate','Period',0.1,'TimerFcn',{@storeDataFromSerial});
            fwrite(xbee,16); 
            disp('Ack Requested.');
            ack = fread(xbee);
            disp('Receiving: ');
            disp(ack);
            if (ack == 17)            
                tenzo = true;
                start(timerXbee);
                fwrite(xbee,18);
                set(connect,'BackgroundColor',[.99 .183 0.09],'String','Disconnect');
                set(conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');
                disp ('Connection established. Rock & Roll!');
            else
                % Received something else
                fwrite(xbee,19);
                disp ('Sending 0x13 to Arduino.');
                tenzo = false;
                set(connect,'BackgroundColor',[.21 .96 .07],'String','Connect');
                set(conTxt,'ForegroundColor',[.99 .183 0.09] ,'String','Offline');
                disp ('Communication problem. Check Hardware and retry.');
                warndlg('Communication busy. Press OK and reconnect','!! Warning !!')
            end
        end

        if get(connect,'Value') == 0
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
    bits = reshape(bitget(version,32:-1:1),8,[]);
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
    xbee.ValuesSent
%    s=1;
%     while s <=  length(bufferSend)
%         fwrite(xbee,bufferSend(s),'uint8');
%         s = s+1;
%     end
%    xbee.ValuesSent
    %% Command
    
    %s = struct('H',value1,'C1',valueN)
end
        
function storeDataFromSerial(obj,event,handles)
       try
           % With the second boolean check this routine won'tbe executed if
           % the connection hasn't been established first
            while (get(xbee, 'BytesAvailable')~=0 && tenzo == true)
                % read until terminator
                [mess,count] = fread(xbee);
                %% Debug stuff
                
                %disp(count);
                %disp(mess);
                %% Parsing the message
                
                if (count == buffSize && mess(2) == matlabAdd && mess(1) == arduinoAdd)
                    % Mess sent from Arduino to MATLAB
                    % Assemble long int sent bytewise
                    versionArd = typecast([uint8(mess(3)), uint8(mess(4)),uint8(mess(5)), uint8(mess(6))], 'int32');
                    if version ~= versionArd
                        warndlg('Your version is different from the one in Tenzo. Sync your repository.','!! Warning !!') 
                    end
                    numCmd = mess(7);
                    %Arduino  tells the receiver where commands start
                    readFrom = mess(8);
%                     disp('ReadFrom');
%                     disp(readFrom);
                    sizeOfEachCmd = mess(9);
                    totMessLength = typecast([uint8(mess(10)), uint8(mess(11))], 'int16');
                    disp('Total message length:');
                    disp(totMessLength);
                    % TODO CRC check for message's integrity
                    crcvalue = typecast([uint8(mess(12)), uint8(mess(13))], 'int16');
                    disp('CRC value');
                    disp(crcvalue);
                    
                    %% Read Commands
                    for i = 0:numCmd
                        typei = (readFrom+1)+i*sizeOfEachCmd;
                        type = mess(typei) 
                        disp('Message #:');
                        disp(i+1);
                        
                        if type == 0 
                            % Motors
                            disp('Unset');                         
                        end
                        if type == 1 
                            % Motors
                            disp('Motors');
                            speed1 = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                            disp('val1:');
                            disp(speed1);
                            
                            speed2 = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                            disp('val2:');
                            disp(speed2);                             
                            
                            speed3 = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                            disp('val3:');
                            disp(speed3);    
                            
                            speed4 = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
                            disp('val4:');
                            disp(speed4);                            
                        end
                        if type == 2 
                            disp('Accelerations');
                            % Acc
                            accXr = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                            disp('val1:');
                            disp(accXr);
                            
                            accYr = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                            disp('val2:');
                            disp(accYr);                             
                            
                            accZr = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                            disp('val3:');
                            disp(accZr);                             
                            
%                             val4 = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'int32');
%                             disp('val4:');
%                             disp(val4); 
                        end
                        if type == 2
                            % Gyro
                            disp('Gyro');
                            wXr = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32');
                            disp('val1:');
                            disp(wXr);
                            
                            wYr = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'int32');
                            disp('val2:');
                            disp(wYr);                             
                            
                            wZr = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'int32');
                            disp('val3:');
                            disp(wZr);
                        end
                        if type == 3 
                            % Magn
                            disp('Magn');
                        end
                        if type == 4
                            % Est
                            disp(' Kalman Est');
                        end
                        if type == 5 
                            % Gps
                            disp('Gps');
                        end
                        if type == 6 
                            % Barometer
                            disp('Barometer');
                        end
                    end
                end
%                 sentence = fscanf( xbee, '%s') % this reads in as a string (until a terminater is reached)
%                 if (strcmp(sentence(1,1),'R'))
%                     %decodes "sentence" seperated (delimted) by commaseck Unit')
%                     [Roll, theta, Pitch, pitch, Yaw, yaw, KalmanRoll, kr, KalmanPitch, kp, OmegaX, wx, OmegaY, wy, OmegaZ, wz, AccX, ax, AccY, ay, AccZ, az, Motor, omegaR] = strread(sentence,'%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f',1,'delimiter',',');
% 
%                     
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
%     %                     Tdata = [ Tdata(2:end) ; TFilt ];
%     %                     Pdata = [ Pdata(2:end) ; PFilt ];
%     %                     Ydata = [ Ydata(2:end) ; YFilt ]; 
% 
%                         Tdata = [ Tdata(2:end) ; theta ];
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
%                 end % header ok
            end %if data available
       end
    end
end