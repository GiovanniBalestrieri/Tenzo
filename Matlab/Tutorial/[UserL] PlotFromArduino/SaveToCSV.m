%% Arduino Gyroscope real time plot

%  The sensor used in this project is the L3G4200D
%  !!Caution!! operative voltage: 3.3 V

    clear all;
    clc;
    clear('instrfindall');
    
    finished = false;
    recording = true;
    if recording 
       disp('Record data enabled');
    end
    
%% Check serial objects & Connect to Port

%   Check the port you are using with the arduino, then run: sudo ln -s /dev/ttyNUMBER /dev/ttyS101
    disp('Linux User? Have you run the simbolic link with ttyS101?');
    disp('sudo rm /dev/ttyS101');
    disp('sudo ln -s /dev/tty_NUMBER_ /dev/ttyS101');
    port = '/dev/ttyS101';
    oldSerial = instrfind('Port', port); 
    % can also use instrfind() with no input arguments to find ALL existing serial objects
    if (~isempty(oldSerial))  % if the set of such objects is not(~) empty
        disp('WARNING:  Port in use. Check your port. Closing.')
        delete(oldSerial)
        a = true;
    else        
        disp('Connection to serial port...');
    end
    
%%  Setting up serial communication
    % XBee expects the end of commands to be delineated by a carriage return.
    xbee = serial(port,'baudrate',57600,'terminator','CR','tag','Quad');

    set(xbee, 'TimeOut', 5);  %I am willing to wait 1.5 secs for data to arive
    % I wanted to make my buffer only big enough to store one message
    set(xbee, 'InputBufferSize',  390 )
    % Before you can write to your serial port, you need to open it:
    fopen(xbee);
    
    disp('Serial port opened');
    
    %% Disp incoming serial message
    fprintf(xbee,'T')
    disp(fscanf(xbee,'%s'));
    
%% Testing Connection 
    
    % Ask for data
    fprintf(xbee,'T');
    disp('Sent: T');
    pause(1);
    ack = fscanf( xbee, '%s');
    if (strcmp(deblank(ack), 'Ok') == 1)
        disp ('Received: OK');
    else
        disp ('Communication problem. Check Hardware and retry.');
    end       
    
    
%% Ask desired Sample Rate in Hz
rate=input('Enter the sampleRate in Hz. Max 500Hz. Non sgravare...      ');
delay = 1/rate;
str = sprintf('SampleRate fixed to: %f.', delay);
disp(str);

%% Initializinig rolling plot

buf_len = 2*rate;
index = 1:buf_len;

% create variables for the Xaxis
gxdata = zeros(buf_len,1);
gydata = zeros(buf_len,1);
gzdata = zeros(buf_len,1);

if recording
    resp = input('Press r to record 2 seconds of data','s');
    if (strcmp(resp,'r'))
        disp('Record!');
        record = true;
        numberOfSamples = 0;
    end
end
%% Setup Plot
% declaring the arduino varialble and establishing connection
Wx = 0; Wy = 0; Wz=0;
% 
% % Figure options
figure(1);
% ax = axes('box','on');
%% Data collection and Plotting
while (abs(Wz) < 2500 && ~finished)
    % Polling 
    pause(delay);
    fprintf(xbee,'M') ; 
    notArrived = false;
    try
        while (get(xbee, 'BytesAvailable')~=0 && ~notArrived)
            % read until terminator
            sentence = fscanf( xbee, '%s'); % this reads in as a string (until a terminater is reached)
            if (strcmp(sentence(1,1),'A'))
                notArrived = true;
                %decodes "sentence" seperated (delimted) by commaseck Unit')
                C = textscan(sentence,'%c %d %d %d %c','delimiter',',');
                Wx = C{2};
                Wy = C{3};
                Wz = C{4};
                
                %% Plotting angles

                %[gx, gy, gz] = [Xacc, Yacc, Zacc];
                % Update the rolling plot. Append the new reading to the end of the
                % rolling plot data
                gxdata = [ gxdata(2:end) ; double(Wx) ];
                gydata = [ gydata(2:end) ; double(Wy) ];
                gzdata = [ gzdata(2:end) ; double(Wz) ]; 
                                
                %Plot the X magnitude
                subplot(3,1,1);
                title('X Axis Omega in deg/s');
                plot(index,gxdata,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                xlabel('Sample');
                ylabel('Wx (deg/sec)');
                %axis([1 buf_len -100 100]);
                %hold on;
                subplot(3,1,2);
                title('Y  Axis Omega in deg/s');
                plot(index,gydata,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                xlabel('Sample');
                ylabel('Wy');
                %axis([1 buf_len -100 100]);
                subplot(3,1,3);
                title('Z  Axis Omega in deg/s');
                %hold on;
                plot(index,gzdata,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                %axis([1 buf_len -100 100]);
                xlabel('Sample');
                ylabel('Wz');
                
                drawnow;
                numberOfSamples = numberOfSamples + 1
                % wait one second then record
                if numberOfSamples==rate*2 && recording
                    disp('saving samples to file');
                    gDataToWrite = [gxdata gydata gzdata];
                    csvwrite('gyro.txt',gDataToWrite);
                    disp('saving file to structure');
                    dat.x = gxdata;
                    dat.y = gydata;
                    dat.z = gzdata;
                    save('GyroSamples1.mat','-struct','dat');
                    disp('Saved.');

                    finished = true;
                end
            end
        end
    catch exeption
        disp('Warning');
    end
end
if abs(Wz) >= 2500
    disp('Threshold reached. Closing');
end
if finished
   disp('Check your local folder.');
end
%% Close connection    

fclose(xbee);
delete(xbee);
clear xbee
clear('instrfindall');
    
clear all
