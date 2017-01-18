% t = linspace(0,100)
% 
% pwm = 650*sin(exp(t))+700
% plot(t,pwm)
% 
% %%
% 
% t = 0:1:100
% 
% pwm = sin(exp(t))
% plot(t,pwm)
% 
% %%
%  Fs=1000; % sample rate 
% tf=2; % 2 seconds
% t=0:1/Fs:tf-1/Fs;
% f1=100;
% f1=f1*t %+(sl.*t/2);
% %f2=f1(end)+f2*semi_t-sl.*semi_t/2;
% %f=[f1 f2];
% f = f1
% y=650*cos(2*pi*f.*t)+ 1300;
% plot(t,y)


%%

%% Arduino RPM real time plot



    clear all;
    clc;
    delete('instrfindall');
    
    testFinished = false;
    testStarted = false;
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
    xbee = serial(port,'baudrate',115200,'terminator','CR','tag','Quad');

    set(xbee, 'TimeOut', 15);  %I am willing to wait 1.5 secs for data to arive
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
    
    % Take Off
    pause(1);
    fprintf(xbee,'c');
    disp('Sent: calibrate command');
    pause(5);
    ack = fscanf( xbee, '%s')
    if (strcmp(deblank(ack), '...') == 1)
        disp ('takiing off');
    else
        disp ('Communication problem. Check Hardware and retry.');
    end     
    
    disp('Sent:  i');
    fprintf(xbee,'i');
    pause(4)
    ack = fscanf( xbee, '%s')
    if (strcmp(deblank(ack), 'Initialized') == 1)
        disp (' OK Initialized');
    else
        disp ('Communication problem. Check Hardware and retry.');
    end  
    
    % Land
    
% 
%     fprintf(xbee,'l');
%     disp('Sent: l');
%     pause(1);
%     ack = fscanf( xbee, '%s')
%     if (strcmp(deblank(ack), 'Landing') == 1)
%         disp ('Landing');
%     else
%         disp ('Communication problem. Check Hardware and retry.');
%     end     
%     pause(3)
%     ack = fscanf( xbee, '%s')
%     if (strcmp(deblank(ack), 'Landed') == 1)
%         disp ('Landed');
%     else
%         disp ('Communication problem. Check Hardware and retry.');
%     end       
    
    
%% Ask desired Sample Rate in Hz
    rate=input('Enter the samplerate in Hz. Max 500Hz.\n Non sgravare...      ');
    delay = 1/rate;
    str = sprintf('SampleRate fixed to: %f.', delay);
    disp(str);

%% Sending commands


    
%% Initializing

buf_len = rate*1000;
    index = 1:buf_len;

    % create variables for the Xaxis
%     time = zeros(buf_len,1);
%     output = zeros(buf_len,1);
%     input = zeros(buf_len,1);

time1 = 0;
output1 = 0;
input1 = 0;
    if recording
        %response = input('Press r to record 2 seconds of data');
        %if (strcmp(response,'r'))
            disp('Record!');
            record = true;
            numberOfSamples = 0;
        %end
    end
    
    fprintf(xbee,'t');
    disp('Sent: t');
    %pause(1);  
%% Data collection and Plotting
    while (~finished && ~testFinished)
        % Polling 
        pause(delay);
        fprintf(xbee,'M') ; 
        notArrived = false;
        try
            while (get(xbee, 'BytesAvailable')~=0 && ~notArrived)
                % read until terminator
                sentence = fscanf( xbee, '%s'); % this reads in as a string (until a terminater is reached)
                
                if (strcmp(sentence,'start'))
                   testStarted = true;
                   disp('Test Started');
                end
                
                if (strcmp(sentence,'Tested'))
                   testFinished = true; 
                   disp('Test Finished');
                end
                if (strcmp(sentence(1,1),'A'))
                    notArrived = true;
                    %decodes "sentence" seperated (delimted) by commaseck Unit')
                    C = textscan(sentence,'%c %d %d %d %c','delimiter',',');
                    Wx = C{2};
                    Wy = C{3};
                    Wz = C{4};

                    % [gx, gy, gz] = [x, y, z];
                    %time = [ time(2:end) ; double(Wx) ];
                    %output = [ output(2:end) ; double(Wy) ];
                    %input = [ input(2:end) ; double(Wz) ]; 
                    
                    time1 = [ time1 ; double(Wx) ];
                    output1 = [ output1 ; double(Wy)];
                    input1 = [ input1 ; double(Wz)];
                    
                    numberOfSamples = numberOfSamples + 1                   
                end
            end
        catch exeption
            disp('Warning');
        end
    end
    % wait one second then record                    if numberOfSamples==rate*30 && recording && testStarted
    %if numberOfSamples==rate*30 && recording && testStarted
    if testFinished
        disp('Writing samples to file:');
        %size(time1,1)
        gDataToWrite = [time1 output1 input1];
        csvwrite('samples.txt',gDataToWrite);
        disp('saving file to structure');
        dat.x = time1;
        dat.y = output1;
        dat.z = input1;
        save('samples.mat','-struct','dat');
        disp('Saved.');

        finished = true;
    end
    if finished
       disp('Check your local folder.');
    end
%% Close connection    


    fprintf(xbee,'l');
    disp('Sent: l');
    pause(1);
    ack = fscanf( xbee, '%s')
    if (strcmp(deblank(ack), 'Landing') == 1)
        disp ('Landing');
    else
        disp ('Communication problem. Check Hardware and retry.');
    end     
    
    fclose(xbee);
    delete(xbee);
    clear xbee
clear('instrfindall');

%% Plot data


dir .
load('samples.mat')
inizio = 115;

x = x(1:end);
y = y(1:end);
z = z(1:end);

disp('Number of Samples');
disp(size(x,1))

iter = ceil(size(x,1)*0.2);

xIde = x(inizio:iter);
uIde = y(inizio:iter);
yIde = z(inizio:iter);

xVal = x(iter:end);
uVal = y(iter:end);
yVal = z(iter:end);

figure(10)
subplot(2,1,1);
plot(x,y,'r');
title('pwm [us]');
subplot(2,1,2);
plot(x,z,'r');
title('Shaft angular velocity [rev/s]');

figure(1)
subplot(2,1,1);
plot(xIde,yIde,'r');
title('Shaft angular velocity [rev/s]');
subplot(2,1,2);
plot(xIde,uIde,'r');
title('pwm [us]');

%figure(2)
%plot(x,z,'r');


% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.045,'low');

% Plot the frequency response (normalized frequency)
figure(13)
z_filtered = filter(b,a,yIde);
plot(xIde,z_filtered,'r');
