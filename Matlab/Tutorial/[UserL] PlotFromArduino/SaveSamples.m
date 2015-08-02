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
    rate=input('Enter the samplerate in Hz. Max 500Hz. Non sgravare...      ');
    delay = 1/rate;
    str = sprintf('SampleRate fixed to: %f.', delay);
    disp(str);

%% Initializing

    buf_len = rate;
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
%% Data collection and Plotting
    while (~finished)
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

                    % [gx, gy, gz] = [x, y, z];
                    gxdata = [ gxdata(2:end) ; double(Wx) ];
                    gydata = [ gydata(2:end) ; double(Wy) ];
                    gzdata = [ gzdata(2:end) ; double(Wz) ]; 

                    numberOfSamples = numberOfSamples + 1

                    % wait one second then record
                    if numberOfSamples==rate*2 && recording
                        disp('saving samples to file');
                        gDataToWrite = [gxdata gydata gzdata];
                        csvwrite('samples.txt',gDataToWrite);
                        disp('saving file to structure');
                        dat.x = gxdata;
                        dat.y = gydata;
                        dat.z = gzdata;
                        save('samples.mat','-struct','dat');
                        disp('Saved.');

                        finished = true;
                    end
                end
            end
        catch exeption
            disp('Warning');
        end
    end
    if finished
       disp('Check your local folder.');
    end
%% Close connection    

    fclose(xbee);
    delete(xbee);
    clear xbee
    clear('instrfindall');