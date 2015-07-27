%% Arduino Matlab communication
clear all;
clc;

%% Check serial objects & Connect to Port

%   Check the port you are using with the arduino, then run: sudo ln -s /dev/ttyNUMBER /dev/ttyS101
    disp('Linux User? Have you run the simbolic link with ttyS101?');
    port = '/dev/ttyS101';
    oldSerial = instrfind('Port', port); 
    % can also use instrfind() with no input arguments to find ALL existing serial objects
    if (~isempty(oldSerial))  % if the set of such objects is not(~) empty
        disp('WARNING:  Port in use. Closing.')
        delete(oldSerial)
    end
    
    disp('Connection to serial port...');
    %%  Setting up serial communication
    % XBee expects the end of commands to be delineated by a carriage return.
    xbee = serial(port,'baudrate',9600,'terminator','CR','tag','Quad');

    set(xbee, 'TimeOut', 5);  %I am willing to wait 1.5 secs for data to arive
    % I wanted to make my buffer only big enough to store one message
    set(xbee, 'InputBufferSize',  390 )
    % Before you can write to your serial port, you need to open it:
    fopen(xbee);
    
    disp('Serial port opened');
%% Disp incoming serial message
    
    % disp(fscanf(xbee,'%s'));
    
%% Testing Wireless communication
    
    % Ask for data
    fprintf(xbee,'T');
    ack = fscanf( xbee, '%s');
    if (strcmp(deblank(ack), 'Ok') == 1)
        yes = [1];
        fwrite(xbee,yes);
        disp ('Ok');
    else
        disp(ack);
    end
    
    
    %% Close connection
    xbee(close)