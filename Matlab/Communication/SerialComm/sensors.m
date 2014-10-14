function [ xbee ] = sensors( ComPortNumber )
%This function initializes the serial port properly 
% for use with the xbee receiver
%COMPORT is the number of the serial port:   ex use 3 for 'COM3'
% port number can be checked in 
% Control Panel/System/Hardware/DeviceManager/Ports
% xbee output is a matlab serial port object
% Giovanni Balestrieri Aka UserK 03/01/2014
 
port = strcat('COM',num2str(ComPortNumber) );
 
out = instrfind('Port', port);  % Check to see if THAT serial port has already 
% been defined in MATLAB
if (~isempty(out))  % It is
    disp('WARNING:  port in use.  Closing.')
    if (~strcmp(get(out(1), 'Status'),'open'))  % Is it open?
        delete(out(1)); % If not, delete
    else  % is open
        fclose(out(1));
        delete(out(1)); 
    end
end


%%  Setting up serial communication
% XBee expects the end of commands to be delineated by a carriage return.
s = serial('COM3','baudrate',57600,'terminator','CR','tag','Quad');

set(s, 'TimeOut', 1.5);  %I am willing to wait 1.5 secs for data to arrive
% I wanted to make my buffer big enough to store one message
set(s, 'InputBufferSize',  390 )
% Before you can write to your serial port, you need to open it:
fopen(s);

pause(1)  % give it a second to start getting data
disp('Xbee Initialized correctly')
