function Filter()
    disp('Arduino Connect ');
    baudrate = 9600;    

    count =0;
    delete(instrfindall);
    %% Check to see if there are existing serial objects 
    % (instrfind) whos 'Port' property is set to 'COM3'

    oldSerial = instrfind('Port', 'COM3'); 
    % can also use instrfind() with no input arguments to find 
    % ALL existing serial objects

    % if the set of such objects is not(~) empty
    if (~isempty(oldSerial))  
        disp('WARNING:  port in use.  Closing.')
        delete(oldSerial)
    end

    %%  Setting up serial communication
    % XBee expects the end of commands to be delineated by a carriage return.

    xbee = serial('COM3','baudrate',baudrate,'terminator','CR');

    % Max wait time
    set(xbee, 'TimeOut', 10);  
    % One message long buffer
    set(xbee, 'InputBufferSize',255)
    % Open the serial
    fopen(xbee);    

    %% Testing Wireless communication
    timerXbee = timer('ExecutionMode','FixedRate','Period',0.1,'TimerFcn',{@storeDataFromSerial});
    start(timerXbee);  
    fwrite(xbee,'m');
    disp('Ack Requested.');

    function storeDataFromSerial(obj,event,handles)
        while (get(xbee, 'BytesAvailable')~=0)
            versionArd = typecast([uint8(mess(3)), uint8(mess(4)),uint8(mess(5)), uint8(mess(6))], 'int32');
            if versionProtocol ~= versionArd
                warndlg('Your version is different from the one in Tenzo. Sync your repository.','!! Warning !!') 
            end
            numCmd = mess(7)
            %Arduino  tells the receiver where commands start
            readFrom = mess(8);
    %               disp('ReadFrom');
    %               disp(readFrom);
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


                % Connection Channel
                disp('Connection channel'); 
                conAck = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'int32')
                if (conAck == 2)
                    tenzo = true;

                    %set(connect,'BackgroundColor',[.99 .183 0.09],'String','Disconnect');

                    set(handles.connect,'String','Disconnect');
                    set(handles.conTxt,'ForegroundColor', [.21 .96 .07],'String','Online');    
                    disp ('Connection established. Rock & Roll!'); 
                    if speakCmd && vocalVerb>=1 
                            %tts('Connessione eseguita',voice);
                            tts('Connection Established',voice);
                    end


                    if (count >= (inputBuffSize) && mess(2) == matlabAdd && mess(1) == arduinoAdd)
                        % Mess sent from Arduino to MATLAB
                        % Assemble long int sent bytewise
                        versionArd = typecast([uint8(mess(3)), uint8(mess(4)),uint8(mess(5)), uint8(mess(6))], 'int32');
                        if versionProtocol ~= versionArd
                            warndlg('Your version is different from the one in Tenzo. Sync your repository.','!! Warning !!') 
                        end
                        numCmd = mess(7);
                        %Arduino  tells the receiver where commands start
                        readFrom = mess(8);
        %               disp('ReadFrom');
        %               disp(readFrom);
                        sizeOfEachCmd = mess(9);
                    end
                end
            end
        end
    end
end