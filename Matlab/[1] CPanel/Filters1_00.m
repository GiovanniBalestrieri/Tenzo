function Filter()
global InputBufferSize;
global arduinoAdd;
global matlabAdd;
global axData;
global ayData;
global azData;
global time;
global axFilt;
global ayFilt;
global azFilt;
global buf_len;
global index;
global alphaAcc;

arduinoAdd = 1;
matlabAdd = 2;
InputBufferSize = 48;

buf_len = 100;
index = 1:buf_len;
alphaAcc = 1;

axData = zeros(buf_len,1);
ayData = zeros(buf_len,1);
azData = zeros(buf_len,1);
time = zeros(buf_len,1);

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

    xbee = serial('COM4','baudrate',baudrate,'terminator','CR');

    % Max wait time
    set(xbee, 'TimeOut', 10);  
    % One message long buffer
    set(xbee, 'InputBufferSize',InputBufferSize)
    % Open the serial
    fopen(xbee);    

    timerXbee = timer('ExecutionMode','FixedRate','Period',1,'TimerFcn',{@storeDataFromSerial});
    start(timerXbee);  
    pause();
    fwrite(xbee,'A');
    
    %%  
    
    function storeDataFromSerial(obj,event,handles)
        while (get(xbee, 'BytesAvailable')~=0)
            [mess,count] = fread(xbee);
            disp('Reading incoming buffer. Dimensions:');
            % Debug stuf
            disp(count);
            %disp(mess(2));
            %disp(mess(1));
            disp(mess);
            if (count == (InputBufferSize))
                versionArd = typecast([uint8(mess(3)), uint8(mess(4)),uint8(mess(5)), uint8(mess(6))], 'int32');
                numCmd = mess(7);
                %Arduino  tells the receiver where commands start
                readFrom = mess(8);
        %               disp('ReadFrom');
        %               disp(readFrom);
        
                sizeOfEachCmd = mess(9);
                totMessLength = typecast([uint8(mess(10)), uint8(mess(11))], 'int16');
                disp('Total message length (header value):');
                disp(totMessLength);
                % TODO CRC check for message's integrity
                %crcvalue = typecast([uint8(mess(12)), uint8(mess(13))], 'int16');
                %disp('CRC value');
                %disp(crcvalue);

                %% Read Commands
                for i = 0:(numCmd-1)
                    typei = (readFrom+1)+i*sizeOfEachCmd;
                    type = mess(typei);
                    %disp('Message #:');
                    %disp(i+1);
                    %disp('Connection channel');
                    if (type == 32)
                        % Pid Roll AGG
                        x = typecast([uint8(mess(typei + 1)), uint8(mess(typei + 2)),uint8(mess(typei + 3)), uint8(mess(typei + 4))], 'single')                        
                        y = typecast([uint8(mess(typei + 5)), uint8(mess(typei + 6)),uint8(mess(typei + 7)), uint8(mess(typei + 8))], 'single')                        
                        z = typecast([uint8(mess(typei + 9)), uint8(mess(typei + 10)),uint8(mess(typei + 11)), uint8(mess(typei + 12))], 'single')                        
                        t = typecast([uint8(mess(typei + 13)), uint8(mess(typei + 14)),uint8(mess(typei + 15)), uint8(mess(typei + 16))], 'single')
                        
                        % Raw Data
                        axData = [ axData(2:end) ; x ];
                        ayData = [ ayData(2:end) ; y ];
                        azData = [ azData(2:end) ; z ]; 
                        
                        %Filt Data
                        axFilt = (1 - alpha)*axFilt + alpha*x;
                        ayFilt = (1 - alpha)*ayFilt + alpha*y;
                        azFilt = (1 - alpha)*azFilt + alpha*z;
                        %Plot
                        
                        time = [ time(2:end) ; t ];
                        %Plot the X magnitude
                            h1 = subplot(3,1,1);
                            %set(hAx,'title','X angular velocity in deg/s');
                            plot(h1,time,axData,'r','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                            hold on;
                            plot(h1,time,axFiltData,'r','LineWidth',1);
                            hold off;
                            %xlabel('Time');
                            %ylabel('Wx');
                            %axis([1 buf_len -80 80]);
                            %hold on;
                            h2 = subplot(3,1,2);
                            %title('Y angular velocity in deg/s');
                            plot(h2,time,ayData,'b','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                            hold on;
                            plot(h1,time,ayFiltData,'r','LineWidth',1);
                            hold off;
                            %xlabel('Time');
                            %ylabel('Wy Acc');
                            %axis([1 buf_len -80 80]);
                            h3 = subplot(3,1,3,'Parent',hTabs(3));
                            %title('Z angular velocity in deg/s');
                            %hold on;
                            plot(h3,time,azData,'g','LineWidth',2);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
                            hold on;
                            plot(h3,time,azFiltData,'g','LineWidth',2);
                            hold off;
                            %axis([1 buf_len -80 80]);
                            %xlabel('Time');
                            %ylabel('Wz Acc');
                    end
                end
            end
        end %while
    end
end