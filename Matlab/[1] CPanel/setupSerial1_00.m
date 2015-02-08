function [arduino,flag] = setupSerial()
    global serialFlag;
    delete(instrfindall);
    %comPortLinux = '/dev/ttyACM0';
    comPortWin = 'COM4';
    flag = 0;
    connected = false;

    oldSerial = instrfind('Port', comPortWin); 
    % if the set of such objects is not(~) empty
    if (~isempty(oldSerial))  
        disp('WARNING:  COM4 in use.  Closing.')
        delete(oldSerial)
    end
    arduino = serial(comPortWin);

    % Max wait time
    set(arduino, 'TimeOut', 5); 
    set(arduino,'terminator','CR');
    set(arduino,'BaudRate',9600);
    fopen(arduino);

    disp('Sending Request.');
    
    timerArduino = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',{@storeSerial});    
    
    timerConnect = timer('ExecutionMode','fixedRate','Period',5,'TimerFcn',{@connect},'StopFcn',{@stoppedCon});    
    start(timerConnect);
    
        
    function connect(obj,event,handles)
        disp('Establishing connection')
        fwrite(arduino,16);  
        start(timerArduino);    
    end
    
    function stoppedCon(obj,event,handles)        
        disp('Connection established');
        serialFlag = 1;
        stop(timerArduino);     
    end

    function storeSerial(obj,event,handles)
        while (get(arduino, 'BytesAvailable')>0)
            [mess,cont] = fread(arduino);
            disp('Received bytes:');
            %disp(cont);
            disp(mess);
            if (mess == 17)             
                display(['Collecting data']);
                fwrite(arduino,18);     
            elseif (mess == 19)  
                stop(connect);
                connected = true;
            elseif (mess == 82)  
                disp('Ricevuto R');
            end
        end
    end
end