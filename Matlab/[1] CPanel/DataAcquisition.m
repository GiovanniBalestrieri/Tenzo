function DataAcquisiton()
    clc; 
    global axdata;
    global aydata;
    global azdata;
    global AccX;
    global AccY;
    global AccZ;
    global time;
    global serialFlag;
    global samplesNum;
    global samplesNumMax;
    global timerArduino;
    global timerConnect;
    
    %% Creates window
    handles.hFig = figure('Menubar','none');
    AccData = warning('off', 'MATLAB:uitabgroup:OldVersion');
    
    
    samplesNumMax = 1000;
    acceleration.s = 0;
    ax=0;
    ay=0;
    az=0;
    t=0;
    serialFlag=0;
    prevTimer = 0;
    deltaT = 0;
    
    buffLen = 100;
    longBuffLen = 1000;
    index=1:buffLen;
    asked = false;
    recording = false;
    receiving = false;
    
    AccX = zeros(samplesNumMax,1);
    AccY = zeros(samplesNumMax,1);
    AccZ = zeros(samplesNumMax,1);
    axdata = zeros(longBuffLen,1);
    aydata = zeros(longBuffLen,1);
    azdata = zeros(longBuffLen,1);
    time  = zeros(longBuffLen,1);
    
    %# Home UI components
    if (~exist('welcomeTxt','var'))
        welcomeTxt = uicontrol('Style','text', 'String','Data Acquisition', ...
            'Position', [55 310 420 50],...
            'FontSize',20,'FontWeight','bold');
    end

    if (~exist('versionTxt','var'))
        versionTxt = uicontrol('Style','text', 'String',version, ...
            'Position', [55 268 420 50],...
            'FontSize',10,'FontWeight','normal');
    end

    if (~exist('handles.start','var'))
        handles.start = uicontrol('Style','pushbutton', 'String','Start', ...
            'Position', [70 50 120 30],...
            'Callback',@startCallback);
    end

    if (~exist('handles.plot','var'))
        handles.record = uicontrol('Style','togglebutton', 'String','Record', ...
            'Position', [210 50 120 30],...
            'Callback',@recordCallback);
    end
    
    if (~exist('handles.plot','var'))
        handles.realTime = uicontrol('Style','togglebutton', 'String','Real time', ...
            'Position', [210 90 120 30],...
            'Callback',@rtCallback);
    end

    if (~exist('handles.stop','var'))
        handles.stop = uicontrol('Style','pushbutton', 'String','Stop', ...
            'Position', [350 50 120 30],...
            'Callback',@stopCallback);
    end



    if ((~exist('h','var') || ~ishandle(h))&& asked)
        h = figure(1);
        ax = axes('box','on');    
    end
    
    function rtCallback(obj,event,h)
        disp('RT pressed');
        asked = ~asked
    
        while asked
            % waiting for the connection to be established
            %disp(abs(az));
            %disp(asked);       
            %disp(serialFlag);
            while (serialFlag == 1 && abs(az) >= -0.5)
                
                % Request acc data RT
                fwrite(acceleration.s,82);
                [ax ay az t,receiving] = readAcc1_05(acceleration);
                figure(2);

                cla;

                axdata = [ axdata(2:end) ; ax ];
                aydata = [ aydata(2:end) ; ay ];
                azdata = [ azdata(2:end) ; az ];    
                time   = [time(2:end) ; t/1000];

                deltaT = t - prevTimer
                prevTimer = t;
                h11 = subplot(3,1,1);

                title('Acc X');    
                axis([1 buffLen -0.5 0.5]);
                xlabel('time [ms]')
                ylabel('Magnitude of X acc [m*s^-2]'); 
                plot(h11,index,axdata(901:1000),'r');
                grid on;

                h12 = subplot(3,1,2);

                title('Acc Y');      
                axis([1 buffLen -0.5 0.5]);          
                xlabel('time [ms]')
                ylabel('Magnitude of Y acc [m*s^-2]');
                plot(h12,index,aydata(901:1000),'r');
                grid on;         

                h13 = subplot(3,1,3);

                title('Acc Z');
                axis([1 buffLen -1.5 1.5]);                
                xlabel('time [ms]')
                ylabel('Magnitude of Z acc [m*s^-2]');
                plot(h13,index,azdata(901:1000),'r');
                grid on;         

                if ((time(900) >= 0) && (time(1000)>0))
                    figure(3);       
                    title('Acc X');
                    axis([time(950) time(1000) -1.5 1.5]);                
                    xlabel('time [ms]')
                    ylabel('Magnitude of X acc [m*s^-2]');
                    plot (time(901:1000),axdata(901:1000),'b');
                    grid on;        
                end
                drawnow;
            end
        end
    end

    function recordCallback(obj,event,handles)
        disp('Record Pressed');
        recording = ~recording
        SerialFlag;
        while recording
            % waiting for the connection to be established
            %disp(abs(az));
            %disp(asked);       
            disp(serialFlag);
            while (serialFlag == 1 && abs(az) >= -0.5)               
                % Request High Res data 
                fwrite(acceleration.s,84);
                
                [ax ay az t,receiving] = readAcc1_05(acceleration)
                while (receiving)
                %while (ax ~= 0 && ay ~= 0 && az ~= 0)
                    [ax ay az t,receiving] = readAcc1_05(acceleration);
                    figure(2);

                    cla;

                    axdata = [ axdata(2:end) ; ax ];
                    aydata = [ aydata(2:end) ; ay ];
                    azdata = [ azdata(2:end) ; az ];    
                    time   = [time(2:end) ; t/1000];

                    deltaT = t - prevTimer
                    prevTimer = t;
                    h11 = subplot(3,1,1);

                    title('Acc X');    
                    axis([1 buffLen -0.5 0.5]);
                    xlabel('time [ms]')
                    ylabel('Magnitude of X acc [m*s^-2]'); 
                    plot(h11,index,axdata(901:1000),'r');
                    grid on;

                    h12 = subplot(3,1,2);

                    title('Acc Y');      
                    axis([1 buffLen -0.5 0.5]);          
                    xlabel('time [ms]')
                    ylabel('Magnitude of Y acc [m*s^-2]');
                    plot(h12,index,aydata(901:1000),'r');
                    grid on;         

                    h13 = subplot(3,1,3);

                    title('Acc Z');
                    axis([1 buffLen -1.5 1.5]);                
                    xlabel('time [ms]')
                    ylabel('Magnitude of Z acc [m*s^-2]');
                    plot(h13,index,azdata(901:1000),'r');
                    grid on;         

                    if ((time(900) >= 0) && (time(1000)>0))
                        figure(3);       
                        title('Acc X');
                        axis([time(950) time(1000) -1.5 1.5]);                
                        xlabel('time [ms]')
                        ylabel('Magnitude of X acc [m*s^-2]');
                        plot (time(901:1000),axdata(901:1000),'b');
                        grid on;        
                    end
                    drawnow;
                end
            end
        end
        
%         fileID = fopen('accx.txt','w');
%         
%         fprintf(fileID,'%6s %12s\n','x','exp(x)');
%         fprintf(fileID,'%6.2f %12.8f\n',A);
%         fclose(fileID);
    end
    
    function startCallback(obj,event,h)
        disp('start pressed');
        asked = ~asked
        if asked == true
            if serialFlag == 0 
                [acceleration.s,serialFlag] = setupSerial();
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
    
    function [s,flag] = setupSerial()
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
        s = serial(comPortWin);

        % Max wait time
        set(s, 'TimeOut', 5); 
        set(s,'terminator','CR');
        set(s,'BaudRate',9600);
        fopen(s);

        disp('Sending Request.');
        acceleration.s = s;
        timerArduino = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',{@storeSerial});    

        timerConnect = timer('ExecutionMode','fixedRate','Period',5,'TimerFcn',{@connect});%,'StopFcn',{@stoppedCon});    
        start(timerConnect);
    end


        function connect(obj,event,h)
            disp('Establishing connection')
            fwrite(acceleration.s,16);  
            if (get(timerArduino,'Running') == 'off')
                start(timerArduino);    
            end
        end

        function stoppedCon(obj,event,h)        
            disp('Connection established');
            serialFlag = 1
            set(handles.start,'String','Stop');
            stop(timerArduino);     
        end

        function storeSerial(obj,event,handles)
            while (get(acceleration.s, 'BytesAvailable')>0)
                [mess,cont] = fread(acceleration.s);
                disp('Received bytes:');
                %disp(cont);
                disp(mess);
                if (mess == 17)             
                    display(['Collecting data']);
                    fwrite(acceleration.s,18);     
                elseif (mess == 19)  
                    stoppedCon();
                elseif (mess == 82)  
                    disp('Ricevuto R');
                end
            end
        end

    function stopCallback(obj,event,handles)
        if (~exist('serialFlag','var')) 
            %[acceleration.arduino,serialFlag] = setupSerial1_00();
            disp('disconnect');
        end 
    end           
end