function DataAcquisiton()
    clc; 
    global axdata;
    global aydata;
    global axFData;
    global angleData;
    global azdata;
    global AccX;
    global AccY;
    global AccZ;
    global time;
    global serialFlag;
    global samplesNum;
    global axF;
    global angle;
    global samplesNumMax;
    global timerArduino;
    global timerConnect;
    global requestPending;
    global firing;
    
    %% Creates window
    handles.hFig = figure('Menubar','none');
    AccData = warning('off', 'MATLAB:uitabgroup:OldVersion');
    
    samplesNumMax = 1000;
    acceleration.s = 0;
    ax=0;
    ay=0;
    az=0;
    axF=0;
    angle=0;
    t=0;
    serialFlag=0;
    prevTimer = 0;
    deltaT = 0;
    contSamples = 0;
    
    buffLen = 300;
    longBuffLen = 300;
    index=1:buffLen;
    asked = false;
    rt = false;
    plotting = false;
    receiving = false;
    requestPending = false;
    firing = false;
    
    AccX = zeros(buffLen,1);
    AccY = zeros(buffLen,1);
    AccZ = zeros(buffLen,1);
    axdata = zeros(buffLen,1);
    aydata = zeros(buffLen,1);
    azdata = zeros(buffLen,1);
    axFData = zeros(buffLen,1);
    angleData = zeros(buffLen,1);
    time  = zeros(buffLen,1);
    
    delete(instrfindall);
    
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
        rt = ~rt
    
        % waiting for the connection to be established
        disp(serialFlag);
        while (abs(az) >= -0.5)               
            % Request High Res data             
            if rt
                fwrite(acceleration.s,83);
                
                [ax axF angle t,firing,contSamplesS] = readAcc1_15_F_Angle(acceleration,contSamples);
                %firing
                %contSamples
                while (firing && rt)
                    
                    figure(6);
                    cla;

                    axdata = [ axdata(2:end) ; ax ];
                    axFData = [ axFData(2:end) ; axF ];
                    angleData = [ angleData(2:end) ; angle ];    
                    time   = [time(2:end) ; t];

                    deltaT = t - prevTimer;
                    prevTimer = t;
                    h11 = subplot(3,1,1);

                    title('Acc X');    
                    axis([1 buffLen -0.5 0.5]);
                    xlabel('time [ms]')
                    ylabel('Magnitude of X acc [m*s^-2]'); 
                    plot(h11,index,axdata,'r');
                    grid on;

                    h12 = subplot(3,1,2);

                    title('Acc X Filtered');      
                    axis([1 buffLen -0.5 0.5]);          
                    xlabel('time [ms]')
                    ylabel('Magnitude of X acc [m*s^-2]');
                    plot(h12,index,axFData,'r');
                    grid on;         

                    h13 = subplot(3,1,3);

                    title('Theta');
                    axis([1 buffLen -1.5 1.5]);                
                    xlabel('time [ms]')
                    ylabel('Theta angle [m*s^-2]');
                    plot(h13,index,angleData,'r');
                    grid on;         

                    if ((time(1) >= 0) && (time(end)>0))
                        figure(5);       
                        title('Acc X');
                        axis([time(1) time(end) -1.5 1.5]);                
                        xlabel('time [ms]')
                        ylabel('Magnitude of X acc [m*s^-2]');
                        plot(time(1:end),angleData(1:end),'b');
                        grid on;        
                    end
                    drawnow;                    
                    [ax axF angle t,firing,contSamples] = readAcc1_15_F_Angle(acceleration,contSamples);
                    contSamples
                    firing
                    rt=firing;
                end
            end
            if ~rt
                disp('saving samples to file');
                accDataToWrite = [axdata,time];
                csvwrite('accs.txt',accDataToWrite);
                disp('saving file to structure accSamples.mat');
                dat.x = axdata;
                dat.y = axFData;
                dat.z = angleData;
                dat.time = time;
                save('accData1.0.mat','-struct','dat');
                disp(deltaT);
                % Reset Arrays
                axdata= zeros(buffLen,1);
                axFData = zeros(buffLen,1);
                angleData = zeros(buffLen,1);
                time  = zeros(buffLen,1);
                break;
            end
        end
    end

    function recordCallback(obj,event,handles)
        disp('Record Pressed');
        plotting = ~plotting
        serialFlag;
        if plotting == 1
            % waiting for the connection to be established
            %disp(abs(az));
            %disp(asked);     
            while (serialFlag == 1 && abs(az) >= -0.5)               
                % Request High Res data 
                fwrite(acceleration.s,84);
                
                [ax ay az t,receiving] = readAcc1_05(acceleration);
                receiving
                if (receiving)
                    figure(3);

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
                    plot(h11,index,axdata,'r');
                    grid on;

                    h12 = subplot(3,1,2);

                    title('Acc Y');      
                    axis([1 buffLen -0.5 0.5]);          
                    xlabel('time [ms]')
                    ylabel('Magnitude of Y acc [m*s^-2]');
                    plot(h12,index,aydata,'r');
                    grid on;         

                    h13 = subplot(3,1,3);

                    title('Acc Z');
                    axis([1 buffLen -1.5 1.5]);                
                    xlabel('time [ms]')
                    ylabel('Magnitude of Z acc [m*s^-2]');
                    plot(h13,index,azdata,'r');
                    grid on;         

                    if ((time(1) >= 0) && (time(end)>0))
                        figure(4);       
                        title('Acc X');
                        axis([time(1) time(end) -1.5 1.5]);                
                        xlabel('time [ms]')
                        ylabel('Magnitude of X acc [m*s^-2]');
                        plot (time,axdata,'b');
                        grid on;        
                    end
                    drawnow;                    
                end
                if ~plotting
                    disp('saving samples to file');
                    accDataToWrite = [axdata,time];
                    csvwrite('accx.txt',accDataToWrite);
                    disp('saving file to structure');
                    dat.x = axdata;
                    dat.y = aydata;
                    dat.z = azdata;
                    dat.time = time;
                    save('AccSamples.mat','-struct','dat');
                    disp(deltaT);

                    % Reset Arrays
                    AccX = zeros(buffLen,1);
                    AccY = zeros(buffLen,1);
                    AccZ = zeros(buffLen,1);
                    axdata = zeros(buffLen,1);
                    aydata = zeros(buffLen,1);
                    azdata = zeros(buffLen,1);
                    time  = zeros(buffLen,1);
                    break;
                end
            end
        end
    end
    
    function startCallback(obj,event,h)
        disp('start pressed');
        asked = ~asked;
        delete(timerfindall);
        if asked == true
            if serialFlag == 0 
                [acceleration.s] = setupSerial();
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
    
    function [s] = setupSerial()
        %comPortLinux = '/dev/ttyACM0';
        comPortWin = 'COM4';
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
        %start(timerConnect);
    end


        function connect(obj,event,h)
            if serialFlag == 0 && requestPending == false
                if isvalid(acceleration.s) == 1
                    fwrite(acceleration.s,90); 
                end
                if (strcmp(get(timerArduino,'Running'),'off'))
                    start(timerArduino);    
                end
            end
        end

        function stoppedCon(obj,event,h)       
            serialFlag = 1;
            requestPending = false;
            stop(timerArduino);   
            stop(timerConnect);      
            set(handles.start,'String','Stop');
            disp('Connection established');
        end

        function storeSerial(obj,event,handles)            
            while (get(acceleration.s, 'BytesAvailable')==1)
                [mess,cont] = fread(acceleration.s)
                %disp('Received bytes:');
                %disp(cont);
                %disp(mess);
                if (mess == 17)             
                    %display(['Collecting data']);
                    fwrite(acceleration.s,18); 
                    requestPending = true;   
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