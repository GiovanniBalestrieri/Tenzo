function DataAcquisiton()
    clc; 
    global axdata;
    global aydata;
    global azdata;
    global time;
    global serialFlag;
    
    %% Creates window
    handles.hFig = figure('Menubar','none');
    AccData = warning('off', 'MATLAB:uitabgroup:OldVersion');
    
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

    axdata = zeros(buffLen,1);
    aydata = zeros(buffLen,1);
    azdata = zeros(buffLen,1);
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
        handles.plott = uicontrol('Style','togglebutton', 'String','Record', ...
            'Position', [210 50 120 30],...
            'Callback',@plotCallback);
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
    
    function startCallback(obj,event,h)
        disp('start pressed');
        asked = ~asked
        if asked == true
            if serialFlag == 0 
                [acceleration.s,serialFlag] = setupSerial();
            end
        else 
            if serialFlag == 1 
                set(handles.start,'String','Start');
                serialFlag = 0;
            end
        end
        %disp(abs(az));
        %serialFlag

        while asked
            % waiting for the connection to be established
            %disp(abs(az));
%             disp(asked);
%             disp(serialFlag);
            while (serialFlag == 1 && abs(az) >= -0.5)

                % Reads acc data
                [ax ay az t] = readAcc1_05(acceleration);
                figure(2);


                cla;
%                 ax
%                 ay
%                 az
%                 t
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
                
                if ((time(900) >= 0) && (time(1000)>0))
                    figure(3);       
                    title('Acc X');
                    axis([time(950) time(1000) -1.5 1.5]);                
                    xlabel('time [ms]')
                    ylabel('Magnitude of X acc [m*s^-2]');
                    plot (time(901:1000),axdata,'b');
                    grid on;        
                end
                drawnow;
            end
        end
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

        timerArduino = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',{@storeSerial});    

        timerConnect = timer('ExecutionMode','fixedRate','Period',5,'TimerFcn',{@connect},'StopFcn',{@stoppedCon});    
        start(timerConnect);


        function connect(obj,event,handles)
            disp('Establishing connection')
            fwrite(s,16);  
            start(timerArduino);    
        end

        function stoppedCon(obj,event,h)        
            disp('Connection established');
            serialFlag = 1;
             set(handles.start,'String','Stop');
            stop(timerArduino);     
        end

        function storeSerial(obj,event,handles)
            while (get(s, 'BytesAvailable')>0)
                [mess,cont] = fread(s);
                disp('Received bytes:');
                %disp(cont);
                disp(mess);
                if (mess == 17)             
                    display(['Collecting data']);
                    fwrite(s,18);     
                elseif (mess == 19)  
                    stop(connect);
                elseif (mess == 82)  
                    disp('Ricevuto R');
                end
            end
        end
    end

    function stopCallback(obj,event,handles)
        if (~exist('serialFlag','var')) 
            %[acceleration.arduino,serialFlag] = setupSerial1_00();
            disp('disconnect');
        end 
    end

    function plotCallback(obj,event,handles)
        asked = ~asked;        
    end

        
end