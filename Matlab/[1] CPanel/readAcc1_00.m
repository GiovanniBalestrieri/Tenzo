function [ax,ay,az,t] = readAcc(out)
    t=0;
    ax=0;
    ay=0;
    az=0;
    
    timerAcc = timer('ExecutionMode','fixedRate','Period',0.05,'TimerFcn',{@storeAcc});
    start(timerAcc);
    
%     while (read & abs(wx) < 1000)% abs(wy) < 1000)
%     %% Polling 
%     fprintf(xbee, 'M') ; 
%     try
%         while (get(xbee, 'BytesAvailable')~=0)
%             % read until terminator
%             sentence = fscanf( xbee, '%s'); % this reads in as a string (until a terminater is reached)
%             if (strcmp(sentence(1,1),'R'))
%             %decodes "sentence" seperated (delimted) by commaseck Unit')
%             [Roll, theta, Pitch, pitch, Yaw, yaw, OmegaX, wx, OmegaY, wy, OmegaZ, wz, AccX, ax, AccY, ay, AccZ, az] = strread(sentence,'%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f',1,'delimiter',',');

   
    function storeAcc(obj,event,handles) 
        fwrite(out.s,82);
        try
            while (get(out.s, 'BytesAvailable')>0)
                sentence = fscanf( out.s, '%s');
                if (strcmp(sentence(1,1),'R'))
                %decodes "sentence" seperated (delimted) by commaseck Unit')
                [R,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
                
%                 disp(ax);
%                 disp(ay);
%                 disp(az);
%                 disp(t);
                
                
%                 [mess,cont] = fread(out.s);
%                 disp('Received bytes:');
%                 disp(cont);
%                 disp(mess);
%                 if (mess(1) == 1)             
%                     display(['Acc data:']);
%                     ax = typecast([int8(mess(2)), int8(mess(3)),int8(mess(4)), int8(mess(5))], 'single')
% 
%                     ay = typecast([int8(mess(6)), int8(mess(7)),int8(mess(8)), int8(mess(9))], 'single')
% 
%                     az = typecast([int8(mess(10)), int8(mess(11)),int8(mess(12)), int8(mess(13))], 'single')
% 
%             % Reads values from accelerometer
%             R = fscanf(out.s,'%s')
%             R = fscanf(out.s,'%s')
%             ax = fscanf(out.s,'%u')
%             R = fscanf(out.s,'%s')
%             ay = fscanf(out.s,'%u')
%             R = fscanf(out.s,'%s')
%             az = fscanf(out.s,'%u')
                end
            end
        end
    end
end