function [ax,ay,az,t,con] = readAcc(out)
    t=0;
    ax=0;
    ay=0;
    az=0;    
    %fwrite(out.s,82);
    if (get(out.s, 'BytesAvailable')~=0)
        sentence = fscanf( out.s, '%s');
        if (strcmp(sentence(1,1),'R'))
        %decodes "sentence" seperated (delimted) by commaseck Unit')
        [R,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
        elseif (strcmp(sentence(1,1),'T'))
            [T,ack] = strread(sentence,'%s%s',1,'delimiter',',')
            if (strcmp(ack,'A'))
                con = true;
            elseif strcmp(ack,'B')
                con =false;
            end
        end        
    end
end