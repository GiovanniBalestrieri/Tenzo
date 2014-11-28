function [ax,ay,az,t,con,cont] = readAcc(out,a)
    t=0;
    ax=0;
    ay=0;
    az=0;    
    cont=0;
    con = false;
    %fwrite(out.s,82);
    if (get(out.s, 'BytesAvailable')>0)
        [sentence,count] = fscanf( out.s, '%s');
        if (~isempty(sentence) && count > 2)
            if (strcmp(sentence(1,1),'R'))
            %decodes "sentence" seperated (delimted) by commaseck Unit')
            [R,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
            con = true;
            elseif (strcmp(deblank(sentence(1,2)), 'T'))
                [T,ack] = strread(sentence,'%s%s',1,'delimiter',',')
                if (strcmp(ack,'A'))
                    con = true;
                elseif strcmp(ack,'B')
                    con = false;
                end
            elseif (strcmp(sentence(1,1),'S'))
                [S,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
                cont = a+1;
                con = true;
            else
                con = false;
            end 
        else
            con = false;
        end
    end
end