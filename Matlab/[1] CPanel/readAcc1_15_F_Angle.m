function [ax,axF,angle,t,con,cont] = readAcc(out,a)
    t=0;
    ax=0;
    axF=0;
    angle=0;    
    cont=0;
    con = false;
    if (get(out.s, 'BytesAvailable')>0)
        [sentence,count] = fscanf( out.s, '%s');
        disp('Mess');
        disp(sentence);
        if (~isempty(sentence))
            if (strcmp(sentence(1,1),'R'))
            %decodes "sentence" seperated (delimted) by commaseck Unit')
            [R,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
            con = true;
            elseif (strcmp(sentence(1,1),'S'))
                [S,ax,ay,az,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
                cont = a+1;
                con = true;
            elseif (strcmp(sentence(1,1),'X'))
                [S,ax,axF,angle,t] = strread(sentence,'%s%f%f%f%f',1,'delimiter',',');
                cont = a+1;
                con = true;
            elseif (strcmp(sentence(1,1),'U'))
                con = false;
            elseif (strcmp(sentence(1,1),'L'))
                disp('VAIII');
            else
                con = false;
            end  
        else
            con = false;
        end
    end
end