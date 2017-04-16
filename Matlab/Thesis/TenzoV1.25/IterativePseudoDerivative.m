function dy = IterativePseudoDerivative(T,y,c,d,mediana,reset)

%T: sampling time (equally spaced samples)
%y: actual data
%c  the length of the two subwindows where the mean/median is evaluated
%d  is total number of data considered to evaluate the pseudo-derivative, d>=2c
%reset: =1 if the data buffer has to be discarded, 0 otherwise
%mediana = 1 the median is evaluated on the burst of the c samples,
%otherwise the mean

% (------d------)
% (-c-)-----(-c-)
% [---]-----[---]


persistent mybuffer counter

if(isempty(mybuffer) || reset == 1)
    mybuffer = zeros(1,d);
    counter = 0;
end



counter = counter +1;
%FUNZIONE PER IL CALCOLO DELLA PSEUDODERIVATA
%----------------------------------
%update the buffer
for k=1:d-1
    mybuffer(k) = mybuffer(k+1);
end
mybuffer(d) = y;

if(counter >= d)%enough data has been collected
    
    if(mediana==1)
        temp1 = median(mybuffer(1:c));
        temp2 = median(mybuffer(d-c+1:d));
    else
        temp1 = 0;
        for k=1:c
            temp1 = temp1 + mybuffer(k);
        end
        temp1 = temp1/c;
        
        temp2 = 0;
        for k=d-c+1:d
            temp2 = temp2 + mybuffer(k);
        end
        temp2 = temp2/c;
    end
    
    dy = (temp2-temp1) / (T*(d-c));
else
    dy = 0;
end
%-------------------------------------
