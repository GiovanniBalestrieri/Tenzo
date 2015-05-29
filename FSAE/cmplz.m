% Centres étrangers 2014 Ex 3

z0=16;
n=10;
for j=0:n
    if j==0
       zn=z0; 
    else
       disp(j);
       zn=(1+i)*zn/2;
       realZn = real(zn)
       imgZn = imag(zn)
       plot(realZn,imgZn,'rx','LineWidth',10);
       hold on
    end
end

%% Pondichéry 2014

z0 = 1;
n = 10;
raison=3/4+sqrt(3)*i/4
for j=0:n
    if j==0
       zn1=z0; 
    else
       disp(j);
       zn1=raison*zn1;
       realZn = real(zn1)
       imgZn = imag(zn1)
       figure(1)
       plot(realZn,imgZn,'rx','LineWidth',10);       
       str1 = ['Z',j];
       text(realZn,imgZn,str1)
       rn=abs(zn1)
       hold on
    end
end

%% Pondichéry 2014

z0 = sqrt(3)-i;
n = 10;
raison=1+i
p=input('Insert p',s)
while (zn<=p)
    if j==0
       zn1=z0; 
    else
       disp(j);
       zn1=raison*zn1;
       realZn = real(zn1)
       imgZn = imag(zn1)
       figure(1)
       plot(realZn,imgZn,'rx','LineWidth',10);       
       str1 = ['Z',num2str(j)];
       text(realZn+realZn/10,imgZn+imgZn/10,str1)
       rn=abs(zn1)
       if (rn>p)
       figure(2)
       plot(j,rn,'rx','LineWidth',10);
       hold on
    end
end
