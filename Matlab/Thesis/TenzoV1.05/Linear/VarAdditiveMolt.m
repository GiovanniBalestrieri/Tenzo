%% Esempio di deduzione di bounds moltiplicativi lm

clear all;
clc;
close all;

%Carica il modello di due massa-molla-smorzatore connessi in serie
%e il modello delle incerteze sull'attuatore
%Modello di due massa-molla-smorzatore connessi in serie
%

m1=1;       %Massa
m2=1;
k1=0;       %Costante elastica
k2=0;
b1=1;       %Attrito viscoso
b2=1;
a1=1;       %Guadagno statico dell'attuatore
a2=1;

A=[zeros(2,2) eye(2); -(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1; k2/m2 -k2/m2 b1/m2 -b2/m2];
B=[zeros(2,2); a1/m1 0;0 a2/m2];
C=[eye(2) zeros(2,2)];
D=zeros(2,2);

modello_nominiale=ss(A,B,C,D)


N=10
p1 = ureal('p1',0,'Range',[-0.1 0.1]); 
p2 = ureal('p2',0,'Range',[-0.2 0.2]); 
% deltaA = [0, p1, p1, p1;
%           p2, 0, p2, p2;
%           p1, 0, 0, 0;
%           p2, 0, 0, 0];
deltaA = [0, p1, p1, p1;
          p1, 0, p1, p1;
          p1, 0, 0, 0;
          p1, 0, 0, 0];
deltaB = [p1, p2;
          p1, p2;
          p2, 0;
          0, p2];  
usys = uss(A+deltaA,B+deltaB,C,0);      
for i=1:1:N
sys{i} = usample(usys);
end


for i=1:1:N
deltaA_sys{i} = tf(sys{i}) - tf(modello_nominiale);
deltaM_sys{i} = (inv(tf(modello_nominiale))) * deltaA_sys{i};
end

figure
for i=1:1:10
sigma(deltaA_sys{i});
hold on;
end
grid on;

pause;

wI = 10^-6
wF = 10^4

[SEmodello_nominale,w] = sigma(modello_nominiale,{wI,wF});
figure
for i=1:1:1
SV{i} = sigma(deltaM_sys{i},w);
sigma(deltaM_sys{i},w);
hold on;
end
grid on;

pause;

for j=1:1:length(w)
    for i = 1:1:N
        value(1,i) = SV{i}(1,j); 
    end
    SV_max(1,j) = max(value);
end
for j=1:1:length(w)
    SV_max(1,j) = 20*log10(SV_max(1,j));
end

semilogx(w,SV_max,'k--','LineWidth',2)
grid on;

lm = 1.5*tf([1000 1],[1])*tf([1],[750 1])*tf([1],[5 1])*tf([1 1],[1])*tf([1 1],[1])*tf([1],[0.001 1])
[lm_value,lm_w] = sigma(lm);
for j=1:1:length(lm_w)
    lm_module(1,j) = 20*log10(lm_value(1,j));
end

semilogx(lm_w,lm_module,'r','LineWidth',2)
% sigma(lm,'r','Linewidth',2)

% lm_value2 = sigma(lm,w);
% for j=1:1:length(w)
%     difference(1,j) = SV_max(1,j)-lm_value2(1,j);
% end
% 
% pause; 
% 
% plot(w,difference,'k','LineWidth',2)
% grid on;
