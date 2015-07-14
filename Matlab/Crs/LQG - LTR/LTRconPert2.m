%% Esempio di progetto LQR per garantire robustezza per perturbazioni
%%moltiplicative sull'in con bound l_m crescente

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

modello_ss=ss(A,B,C,D)

disp('Eigenvalues of the matrix A')
disp(eig(A))

disp('Transfer matrix of the model')
modello_tf=tf(modello_ss)


% Verifica di rilevabilita' e stabilizzabilita'
disp('Reachability and Observability')
ctrb(A,B)
rank(ctrb(A,B))
ctrb(A',C')
rank(ctrb(A',C'))

figure
sigma(modello_tf)
grid on

%%
close all;
disp('Consider the following lm bound')
w=logspace(-2,2,100);
lm = tf([1 1], 1.1);

figure
sigma(lm,'r--')
hold on
grid on

example = [tf(0.1*[1 3 5 5], [1 1 1]); tf([1 2], [1 3 5]); tf(0.2*[1 3 5 3], [1 0.7 1])]
sigma(example(1,1),'b')
sigma(example(2,1),'b')
sigma(example(3,1),'b')

%%

close all;
disp('Design of the LQR controller')

rho = 10;
Q = C'*C;
R = rho*eye(2);
Kopt = lqr(A,B,Q,R);
H = ss(A,B,Kopt,zeros(2,2));

CL = feedback(H,eye(2));
w=logspace(-2,2,100);
lm = tf([1 1], 1.1)


figure
hold on
sigma(H,'b')
sigma(lm,'r--')
sigma(CL,'k')
sigma(1/lm,'r')
grid on
legend('Anello aperto','Bound l_m','U0','1/lm')


%%
close all;
disp('Performance of the LQR controller')

p1 = 0.1*rho;
p2 = rho;
p3 = 10*rho;

K1 = lqr(A,B,Q,p1*eye(2));
K2 = lqr(A,B,Q,p2*eye(2));
K3 = lqr(A,B,Q,p3*eye(2));

H1 = ss(A,B,K1,zeros(2,2));
H2 = ss(A,B,K2,zeros(2,2));
H3 = ss(A,B,K3,zeros(2,2));

CL1 = feedback(H1,eye(2));
CL2 = feedback(H2,eye(2));
CL3 = feedback(H3,eye(2));

figure 
step(CL1,'k')
hold on;
step(CL2,'r')
step(CL3,'k--')
legend('R=0.1*rho*I','R= rho*I','R=10*rho*I')

grid on;

%%
close all;
disp('Rubustness of the LQR controller')

H0 = ss(A,B,Kopt,zeros(2,2));
nominale=feedback(H0,eye(2));

perturbato1=feedback(H,eye(2)+example(1,1)*eye(2));
perturbato2=feedback(H,eye(2)+example(2,1)*eye(2));
perturbato3=feedback(H,eye(2)+example(3,1)*eye(2));

% figure
% hold on;
% sigma(example(1,1))
% sigma(example(2,1))
% sigma(example(3,1))
% sigma(lm,'k')
% grid on 

figure
step(nominale,'r');
grid on;
hold on
step(perturbato1,'b');
step(perturbato2,'b');
step(perturbato3,'b');
legend('Nominale','Perturbato')

%%
close all;
disp('Rubustness of the LQR controller: excess of perturbation')

% example(4,1) = tf(0.4*[ 1 1 1 3 5 3], [1 1 3 2])
% prova per 1,10,20,40,70
example(4,1) = 100*tf([1 -2 1], [1])*tf([1], [1 20 100])

figure
sigma(example(4,1))
grid on
hold on
sigma(lm,'r')

pause;
close all;

perturbato4=feedback(H,eye(2)+example(4,1)*eye(2));
figure
step(nominale,'r');
grid on;
hold on
step(perturbato4,'k');
legend('Nominale','Perturbato')

%%
close all;
disp('LTR recovery')

sigmas1=0.1;           
W1=sigmas1*B*B';
sigmas2=1000;           
W2=sigmas2*B*B';

V=eye(2);
L1= lqr(A',C',W1,V)';
L2= lqr(A',C',W2,V)';

Ac1 = A-B*Kopt-L1*C;
Ac2 = A-B*Kopt-L2*C;

Bc1 = L1;
Bc2 = L2;

Cc = Kopt;
Dc = zeros(2,2);

controllore1 = ss(Ac1,Bc1,Cc,Dc);
controllore2 = ss(Ac2,Bc2,Cc,Dc);

H_recovery1 = series(modello_ss,controllore1);
H_recovery2 = series(modello_ss,controllore2);

sigma(H_recovery1,'b')
hold on;
grid on;
sigma(H_recovery2,'k')
sigma(H0,'r');
legend('LTR: sigma = 0.1','LTR: sigma = 1000','LQR')

%%
close all;
disp('LTR recovery: robustness, sigma=0.1;')

nominaleLTR=feedback(H_recovery1,eye(2));
perturbato1=feedback(H_recovery1,eye(2)+example(1,1)*eye(2));
perturbato2=feedback(H_recovery1,eye(2)+example(2,1)*eye(2));
perturbato3=feedback(H_recovery1,eye(2)+example(3,1)*eye(2));

figure
step(nominale,'r--');
grid on;
hold on
step(nominaleLTR,'r');
step(perturbato1,'k');
step(perturbato2,'k');
step(perturbato3,'k');
legend('LQR Nominale','LTR nominale: sigma = 0.1','LTR perturbato: sigma = 0.1')


%%
close all;
disp('LTR recovery: robustness, sigma=1000;')

nominaleLTR=feedback(H_recovery2,eye(2));
perturbato1=feedback(H_recovery2,eye(2)+example(1,1)*eye(2));
perturbato2=feedback(H_recovery2,eye(2)+example(2,1)*eye(2));
perturbato3=feedback(H_recovery2,eye(2)+example(3,1)*eye(2));

figure
step(nominale,'r--');
grid on;
hold on
step(nominaleLTR,'r');
step(perturbato1,'k');
step(perturbato2,'k');
step(perturbato3,'k');
legend('Nominale','LTR nominale: sigma = 1000','LTR perturbato: sigma = 1000')


%%
close all;
disp('LTR recovery: excess of perturbation')

perturbato4=feedback(H_recovery2,eye(2)+example(4,1)*eye(2));
figure
step(perturbato4,'k');
grid on;
legend('LTR perturbato: sigma = 1000')
