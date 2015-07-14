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
k1=0.1;       %Costante elastica
k2=0.1;
b1=.1;       %Attrito viscoso
b2=.1;
a1=1;       %Guadagno statico dell'attuatore
a2=1;

A=[zeros(2,2) eye(2); -(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1; k2/m2 -k2/m2 b1/m2 -b2/m2];
B=[zeros(2,2); a1/m1 0;0 a2/m2];
C=[eye(2) zeros(2,2)];
D=zeros(2,2);
n=length(A);
q=size(C,1);
p=size(B,2);

modello_ss=ss(A,B,C,D)

disp('Eigenvalues of the matrix A')
disp(eig(A))

disp('Transfer matrix of the model')
modello_tf=tf(modello_ss)


%% Verifica di rilevabilita' e stabilizzabilita'
disp('Reachability and Observability')
ctrb(A,B)
rank(ctrb(A,B))
ctrb(A',C')
rank(ctrb(A',C'))

figure
sigma(modello_tf)
grid on

% verifica minimum phase
disp('Transmission zeros')
tzero(modello_ss)

%%  LQR controller

close all;
disp('Design a LQR controller')

rho = .10;
Q = C'*C;
R = rho*eye(2);
Kopt = lqr(A,B,Q,R);
H = ss(A,B,Kopt,zeros(2,2));
CL = feedback(H,eye(2));
step(CL);

H_lqr = ss(A-B*Kopt,B,C,D)
figure
step(H_lqr)
%%
close all;
disp('Derive lm bounds for the designed LQR controller')
omega = logspace(-2,3); 
pp = sigma(CL,omega); %compute sing values at each freq
sysg = pp(1,:);   %pick the max sing value at each freq

% % Verifica che sysg è il max valor singolare
% sigma(CL,omega);
% hold on;
% semilogx(omega,20*log10(sysg),'r');
pre_lm = frd(sysg.^-1,omega);

% fit razionale e min phase per ricavare il bound
ord = 1; %Ordine della funzione di fitting 
lm1 = fitmagfrd(pre_lm,ord,[],[],-1); 
lmg1 = frd(lm1,omega); 

ord = 2; %Ordine della funzione di fitting 
lm2 = fitmagfrd(pre_lm,ord,[],[],-1); 
lmg2 = frd(lm2,omega); 

bodemag(pre_lm,'r',lmg1,'k:',lmg2,'b--');
legend('\sigma(U_0)^{-1}','lmtilde, order 1','lmtilde, order 2','Location','NorthWest');

%% ridimensiono scritte linee etc
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',20)

%%
close all;
disp('confronto fra anello chiuso e aperto, e vari bound')
figure
sigma(H,'k',lm2,'r--',lm1,'g--',CL,'b-o',1/lm2,'r',1/lm1,'g',omega);
grid on
legend('Anello aperto','Bound lm_2','Bound lm_1','U0','1/lm_2','1/lm_1','Location','SouthWest');
lm_til = lm2;

%ridimensiono scritte linee etc
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',20)

%%
close all;
disp('LTR recovery')

%Funzione d'anello originale
sigma(H,'ko-',omega);
grid on;
hold on;
%ridimensiono scritte linee etc
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',20)

%Inizio il recovery
Xi = eye(n);
Th = eye(q);
rho = [0 1000 1000000]; % LTR recovery gains
[Kltr,SVL,W1] = ltrsyn(modello_ss,Kopt,Xi,Th,rho,omega);

%% ssdata
close all;
disp('Confronto con LQG standard');

Klqg = lqg(modello_ss,blkdiag(Q,R),blkdiag(Xi,Th));

CL_LQG = feedback(series(Klqg,modello_ss),eye(q),+1);
CL_LTR = feedback(series(Kltr,modello_ss),eye(q),+1);
CL_LQR = CL;

figure
sigma(lm_til,'r--',1/lm_til,'r',CL_LQG,'b',CL_LTR,'g',CL_LQR,'k',omega);
grid on
legend('Bound lm','1/lm','U0 LQG','U0 LTR','U0 LQR','Location','SouthWest');

%ridimensiono scritte linee etc
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',20)

