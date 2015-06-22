%% Esempio di progetto LQR per garantire robustezza per perturbazioni
%%moltiplicative sull'in con bound l_m crescente

clear all;
clc;
close all;

%%Carica il modello di due massa-molla-smorzatore connessi in serie
%e il modello delle incerteze sull'attuatore
%Modello di due massa-molla-smorzatore connessi in serie
%

% m1=1;       %Massa
% m2=1;
% k1=0.1;       %Costante elastica
% k2=0.1;
% b1=.1;       %Attrito viscoso
% b2=.1;
% a1=1;       %Guadagno statico dell'attuatore
% a2=1;
% 
% A=[zeros(2,2) eye(2); -(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1; k2/m2 -k2/m2 b1/m2 -b2/m2];
% B=[zeros(2,2); a1/m1 0;0 a2/m2];
% C=[eye(2) zeros(2,2)];
% D=zeros(2,2);
% n=length(A);
% q=size(C,1);
% p=size(B,2);

%%
%% Tenzo Control System
%  gen-23-2015 UserK
%  info: balestrieri.gepp@gmail.com

%% Physical properties of the quadrotor

g=9.81; %Acceleration of gravity (m)
rho=1.2; %Density of air (m^3.kg^-1)

% Total mass of the quadrotor [Kg]
mq=0.82;
% Mass of a motor (kg). All motors have equal mass.
mm=0.068; 
% Motor length along x-axis (m). All motors have equal sizes.
lx=28.8e-3;
% Motor length along y-axis (m)
ly=28.8e-3;
% Motor length along z-axis (m)
lz=26e-3;

% Distance from the center of gravity to the center of a motor (m).
% The quadrotor is symmetric regarding the XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

% Moment of inertia (x-axis) for motors 1 and 3
% (kg.m^2).
Ix1=(1/12)*mm*(ly^2+lz^2); 
% Moment of inertia (x-axis) for motors
% 2 and 4 (kg.m^2).
Ix2=(1/12)*mm*(ly^2+lz^2)+mm*dcg^2;
% Total moment of inertia along the x-axis (kg.m^2)
Ixx=2*Ix1+2*Ix2; 
% Moment of inertia (y-axis) for motors
% 1 and 3 (kg.m^2).
Iy1=(1/12)*mm*(lx^2+lz^2)+mm*dcg^2; 
% Moment of inertia (y-axis) for motors 2 and 4
% (kg.m^2).
Iy2=(1/12)*mm*(lx^2+lz^2); 
% Total moment of inertia along the y-axis (kg.m^2)
Iyy=2*Iy1+2*Iy2; 
% Moment of inertia (z-axis) for motors
% 1 and 3 (kg.m^2).
Iz1=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Moment of inertia (z-axis) for motors
% 2 and 4 (kg.m^2).
Iz2=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Total moment of inertia along the z-axis (kg.m^2)
Izz=2*Iz1+2*Iz2;
% Inertia matrix
II=diag([Ixx Iyy Izz]); 

% Inflow coefficient
If = -0.3559;

% Thrust coefficient of the propeller and Power coefficient
cp=0.0743; 
ct=0.1154;

% Propeller radius (m)
rp=25.4e-3; 
% Constant value to calculate the moment provided
% by a propeller given its angular speed (kg.m^2.rad^-1)
Km=cp*4*rho*rp^5/pi()^3; 
% Constant value to calculate the thrust provided
% by a propeller given its angular speed (kg.m.rad^-1) 
%Kt=ct*4*rho*rp^4/pi()^2; 
Kt=ct*rho*rp^4*pi();
% Constant that relates thrust and moment of a propeller.
Ktm=Km/Kt;
k1=2.028; %Transfer-function gain of motor 1
k2=1.869; %Transfer-function gain of motor 2
k3=2.002; %Transfer-function gain of motor 3
k4=1.996; %Transfer-function gain of motor 4
tc=0.436; %Time-constant of the motors (assumed equal for all motors)

dz1=1247.4; %PWM dead-zone of motor 1 given its first-order transfer
% function (micro-seconds)
dz2=1247.8; % PWM dead-zone of motor 2 given its first-order transfer
%function (micro-seconds)
dz3=1246.4; % PWM dead-zone of motor 3 given its first-order transfer
%function (micro-seconds)
dz4=1247.9; % PWM dead-zone of motor 4 given its first-order transfer

g_m1=tf(k1,[tc 1]); % First-order transfer-function of motor 1
g_m2=tf(k2,[tc 1]); % First-order transfer-function of motor 2
g_m3=tf(k3,[tc 1]); % First-order transfer-function of motor 3
g_m4=tf(k4,[tc 1]); % First-order transfer-function of motor 4

%vNote: If the behavior of the motors is not linear, linearization around
% an operating point is required to attain these transfer-functions.
PWM_max=2200; %Upper limit of the PWM signal provided to
%the motors (micro-seconds)
w0c=sqrt(g*mq/Kt);
%X0c=w0c;
Ft0=mq*g;

%% 
% Definiamo il sottosistema osservabile e raggiungibile
disp('Semplificazione del modello. Press X per mostrare il modello semplificato');
  
%If=ureal('If',-0.3559,'Range',[-0.5 -0.001]);
%%mq = ureal('m1',0.82,'Range',[0.8 1.2]);
%Ixx = ureal('Ixx',0.0113,'Range',[0.005 0.05]);
%Iyy = ureal('Ixx',0.0113,'Range',[0.005 0.05]);
%Izz = ureal('Ixx',0.0226,'Range',[0.05 0.05]);


  A = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  
  B = [
    zeros(1,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz];
D = zeros(4,4);
  
outputsLocal = {'phi'; 'theta';'psi';'ze'};
C = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
n=8;      
p=4;
q=4;
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};
%model_unc=uss(AMin,BMin,ClocalMin,D);
%modello_nominale=modello_unc.NominalValue;

%%
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

% verifica minimum phase
disp('Transmission zeros')
tzero(modello_ss)


%%

close all;
disp('Design a LQR controller')

rho = .10;
Q = C'*C;
R = rho*eye(p);
Kopt = lqr(A,B,Q,R);
H = ss(A,B,Kopt,zeros(p,p));
CL = feedback(H,eye(p));
step(CL);

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

%ridimensiono scritte linee etc
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

%%
close all;
disp('Confronto con LQG standard');

Klqg = lqg(modello_ss,blkdiag(Q,R),blkdiag(Xi,Th));

CL_LQG = feedback(series(Klqg,modello_ss),eye(q),+1);
CL_LTR = feedback(series(Kltr,modello_ss),eye(q),+1);
CL_LQR = CL;

figure
sigma(lm_til,'r--',1/lm_til,'r',CL_LQG,'b:',CL_LTR,'g:',CL_LQR,'k:',omega);
grid on
legend('Bound lm','1/lm','U0 LQG','U0 LTR','U0 LQR','Location','SouthWest');

%ridimensiono scritte linee etc
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',20)

