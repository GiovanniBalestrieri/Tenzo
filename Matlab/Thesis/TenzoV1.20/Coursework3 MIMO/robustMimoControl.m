%% Tenzo Control System
%  oct-22-2015 UserK
%  www.userk.co.uk
%  info: userk@protonmail.ch

clear all;
clc;

%% I) Physical properties of the quadcopter

g=9.81; %Acceleration of gravity (m)

% Density of air [ kg*m^-3 ] 
% From 35°C to - 25°C
rho = ureal('rho',1.2250,'Range',[1.1455 1.4224]);

% Total mass of the quadrotor [Kg]
mq = ureal('mq',1.150,'Range',[0.820 1.4224]);

% Mass of a motor (kg). All motors have equal mass.
mm = ureal('mm',0.068,'Range',[0.020 0.075]);
% Motor length along x-axis (m). All motors have equal sizes.
lx = ureal('lx',28.8e-3,'Range',[0.020 0.035]);
% Motor length along y-axis (m)
ly = ureal('ly',28.8e-3,'Range',[0.020 0.035]);
% Motor length along z-axis (m)
lz = ureal('lz',0.08,'Range',[0.05 0.1]);

% Distance from the center of gravity to the center of a motor (m).
% The quadrotor is symmetric regarding the XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

dcgX = ureal('dcgX',0.288,'Range',[0.25 0.30]);
dcgY = ureal('dcgY',0.288,'Range',[0.25 0.30]);
dcgZ = ureal('dcgZ',0.03,'Range',[0.02 0.1]);

% Moment of inertia (x-axis) for motors 1 and 3
% (kg.m^2).
Ix1=(1/12)*mm*(ly^2+lz^2); 
% Moment of inertia (x-axis) for motors
% 2 and 4 (kg.m^2).
Ix2=(1/12)*mm*(ly^2+lz^2)+mm*dcgX^2;
% Total moment of inertia along the x-axis (kg.m^2)
Ixx=2*Ix1+2*Ix2; 
% Moment of inertia (y-axis) for motors
% 1 and 3 (kg.m^2).
Iy1=(1/12)*mm*(lx^2+lz^2)+mm*dcgY^2; 
% Moment of inertia (y-axis) for motors 2 and 4
% (kg.m^2).
Iy2=(1/12)*mm*(lx^2+lz^2); 
% Total moment of inertia along the y-axis (kg.m^2)
Iyy=2*Iy1+2*Iy2; 
% Moment of inertia (z-axis) for motors
% 1 and 3 (kg.m^2).
Iz1=(1/12)*mm*(lx^2+ly^2)+mm*dcgZ^2; 
% Moment of inertia (z-axis) for motors
% 2 and 4 (kg.m^2).
Iz2=(1/12)*mm*(lx^2+ly^2)+mm*dcgZ^2; 
% Total moment of inertia along the z-axis (kg.m^2)
Izz=2*Iz1+2*Iz2;
% Inertia matrix
II=diag([Ixx Iyy Izz]); 

% Inflow coefficient
If = ureal('If',-0.3559,'Range',[-0.4000 -0.1559]);

% Thrust coefficient of the propeller and Power coefficient
cp = ureal('cp',0.0314,'Range',[0.0311 0.0465]);

ct = ureal('ct',0.0726,'Range',[0.0548 0.0980]);

% Propeller radius (m)
rp = ureal('rp',13.4e-2,'Range',[0.10 0.19]);
% Constant value to calculate the moment provided
% by a propeller given its angular speed (kg.m^2.rad^-1)
Km=cp*4*rho*rp^5/pi()^3; 
% Constant value to calculate the thrust provided
% by a propeller given its angular speed (kg.m.rad^-1) 
%Kt=ct*4*rho*rp^4/pi()^2; 
Kt=ct*rho*rp^4*pi();
% Constant that relates thrust and moment of a propeller.
Ktm=Km/Kt;

Ft0=mq*g;

%% II) Definiamo il sottosistema osservabile e raggiungibile
disp('Let us remove the unobservable modes/components from the state.');
  
  AMin = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

% Define B matrix related to ui as torques and thrust

B = [zeros(5,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];

% Now we want the inputs to be wi^2 of the i-th motor 

  BMin = [
    zeros(1,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];

WTF = [     0   -lx*Kt  0  lx*Kt;
            ly*Kt  0  -ly*Kt  0;
            Ktm -Ktm Ktm -Ktm;
            -Kt -Kt -Kt -Kt];  

Bw = B*WTF;

% define B matrix related to wi anuglar velocities
BMinw = [Bw(3,:);Bw(6:end,:)]
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
D=zeros(4,4);
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};
inputName = {'w1^2','w2^2','w3^2','w4^2'};

disp('Equivalent system: We have removed the x,y position and x,y linear velocities from the plant.');
disp('When we linearize the non linear model, these informations are lost');
disp('Anyway, we do not need them for th Low Level Controller (attitude control)');


tenzo_min_unc = uss(AMin,BMinw,ClocalMin,D,'statename',statesMin,'inputName',inputName,'outputname',outputsLocal);
tenzo_min_nominale=tenzo_min_unc.NominalValue

% Sottosistema ragg e oss con decomposizione di Kalman
% Check wether the solution is still valid with this subsys
disp('Let us see if Matlab confirms our guess');
disp('  Press X ');


[sysr,U] = minreal(tenzo_min_nominale,0.1);

KalmanA = U*tenzo_min_nominale.a*U';
KalmanB = U*tenzo_min_nominale.b;
KalmanC = tenzo_min_nominale.c*U';

%sysr

disp('Yeah confirmed! Correct observable and reachable subsystem');

%% 1) Verifiche preliminari proprietà strutturali 
% #Osservabilità #Controllabilità #Invariant zeros 

 n=size(tenzo_min_nominale.a,1);

% Verifica Raggiungibilità e Osservabilità
if (rank(ctrb(tenzo_min_nominale.a,tenzo_min_nominale.b))==size(tenzo_min_nominale.a,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(tenzo_min_nominale.a,tenzo_min_nominale.b)));
else
    disp('Sistema Irraggiungibile');
end

if (rank(obsv(tenzo_min_nominale.a,tenzo_min_nominale.c))==size(tenzo_min_nominale.a,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(tenzo_min_nominale.a,tenzo_min_nominale.c)));

% Transfer function
modello_tf = tf(tenzo_min_nominale);

% invarianzZero
disp('Invariant Zeros:');
tzero(tenzo_min_nominale.a,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d,eye(8))

disp('Transmission Zeros');
tzero(modello_tf)

% valori singolari 
figure(2)
sigma(modello_tf,[],'o-')
grid on;

%% 2.1) LQR

close all;
clc;

disp('Design a LQR controller')
rho = 0.08

% Check (A,Cq) Rilevabile
if (rank(obsv(tenzo_min_nominale.a,tenzo_min_nominale.c)) == n)
    disp('Checking (A,Cq) ... Rilevabile');
elseif (rank(obsv(tenzo_min_nominale.a,tenzo_min_nominale.c)) < n)
    disp('Checking (A,Cq) ...  NON Rilevabile');
end


% Check Cq(sI-A)B Squared
if (size(tf(tenzo_min_nominale)) == size(tenzo_min_nominale.d))
    disp('Checking tf ... Squared TF');
else
    disp('Checking tf ... Non squared tf');
end

disp('Premere un tasto per continuare...')
pause;

%%
clc;
disp('Design a LQR controller')
rho = 0.08;

disp('Let us compute the optimal gain Kinf u=(Kinf)x');

Q = tenzo_min_nominale.c'*tenzo_min_nominale.c;
R = rho*eye(size(tenzo_min_nominale.d));

Kopt = lqr(tenzo_min_nominale.a,tenzo_min_nominale.b,Q,R);

disp('Matrice di guadagno K: [comando K = lqr(A,B,Q,R)]');
H = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,Kopt,zeros(size(tenzo_min_nominale.d)));
CL = feedback(H,eye(size(tenzo_min_nominale.d)));
step(CL);

disp('Eigenvalues closed loop sys: eig(A-B*Kinf)');
eK = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt);
disp(eK);

disp('Kopt');
Kopt

disp('Premere un tasto per continuare...')
pause;

%% 2.2) U0 largest singular values

w=logspace(-2,2,100);
lm = tf([1 100], 100);

figure(3)
hold on
sigma(H,'b')
sigma(lm,'r--')
sigma(CL,'k')
sigma(1/lm,'r')
grid on
legend('Anello aperto','Bound l_m','U0','1/lm')

disp('Press X to continue');
pause()
%% lm bounds for the designed LQR controller
close all;
disp('Derive lm bounds for the designed LQR controller')
omega = logspace(-4,4); 
pp = sigma(CL,omega); %compute sing values at each freq
sysg = pp(1,:);   %pick the max sing value at each freq

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
set(uuu,'FontSize',14)

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
set(uuu,'FontSize',14)

%% 2.3) LTR

% a) verify TF is nS, without Transmission zeros and squared
if (size(tf(tenzo_min_nominale)) == size(tenzo_min_nominale.d))
    disp('Squared TF');
else
    disp('Nein');
end
%%%%% manca il check sul rank della tf

% b) LQG
clc;

disp('LTR Recovery')

omega = logspace(-4,4); 

%Funzione d'anello originale
sigma(H,'k',omega);
grid on;
hold on;

% graphic adjustments
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
uuu = gca;
set(uuu,'FontSize',13)

%Inizio il recovery
Xi = eye(n);
Th = eye(size(tenzo_min_nominale.c,1));
rho = [0 1000 1000000]; % LTR recovery gains
[Kltr,SVL,W1] = ltrsyn(tenzo_min_nominale,Kopt,Xi,Th,rho,omega);

disp('Premere un tasto per continuare...')
pause;




%% Comparison LQG standard

close all;
disp('Confronto con LQG standard');

Klqg = lqg(tenzo_min_nominale,blkdiag(Q,R),blkdiag(Xi,Th));

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




