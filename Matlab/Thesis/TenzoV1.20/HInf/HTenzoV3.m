%% Tenzo Control System
%  oct-11-2015 UserK
%  info: userk@protonmail.ch
clear all;
clc;


% Physical properties of the quadrotor

g=9.81; %Acceleration of gravity (m)

% Density of air [ kg*m^-3 ] 
% From 35°C to - 25°C
rho = ureal('rho',1.2250,'Range',[1.1455 1.4224]);

% Total mass of the quadrotor [Kg]
mq = ureal('mq',1.30,'Range',[0.620 2.0]);

% Mass of a motor (kg). All motors have equal mass.
mm = ureal('mm',0.068,'Range',[0.020 0.085]);
% Motor length along x-axis (m). All motors have equal sizes.
lx = ureal('lx',28.8e-3,'Range',[0.015 0.030]);
% Motor length along y-axis (m)
ly = ureal('ly',28.8e-3,'Range',[0.015 0.030]);
% Motor length along z-axis (m)
lz = ureal('lz',0.08,'Range',[0.03 0.6]);

% Distance from the center of gravity to the center of a motor (m).
% The quadrotor is symmetric regarding the XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

dcgX = ureal('dcgX',0.288,'Range',[0.12 0.37]);
dcgY = ureal('dcgY',0.288,'Range',[0.12 0.37]);
dcgZ = ureal('dcgZ',0.03,'Range',[-0.1 0.1]);

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
cp = ureal('cp',0.0314,'Range',[0.0111 0.0465]);

ct = ureal('ct',0.0726,'Range',[0.0348 0.0980]);

% Propeller radius (m)
rp = ureal('rp',13.4e-2,'Range',[0.05 0.15]);
% Constant value to calculate the moment provided
% by a propeller given its angular speed (kg.m^2.rad^-1)
Km=cp*4*rho*rp^5/pi()^3; 
% Constant value to calculate the thrust provided
% by a propeller given its angular speed (kg.m.rad^-1) 
%Kt=ct*4*rho*rp^4/pi()^2; 
Kt=ct*rho*rp^4*pi();
% Constant that relates thrust and moment of a propeller.
Ktm=Km/Kt;

dz1=1247.4; %PWM dead-zone of motor 1 given its first-order transfer
% function (micro-seconds)
dz2=1247.8; % PWM dead-zone of motor 2 given its first-order transfer
%function (micro-seconds)
dz3=1246.4; % PWM dead-zone of motor 3 given its first-order transfer
%function (micro-seconds)
dz4=1247.9; % PWM dead-zone of motor 4 given its first-order transfer

%vNote: If the behavior of the motors is not linear, linearization around
% an operating point is required to attain these transfer-functions.
PWM_max=2200; %Upper limit of the PWM signal provided to
%the motors (micro-seconds)
%w0c=sqrt(g*mq/Kt);
%X0c=w0c;
Ft0=mq*g;

%% Linearized Model
disp('Welcome to Tenzo!');
disp('Using Linearized Model: press tenzo_nominale for infos');

states = {'xe','ye','ze','vxe','vye','vze','phi','theta','psi','wxb','wyb','wzb'};

A = [ 0 0 0 1 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 0 0;
      0 0 0 0 0 1 0 0 0 0 0 0;
      0 0 0 0 0 0 0 -g 0 0 0 0;
      0 0 0 0 0 0 g 0 0 0 0 0;
      0 0 0 0 0 If 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 1 0 0; 
      0 0 0 0 0 0 0 0 0 0 1 0; 
      0 0 0 0 0 0 0 0 0 0 0 1; 
      zeros(3,12)];
  
n = size(A,2);

  
B = [zeros(5,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];

p = size(B,2);

WTF = [     0   -lx*Kt  0  lx*Kt;
            ly*Kt  0  -ly*Kt  0;
            Ktm -Ktm Ktm -Ktm;
            -Kt -Kt -Kt -Kt];  

Bw = B*WTF;
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
Clocal = [ 0 0 0 0 0 0 1 0 0 0 0 0; 
           0 0 0 0 0 0 0 1 0 0 0 0; 
           0 0 0 0 0 0 0 0 1 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0];
       
outputsRemote = {'xw'; 'ye';'ze';'psi'};
Cremote = [ 1 0 0 0 0 0 0 0 0 0 0 0;
           0 1 0 0 0 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 1 0 0 0];
       
q = size(Clocal,1);       
       
inputs = {'w1^2','w2^2','w3^2','w4^2'};

D = zeros(4,4);

tenzo_unc = uss(A,Bw,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
tenzo_nominale=tenzo_unc.NominalValue;

disp('Verifica preliminare, autovalori del processo [eig(A)]:');

%Definizione intervallo delle frequenze di interesse
omega = logspace(-2,3,250);

% Motors

% V - Include mixer in B 
% V - Include motor's dynamic in A with state space canonical companion from 

disp('Creating motors transfer functions...');

ktfM1 = 2023;
p1m1 = -10;
p2m1 = -220;
ktfM2 = 2023;
p1m2 = -10;
p2m2 = -220;
ktfM3 = 2023;
p1m3 = -10;
p2m3 = -220;
ktfM4 = 2023;
p1m4 = -10;
p2m4 = -220;

% Motor 1
tfM1 = zpk([],[p1m1 p2m1],ktfM1);
canonMotor1 = canon(tfM1,'companion');
% Motor 2
tfM2 = zpk([],[p1m2 p2m2],ktfM2);
canonMotor2 = canon(tfM2,'companion');

% Motor 3
tfM3 = zpk([],[p1m3 p2m3],ktfM3);
canonMotor3 = canon(tfM3,'companion');

% Motor 4
tfM4 = zpk([],[p1m4 p2m4],ktfM4);
canonMotor4 = canon(tfM4,'companion');

disp('Display Motor step response');
% Analyze motors dynamics
opt = stepDataOptions;
opt.StepAmplitude = 1000;
title('Step response for motors');
step(tfM1,opt);

% Controllable
%rank(ctrb(canonMotor.a,canonMotor.b));
% Observable
%rank(ctrb(canonMotor.a',canonMotor.c'));

disp('Motors Tf created.');

mot= zpk([],[-10 -220],1273);

m = canon(mot,'modal');
mot1= m^2;

m1 = canon(mot1,'modal');

% open('testMotors');
% sim('testMotors');

% Eigenvalues of the system
alpha = 0;
stab=1;
eOp = eig(tenzo_nominale.a);
[dn,dm]=size(eOp);
moltZero = 0;
for i=1:dn,
  if (real(eOp(i)) > 0) 
      stab=0; 
      disp('elemento a parte reale positiva:');
      disp(eOp(i)); 
  elseif (real(eOp(i))==0 && moltZero == 0) 
      moltZero =+ 1;
  elseif (real(eOp(i))==0 && moltZero > 0)
      stab=2;
  end
end
if (stab==0) disp('Sistema instabile! Gli autovalori a ciclo aperto sono:'); end
if (stab==1) disp('Sistema stabile! OLE!! Gli autovalori a ciclo aperto sono:'); end
if (stab==2) disp('Sistema instabile! Sono presenti autovalori pari a zero con molteplicità > 1'); end
disp(eOp);

% Analisi risposta a gradino
disp('Press any for Step Response:');
pause;
step(tenzo_nominale);

%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

raggiungibile = false;
osservabile = false;
if (rank(ctrb(tenzo_nominale.a,tenzo_nominale.b))==size(tenzo_nominale.a,1))
    disp('Sistema raggiungibile');
    raggiungibile = true;
else
    disp('Sistema Irraggiungibile');
    
end
disp('Rank Reach Matrix:');
disp(rank(ctrb(tenzo_nominale.a,tenzo_nominale.b)));

if (rank(obsv(tenzo_nominale.a,tenzo_nominale.c))==size(tenzo_nominale.a,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp('Rank Obsv matrix:');
disp(rank(obsv(tenzo_nominale.a,tenzo_nominale.c)));


    stab_flag=0;
% PBH test of reachability
if raggiungibile == false
    stab_flag=0;
    for i=1:length(eOp)
        if(real(eOp(i))>=-alpha)
            pbh_matrix_reach = [(tenzo_nominale.a - eOp(i)*eye(n)) tenzo_nominale.b];
            if(rank(pbh_matrix_reach)<n)
                disp('PBH VIOLATO');
                stab_flag=1;
                break;
            end
        end
    end
    
    if stab_flag > 0
        disp('Confermato anche dal Pbh test. Sys Non C-Buono stabile'); 
    end
end

%PBH test of observability
if osservabile == false
    for i=1:length(eOp)
        if(real(eOp(i))>=-alpha)
            pbh_matrix_obsv = [(tenzo_nominale.a - eOp(i)*eye(n)); tenzo_nominale.c];
            if(rank(pbh_matrix_obsv)<n)
                disp('PBH VIOLATO');
                rel_flag=1;
                break;
            end
        end
    end
    
    if stab_flag > 0
       disp('Confermato anche dal Pbh test. Sys Non C-Buono stabile'); 
    end
end

% Il sys non è osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile
disp('Let us remove the unobservable modes/components from the state.');
  
  AMin = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

% Define B matrix related to ui as torques and thrust
  BMin = [
    zeros(1,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];

% define B matrix related to wi anuglar velocities
BMinw = [Bw(3,:);Bw(6:end,:)]
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
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

[sysr,U] = minreal(tenzo_nominale,0.1);

KalmanA = U*tenzo_nominale.a*U';
KalmanB = U*tenzo_nominale.b;
KalmanC = tenzo_nominale.c*U';

%sysr

disp('Yeah confirmed! Correct observable and reachable subsystem');

% Proprietà strutturali:

% Sistema Ben Connessi: Somma dei singoli stati che compongono i sistemi 
% sia pari alla dimensione dello stato del sistema complessivo
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

disp('It is fine now');

% Invariant zeros 

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
grid on


% Controllo che P(s) non sia singolare nel calmpo razionale
syms s
if (det(tenzo_min_nominale.c *(s*eye(n)-tenzo_min_nominale.a)^(-1)*tenzo_min_nominale.b) ~= 0)
    disp('P(s) Non singular nel campo Razionale');
else
    disp('P(s) singolare nel campo razionale');
end

%% Passo B:

%          Scegliere R=rho*I con rho>0 e calcolare la matrice dei guadagni 
%          ottimi per tali Q e R. 
%          Scelgo rho tenendo conto che:
%          1. il controllo non deve dar luogo ad una risposta troppo lenta.
%          2. l'andamento della curva del massimo valor singolare della 
%              matrice U0 del sistema a ciclo chiuso non deve essere troppo
%              alto ad alte frequenze 



% Ricostruzione dello stato con Kalman

disp('Ricostruzione dello stato con Kalman');
disp('Press any key to continue.');
pause();

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
alphaK = 10;
Q = eye(size(tenzo_min_nominale.a));
W = eye(size(tenzo_min_nominale.a));
R = eye(size(tenzo_min_nominale.c,1));
%disp('matrice  V per Kalman:');
V=lqr((tenzo_min_nominale.a+alphaK*eye(n))',tenzo_min_nominale.c',Q,R)';
%disp('Dimensione attesa [nxq]');
disp(size(V));
Aoss=tenzo_min_nominale.a-V*tenzo_min_nominale.c;
Bossw=[tenzo_min_nominale.b-V*D V]; % perche ho u,y,d come ingressi, si noti che B-vD ha dim di B ma anche V ha dim di B
Coss=eye(q);
Doss=zeros(q,p);

disp('Autovalori A-V*C');
disp(eig(tenzo_min_nominale.a-V*tenzo_min_nominale.c));

% Stabilizzazione LQR

disp('Stabilizzazione mediante LQR dallo stato stimato');
disp('Press any key to continue.');
pause();

rho1 = 0.001;
rho2 = 1;
rho3 = 10;
alphaK = 5;
%QieCmp = blkdiag([0.00001 0; 0 0.00001],100*eye(3),eye(3));
%Qie = blkdiag([0.01 0; 0 0.01],100000*eye(3),zeros(3,3));
Q = eye(size(AMin));

Q = tenzo_min_nominale.c'*tenzo_min_nominale.c;

%RieCmp = [1 0 0 0; 0 100000 0 0; 0 0 100000 0; 0 0 0 10000];
R = eye(size(BMinw,2));

R1 = rho1*eye(p);
R2 = rho2*eye(p);
R3 = rho3*eye(p);

Kopt_1 = lqr(tenzo_min_nominale.a+alphaK*eye(n) , tenzo_min_nominale.b, Q, R1);
Kopt_2 = lqr(tenzo_min_nominale.a+alphaK*eye(n) , tenzo_min_nominale.b, Q, R2);
Kopt_3 = lqr(tenzo_min_nominale.a+alphaK*eye(n) , tenzo_min_nominale.b, Q, R3);

disp('');

disp('Eig sys 1 CC retroazione dallo stato:');
eig(tenzo_min_nominale.a - tenzo_min_nominale.b*Kopt_1)

disp('');

disp('Eig sys 2 CC retroazione dallo stato:');
eig(tenzo_min_nominale.a - tenzo_min_nominale.b*Kopt_2)

disp('');

disp('Eig sys 3 CC retroazione dallo stato:');
eig(tenzo_min_nominale.a - tenzo_min_nominale.b*Kopt_3)

disp('Premere un tasto per visualizzare la Step Response...');
pause;

tenzoLQR1=ss(tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt_1,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
tenzoLQR2=ss(tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt_2,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
tenzoLQR3=ss(tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt_3,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);

disp('Displaying LQR 1');
pause();
figure(3);
step(tenzoLQR1);

disp('Displaying LQR 2');
pause();
figure(4)
step(tenzoLQR2);

disp('Displaying LQR 3');
pause();
figure(5)
step(tenzoLQR3);


%% Passo 2

F1_ss = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,Kopt_1,zeros(q,q)); 
F2_ss = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,Kopt_2,zeros(q,q));
F3_ss = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,Kopt_3,zeros(q,q));

% Retroazione del sistema Fi_ss per ottenere la U0i utile nel seguito

U0_1 = feedback(F1_ss,eye(q));
U0_2 = feedback(F2_ss,eye(q));
U0_3 = feedback(F3_ss,eye(q));

% Verifica velocità della risposta nei 3 diversi casi 

figure(6)
step(U0_1,'b',U0_2,'r',U0_3,'g')
legend('rho1 = 0.1','rho2 = 1','rho3 = 10')
title('Verifica velocità della risposta a ciclo chiuso per tutti i Kopt');

% Verifico che l'andamento della curva del massimo valor singolare della
% matrice U0 del sistema a ciclo chiuso non sia troppo alto 


U0_1_vs = sigma(U0_1,omega); % Vettore con i valori singolari (su tutte le
                             % frequenze di interesse di U0_1)
U0_2_vs = sigma(U0_2,omega); % Vettore con i valori singolari (su tutte le
                             % frequenze di interesse di U0_2)
U0_3_vs = sigma(U0_3,omega); % Vettore con i valori singolari (su tutte le
                             % frequenze di interesse di U0_3)
m_U0_1_vs = U0_1_vs(1,:);
m_U0_2_vs = U0_2_vs(1,:);
m_U0_3_vs = U0_3_vs(1,:);

% Grafici dei massimi valori singolari delle funzioni U0_i, i = 1,2,3

figure(7)
semilogx(omega,20*log10(m_U0_1_vs),'r'); 
hold on
semilogx(omega,20*log10(m_U0_2_vs),'b');
semilogx(omega,20*log10(m_U0_3_vs),'g');
grid on
legend('U01 MVS','U02 MVS','U03 MVS')
title('Verifica andamento massimo valor singolare di U0');

% Scelta di Kopt3

R    = R3;
U0   = U0_3;
Kopt = Kopt_3;

%% Passo 3 - Loop Transfer Recovery

% TASK:    sostituire la retroazione dallo stato con quella da una stima 
%          data da un filtro di Kalman, scegliendo V=sigma^2*B0*B0'. 
%          Adottiamo scelte via via crescenti di sigma in modo che il max
%          valor singolare della matrice U0(j*omega) del sistema a ciclo chiuso 
%          in una banda [0,omega] di larghezza molto ampia, sia pari all'incirca 
%          al max valor singolare della matrice U0(j*omeaga) corrispondente 
%          al sistema di controllo con retroazione dallo stato (calcolato al 
%          secondo passo)

% 1st Attempt 

sigma_1 = 0.5;
V_1 = sigma_1^2*tenzo_min_nominale.b*tenzo_min_nominale.b'; % matrice di intensità
W_1 = eye(p);
L_1 = lqr(tenzo_min_nominale.a',tenzo_min_nominale.c',V_1,W_1)';

% Definisco le matrici (Ac_i,Bc_i,Cc_i,Dc_i) di un sistema che, in serie
% all'impianto nominale, darà luogo ad una matrice di trasferimento ad
% anello aperto che è pari a quella ottenuta con l'utilizzo di un filtro di
% kalman con guadagno ottimo L_i

Ac_1 = tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt-L_1*tenzo_min_nominale.c-L_1*tenzo_min_nominale.d*Kopt;
Bc_1 = L_1;
Cc_1 = Kopt;
Dc_1 = zeros(q,q);
G_1  = ss(Ac_1,Bc_1,Cc_1,Dc_1);     % Kalman Filter + Optimal K
H_LTR_1 = series(tenzo_min_nominale,G_1);   
%CC_H_LTR_1 = feedback(H_LTR_1,eye(q));
%step(CC_H_LTR_1);
U_LTR_1 = feedback(H_LTR_1,eye(q)); % Nuova matrice U_1 dopo LTR

% Second attempt 

sigma_2 = 1000;
V_2 = sigma_2^2*tenzo_min_nominale.b*tenzo_min_nominale.b'; % matrice di intensità
W_2 = eye(p);
L_2 = lqr(tenzo_min_nominale.a',tenzo_min_nominale.c',V_2,W_2)';

Ac_2 = tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt-L_2*tenzo_min_nominale.c-L_2*tenzo_min_nominale.d*Kopt;
Bc_2 = L_2;
Cc_2 = Kopt;
Dc_2 = zeros(q,q);

G_2  = ss(Ac_2,Bc_2,Cc_2,Dc_2);       % Kalman Filter + Optimal K
H_LTR_2 = series(tenzo_min_nominale,G_2);  
U_LTR_2 = feedback(H_LTR_2,eye(q)); 


% Third attempt

sigma_3 = 10^12;
V_3 = sigma_3^2*tenzo_min_nominale.b*tenzo_min_nominale.b';
W_3 = eye(p);
L_3=  lqr((tenzo_min_nominale.a+alpha*eye(n))',tenzo_min_nominale.c',V_3,W_3)';

Ac_3 = tenzo_min_nominale.a-tenzo_min_nominale.b*Kopt-L_3*tenzo_min_nominale.c-L_3*tenzo_min_nominale.d*Kopt;
Bc_3 = L_3;
Cc_3 = Kopt;
Dc_3 = zeros(q,q);

G_3 = ss(Ac_3,Bc_3,Cc_3,Dc_3);      % Sistema filtro di kalman + guadagno k ottimo
H_LTR_3 = series(tenzo_min_nominale,G_3);   % Connessione in serie all'impianto nominale
U_LTR_3 = feedback(H_LTR_3,eye(q)); % Nuova matrice U_3 dopo LTR

% Valutiamo graficamente il risultato migliore

U_LTR_1_vs = sigma(U_LTR_1,omega);  % Vettore con i valori singolari (su tutte le
                                    % frequenze di interesse di U_LTR_1)
U_LTR_2_vs = sigma(U_LTR_2,omega);  % Vettore con i valori singolari (su tutte le
                                    % frequenze di interesse di U_LTR_2)
U_LTR_3_vs = sigma(U_LTR_3,omega);  % Vettore con i valori singolari (su tutte le
                                    % frequenze di interesse di U_LTR_3)
m_U_LTR_1_vs = U_LTR_1_vs(1,:);
m_U_LTR_2_vs = U_LTR_2_vs(1,:);
m_U_LTR_3_vs = U_LTR_3_vs(1,:);
m_U0_vs = m_U0_3_vs;

% Grafici dei massimi valori singolari delle funzioni U0 e U_LTR_i , i = 1,2,3

figure(8)
semilogx(omega,20*log10(m_U0_vs),'bo');
hold on
semilogx(omega,20*log10(m_U_LTR_1_vs),'r'); 
semilogx(omega,20*log10(m_U_LTR_2_vs),'m');
semilogx(omega,20*log10(m_U_LTR_3_vs),'g');

grid on
legend('U0 MVS','U01 LTR MVS','U02 LTR MVS','U03 LTR MVS')
title('Max val sing of U0');

% Scelgo la terza sigma

G  = G_3; % Sistema filtro di kalman + guadagno k ottimo scelto per LTR
lma = frd(m_U_LTR_3_vs.^-1,omega);

%% Quarto Task - Robustezza

% Number of perturbations
N=10

% Prende alcuni campioni del sistema incerto e calcola bound su incertezze
for i=1:1:N
sys{i} = usample(tenzo_min_unc);
% Additive
deltaA_sys{i} = tf(sys{i}) - tf(tenzo_min_nominale);
% Moltiplicative riportate sull'Ingresso
deltaMin_sys{i} = (inv(tf(tenzo_min_nominale))) * deltaA_sys{i};
% Moltiplicative riportate sull'Usicta
deltaMout_sys{i} = deltaA_sys{i} * (inv(tf(tenzo_min_nominale)));
end

% generates 250 points between decades 10^( -2 ) and 10^( 3 ).
omega = logspace(-2,3,250);

% Plots the singular values of the frequency response of a model nominale
% specifies the frequency range or frequency points to be used for the plot
temp = sigma(tenzo_nominale,omega);
% Select only max sing values
max_sig_nom = temp(1,:);

for i=1:1:N
  temp = sigma(sys{i},omega);
  max_sig_unc(i,:) = temp(1,:);
  temp = sigma(deltaA_sys{i},omega);
  max_sig_dA(i,:) = temp(1,:);
  temp = sigma(deltaMin_sys{i},omega);
  max_sig_dMin(i,:) = temp(1,:);
  temp = sigma(deltaMout_sys{i},omega);
  max_sig_dMout(i,:) = temp(1,:);
end

max_sig_dMout(1,:);
% Returns a row vector containing the maximum element from each column.
top_unc = max(max_sig_unc);
top_dA = max(max_sig_dA);
top_dMin = max(max_sig_dMin);
top_dMout = max(max_sig_dMout);



%% Input Moltiplicative uncertainties
% 
% figure(3);
% semilogx(omega,mag2db(top_dMin),'b','LineWidth',5)
% grid on;
% hold on;
% for i=1:1:N
%   semilogx(omega,mag2db(max_sig_dMin(i,:)),'r:','LineWidth',2)
% end
% title('Max sing values: input multiplicative uncertainties (red), bound (blue)')

%% output multiplicative Out uncertainties

figure(9);
semilogx(omega,mag2db(top_dMout),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dMout(i,:)),'r:','LineWidth',2)
end
title('Max sing values: output multiplicative uncertainties (red), bound (blue)')


%% Upper bound MOLT IN lm(w) razionale stabile e fase minima
% pre_bound_dMin = frd(top_dMin,omega);
% 
% % fit razionale e min phase per ricavare il bound
% %ord = 2; %Ordine della funzione di fitting 
% %bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
% %bb_dMin2 = sigma(bound_dM,omega);
% ord = 5; %Ordine della funzione di fitting 
% bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
% bb_dMin5 = sigma(bound_dM,omega);
% %ord = 7; %Ordine della funzione di fitting 
% %bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
% %bb_dMin7 = sigma(bound_dM,omega);
% 
% figure(6);
% semilogx(omega,mag2db(top_dMin),'b','LineWidth',2)
% grid on;
% hold on;
% %semilogx(omega,mag2db(bb_dMin2(1,:)),'r','LineWidth',2)
% semilogx(omega,mag2db(bb_dMin5(1,:)),'k','LineWidth',2)
% %semilogx(omega,mag2db(bb_dMin7(1,:)),'m','LineWidth',2)
% title('Bound on multiplicative uncertainties');
% legend('strict bound', 'Rational stable min phase, order 2',...
%   'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
%   'Location','SouthWest');

%% Upper bound Molt OUT lm~(w) razionale stabile e fase minima
pre_bound_dMout = frd(top_dMout,omega);

% fit razionale e min phase per ricavare il bound
ord = 2; %Ordine della funzione di fitting 
bound_dMout = fitmagfrd(pre_bound_dMout,ord,[],[],1); 
bb_dMout2 = sigma(bound_dMout,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dMout = fitmagfrd(pre_bound_dMout,ord,[],[],1); 
bb_dMout5 = sigma(bound_dMout,omega);
ord = 7; %Ordine della funzione di fitting 
bound_dMout = fitmagfrd(pre_bound_dMout,ord,[],[],1); 
bb_dMout7 = sigma(bound_dMout,omega);

figure(10);
semilogx(omega,mag2db(top_dMout),'b','LineWidth',2)
grid on;
hold on;
semilogx(omega,mag2db(bb_dMout2(1,:)),'r','LineWidth',2)
semilogx(omega,mag2db(bb_dMout5(1,:)),'k','LineWidth',2)
title('Bound on multiplicative Output uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5',...
  'Location','SouthWest');

disp('Step response for uncertain systems');
figure(11)
% Compute Closed Loop transfer functions
for i=1:N
    Ac_3 = sys{i}.a-sys{i}.b*Kopt_3-L_3*sys{i}.c;
    Bc_3 = L_3;
    Cc_3 = Kopt;
    Dc_3 = zeros(q,q);

    G_3 = ss(Ac_3,Bc_3,Cc_3,Dc_3);      % Sistema filtro di kalman + guadagno k ottimo
    H_LTR_3 = series(sys{i},G_3);   % Connessione in serie all'impianto nominale
    Closed_Loop_LTR{i} = feedback(H_LTR_3,eye(q)); % Nuova matrice U_3 dopo LTR
    step(Closed_Loop_LTR{i});
    hold on
    grid on
end

%% AUTOVALORI
disp('Print eigenvalues of uncertain systems');
pause();
clc

for i=1:N
 disp('Autovalori del sistema')
 i
 eig(Closed_Loop_LTR{i})     % Autovalori del sistema di controllo perturbato 1
end

%% TASK 3 

%  Calcolo di strumenti da utilizzare
%  nell'applicazione del controllo Hinf 

% -----> PROBLEM F0 ha poli lungo asse IM  -> sposta C-buono

F0 = series(G,tenzo_min_nominale);
% Matrice di sensibilità
S0_LTR = feedback(eye(q),F0);
S0_vs = sigma(S0_LTR,omega);
max_S0_vs = S0_vs(1,:);

F0_vs = sigma(F0,omega);
max_F0_vs = F0_vs(1,:);
P0G = frd(max_F0_vs,omega);

ps_sign = frd(max_S0_vs.^-1,omega);
mx_SO_vs = frd(max_S0_vs,omega);

%approssmazione si 1/ps con w1                                
w1 = zpk([],[0 -2],1200);

[MODX,FAS]=bode(w1,omega);
w1M = frd(MODX,omega); % Otteniamo la funzione ps imponendola pari al modulo
                     % di w1 per ogni omega

% Grafico di ps~ (inverso dell'andamento dei massimi
% valori singolari di S0 al variare di omega) 
% e ps ricavata tale che:
% - tenda a zero per w -> inf
% - ps >> 1 per w < wX


figure(12)
bodemag(ps_sign,'b',mx_S0_vs,'r',w1M,'k',omega);
legend('1/ps','max_S0','w1');
grid on

%% 3.2) 

 
figure(13);
semilogx(omega,mag2db(max_sig_nom),'k--','LineWidth',2)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_unc(i,:)),'r:','LineWidth',2)
end
semilogx(omega,mag2db(top_unc),'b','LineWidth',5)
semilogx(omega,mag2db(max_sig_nom),'k','LineWidth',5)
title('Max sing values: Nominal (bk), Pert models (R), bound (blue)')

%  Additive
figure(14);
semilogx(omega,mag2db(top_dA),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dA(i,:)),'r:','LineWidth',2)
end
title('Max sing values: additive uncertainties (red), bound (blue)')

% Upper bound razionale stabile e fase minima
pre_bound_dA = frd(top_dA,omega);

% fit razionale e min phase per ricavare il bound
ord = 2; %Ordine della funzione di fitting 
bound_dA2 = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA2 = sigma(bound_dA2,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dA5 = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA5 = sigma(bound_dA5,omega);
ord = 7; %Ordine della funzione di fitting
bound_dA7 = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA7 = sigma(bound_dA7,omega);

figure(15);
semilogx(omega,mag2db(top_dA),'b','LineWidth',2);
grid on;
hold on;
semilogx(omega,mag2db(bb_dA2(1,:)),'r-','LineWidth',2);
semilogx(omega,mag2db(bb_dA5(1,:)),'ko','LineWidth',2);
semilogx(omega,mag2db(bb_dA7(1,:)),'m','LineWidth',2);
title('Bound on additive uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
  'Location','SouthWest');

%% Costruzione V0

% let us use as la the rational fit previously computed
la = bound_dA2;

V0_LTR = feedback(G,tenzo_min_nominale);

% Max val sing di V0
V0_vs = sigma(V0_LTR,omega);
max_V0_vs = V0_vs(1,:);
MAX_V0_vs = frd(max_V0_vs,omega);

% la segnato
la_signed = frd(max_V0_vs.^-1,omega);

%approssmazione si la con w2                                
w2 = zpk([-3 -5 -1],[0 0 -2],5);

[MODX,FAS]=bode(w2,omega);
w2M = frd(MODX,omega); % Otteniamo la funzione ps imponendola pari al modulo
                     % di w1 per ogni omega

figure(16)
bodemag(la_signed,'b',MAX_V0_vs,'r',la,'m',w2M,'k',omega);
legend('la_sign','maxV0','la','w2');
grid on

%% Definizione dei bounds

ps = gamma1*tf(1,[1 1])
la{i} = deltaA_sys{i}
lmD{i} = deltaMin_sys{i}
lm{i} = deltaMout_sys{i}

la = bb_dA5;
lm = bb_dMin5;

lm = tf([1 1], 1.1);
lmD = lm

figure
sigma(lm,'r--')
hold on
grid on
semilogx(omega,mag2db(lm),'b','LineWidth',2)
semilogx(omega,mag2db(la),'b','LineWidth',2)

bode(mag2db(lm))


% disp('End');


%% Verifica condizioni Astatismo

% disp('Condizioni Astatismo');
% pause;
% n=size(AMin,2);
% p=size(BMinw,2);
% q=size(ClocalMin,1);
% 
% % Definizione segnali esogeni
% disp('Definizione dei Disturbi da reiettare:');
% disp('d(t)=');
% alpha=3; omega=0.5; gamma1=0;
% k1=1; h1=1; h2=1; gamma2=complex(0,omega);
% 
% disp('Definizione segnali esogeni');
% disp('gamma 1:='); disp(gamma1);
% disp('gamma 2:='); disp(gamma2);
% 
% if (rank(ctrb(AMin+alpha*eye(n),BMinw))==n)
%     disp('(a3) -> verificata, la coppia (AMin,BMin) raggiungibile, rank(P)'); 
%     disp(rank(ctrb(AMin,BMinw))); 
% end
% if (rank(obsv(AMin+alpha*eye(n),ClocalMin))==n) 
%     disp('(a3) -> verificata, la coppia (A,C) osservabile, rank(Q)');
%     disp(rank(obsv(AMin,ClocalMin))); 
% end
% 
% R1=[ AMin-gamma1*eye(size(AMin)) BMinw ;
%     ClocalMin D];
% 
% if (rank(R1)==n+q) disp('b) verificata ,rango della matrice 5.4.23 per gamma1 �:'); disp(rank(R1)); end
% R2=[ AMin-gamma2*eye(size(AMin)) BMinw ; ClocalMin D];
% if (rank(R2)==n+q) disp('b) verificata ,rango della matrice 5.4.23 per gamma2 �:'); disp(rank(R2)); end

%% Calcolo del modello interno
% 
% disp('specifica 2) Calcolo modello interno KM1');
% k1_segnato=max(k1,h1);
% k2_segnato=h2;
% disp('max(k1,h1):'); 
% disp(k1_segnato); 
% disp('max(k2,h2):'); 
% disp(k2_segnato);
% syms s;
% disp('Definiamo il polinomio phi(lambda)');
% phi1=((s-gamma1)^k1_segnato)*((s-gamma2)^k2_segnato)*((s-conj(gamma2))^k2_segnato)
% disp('phi(lambda)=');
% disp((phi1));
% mu=3;
% disp('mu=');
% disp(mu);
% 
% Aphi=compan(sym2poly(phi1));
% APhi=zeros(3);
% APhi(3,:)=Aphi(1,:);
% APhi(1,2)=Aphi(2,1);
% APhi(2,3)=Aphi(3,2);
% disp('matrice in forma compagna Aphi:');
% disp(APhi);
% disp('Bphi:');
% BPhi=[0;0;1];
% disp(BPhi);
% 
% disp('La matrice dinamica AK1 del modello interno KM1:');
% AK1=blkdiag(APhi,APhi,APhi,APhi);
% disp(AK1);
% disp('La matrice dinamica BK1 del modello interno KM1:');
% BK1=blkdiag(BPhi,BPhi,BPhi,BPhi);
% disp(BK1);
% 
% %% Calcolo delle matrici F1,F2 + V di Kalman 
% alpha = 0.5;
% 
% disp('Calcolo delle matrici F1,F2 per S1-S2 +  V per Kalman');
% Asig =[ AMin-BMinw*K zeros(n,size(AK1,2)); -BK1*ClocalMin AK1];
% Bsig = [ BMinw; -BK1*D];
% 
% R = eye(size(Asig,2));
% %Q = blkdiag([1 0; 0 1],1*eye(3),eye(3),eye(12));
% %Q = 1000000*eye(20);
% R = eye(size(Bsig,2));
% F=-lqr(Asig+alpha*eye(size(Asig)),Bsig,Q,R);
% 
% F2=F(:,1:size(AMin,1));
% disp('dimensioni [pxn]:');
% disp(size(F2));
% F1=F(:,size(AMin,1)+1:size(F,2));
% disp('dimensioni [pxq*mu]:');
% disp(size(F1));
% 
% % disp('matrice per Kalman:');
% % V=lqr((AMin-BMin*K)',ClocalMin',Q,R)';
% disp('dimensione attesa [nxq]');
% disp(size(V));
% 
% disp('verifica spostamento autovalori:');
% disp('autovalori A+B*F2');
% disp(eig(AMin-BMinw*K+BMinw*F2));
% disp('autovalori A-V*C');
% disp(eig(AMin-BMinw*K-V*ClocalMin));
% 
% % specifica 3 ) definizione matrici per simulink:
% 
% disp('specifica 3 ) definizione matrici per simulink:');
% 
% M=[ 1 0 0 0 1 1 1 1]';
% N=[ 0; 0; 0; 0];
% %per disturbo sul processo definisco Bmod:
% Bmodw = [M BMinw]; %nota prima d e poi u scambio la somma per comodit�
% Dmod  = [N D];
% 
% %si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
% Aoss=AMin-V*ClocalMin;
% Bossw=[BMinw-V*D V M-V*N]; %perche ho u,y,d   come ingressi, si noti che B-vD ha dim di B ma anche V ha dimn di B
% Coss=eye(n);
% Doss=zeros(size(Bossw));
% 
% % Ricordando che delta xi1=AK1*xi+ Bk1*e
% AMI=AK1;
% BMI=BK1;
% CMI=eye(q*mu);
% DMI=zeros(q*mu,q);
% 
% satW = 8000;
% saMenoW = -400;
% 
% disp('avvio simulazione 1');
% %pause;
% 
% open('progetto3Tenzo.mdl')
% sim('progetto3Tenzo.mdl')

