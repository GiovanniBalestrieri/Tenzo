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
%% Linearized Model
disp('Welcome to Tenzo!');
disp('Press X to show the Linearized Model:');
pause();

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

B = [zeros(5,4);
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

%% Modifiche
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

% Analyze motors dynamics
opt = stepDataOptions;
opt.StepAmplitude = 1000;
step(tfM1,opt);

% Controllable
%rank(ctrb(canonMotor.a,canonMotor.b));
% Observable
%rank(ctrb(canonMotor.a',canonMotor.c'));

disp('Motors Tf created.');

%% 
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
       
CFull = eye(size(A));       
DFull = zeros(size(A,1),4);
inputs = {'w1^2','w2^2','w3^2','w4^2'};
%inputs = {'TauPhi','TauTheta','TauPsi','Thrust'};
D = zeros(4,4);

tenzo=ss(A,Bw,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal)
tenzoFull=ss(A,Bw,CFull,DFull);

%disp('Transfer matrix of the model')
%modello_tf=tf(tenzo)

disp('Assunzione: Esiste almeno una classe di segnali esogeni rispetto alla quale regolare/far inseguire asintoticamente y(t) Verifica preliminare, autovalori del processo [eig(A)]:')
pause();

disp('Verifica preliminare, autovalori del processo [eig(A)]:')
pause();

%% Eigenvalues of the system

stab=1;
eOp = eig(A);
[dn,dm]=size(eOp);
moltZero = 0;
for i=1:dn,
  if (real(eOp(i))>0) 
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
tenzoRetro=ss(A,Bw,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);

%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

disp('Press any key to check proprietà strutturali:');
pause();

disp('Verifica Raggiungibilità. Rango della matrice P:')
if (rank(ctrb(A,Bw))==size(A,1))
    disp('Sistema raggiungibile');
else
    disp('Sistema Irraggiungibile');
end
disp(rank(ctrb(A,Bw)));


disp('Verifica osservabilità. Rango della matrice Q:')
if (rank(obsv(A,Clocal))==size(A,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(A,Clocal)));

pause();
clc
%% Il sys non è osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile
disp('Semplificazione del modello. Press X per mostrare il modello semplificato');
  
  AMin = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  BMin = [
    zeros(1,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];

BMinw = [Bw(3,:);Bw(6:end,:)]
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};
inputName = {'w1^2','w2^2','w3^2','w4^2'};

tenzoMin=ss(AMin,BMinw,ClocalMin,D,'statename',statesMin,'inputName',inputName,'outputname',outputsLocal);

tenzoRetro=ss(AMin,BMinw,ClocalMin,D,'statename',statesMin,'inputName',inputName,'outputname',outputsLocal);
%step(tenzoRetro)


% Transfer function
Ps = tf(tenzoMin);

% invarianzZero
disp('Invariant Zeros:');
tzero(AMin,BMinw,ClocalMin,D,eye(8))

disp('Transmission Zeros');
tzero(Ps)

% Sottosistema ragg e oss con decomposizione di Kalman
% Check wether the solution is still valid with this subsys
[sysr,U] = minreal(tenzo,0.1);

KalmanA = U*A*U';
KalmanB = U*Bw;
KalmanC = Clocal*U';

pause();
sysr

%% Proprietà strutturali:

% Sistema Ben Connessi: Somma dei singoli stati che compongono i sistemi 
% sia pari alla dimensione dello stato del sistema complessivo
 n=size(AMin,1);

% Verifica Raggiungibilità e Osservabilità
disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
pause();
if (rank(ctrb(AMin,BMinw))==size(AMin,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(AMin,BMinw)));
else
    disp('Sistema Irraggiungibile');
end

disp('Verifica osservabilità. Rango della matrice Q:')
if (rank(obsv(AMin,ClocalMin))==size(AMin,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(AMin,ClocalMin)));

if (rank(ctrb(AMin,BMin))==size(AMin,1))
    disp('(a1) -> verificata, la coppia (AMin,BMin) Cb stabilizzabile'); 
    disp(rank(ctrb(AMin,BMin))); 
end

pause();

%% Ricostruzione dello stato con Kalman

disp('Ricostruzione dello stato con Kalman');
disp('Press any key to continue.');
pause();

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
alphaK = 100;
Q = eye(size(AMin));
W = eye(size(AMin));
R = eye(size(ClocalMin,1));
%disp('matrice  V per Kalman:');
V=lqr((AMin+alphaK*eye(size(AMin)))',ClocalMin',Q,R)';
%disp('Dimensione attesa [nxq]');
disp(size(V));
Aoss=AMin-V*ClocalMin;
Bossw=[BMinw-V*D V]; % perche ho u,y,d come ingressi, si noti che B-vD ha dim di B ma anche V ha dim di B
Coss=eye(size(AMin));
Doss=zeros(size(Bossw));

disp('Autovalori A-V*C');
disp(eig(AMin-V*ClocalMin));
pause();
clc;

%% Stabilizzazione LQR

disp('Stabilizzazione mediante LQR dallo stato stimato');
disp('Press any key to continue.');
pause();

alphaK = 2;
QieCmp = blkdiag([0.00001 0; 0 0.00001],100*eye(3),eye(3));
Qie = blkdiag([0.01 0; 0 0.01],100000*eye(3),zeros(3,3));
Q = eye(size(AMin));
RieCmp = [1 0 0 0; 0 100000 0 0; 0 0 100000 0; 0 0 0 10000];
R = eye(size(BMinw,2));
K = lqr(AMin,BMinw,Q,R);
disp('Autovalori del sys a ciclo chiuso ottenuto per retroazione dallo stato:');
eig(AMin-BMinw*K)

disp('Premere un tasto per visualizzare la Step Response...');
pause;

tenzoLQR=ss(AMin-BMinw*K,BMinw,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoLQR);

%% Verifica condizioni Astatismo

disp('Condizioni Astatismo');
pause;
n=size(AMin,2);
p=size(BMinw,2);
q=size(ClocalMin,1);

% Definizione segnali esogeni
disp('Definizione dei Disturbi da reiettare:');
disp('d(t)=');
alpha=3; omega=0.5; gamma1=0;
k1=1; h1=1; h2=1; gamma2=complex(0,omega);

disp('Definizione segnali esogeni');
disp('gamma 1:='); disp(gamma1);
disp('gamma 2:='); disp(gamma2);

if (rank(ctrb(AMin+alpha*eye(n),BMinw))==n)
    disp('(a3) -> verificata, la coppia (AMin,BMin) raggiungibile, rank(P)'); 
    disp(rank(ctrb(AMin,BMinw))); 
end
if (rank(obsv(AMin+alpha*eye(n),ClocalMin))==n) 
    disp('(a3) -> verificata, la coppia (A,C) osservabile, rank(Q)');
    disp(rank(obsv(AMin,ClocalMin))); 
end

R1=[ AMin-gamma1*eye(size(AMin)) BMinw ;
    ClocalMin D];

if (rank(R1)==n+q) disp('b) verificata ,rango della matrice 5.4.23 per gamma1 �:'); disp(rank(R1)); end
R2=[ AMin-gamma2*eye(size(AMin)) BMinw ; ClocalMin D];
if (rank(R2)==n+q) disp('b) verificata ,rango della matrice 5.4.23 per gamma2 �:'); disp(rank(R2)); end

%% Calcolo del modello interno

disp('specifica 2) Calcolo modello interno KM1');
k1_segnato=max(k1,h1);
k2_segnato=h2;
disp('max(k1,h1):'); 
disp(k1_segnato); 
disp('max(k2,h2):'); 
disp(k2_segnato);
syms s;
disp('Definiamo il polinomio phi(lambda)');
phi1=((s-gamma1)^k1_segnato)*((s-gamma2)^k2_segnato)*((s-conj(gamma2))^k2_segnato)
disp('phi(lambda)=');
disp((phi1));
mu=3;
disp('mu=');
disp(mu);

Aphi=compan(sym2poly(phi1));
APhi=zeros(3);
APhi(3,:)=Aphi(1,:);
APhi(1,2)=Aphi(2,1);
APhi(2,3)=Aphi(3,2);
disp('matrice in forma compagna Aphi:');
disp(APhi);
disp('Bphi:');
BPhi=[0;0;1];
disp(BPhi);

disp('La matrice dinamica AK1 del modello interno KM1:');
AK1=blkdiag(APhi,APhi,APhi,APhi);
disp(AK1);
disp('La matrice dinamica BK1 del modello interno KM1:');
BK1=blkdiag(BPhi,BPhi,BPhi,BPhi);
disp(BK1);

%% Calcolo delle matrici F1,F2 + V di Kalman 
alpha = 0.5;

disp('Calcolo delle matrici F1,F2 per S1-S2 +  V per Kalman');
Asig =[ AMin-BMinw*K zeros(n,size(AK1,2)); -BK1*ClocalMin AK1];
Bsig = [ BMinw; -BK1*D];

R = eye(size(Asig,2));
Q = blkdiag([1 0; 0 1],1*eye(3),eye(3),eye(12));
%Q = 1000000*eye(20);
R = eye(size(Bsig,2));
F=-lqr(Asig+alpha*eye(size(Asig)),Bsig,Q,R);

F2=F(:,1:size(AMin,1));
disp('dimensioni [pxn]:');
disp(size(F2));
F1=F(:,size(AMin,1)+1:size(F,2));
disp('dimensioni [pxq*mu]:');
disp(size(F1));

% disp('matrice per Kalman:');
% V=lqr((AMin-BMin*K)',ClocalMin',Q,R)';
disp('dimensione attesa [nxq]');
disp(size(V));

disp('verifica spostamento autovalori:');
disp('autovalori A+B*F2');
disp(eig(AMin-BMinw*K+BMinw*F2));
disp('autovalori A-V*C');
disp(eig(AMin-BMinw*K-V*ClocalMin));

% specifica 3 ) definizione matrici per simulink:

disp('specifica 3 ) definizione matrici per simulink:');
% Cosa influenza il disturbo 
M=[ 1 0 0 0 1 1 1 1]';
N=[ 0; 0; 0; 0];
%per disturbo sul processo definisco Bmod:
Bmodw = [M BMinw]; %nota prima d e poi u scambio la somma per comodit�
Dmod  = [N D];

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
Aoss=AMin-V*ClocalMin;
Bossw=[BMinw-V*D V M-V*N]; %perche ho u,y,d   come ingressi, si noti che B-vD ha dim di B ma anche V ha dimn di B
Coss=eye(n);
Doss=zeros(size(Bossw,2));

% Ricordando che delta xi1=AK1*xi+ Bk1*e
AMI=AK1;
BMI=BK1;
CMI=eye(q*mu);
DMI=zeros(q*mu,q);

satW = 8000;
saMenoW = -400;

disp('avvio simulazione 1');
%pause;

open('progetto3Tenzo.mdl')
sim('progetto3Tenzo.mdl')

disp('End');