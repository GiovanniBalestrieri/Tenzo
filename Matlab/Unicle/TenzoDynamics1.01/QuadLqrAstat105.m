%Description: This m-file initializes all necessary variables required to
%buid the dynamic model of a quadrotor in the state-space form.
%
%clear all;
%% References
X0c = 3950;

w1=3050;
w2=5820;
w3=4250;
w4=5940;
rif = [ w1 w2 w3 w4];

% Set PulseOff = 0 to disable pulse mode, PulseOff=1 to ena
PulseOff=1;
pulseAmp=100;
pulseP=20;
pulsePh=7; % simulation time (sec)

%% Physical properties of the environment
g=9.81; %Acceleration of gravity (m)
rho=1.2; %Density of air (m^3.kg^-1)

%% Physical properties of the quadrotor

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

%% Arduino variables
freq = 20; %Sampling frequency (Hz);
ardres = 10; %Arduino analog resolution (bits)
% %% Sensors
daccel_x = 0.003; %Accelerometer distance to the quadrotor’s center of
%gravity along the x-axis (m)
daccel_y = 0; %Accelerometer distance to the quadrotor’s center of
%gravity along the y-axis (m)
daccel_z = -0.0490; %Accelerometer distance to the quadrotor’s center of
%gravity along the z-axis (m)
daccel=[daccel_x daccel_y daccel_z];
accel_max = 3*g; %Accelerometer maximum possible reading
accel_min = -3*g; %Accelerometer minimum possible reading

% Accelerometer reading resolution
ares = (accel_max-accel_min)/(2^ardres);

% Compass reading resolution (rad) = i degree x 180 degrees / pi() rad.
% Note: this can be configured in the Arduino to be either 1 or 0.5 degrees
cres = 1*pi()/180;

%% Initial states
k1 = 2.0276; %Constant that relates Rotation speed of propeller 1 with the
%corresponding PWM pulse of the Arduino
k2 = 1.8693; %Constant that relates Rotation speed of propeller 2 with theB.1. LOADVARS.M
%corresponding PWM pulse of the Arduino
k3 = 2.0018; %Constant that relates Rotation speed of propeller 3 with the
%corresponding PWM pulse of the Arduino
k4 = 1.9986; %Constant that relates Rotation speed of propeller 4 with the
%corresponding PWM pulse of the Arduino

PWMdz1 = 1256; %Dead zone of the motor 1 (PWM pulse-width in microseconds)
PWMdz2 = 1264; %Dead zone of the motor 2 (PWM pulse-width in microseconds)
PWMdz3 = 1256; %Dead zone of the motor 3 (PWM pulse-width in microseconds)
PWMdz4 = 1256; %Dead zone of the motor 4 (PWM pulse-width in microseconds)

%Calculate the PWM pulse necessary to beat the force of gravity
syms PWMg1 PWMg2 PWMg3 PWMg4
PWMg1=solve((PWMg1-PWMdz1)^2-(1/4)*mq*g/(Kt*(k1^2)),PWMg1);
PWMg1=double(PWMg1);
PWMg1=PWMg1(PWMg1>PWMdz1);
PWMg2=solve((PWMg2-PWMdz2)^2-(1/4)*mq*g/(Kt*(k2^2)),PWMg2);
PWMg2=double(PWMg2);
PWMg2=PWMg2(PWMg2>PWMdz2);
PWMg3=solve((PWMg3-PWMdz3)^2-(1/4)*mq*g/(Kt*(k3^2)),PWMg3);
PWMg3=double(PWMg3);
PWMg3=PWMg3(PWMg3>PWMdz3);
PWMg4=solve((PWMg4-PWMdz4)^2-(1/4)*mq*g/((Kt*k4^2)),PWMg4);
PWMg4=double(PWMg4);
PWMg4=PWMg4(PWMg4>PWMdz4);

%Compute the necessary rotation speed (rad.s^-1) of each propeller to beat
%the force of gravity
wm1 = double(k1*(PWMg1-PWMdz1));
wm2 = double(k2*(PWMg2-PWMdz2));
wm3 = double(k3*(PWMg3-PWMdz3));
wm4 = double(k4*(PWMg4-PWMdz4));

%% Initial Conditions

% Initial angular speed of motors 1 to 4 (rad.s^-1)
w0=[wm1 wm2 wm3 wm4]; 
% Initial linear velocity [m.s^-1] along the axis:
% VXb -> U
% VYb -> V
% VZb -> W

U0=0; 
V0=0;
W0=0;

% Initial angular velocity [rad.s^-1] along the axis:
% WXb -> P
% WYb -> Q
% WZb -> R
P0=0;
Q0=0; 
R0=0; 

% Initial quadrotor position 

X0=0; 
Y0=0; 
Z0=0; 

% Initial angular position [rad]
%   pitchE -> phi
%   rollE  -> theta
%   psiE   -> psi
PHI0=0; 
THETA0=0; 
PSI0=0; 

%% Linearization
% 
% tenzoSS = load('linsys1.mat', 'linsys1')
loadA=load('A.mat');
loadB=load('B.mat');
loadC=load('C.mat');
loadD=load('D.mat');
A=loadA.A;
B=loadB.B;
C=loadC.C;
D=loadD.D;

states = {'xe','ye','ze','vxe','vye','vze','phi','theta','psi','wxb','wyb','wzb'};
inputs = {'Thrust','TauX','Tauy','Tauz'};
%outputs = {'phi'; 'theta';'psi';'ze'};
outputs = {'xe','ye','ze','vxe','vye','vze','phi','theta','psi','wxb','wyb','wzb'};

%C = [0 0 0 0 0 0 1 0 0 0 0 0;
%         0 0 0 0 0 0 0 1 0 0 0 0;
%         0 0 0 0 0 0 0 0 1 0 0 0;
%         0 0 1 0 0 0 0 0 0 0 0 0];
    
%D=zeros(4,4);

tenzo=ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% B=tenzoSS.b;
% C=tenzoSS.c;
% D=tenzoSS.d;

%% specifica1 Verificare che esiste un cotrollore che soddisfa le specifiche A1,B,C1
%clc;


alpha=2;
omega=4;
%f=omega/(2*pi);
gamma1=1;
k1=1;
h1=1;
h2=1;
gamma2=complex(0,omega);
disp('definizione segnali esogeni');
disp('gamma1='); disp(gamma1);
%disp('gamma2='); disp(gamma2);
disp('Premere un tasto per continuare...');
pause;


disp('specifica 1) Verifichare che esiste un cotrollore che soddisfa le specifiche A1,B,C1 (teorema 5.4.2):');
disp('verifica preliminare, autovalori del processo [eig(A)]:')
stab=1;
eOp = eig(A);
[dn,dm]=size(eOp);
  for i=1:dn,
      if (real(eOp(i))>0 ) stab=0; disp('elemento a parte reale positiva:'); disp(eOp(i)); end    
  end
if (stab==0) disp('Sistema instabile: gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
if (stab==1) disp('Sistema stabile: gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
disp(eOp);

if (rank(ctrb(A-alpha*eye(size(A)),B))==size(A)) 
    disp('a1) verificata, la coppia A,B � raggiungibile, rank(matrice controllabilit� �)');
    disp(rank(ctrb(A,B))); 

% if (rank(ctrb(A+alpha*eye(size(A,1)),B))==rank(A)) 
%     disp('a1) verificata, la coppia A,B � raggiungibile, rank(matrice controllabilit� �)');
%     disp(rank(ctrb(A,B))); 
end
% if (rank(obsv(A+alpha*eye(size(A,1)),C))==rank(A)) 
%     disp('a1) verificata, la coppia A,C � osservabile, rank(matrice osservabilit� �)'); 
%     disp(rank(obsv(A,C))); 
% end

if (rank(obsv(A-alpha*eye(size(A)),C))==size(A)) 
    disp('a1) verificata, la coppia A,C � osservabile, rank(matrice osservabilit� �)'); 
    disp(rank(obsv(A,C))); 
end

disp('Premere un tasto per continuare...');
pause;


R1=[ A-gamma1*eye(size(A)) B ; C D]
disp('Rank R1:');
disp(rank(R1));
if (rank(R1)==size(A,1)+size(C,1))
    disp('b) verificata ,rango della matrice 5.4.23 per gamma1 �:');
    disp(rank(R1));
else
    disp('test fallito per gamma1');
end
R2=[ A-gamma2*eye(size(A)) B ; C D];
disp('Rank R2:');
disp(rank(R2)); 
if (rank(R2)==size(A,1)+12) 
    disp('b) verificata ,rango della matrice 5.4.23 per gamma2 �:');
    disp(rank(R2)); 
else
    disp('Test Fallito per gamma2');
end


%% calcolo modello interno
disp('specifica 2) Calcolo modello interno KM1');
disp('Premere un tasto per continuare...');
pause;
k1_segnato=max(k1,h1);
k2_segnato=h2;
disp('definiamo k1 segnato come massimo tra k e h dei disturbi e riferimenti i-esimi');
disp('max(k1,h1):'); disp(k1_segnato); disp('max(k2,h2):'); disp(k2_segnato);
syms s;
phi1=((s-gamma1)^k1_segnato)*((s-gamma2)^k2_segnato)*((s-conj(gamma2))^k2_segnato);
disp('phi(lambda)='); disp((phi1));
mu=3;
disp('mu='); disp(mu);

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

disp('la matrice dinamica AK1 del modello interno KM1 :');
AK1=blkdiag(APhi,APhi,Aphi,Aphi,Aphi,Aphi,Aphi,Aphi,Aphi,Aphi,Aphi,Aphi);
disp(AK1);
disp('la matrice dinamica BK1, con q blocchi diagonali, del modello interno KM1 :');
BK1=blkdiag(BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi,BPhi);
disp(BK1);

disp('Premere un tasto per continuare...');
pause;

%% Calcolo delle matrici F1,F2 pe3r stabilizzare la scascata in anello aperto, V per Kalman 

% disp('Calcolo delle matrici F1,F2 per stabilizzare la cascata S1-S2, ovvero l anello aperto,e  V per Kalman ');
% Asig =[ A zeros(size(A),size(AK1,2)); -BK1*C AK1];
% Bsig = [ B; -BK1*D];
% Q = eye(size(A)+size(AK1));
% R = eye(size(Bsig,2));
% F=-lqr(Asig+alpha*eye(size(Asig)),Bsig,Q,R);
% 
% disp('matrici stablizzanti:')
% F2=F(:,1:10)
% disp('dimensioni [pxn]:'); disp(size(F2));
% F1=F(:,11:16)
% disp('dimensioni [pxq*mu]:'); disp(size(F1));
% 
% disp('Premere un tasto per continuare...');
% pause;
%% Calcolo LQR 

Q = eye(size(A));
R = eye(size(B,2));
disp('matrice per lqr:')
K=lqr(A,B,Q,R);
disp('dimensione attesa [qxn]');
disp(size(K));

AK=[(A-B*K)];
Bc=B;
Cc=C;
Dc=D;

disp('verifica spostamento autovalori:');
%disp('autovalori A+B*F2');
%disp(eig(A+B*F2));
disp('autovalori A-B*K');
disp(eig(AK));

disp('Premere un tasto per continuare...');
pause;
%% Calcolo Filtro di Kalman

alphaK=30;
Qk = eye(size(A));
Rk = eye(size(C,2));
disp('matrice per Kalman:')
V=lqr((A+alphaK*eye(size(A)))',C',Qk,Rk)'
disp('dimensione attesa [nxq]');
disp(size(V));

disp('verifica spostamento autovalori:');
disp('autovalori A-V*C');
disp(eig(A-V*C));

disp('Premere un tasto per continuare...');
pause;
%% specifica 3 ) definizione matrici per simulink:
disp('specifica 3 ) definizione matrici per simulink:');
%per disturbo sul processo defibnisco Bmod:
%Bmod=[M B]; %nota prima d e poi u scambio la somma per comodit�
%Dmod=[N D];

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
Aoss=A-V*C;
Boss=[B-V*D V]; %M-V*N]; %perche ho u,y,d   come ingressi, si noti che B-vD ha dim di B ma anche V ha dimn di B
Coss=eye(size(A));
Doss=zeros(size(Boss));

%ricordando che delta xi1=AK1*xi+ Bk1*e
%AMI=AK1;
%BMI=BK1;
%CMI=eye(q*mu);
%DMI=zeros(q*mu,q);

%% Open symulink model

disp('Avvio Simulazione');
sys = 'tenzo6dof';
open_system(sys)
SimOut = sim('tenzo6dof');
%%
disp('avvio il programma per il calcolo ottimizzato');
alpha=1;


% %Initial state quaternion
% q0=angle2quat(PHI0,THETA0,PSI0); 
% 
% %Initial state vector
% X0=[U0 V0 W0 P0 Q0 R0 X0 Y0 Z0 q0(1) q0(2) q0(3) q0(4)];
% 
% Z0lin = -1; %Linearization point with the quadrotor stabilized (m)
% PHIlin = 0; %Linearization point pitch angle of the quadrotor (rad)
% THETAlin = 0; %Linearization point roll angle of the quadrotor (rad)
% PSIlin = 0; %Linearization point yaw angle of the quadrotor (rad)
% qlin=angle2quat(PHIlin,THETAlin,PSIlin); %Initial state quaternion
% 
% %Linearization point for generating the state-space system representation of the quadrotor
% %X0lin=[0 0 0 0 0 0 0 0 Z0lin PHIlin THETAlin PSIlin];
% X0lin=[0 0 0 0 0 0 0 0 Z0lin qlin];
% 
% [A,B,C,D]=quadss(X0lin,mq,I,w0,Kt,Ktm,dcg);%,daccel); %Get linearized state
% %space matrices of the quadrotor
% 
% %Initial quadrotor reference
% Y0ref = [0 0 0 0 0 0]; 
% 
% %Get Kalman filter
% quadkalm; 
% %Get LQR controller
% quadLQRcontrol; 
% 
% %quadLQRcontrol_12states %Uncomment this line if you wish 12 states control
% %with pure feedback (i.e. no Kalman filter nor sensors)
% 
% damp(A-B*[zeros(4,3) Klqr(:,1:3) zeros(4,3) Klqr(:,4:6)]) % Eigen values of
% % the closed loop. If an eigenvalue is not stable, the dynamics of this
% % eigenvalue will be present in the closed-loop system which therefore will
% % be unstable.