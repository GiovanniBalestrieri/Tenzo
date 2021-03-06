%% Variable declaration

% Initial Conditions
ze0=1.1;
phi0 = 1.1;
theta0 = -1.2;
psi0 = 0.2;

% Inflow coefficient
If = -0.3559;

% Acceleration of gravity [m*s^-2]
g=9.81; 
% Density of air [m^3.kg^-1]
rho=1.2;

% Total mass of the quadrotor [Kg]
mq=1.20;
% Mass of a motor [kg]. All motors have equal mass.
mm=0.068; 

% Motor length along x,y,z-axis [m] 
lx=28.8e-3;
ly=28.8e-3;
lz=26e-3;
% Distance from the center of gravity to the center of a motor [m].
% The quadrotor is symmetric regarding to XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

% Moment of inertia (x-axis) for motors 1 and 3 [kg.m^2].
Ix1=(1/12)*mm*(ly^2+lz^2); 
% Moment of inertia (x-axis) for motors 2 and 4 [kg.m^2].
Ix2=(1/12)*mm*(ly^2+lz^2)+mm*dcg^2;
% Total moment of inertia along the x-axis [kg.m^2]
Ixx=2*Ix1+2*Ix2; 
% Moment of inertia (y-axis) for motors 1 and 3 [kg.m^2].
Iy1=(1/12)*mm*(lx^2+lz^2)+mm*dcg^2; 
% Moment of inertia (y-axis) for motors 2 and 4 [kg.m^2].
Iy2=(1/12)*mm*(lx^2+lz^2); 
% Total moment of inertia along the y-axis [kg.m^2]
Iyy=2*Iy1+2*Iy2; 
% Moment of inertia (z-axis) for motors 1 and 3 [kg.m^2]
Iz1=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Moment of inertia (z-axis) for motors 2 and 4 [kg.m^2]
Iz2=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Total moment of inertia along the z-axis [kg.m^2]
Izz=2*Iz1+2*Iz2;
% Inertia matrix
II=diag([Ixx Iyy Izz]); 


% Thrust coefficient of the propeller and Power coefficient
% Refs: http://m-selig.ae.illinois.edu/props/plots/apcsf_9x4.7_cp.png
cp=0.045;
% Refs: http://m-selig.ae.illinois.edu/props/plots/apcsf_9x4.7_ct.png
ct=0.1154;

% Propeller radius (m) 10" = 25.4e-3 9" = 22.6800
rp=22.68e-2; 
% Constant value to calculate the moment provided
% by a propeller given its angular speed (kg.m^2.rad^-1)
Km=cp*4*rho*rp^5/pi()^3; 
% Constant value to calculate the thrust provided
% by a propeller given its angular speed (kg.m.rad^-1) 
%Kt=ct*4*rho*rp^4/pi()^2; 
Kt=ct*rho*rp^4*pi();

% Constant that relates thrust and moment of a propeller.
Ktm=Km/Kt;

%% Preprocessing

% Compute angular speed required to beat the gravitational force

%Dead zone of the motor (PWM pulse-width in microseconds)
PWMdz1 = 1256;
PWMdz2 = 1256;
PWMdz3 = 1256;
PWMdz4 = 1256;

%Calculate the PWM pulse necessary to beat the force of gravity
syms wg1 wg2 wg3 wg4
% wg1=solve((wg1-PWMdz1)^2-(1/4)*mq*g/Kt,wg1);
% wg1=double(wg1);
% wg1=wg1(wg1>PWMdz1);
% wg2=solve((wg2-PWMdz2)^2-(1/4)*mq*g/Kt,wg2);
% wg2=double(wg2);
% wg2=wg2(wg2>PWMdz2);
% wg3=solve((wg3-PWMdz3)^2-(1/4)*mq*g/Kt,wg3);
% wg3=double(wg3);
% wg3=wg3(wg3>PWMdz3);
% wg4=solve((wg4-PWMdz4)^2-(1/4)*mq*g/Kt,wg4);
% wg4=double(wg4);
% wg4=wg4(wg4>PWMdz4);


wg1=double(solve((wg1)^2-(1/4)*mq*g/Kt,wg1));
wg1=wg1(1,1);
wg2=double(solve((wg2)^2-(1/4)*mq*g/Kt,wg2));
wg2=wg2(1,1);
wg3=double(solve((wg3)^2-(1/4)*mq*g/Kt,wg3));
wg3=wg3(1,1);
wg4=double(solve((wg4)^2-(1/4)*mq*g/Kt,wg4));
wg4=wg4(1,1);

disp('Angular Velocity required for hovering:');

U0=[wg1;wg2;wg3;wg4]
%U0=[0;0;0;0]


%% State space system

% Dopo linearizzazione con punto di lavoro
% (z=X,U=[1.3066e+03,1.3066e+03,1.3066e+03,1.3066e+03])

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
      zeros(3,12)]

B = [zeros(5,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz]
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
Clocal = [ 0 0 0 0 0 0 1 0 0 0 0 0; 
           0 0 0 0 0 0 0 1 0 0 0 0; 
           0 0 0 0 0 0 0 0 1 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0];
       
inputs = {'Thrust','TauPhi','TauTheta','TauPsi'};
D = zeros(4,4);
tenzo=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);

% Transfer function
PsTot = tf(tenzo);

% invarianzZero
disp('Invariant Zeros:');
tzero(A,B,Clocal,D,eye(12))

disp('Transmission Zeros');
tzero(PsTot)

disp('X')
pause();
%% Eigenvalues of the system

disp('Verifica preliminare, autovalori del processo [eig(A)]:')
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
if (stab==0) disp('Sistema instabile! Gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
if (stab==1) disp('Sistema stabile! OLE!! Gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
if (stab==2) disp('Sistema instabile! Sono presenti autovalori pari a zero con molteplicità > 1'); end
disp(eOp);


% Analisi risposta a gradino
disp('Press any for Step Response:');
pause;

tenzoRetro=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);

%% Propriet� strutturali:
% Verifica Raggiungibilit� e Osservabilit�

disp('Press any key to check propriet� strutturali:');
pause();

disp('Verifica Raggiungibilit�. Rango della matrice P:')
if (rank(ctrb(A,B))==size(A,1))
    disp('Sistema raggiungibile');
else
    disp('Sistema Irraggiungibile');
end
disp(rank(ctrb(A,B)));


disp('Verifica osservabilit�. Rango della matrice Q:')
if (rank(obsv(A,Clocal))==size(A,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(A,Clocal)));

pause();
clc;

%% Il sys non � osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile
disp('Semplificazione del modello');
disp('Press any key to continue.');
pause();
  
  AMin = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  
  BMin = [
    zeros(1,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz];
  
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};

tenzoMin=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal)

Ps=tf(tenzoMin);

% Transmission zeros
tzero(tenzoMin)

% Invariant Zeros
tzero(AMin,BMin,ClocalMin,D,eye(8))
%% Propriet� strutturali:
% Verifica Raggiungibilit� e Osservabilit�

disp('Verifica raggiungibil�: rank([A-gI,B]) : per tutti g � spec(A)')
if (rank(ctrb(AMin,BMin))==size(AMin,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(AMin,BMin)));
else
    disp('Sistema Irraggiungibile');
end

disp('Verifica osservabilit�. Rango della matrice Q:')
if (rank(obsv(AMin,ClocalMin))==size(AMin,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(AMin,ClocalMin)));

pause();

%% Luenberger Observer

P =[-1; -1.1 ;-1.2 ;-1.3; -1.4 ; -1.5 ; -1.6 ;-1.7]*1000;
L=place(AMin',ClocalMin',P)';

disp('Ricostruzione dello stato con Luenberger');
disp('X');
pause();

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
disp(size(L));
Aoss=AMin-L*ClocalMin;
Boss=[BMin-L*D L]; %perche ho u,y,d   come ingressi, si noti che B-vD ha dim di B ma anche V ha dimn di B
Coss=eye(size(AMin));
Doss=zeros(size(Boss));

disp('Autovalori A-L*C');
disp(eig(AMin-L*ClocalMin));

%% LQR

disp('Stabilizzazione mediante LQR dallo stato stimato');

alphaK = 2;
Qie = blkdiag([0.1 0 ;0 1],0.01*eye(3),eye(3));
Q = eye(size(AMin));
% [T,tx,ty,tz]
Rie = [10 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 0.0000001];
R = eye(size(BMin,2));
K = lqr(AMin,BMin,Q,Rie);
disp('Autovalori del sys a ciclo chiuso ottenuto per retroazione dallo stato:');
eig(AMin-BMin*K)

disp('X');

tenzoLQR=ss(AMin-BMin*K,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoLQR);

%% Simulazione

refs=[5 0 1 1 2 0 0 0 ]; 
tc=0.436;
X0c = 3000;
dz1=1247.4;
PulseOff = 1;
pulseAmp = 500;
pulsePh = 0;
pulseP = 20;
pulseP2 = 10;
pulseP4= 10;
w1=3100;
w2=3100;
w3=3100;
w4=3100;

Knl = [-1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

sys = 'lqr_120';
open_system(sys)
SimOut = sim(sys);

disp('End');
%% Optimal observer with Kalman