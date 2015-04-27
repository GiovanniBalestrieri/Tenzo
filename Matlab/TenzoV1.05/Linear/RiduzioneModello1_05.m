%% Tenzo Control System
%  gen-23-2015 UserK
%  info: balestrieri.gepp@gmail.com

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
w0c=sqrt(g*mq/Kt);
%X0c=w0c;
Ft0=mq*g;
%% Linearized Model

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
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz];
 
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
DFull = zeros(size(A),4);
inputs = {'Thrust','TauPhi','TauTheta','TauPsi'};
D = zeros(4,4);


tenzo=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
%tenzo=ss(A,B,CFull,DFull);

%disp('Transfer matrix of the model')
%modello_tf=tf(tenzo)

pause;
%% Eigenvalues of the system

disp('specifica 1) Verifichare che esiste un cotrollore che soddisfa le specifiche A1,B,C1 (teorema 5.4.2):');
disp('verifica preliminare, autovalori del processo [eig(A)]:')
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

pause;
disp(eOp);

%% Analisi risposta a gradino
tenzoRetro=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);


%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
if (rank(ctrb(A,B))==size(A,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(A,B)));
else
    disp('Sistema Irraggiungibile');
end


disp('Verifica osservabilità. Rango della matrice di osservabilità:')
if (rank(obsv(A,Clocal))==size(A,1))
    disp('Sistema controllabile');
    disp(rank(ctrb(A,B)));
else    
    disp('Sistema Non controllabile');
end
disp(rank(obsv(A,Clocal)));

pause();

%% Il sys non è osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile

AMin = [ 0 0 0 0 0 0 0  0;
       0 0 0 -g 0 0 0 0;
       0 1 g 0 0 0 0 0;
       0 If 0 0 0 0 0 0;
       0 0 0 0 0 1 0 0; 
       0 0 0 0 0 0 1 0; 
       0 0 0 0 0 0 0 1; 
      zeros(1,8)];
  
  AMin = [ 0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  
  BMin = [zeros(1,4);
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

tenzoMin=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);

tenzoRetro=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro)

% Check del sottosistema ragg e oss con decomposizione di Kalman
[sysr,U] = minreal(tenzoMin,0.1)

KalmanA = U*AMin*U'
KalmanB = U*BMin
KalmanC = ClocalMin*U'

%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
if (rank(ctrb(AMin,BMin))==size(AMin,1))
    disp('Sistema raggiungibile');
    disp((ctrb(AMin,BMin)));
else
    disp('Sistema Irraggiungibile');
end


disp('Verifica osservabilità. Rango della matrice di osservabilità:')
if (rank(obsv(AMin,ClocalMin))==size(AMin,1))
    disp('Sistema controllabile');
    disp(rank(obsv(AMin,ClocalMin)));
else    
    disp('Sistema Non controllabile');
end

pause();

%% Ricostruzione dello stato con Kalman

%si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
alphaK = 100;
Q = eye(size(AMin));
W = eye(size(AMin));
R = eye(size(ClocalMin,1));
disp('matrice per Kalman:')
V=lqr((AMin+alphaK*eye(size(AMin)))',ClocalMin',Q,R)'
disp('dimensione attesa [nxq]');
disp(size(V));
Aoss=AMin-V*ClocalMin;
Boss=[BMin-V*D V]; %perche ho u,y,d   come ingressi, si noti che B-vD ha dim di B ma anche V ha dimn di B
Coss=eye(size(AMin));
Doss=zeros(size(Boss));


disp('autovalori A-V*C');
disp(eig(AMin-V*ClocalMin));

disp('Premere un tasto per continuare...');
pause;

%% Stabilizzazione LQR

alphaK = 2;
Qie = blkdiag([0.00001 0 0; 0 0.00001 0; 0 0 0.00001],zeros(3),100*eye(3),eye(3));
Q = eye(size(AMin));
Rie = [1 0 0 0; 0 0.00001 0 0; 0 0 0.00001 0; 0 0 0 0.00001];
R = eye(size(BMin,2));
K = lqr(AMin,BMin,Q,R);
disp('Autovalori del sys a ciclo chiuso ottenuto per retroazione dallo stato:');
eig(AMin-BMin*K)

disp('Premere un tasto per continuare...');
pause;


tenzoLQR=ss(AMin-BMin*K,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoLQR);


%% Simulazione

refs=[0 0 -20 0 0 0 0 0 0 0 0 0]; 
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
% w11e3=0;

sys = 'tenzo1_10_lqr';
sys = 'tenzo1_10_lqr0';
open_system(sys)
SimOut = sim(sys);

disp('End');
pause();