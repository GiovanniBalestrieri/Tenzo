%% Variable declaration

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

%% State space system

states = {'xe','ye','ze','vxe','vye','vze','phi','theta','psi','wxb','wyb','wzb'};
A = [ 0 0 0 1 0 0 0 0 0 0 0 0;      0 0 0 0 1 0 0 0 0 0 0 0;
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
if (stab==2) disp('Sistema instabile! Sono presenti autovalori pari a zero con molteplicitÃ  > 1'); end
disp(eOp);


% Analisi risposta a gradino
disp('Press any for Step Response:');
pause;
tenzoRetro=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);

%% Proprietà  strutturali:
% Verifica Raggiungibilità  e Osservabilità

disp('Press any key to check proprietà strutturali:');
pause();

disp('Verifica Raggiungibilità . Rango della matrice P:')
if (rank(ctrb(A,B))==size(A,1))
    disp('Sistema raggiungibile');
else
    disp('Sistema Irraggiungibile');
end
disp(rank(ctrb(A,B)));


disp('Verifica osservabilità . Rango della matrice Q:')
if (rank(obsv(A,Clocal))==size(A,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(A,Clocal)));

pause();
clc;

%% Il sys non è osservabile. 
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
%% Proprietà  strutturali:
% Verifica Raggiungibilità  e Osservabilità

disp('Verifica raggiungibilà : rank([A-gI,B]) : per tutti g € spec(A)')
if (rank(ctrb(AMin,BMin))==size(AMin,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(AMin,BMin)));
else
    disp('Sistema Irraggiungibile');
end

disp('Verifica osservabilità . Rango della matrice Q:')
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
clc;

%% LQR

disp('Stabilizzazione mediante LQR dallo stato stimato');

alphaK = 2;
Qie = blkdiag([10 0 ;0 1],50*eye(3),eye(3));
Q = eye(size(AMin));
Rie = [0.01 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10];
R = eye(size(BMin,2));
K = lqr(AMin,BMin,Qie,Rie);
disp('Autovalori del sys a ciclo chiuso ottenuto per retroazione dallo stato:');
eig(AMin-BMin*K)

disp('X');
pause;

tenzoLQR=ss(AMin-BMin*K,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoLQR);


%% Simulazione

refs=[-3 0 25 25 45 0 0 0 ]; 
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

sys = 'lqr_115';
open_system(sys)
SimOut = sim(sys);

disp('End');
%% Optimal observer with Kalman