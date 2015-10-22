%% Tenzo Control System
%  oct-22-2015 UserK
%  info: userk@protonmail.ch
clear all;
clc;

% Physical properties of the quadrotor

g=9.81; %Acceleration of gravity (m)

% Density of air [ kg*m^-3 ] 
% From 35°C to - 25°C
rho = ureal('rho',1.2250,'Range',[1.1455 1.4224]);

% Total mass of the quadrotor [Kg]
mq = ureal('mq',1.150,'Range',[0.820 1.4224]);

% Mass of a motor (kg). All motors have equal mass.
mm = ureal('mm',0.068,'Range',[0.020 0.055]);
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
rp = ureal('rp',25.4e-2,'Range',[0.10 0.15]);
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

%% % Definiamo il sottosistema osservabile e raggiungibile
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

%% Invariant zeros 

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

%% Campioni de sistema
N=10

for i=1:1:N
sys{i} = usample(tenzo_min_unc);
% Additive
deltaA_sys{i} = tf(sys{i}) - tf(tenzo_min_nominale);
% Moltiplicative riportate sull'Ingresso
deltaMin_sys{i} = (inv(tf(tenzo_min_nominale))) * deltaA_sys{i};
% Moltiplicative riportate sull'Usicta
deltaMout_sys{i} = deltaA_sys{i} * (inv(tf(tenzo_min_nominale)));
end

%% Calcolo di un compensatore stabilizzante (1/3)
clc;
%Calcolo retroazione dallo stato
disp('Uso la funzione lqr per calcolare il guadagno K (la retroazione dallo stato u=Kx)')
Q=eye(8)
R=eye(4)
K = lqr(tenzo_min_nominale.a,tenzo_min_nominale.b,Q,R);
disp('Matrice di guadagno K: [comando K = lqr(A,B,Q,R)]');
disp(K);
disp('Autovalori a ciclo chiuso: [comando eig(A-B*K)]');
eK = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*K);
disp(eK);
disp('[Nota: retroazione negativa!]')

disp('Premere un tasto per continuare...')
pause;

%% Calcolo di un compensatore stabilizzante (2/3)
clc;
%Calcolo guadagno del filtro di Kalman
disp('Uso la funzione lqr per calcolare il guadagno L del filtro di Kalman')
W=eye(8)
V=eye(4)
L= lqr(tenzo_min_nominale.a',tenzo_min_nominale.c',W,V)';
disp('Matrice di guadagno L: [comando L = lqr(A'',C'',W,V)'']');
disp(L);
disp('Autovalori del filtro di Kalman: [comando eig(A-L*C)]');
eL = eig(tenzo_min_nominale.a-L*tenzo_min_nominale.c);
disp(eL);

disp('Premere un tasto per continuare...')
pause;

%% Calcolo di un compensatore stabilizzante (3/3)
clc;
%Costruisco il regolatore
disp('Costruisco il regolatore come filtro di Kalman + retroazione dallo stato stimato')
Ac = tenzo_min_nominale.a-tenzo_min_nominale.b*K-L*tenzo_min_nominale.c+L*tenzo_min_nominale.d*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));
disp('Ac = A-B*K-L*C+L*D*K');
disp(Ac);
disp('Bc = L, Cc = -K, Dc = zeros(size(K,1),size(L,2))')
disp('Autovalori a ciclo chiuso:');
sistema = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d);
controllore = ss(Ac,Bc,Cc,Dc);
CicloChiuso = feedback(series(controllore, sistema),eye(4), 1); %Retroazione positiva
[Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
eCl = eig(Acl);
disp(eCl);

disp('La figura mostra la risposta al gradino del sistema di controllo');

step(CicloChiuso);
disp('Premere un tasto per continuare...')
pause;

%% Provo una sintesi piu' veloce modificando i pesi
clc;
disp('Cambio i pesi per rendere piu'' rapida la convergenza')
disp('riducendo il peso relativo di R=V rispetto a Q=W')
Q=eye(8)
R=0.001*eye(4)
W=eye(8);
V=0.001*eye(4);
K = lqr(tenzo_min_nominale.a,tenzo_min_nominale.b,Q,R);
L= lqr(tenzo_min_nominale.a',tenzo_min_nominale.c',W,V)';
disp('Matrici di guadagno K ed L:');
disp(K);
disp(L);
eK1 = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*K);
eL1 = eig(tenzo_min_nominale.a-L*tenzo_min_nominale.c);
disp('Autovalori a ciclo chiuso con le due sintesi:');
disp('   K                 K1                 L                 L1:');
disp([eK eK1 eL eL1]);

Ac = tenzo_min_nominale.a-tenzo_min_nominale.b*K-L*tenzo_min_nominale.c+L*tenzo_min_nominale.d*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));

sistema = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d);
controllore1 = ss(Ac,Bc,Cc,Dc);
CicloChiuso1 = feedback(series(controllore1, sistema),eye(4), 1); %Retroazione positiva
% [Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
% eCl = eig(Acl);
% disp(eCl);

disp('La figura mostra la risposta al gradino del nuovo sistema di controllo');

hold on;
step(CicloChiuso1);

disp('Premere un tasto per continuare...')
pause;

%% Motivazione della traslazione

clc;
disp('La seconda sintesi ha reso la convergenza piu'' rapida ma ha ');
disp('modificato solo di poco gli autovalori a ciclo chiuso piu'' lenti!');
disp(' ');
disp('Parte reale degli autovalori a ciclo chiuso con le due sintesi:');
disp('   K        K1         L         L1:');
disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eL),'descend') sort(real(eL1),'descend')]);

disp('Per rendere tutti gli autovalori a ciclo chiuso piu'' rapidi');
disp('(uniformemente), posso usare la traslazione');
disp('[comando K=lqr(A+alfa*I,B,Q,R) con alfa > 0]');
disp('che garantisce che la parte reale di eig(A-B*K)<-alfa ');

disp(' ');
disp(' ');
disp('Premere un tasto per continuare...');
pause;

%% Over the top
clc;
disp('Con i pesi originali, rendo piu'' rapida la convergenza')
disp('traslando la A con una traslazione alfa=5')
Q=eye(8)
R=eye(4)
W=eye(8);
V=eye(4);
alfa=3;
K = lqr(tenzo_min_nominale.a+alfa*eye(size(tenzo_min_nominale.a)),tenzo_min_nominale.b,Q,R);
L= lqr(tenzo_min_nominale.a'+alfa*eye(size(tenzo_min_nominale.a)),tenzo_min_nominale.c',W,V)';
disp('Matrici di guadagno K ed L:');
disp(K);
disp(L);
disp('Autovalori a ciclo chiuso:');
eK2 = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*K);
eL2 = eig(tenzo_min_nominale.a-L*tenzo_min_nominale.c);
disp('Parte reale degli autovalori a ciclo chiuso con le tre sintesi:');
disp('   K        K1         K2        L         L1         L2:');
disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eK2),'descend') sort(real(eL),'descend') sort(real(eL1),'descend') sort(real(eL2),'descend')]);

Ac = tenzo_min_nominale.a-tenzo_min_nominale.b*K-L*tenzo_min_nominale.c+L*tenzo_min_nominale.d*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));

disp('Autovalori a ciclo chiuso:');


controllore2 = ss(Ac,Bc,Cc,Dc);
CicloChiuso2 = feedback(series(controllore2, sistema),eye(4), 1); 

step(CicloChiuso2);

figure(3);
for i=1:N
    sys{i} = usample(tenzo_min_unc);
    CicloChiusoUnc{i} =  feedback(series(controllore2, sys{i}),eye(4), 1); 
    step(CicloChiusoUnc{i});
    pause();
    hold on;
end
hold off;
% eCl = eig(Acl);
% disp(eCl);


