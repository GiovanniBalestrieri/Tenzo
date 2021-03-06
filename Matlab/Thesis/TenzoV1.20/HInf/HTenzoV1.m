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
disp('Linearized Model:');

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
       
inputs = {'w1^2','w2^2','w3^2','w4^2'};

D = zeros(4,4);

tenzo_unc = uss(A,Bw,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
tenzo_nominale=tenzo_unc.NominalValue

disp('Assunzione: Esiste almeno una classe di segnali esogeni rispetto alla quale regolare/far inseguire asintoticamente y(t) Verifica preliminare, autovalori del processo [eig(A)]:')

disp('Verifica preliminare, autovalori del processo [eig(A)]:')

%% Motors

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

mot= zpk([],[-10 -220],1273);

m = canon(mot,'modal');
mot1= m^2;

m1 = canon(mot1,'modal');

% open('testMotors');
% sim('testMotors');

%% Eigenvalues of the system

stab=1;
eOp = eig(tenzo_nominale.a);
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
step(tenzo_nominale);

%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

if (rank(ctrb(tenzo_nominale.a,tenzo_nominale.b))==size(tenzo_nominale.a,1))
    disp('Sistema raggiungibile');
else
    disp('Sistema Irraggiungibile');
end
disp(rank(ctrb(tenzo_nominale.a,tenzo_nominale.b)));

if (rank(obsv(tenzo_nominale.a,tenzo_nominale.c))==size(tenzo_nominale.a,1))
    disp('Sistema osservabile');
else    
    disp('Sistema Non osservabile');
end
disp(rank(obsv(tenzo_nominale.a,tenzo_nominale.c)));

%% Il sys non è osservabile. 
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

%% Proprietà strutturali:

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


%% Prende alcuni campioni del sistema incerto e calcola bound su incertezze

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

%%

% generates 200 points between decades 10^( -2 ) and 10^( 2 ).
omega = logspace(-3,3,250);

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

max_sig_unc
% Returns a row vector containing the maximum element from each column.
top_unc = max(max_sig_unc);
top_dA = max(max_sig_dA);
top_dMin = max(max_sig_dMin);
top_dMout = max(max_sig_dMout);

%% 

figure(1);
semilogx(omega,mag2db(max_sig_nom),'k--','LineWidth',2)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_unc(i,:)),'r:','LineWidth',2)
end
semilogx(omega,mag2db(top_unc),'b','LineWidth',5)
semilogx(omega,mag2db(max_sig_nom),'k','LineWidth',5)
title('Max sing values: Nominal (bk), Pert models (R), bound (blue)')

%%  Additive
figure(2);
semilogx(omega,mag2db(top_dA),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dA(i,:)),'r:','LineWidth',2)
end
title('Max sing values: additive uncertainties (red), bound (blue)')


%% Input Moltiplicative uncertainties

figure(3);
semilogx(omega,mag2db(top_dMin),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dMin(i,:)),'r:','LineWidth',2)
end
title('Max sing values: input multiplicative uncertainties (red), bound (blue)')

%% output multiplicative uncertainties

figure(4);
semilogx(omega,mag2db(top_dMout),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dMout(i,:)),'r:','LineWidth',2)
end
title('Max sing values: output multiplicative uncertainties (red), bound (blue)')

%% Upper bound razionale stabile e fase minima
pre_bound_dA = frd(top_dA,omega);

% fit razionale e min phase per ricavare il bound
ord = 2; %Ordine della funzione di fitting 
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA2 = sigma(bound_dA,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA5 = sigma(bound_dA,omega);
ord = 7; %Ordine della funzione di fitting
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA7 = sigma(bound_dA,omega);

figure(5);
semilogx(omega,mag2db(top_dA),'b','LineWidth',2);
grid on;
hold on;
semilogx(omega,mag2db(bb_dA2(1,:)),'r','LineWidth',2);
semilogx(omega,mag2db(bb_dA5(1,:)),'k','LineWidth',2);
semilogx(omega,mag2db(bb_dA7(1,:)),'m','LineWidth',2);
title('Bound on additive uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
  'Location','SouthWest');

%% Upper bound razionale stabile e fase minima
pre_bound_dMin = frd(top_dMin,omega);

% fit razionale e min phase per ricavare il bound
%ord = 2; %Ordine della funzione di fitting 
%bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
%bb_dMin2 = sigma(bound_dM,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
bb_dMin5 = sigma(bound_dM,omega);
%ord = 7; %Ordine della funzione di fitting 
%bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
%bb_dMin7 = sigma(bound_dM,omega);

figure(6);
semilogx(omega,mag2db(top_dMin),'b','LineWidth',2)
grid on;
hold on;
%semilogx(omega,mag2db(bb_dMin2(1,:)),'r','LineWidth',2)
semilogx(omega,mag2db(bb_dMin5(1,:)),'k','LineWidth',2)
%semilogx(omega,mag2db(bb_dMin7(1,:)),'m','LineWidth',2)
title('Bound on multiplicative uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
  'Location','SouthWest');

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

% %% Ricostruzione dello stato con Kalman
% 
% disp('Ricostruzione dello stato con Kalman');
% disp('Press any key to continue.');
% pause();
% 
% %si ricorda che delta zita0=(A-VC)*zita0 +(B-VD)u + sommatoria (M-VN)*d +V*y
% alphaK = 100;
% Q = eye(size(AMin));
% W = eye(size(AMin));
% R = eye(size(ClocalMin,1));
% %disp('matrice  V per Kalman:');
% V=lqr((AMin+alphaK*eye(size(AMin)))',ClocalMin',Q,R)';
% %disp('Dimensione attesa [nxq]');
% disp(size(V));
% Aoss=AMin-V*ClocalMin;
% Bossw=[BMinw-V*D V]; % perche ho u,y,d come ingressi, si noti che B-vD ha dim di B ma anche V ha dim di B
% Coss=eye(size(AMin));
% Doss=zeros(size(Bossw));
% 
% disp('Autovalori A-V*C');
% disp(eig(AMin-V*ClocalMin));
% pause();
% clc;
% 
% %% Stabilizzazione LQR
% 
% disp('Stabilizzazione mediante LQR dallo stato stimato');
% disp('Press any key to continue.');
% pause();
% 
% alphaK = 2;
% QieCmp = blkdiag([0.00001 0; 0 0.00001],100*eye(3),eye(3));
% Qie = blkdiag([0.01 0; 0 0.01],100000*eye(3),zeros(3,3));
% Q = eye(size(AMin));
% RieCmp = [1 0 0 0; 0 100000 0 0; 0 0 100000 0; 0 0 0 10000];
% R = eye(size(BMinw,2));
% K = lqr(AMin,BMinw,Q,R);
% disp('Autovalori del sys a ciclo chiuso ottenuto per retroazione dallo stato:');
% eig(AMin-BMinw*K)
% 
% disp('Premere un tasto per visualizzare la Step Response...');
% pause;
% 
% tenzoLQR=ss(AMin-BMinw*K,BMinw,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
% step(tenzoLQR);
% 
% %% Verifica condizioni Astatismo
% 
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
% 
% %% Calcolo del modello interno
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
% 
% disp('End');