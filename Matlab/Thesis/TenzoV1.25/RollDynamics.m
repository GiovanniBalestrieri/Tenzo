%% Roll Dynamics 22 mar 2017
% Giovanni Balestrieri 
% Info @ userk.co.uk
clc
clear all
 

Whovering = 750;
Ts = 0.021;

%% Loading identified motor dynamics
motorDynamics = load('discreteMotortf.mat')
motorDynamics = motorDynamics.mv



disp('Evaluating step response for motor dynamics. Press X');
%pause()
opt = stepDataOptions('InputOffset',0,'StepAmplitude',750);
step(motorDynamics,opt)
motorDynamics = d2d(motorDynamics,0.021)

% get numerator and denominator Roll
[motor_num_tf_discrete , motor_den_tf_discrete] = tfdata(motorDynamics,'v')

% Computing observator canonical form
motorC = canon(motorDynamics,'companion');
motorC1 = motorC;
motorC2 = motorC;

%% Loading identified Roll dynamics

rollDynamics = load('discreteDynamicTenzo.mat');
rollDynamics = rollDynamics.mts

disp('Evaluating step response for roll dynamics. Press X');
%pause()
step(rollDynamics)

% Computing observator canonical form
rollC = canon(rollDynamics,'companion');

%% Computing linearized 
% Computing Thrust force. It is the result of vertical forces acting on all
% blade elements of one propeller
Radius = 0.115; % m
Radius_in = 9; % in
Ct = 0.18;
rho = 1.225; % kg/m^3
Aprop = pi*Radius^2;

% Convert to RPM
Kforce = Ct*rho*Aprop*2*Whovering*Radius^2;
%Thrust_newton = rpm*Kforce;
%Thrust_kg = Thrust_newton/9.81



% get numerator and denominator Roll
[roll_num_tf_discrete , roll_den_tf_discrete] = tfdata(rollDynamics,'v');







%% Constant definition and simulation

% initial value of error derivative
de = 0;
em1 = 0;
thresholdError = 5;

% Affine constants
k = 10;
M = 200;

% Bounding box
Me = 15;
Mde = 70;

% iperbole
lambdaErr = 0.00000001;


armLength = 0.23;

%Saturation
pwmUpperBound = 1700;
pwmLowerBound = 1100;

dmUpperBound = 300;
dmLowerBound = -300;

rpmUpperBound = 90;
rpmLowerBound = 0;

% saturations 
enablePwmSaturation = -1;
enableRpmSaturation = 1;

% Measurement Error
enableMisErr = 1;

% Output Perturbation
enableOutputPert = -1;


% nonlinear
mode = -1;

%open('testRollContSolo');
open('rollDynamicsNonLinear');
sim('rollDynamicsNonLinear');

%% Plot switching criteria

time = -5:0.1:5;
f = 0.00000001./time;
f1 = -f;
plot(f,time)
hold on 
plot(f1,time)
grid on
title("Switching function")
xlabel("error")
ylabel("d error")

%% Valuta Err e DErr
size(derErr.Data);
plot3(derErr.Data(120:end-2),Err.Data(120:end-2),Err.Time(120:end-2),'--gs',...
'MarkerSize',10,...
    'MarkerEdgeColor','b')
xlabel('Error derivative')
ylabel('Error ')
zlabel('Time')
title('de Vs e vs T')
grid on

%% LQ regulator
% linRollSys = open('rollOpenLoopLin.mat')
% % stable system
% roll = linRollSys.openLoop;
% roll = minreal(roll)
% rank(ctrb(roll.a,roll.b))
% nRoll = size(roll.a,1)
% pRoll = size(roll.b,2)
% qRoll = size(roll.c,1)
% eig(roll)
% step(roll);
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% %Calcolo retroazione dallo stato
% disp('Uso la funzione lqr per calcolare il guadagno K (la retroazione dallo stato u=Kx)')
% Q=eye(nRoll)
% R=eye(pRoll)
% K = dlqr(roll.a,roll.b,Q,R);
% disp('Matrice di guadagno K: [comando K = dlqr(A,B,Q,R)]');
% disp(K);
% disp('Autovalori a ciclo chiuso: [comando eig(A-B*K)]');
% eK = eig(roll.a-roll.b*K);
% disp(eK);
% 
% disp('Premere un t1asto per continuare...')
% pause;
% 
% %% Calcolo di un compensatore stabilizzante (2/3)
% clc;
% %Calcolo guadagno del filtro di Kalman
% disp('Uso la funzione lqr per calcolare il guadagno L del filtro di Kalman')
% W=eye(nRoll)
% V=eye(qRoll)
% L= dlqr(roll.a',roll.c',W,V)';
% disp('Matrice di guadagno L: [comando L = lqr(A'',C'',W,V)'']');
% disp(L);
% disp('Autovalori del filtro di Kalman: [comando eig(A-L*C)]');
% eL = eig(roll.a-L*roll.c);
% disp(eL);
% 
% disp('Premere un tasto per continuare...')
% pause;
% 
% %% Calcolo di un compensatore stabilizzante (3/3)
% clc;
% %Costruisco il regolatore
% disp('Costruisco il regolatore come filtro di Kalman + retroazione dallo stato stimato')
% Ac = roll.a-roll.b*K-L*roll.c+L*roll.d*K;
% Bc = L;
% Cc = -K;
% Dc = zeros(size(K,1),size(L,2));
% disp('Ac = A-B*K-L*C+L*D*K');
% disp(Ac);
% disp('Bc = L, Cc = -K, Dc = zeros(size(K,1),size(L,2))')
% disp('Autovalori a ciclo chiuso:');
% controllore = ss(Ac,Bc,Cc,Dc,Ts);
% CicloChiuso = feedback(series(controllore, roll),eye(pRoll), 1); %Retroazione positiva
% [Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
% eCl = eig(Acl);
% disp(eCl);
% 
% Aoss=roll.a-V*roll.c;
% Bossw=[roll.b-V*roll.d V]; % perche ho u,y,d come ingressi, si noti che B-vD ha dim di B ma anche V ha dim di B
% Coss=eye(size(AMin));
% Doss=zeros(size(Bossw));
% 
% 
% disp('La figura mostra la risposta al gradino del sistema di controllo');
% 
% step(CicloChiuso);
% disp('Premere un tasto per continuare...')
% pause;

%% Provo una sintesi piu' veloce modificando i pesi
% clc;
% disp('Cambio i pesi per rendere piu'' rapida la convergenza')
% disp('riducendo il peso relativo di R=V rispetto a Q=W')
% Q=eye(8)
% R=0.001*eye(4)
% W=eye(8);
% V=0.001*eye(4);
% K = lqr(tenzo_min_nominale.a,tenzo_min_nominale.b,Q,R);
% L= lqr(tenzo_min_nominale.a',tenzo_min_nominale.c',W,V)';
% disp('Matrici di guadagno K ed L:');
% disp(K);
% disp(L);
% eK1 = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*K);
% eL1 = eig(tenzo_min_nominale.a-L*tenzo_min_nominale.c);
% disp('Autovalori a ciclo chiuso con le due sintesi:');
% disp('   K                 K1                 L                 L1:');
% disp([eK eK1 eL eL1]);
% 
% Ac = tenzo_min_nominale.a-tenzo_min_nominale.b*K-L*tenzo_min_nominale.c+L*tenzo_min_nominale.d*K;
% Bc = L;
% Cc = -K;
% Dc = zeros(size(K,1),size(L,2));
% 
% sistema = ss(tenzo_min_nominale.a,tenzo_min_nominale.b,tenzo_min_nominale.c,tenzo_min_nominale.d);
% controllore1 = ss(Ac,Bc,Cc,Dc);
% CicloChiuso1 = feedback(series(controllore1, sistema),eye(4), 1); %Retroazione positiva
% % [Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
% % eCl = eig(Acl);
% % disp(eCl);
% 
% disp('La figura mostra la risposta al gradino del nuovo sistema di controllo');
% 
% hold on;
% step(CicloChiuso1);
% 
% disp('Premere un tasto per continuare...')
% pause;
% 
% %% Motivazione della traslazione
% 
% clc;
% disp('La seconda sintesi ha reso la convergenza piu'' rapida ma ha ');
% disp('modificato solo di poco gli autovalori a ciclo chiuso piu'' lenti!');
% disp(' ');
% disp('Parte reale degli autovalori a ciclo chiuso con le due sintesi:');
% disp('   K        K1         L         L1:');
% disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eL),'descend') sort(real(eL1),'descend')]);
% 
% disp('Per rendere tutti gli autovalori a ciclo chiuso piu'' rapidi');
% disp('(uniformemente), posso usare la traslazione');
% disp('[comando K=lqr(A+alfa*I,B,Q,R) con alfa > 0]');
% disp('che garantisce che la parte reale di eig(A-B*K)<-alfa ');
% 
% disp(' ');
% disp(' ');
% disp('Premere un tasto per continuare...');
% pause();
% 
% %% Over the top
% clc;
% disp('Con i pesi originali, rendo piu'' rapida la convergenza')
% disp('traslando la A con una traslazione alfa=5')
% Q=eye(8)
% R=eye(4)
% W=eye(8);
% V=eye(4);
% alfa=3;
% K = lqr(tenzo_min_nominale.a+alfa*eye(size(tenzo_min_nominale.a)),tenzo_min_nominale.b,Q,R);
% L= lqr(tenzo_min_nominale.a'+alfa*eye(size(tenzo_min_nominale.a)),tenzo_min_nominale.c',W,V)';
% disp('Matrici di guadagno K ed L:');
% disp(K);
% disp(L);
% disp('Autovalori a ciclo chiuso:');
% eK2 = eig(tenzo_min_nominale.a-tenzo_min_nominale.b*K);
% eL2 = eig(tenzo_min_nominale.a-L*tenzo_min_nominale.c);
% disp('Parte reale degli autovalori a ciclo chiuso con le tre sintesi:');
% disp('   K        K1         K2        L         L1         L2:');
% disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eK2),'descend') sort(real(eL),'descend') sort(real(eL1),'descend') sort(real(eL2),'descend')]);
% 
% Ac = tenzo_min_nominale.a-tenzo_min_nominale.b*K-L*tenzo_min_nominale.c+L*tenzo_min_nominale.d*K;
% Bc = L;
% Cc = -K;
% Dc = zeros(size(K,1),size(L,2));
% 
% disp('Autovalori a ciclo chiuso:');
% 
% 
% controllore2 = ss(Ac,Bc,Cc,Dc);
% CicloChiuso2 = feedback(series(controllore2, sistema),eye(4), 1); 
% 
% step(CicloChiuso2);
% 
% figure(3);
% for i=1:N
%     sys{i} = usample(tenzo_min_unc);
%     CicloChiusoUnc{i} =  feedback(series(controllore2, sys{i}),eye(4), 1); 
%     step(CicloChiusoUnc{i});
%     pause();
%     hold on;
% end
% hold off;
% % eCl = eig(Acl);
% % disp(eCl);

















%% Computing augmentedsystem
% 
% Brall = rollC.b*armLength*Kforce
% Br_segn = Brall*[1 -1]
% 
% Br_segn11 = Br_segn(1,1)
% Br_segn12 = Br_segn(1,2)
% Br_segn21 = Br_segn(2,1)
% Br_segn22 = Br_segn(2,2)
% 
% Br_segn1 = Br_segn(1:2,1)
% Br_segn2 = Br_segn(1:2,2)
% 
% 
% states = {'x1m1','x2m1','x1m2','x2m2','xr1','xr2'}
% output = {'phi'}
% input= {'PwmMotor1','PwmMotor2'}
% 
% Acomplete = [ motorC.a zeros(3,4) ; zeros(3,2) motorC.a zeros(3,2); (Br_segn1)*motorC.c (Br_segn2)*motorC.c rollC.a ]
% 
% Bcomplete = [ motorC.b zeros(2,1) ; zeros(2,1) motorC.b; zeros(2,2) ]
% 
% Ccomplete = [ zeros(1,4) rollC.c ]
% 
% Dcomplete = [ zeros(1,2) ]
% 
% rollComplete = ss(Acomplete,Bcomplete,Ccomplete,Dcomplete,Ts,'statename',states,'inputname',input,'outputname',output)
% 
% step(rollComplete)
% 
% tfRollComplete = tf(rollComplete)
% tfRollComplete(1)
% % get numerator and denominator Roll
% [roll_c_num_1 , roll_c_den_1] = tfdata(tfRollComplete(1),'v');
% [roll_c_num_2 , roll_c_den_2] = tfdata(tfRollComplete(2),'v');
% 
% N = {roll_c_num_1;roll_c_den_1};   % Cell array for N(s)
% D = {roll_c_num_2;roll_c_den_2}; % Cell array for D(s)
% Hmimo = tf(N,D,Ts)
% 
% figure
% step(Hmimo)
% hold on
% step(tfRollComplete)
