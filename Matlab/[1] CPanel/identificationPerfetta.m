clc

%% Load data

%a = load('ideTest1.mat')
a = load('errorTenzo.mat')

% Test Set 1
inizio = 1702
fine = 1820


% Test Set 2
inizio = 1552
fine = 1731


% Test 
inizio = 1552
fine = 1755



%% Prepare Data

timeTenzo = a.IdeDataTimeTenzo;
timeTenzoSec = (timeTenzo-1000)*0.001;
timeVitruvio = a.IdeDataTimeVitruvio;
yIde = a.IdeDataRoll;
yIdeEst = a.IdeDataEstRoll;
uDelta = a.IdeDataDelta;
uBase = 1420;
uIdeM1 = uDelta + uBase;
uIdeM2 = - uDelta + uBase;
%% Plot Data

figure(13)
subplot(2,1,1)
plot(timeTenzoSec(inizio:fine),a.IdeDataRoll(inizio:fine),'r');
grid on
title('Output - Roll [°]');
subplot(2,1,2)
plot(timeTenzoSec(inizio:fine),a.IdeDataDelta(inizio:fine));
grid on
title('Input - duty cycle difference between M1 and M2 [us]');

figure(43)
subplot(2,1,1)
plot(a.IdeDataRoll(inizio:fine),'r');
grid on
title('Output - Roll [°]');
subplot(2,1,2)
plot(a.IdeDataDelta(inizio:fine));
grid on
title('Input - duty cycle difference between M1 and M2 [us]');


figure(44)
subplot(2,1,1)
plot(a.IdeDataRoll,'r');
grid on
title('Output - Roll [°]');
subplot(2,1,2)
plot(a.IdeDataDelta(inizio:fine));
grid on
title('Input - duty cycle difference between M1 and M2 [us]');



m1 = a.IdeDataDelta;
m2 = -a.IdeDataDelta;

% figure(15)
% plot(timeTenzoSec,uIdeM1,'r');
% hold on 
% plot(timeTenzoSec,uIdeM2,'b');



%% Discrete Transfer function of motors
clear tf
Ts_motor = 0.009 ; %ms
%H = tf(0.0001195,[1 -0.9368],Ts_motor,'InputDelay',8)

% Prepare simulated output fitting 85%

% Motor dynamics tf1 
H = load('estimation/motor87perc.mat')
Hd1 = c2d(H.tf1,Ts_motor)

% Motor dynamics with n4sid

H = load('estimation/discreteMotortf.mat')
H = H.mv
% figure(21)
% step(Hd1,'b');
% hold on
% step(H,'r');

% Simulate motor output
t = 0:Ts_motor:Ts_motor*(size(uIdeM1,1)-1);

[yM1,t,x] = lsim(Hd1,uIdeM1',t,0);
[yM2,t,x] = lsim(Hd1,uIdeM2',t,0);
[yM3,t,x] = lsim(H,uIdeM1',t);
[yM4,t,x] = lsim(H,uIdeM2',t);

figure(22)
hold on
subplot(2,1,1)
plot(t,yM3,'r');
hold on
plot(t,yM4,'b');
grid on
title('Output - Shaft angular velocity [rev/s]');
subplot(2,1,2)
plot(t,uIdeM1,'b');
hold on 
plot(t,uIdeM2,'r');
title('Input - pwm [us]');
grid on
 


%% Convert to torque input signal

% Computing Thrust force. It is the result of vertical forces acting on all
% blade elements of one propeller
Radius = 0.115 % m
Radius_in = 9 % in
Ct = 0.18
rho = 1.225 % kg/m^3
Aprop = pi*Radius^2

%MOTOR 3
% Convert to RPM
yM3_rpm= yM3*60
Thrust_newton_3 = Ct*rho*Aprop*(yM3).^2*Radius^2;
Thrust_kg_3 = Thrust_newton_3/9.81


%MOTOR 4
% Convert to RPM
yM4_rpm= yM4*60
Thrust_newton_4 = Ct*rho*Aprop*(yM4).^2*Radius^2;
Thrust_kg_4 = Thrust_newton_4/9.81

figure(20)
title('Thrust in kg')
plot(Thrust_kg_4,'r')
hold on 
plot(Thrust_kg_3,'b')
grid on
legend('Motor3','Motor4')


figure(21)
title('Thrust in Newton')
plot(Thrust_newton_4,'r')
hold on 
plot(Thrust_newton_3,'b')
grid on
legend('Motor3','Motor4')

% Computing Torque
arm = 0.23 %m
%Tau1 = arm*(Thrust_newton_4-Thrust_newton_3)

% Oppure
TauDynamic = arm*Ct*rho*Aprop*Radius^2*((yM4).^2 - (yM3).^2)
TauRaw = arm*Ct*rho*Aprop*Radius^2*((uIdeM2).^2 - (uIdeM1).^2)

figure(23)
plot(timeTenzoSec(inizio:fine),TauDynamic(inizio:fine),'r');
%hold on
%plot(a.IdeDataTimeTenzo(inizio:fine),TauRaw(inizio:fine),'b');
grid on
title('Output - Torque with motor Dynamic included [nM]');

figure(24)
plot(timeTenzoSec(inizio:fine),TauRaw(inizio:fine),'b');
grid on
title('Output - Torque with motor Dynamic Neglected [nM]');


%% Identification
% Create time series Y-U data

% Computing deltaT for Tenzo and Vitruviano
dTtenzo = timeTenzoSec(inizio+1:fine+1) - timeTenzoSec(inizio:fine);
dtt = mean(dTtenzo)
Ts = dtt;
%Ts = 0.019
Ts = 0.021

TauMotorDynamic = TauDynamic(inizio:fine);
Theta = a.IdeDataRoll(inizio:fine);

% Validation Set
TauMotorRaw = TauRaw(inizio:fine);

IOdynamic = iddata(Theta,TauMotorDynamic,Ts);
IOraw = iddata(Theta,TauMotorRaw,Ts);


[IOtestDetrend, Tiotest ]= detrend(IOdynamic,1);
[IOvalDetrend, Tioval ]= detrend(IOraw,1);

delay = delayest(IOvalDetrend)
delay1 = delay;
delay2 = delayest(IOtestDetrend)

%% 

% V = arxstruc(IOtestDetrend,IOvalDetrend,struc(2,2,1:10));
% [nn,Vm] = selstruc(V,0);
% 
% FIRModel = impulseest(IOtestDetrend);
% clf
% h = impulseplot(FIRModel);
% showConfidence(h,3)
% %  
% V = arxstruc(IOtestDetrend,IOvalDetrend,struc(1:10,1:10,delay));
% nns = selstruc(V)
% 
% %% Identifiy linear discrete time model with arx
% % 
% th2 = arx(IOtestDetrend,nns);
% [ th4] = arx(IOtestDetrend,nns);
% compare(IOvalDetrend(1:end),th2,th4);
% compare(IOtestDetrend(1:end),th2,th4);
% compare(IOcompleteDetrend(1:end),th2,th4);

%% Identifiy linear discrete time model with n4sid

% Training
[mt, x0t] = n4sid(IOtestDetrend,2,'InputDelay',2,'Ts',Ts);
[mts,x0ts] = n4sid(IOtestDetrend,1:10,'InputDelay',8,'Ts',Ts);


% Validation
[mv, x0v] = n4sid(IOvalDetrend,2,'InputDelay',delay1,'Ts',Ts);
[mvs,x0vs] = n4sid(IOvalDetrend,1:10,'InputDelay',delay1,'Ts',Ts);


%% Compare n4Sid trained with TestSettf()
% Dynamic set
figure
disp('Comparing ms1 and ms')
compare(IOtestDetrend,mts)
% Raw set
figure
disp('Comparing th2 and ms')
compare(IOvalDetrend,mv,mvs)
%% Save Identified Discrete Time System


save('discreteDynamicTenzo.mat','mts')



