clc

%% Load data

%a = load('ideTest1.mat')
a = load('errorTenzo.mat')




%% Prepare Data

timeTenzo = a.IdeDataTimeTenzo;
timeTenzoSec = timeTenzo*0.001;
timeVitruvio = a.IdeDataTimeVitruvio;
yIde = a.IdeDataRoll;
yIdeEst = a.IdeDataEstRoll;
uDelta = a.IdeDataDelta;
uBase = a.IdeDataThrottle;
uIdeM1 = uDelta + uBase;
uIdeM2 = - uDelta + uBase;
%% Plot Data

figure(11)
plot(a.IdeDataTimeTenzo,a.IdeDataEstRoll);

figure(13)
plot(a.IdeDataTimeVitruvio,a.IdeDataRoll);


figure(14)
plot(a.IdeDataRoll);

figure(12)
plot(a.IdeDataTimeTenzo,a.IdeDataDelta);

m1 = a.IdeDataDelta;
m2 = -a.IdeDataDelta;
plot(m1,'b')
hold on
plot(m2,'r')
diff = m1-m2
figure(30)
plot(diff)

figure(15)
plot(uIdeM1,'r');
hold on 
plot(uIdeM2,'b');

%% Discrete Transfer function of motors
clear tf
Ts_motor = 0.009 ; %ms
%H = tf(0.0001195,[1 -0.9368],Ts_motor,'InputDelay',8)

%% Prepare simulated output fitting 85%

% Motor dynamics tf1 
H = load('estimation/motor87perc.mat')
Hd1 = c2d(H.tf1,Ts_motor)

% Motor dynamics with n4sid
H = load('estimation/discreteMotortf.mat')
H = H.mv
step(Hd1,'b');
hold on
step(H,'r');

% Simulate motor output
t = 0:Ts_motor:Ts_motor*(size(uIdeM1,1)-1);

[yM1,t,x] = lsim(Hd1,uIdeM1',t,0);
[yM2,t,x] = lsim(Hd1,uIdeM2',t,0);
[yM3,t,x] = lsim(H,uIdeM1',t);
[yM4,t,x] = lsim(H,uIdeM2',t);
hold on
subplot(2,1,1)
%plot(t,yM1,'b');
%hold on
%plot(t,yM2,'r');
%hold on
plot(t,yM3,'r');
hold on
plot(t,yM4,'b');
subplot(2,1,2)
plot(t,uIdeM1,'b');
hold on 
plot(t,uIdeM2,'r');


figure(16)
subplot(2,1,1)
plot(t,yM3,'r');
grid on
title('Output - Shaft angular velocity [rev/s]');
subplot(2,1,2)
plot(t,uIdeM1,'b');
grid on
title('Input - pwm [us]');

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
Tau1 = arm*(Thrust_newton_4-Thrust_newton_3)

% Oppure
Tau = arm*Ct*rho*Aprop*Radius^2*((yM4).^2 - (yM3).^2)


figure(23)
plot(Tau1-Tau)


%% Identification
% Computing deltaT for Tenzo and Vitruviano

% Diff
plotDiff = 0;
dTtenzo = timeTenzo(2:end) - timeTenzo(1:end-1);
dTvitruviano = timeVitruvio(2:end) - timeVitruvio(1:end-1);

if (plotDiff) 
%     figure(3)
%     subplot(2,1,1);
%     % show results
%     title('dy/dt [rad/s^2]');
%     plot(dy./dt,'r');
%     subplot(2,1,2);
%     title('du/dt [us/s]');
%     plot(du./dt,'r');
end

disp('Ts from data')
dtt = mean(dTtenzo)
dtv = mean(dTvitruviano)
Ts = dtt;

%% Create time series Y-U data
sTauTest = size(Tau,1)
sThetaTest = size(a.IdeDataRoll,1)

sampleTest = sTauTest*0.3
sampleVal = sTauTest*0.7

% Test Set
TauTest = Tau(1:sampleTest)
ThetaTest = a.IdeDataRoll(1:sampleTest)

% Validation Set
TauVal = Tau(sampleTest+1:sampleVal)
ThetaVal = a.IdeDataRoll(sampleTest+1:sampleVal)

IOtest = iddata(ThetaTest,TauTest,Ts);
IOval = iddata(ThetaVal,TauVal,Ts);
IOcomplete = iddata(a.IdeDataRoll,Tau,Ts);

[IOcompleteDetrend, Tio ]= detrend(IOcomplete,0);
[IOtestDetrend, Tiotest ]= detrend(IOtest,0);
[IOvalDetrend, Tioval ]= detrend(IOval,0);

delay1 = delayest(IOvalDetrend)

delay2 = delayest(IOtestDetrend)

delay3 = delayest(IOcompleteDetrend)



%% 

V = arxstruc(IOtestDetrend,IOvalDetrend,struc(2,2,1:10));
[nn,Vm] = selstruc(V,0);

FIRModel = impulseest(IOtestDetrend);
clf
h = impulseplot(FIRModel);
showConfidence(h,3)
 
V = arxstruc(IOtestDetrend,IOvalDetrend,struc(1:10,1:10,delay));
nns = selstruc(V)

%% Identifiy linear discrete time model with arx

th2 = arx(IOtestDetrend,nns);
[ th4] = arx(IOtestDetrend,nns);
compare(IOvalDetrend(1:end),th2,th4);
compare(IOtestDetrend(1:end),th2,th4);
compare(IOcompleteDetrend(1:end),th2,th4);

%% Identifiy linear discrete time model with n4sid

% Training
[mt, x0t] = n4sid(IOtestDetrend,2,'InputDelay',delay1,'Ts',Ts);
[mts,x0ts] = n4sid(IOtestDetrend,1:10,'InputDelay',delay1,'Ts',Ts);


% Validation
[mv, x0v] = n4sid(IOvalDetrend,2,'InputDelay',delay1,'Ts',Ts);
[mvs,x0vs] = n4sid(IOvalDetrend,1:10,'InputDelay',delay1,'Ts',Ts);


% Complete
[mc, x0c] = n4sid(IOcompleteDetrend,2,'InputDelay',delay1,'Ts',Ts);
[mcs,x0cs] = n4sid(IOcompleteDetrend,1:10,'InputDelay',delay1,'Ts',Ts);


%% Compare n4Sid trained with TestSet

figure
disp('Comparing ms and arx')
compare(IOtestDetrend,mts,mt)
% Test set
figure
disp('Comparing ms1 and ms')
compare(IOvalDetrend,mts,mt)
% Complete set
figure
disp('Comparing th2 and ms')
compare(IOcompleteDetrend,mts,mt)
%% Compare n4Sid trained with validation set

figure
disp('Comparing ms and arx')
compare(IOtestDetrend,mv,mvs)
% Test set
figure
disp('Comparing ms1 and ms')
compare(IOvalDetrend,mv,mvs)
% Complete set
figure
disp('Comparing th2 and ms')
compare(IOcompleteDetrend,mv,mvs)
%% Compare n4Sid trained with Full set

figure
disp('Comparing ms and arx')
compare(IOtestDetrend,mcs)
% Test set
figure
disp('Comparing ms1 and ms')
compare(IOvalDetrend,mcs)
% Complete set
figure
disp('Comparing th2 and ms')
compare(IOcompleteDetrend,mcs)

%% Manual compare
% 
% t = 0:Ts:Ts*size(yOriginal,1)-1;
% [y,t,x] = lsim(ms,yOriginal,t,x0);
% hold on
% plot(t,zOriginal,'r');
% hold on
% plot(t,y,'b');

