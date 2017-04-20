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
plot(a.IdeDataRoll(inizio:fine),'r');
grid on
title('Output - Roll [°]');
subplot(2,1,2)
plot(a.IdeDataDelta(inizio:fine));
grid on
title('Input - duty cycle difference between M1 and M2 [us]');


%% Identification
% Create time series Y-U data

Ts = 0.021
IOraw = iddata(a.IdeDataRoll(inizio:fine),a.IdeDataDelta(inizio:fine),Ts);

[IOrawDetrend, Tioraw ]= detrend(IOraw);
plot(IOrawDetrend)

% Computing deltaT for Tenzo and 

delay = delayest(IOrawDetrend)
delay1 = delay;


%% Identifiy linear discrete time model with n4sid

% Training
[mt, x0t] = n4sid(IOrawDetrend,1:10,'InputDelay',delay,'Ts',Ts);
[mts,x0ts] = n4sid(IOrawDetrend,1:10,'Ts',Ts);

%% Compare n4Sid trained with TestSettf()
% Dynamic set
figure
disp('Comparing ms1 and ms')
compare(IOrawDetrend,mts,mt)
%% Save Identified Discrete Time System

rollDynamic76 = d2c(mts)
rollDynamic68 = d2c(mt)
save('DynamicTenzo.mat','mts')
save('Results/Roll76.mat','rollDynamic76')
save('Results/Roll68.mat','rollDynamic68')

%% Forma canonica di osservatore
roll76 = ss(rollDynamic76)

[Abar,Bbar,Cbar,T,k] = obsvf(roll76.A,roll76.B,roll76.C)

rollX = ss(Abar,Bbar,Cbar,roll76.D)

step(roll76,'r',rollX,'b')
