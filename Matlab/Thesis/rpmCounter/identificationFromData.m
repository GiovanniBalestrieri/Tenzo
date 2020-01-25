%% UserK 14-01-2017
% Paola got married 
clc
clear all
plotDiff = false;

%% Identification Data
dir .
load('samples.mat')

x = x(2:end);
y = y(2:end);
z = z(2:end);


xOriginal = x(2:end);
yOriginal = y(2:end);
zOriginal = z(2:end);

disp('Number of Samples');
disp(size(x,1))

iter = ceil(size(x,1)*0.2);

xIde = x(200:iter);
uIde = y(200:iter);
yIde = z(200:iter);

xVal = x(iter:end);
uVal = y(iter:end);
yVal = z(iter:end);

figure(10)
subplot(2,1,1);
plot(x,y,'r');
title('pwm [us]');
subplot(2,1,2);
plot(x,z,'r');
title('Shaft angular velocity [rev/s]');

figure(1)
subplot(2,1,1);
plot(xIde,yIde,'r');
title('Shaft angular velocity [rev/s]');
subplot(2,1,2);
plot(xIde,uIde,'r');
title('pwm [us]');

% %% Designs a second order filter using a butterworth design guidelines
% [b a] = butter(2,0.045,'low');
% 
% % Plot the frequency response (normalized frequency)
% figure(13)
% z_filtered = filter(b,a,yIdeDetrend);
% plot(xIde,z_filtered,'r');

%% Diff
dt = xIde(2:end) - xIde(1:end-1);
dy = yIde(2:end) - yIde(1:end-1);
du = uIde(2:end) - uIde(1:end-1);
% 
% if (plotDiff) 
%     figure(3)
%     subplot(2,1,1);
%     % show results
%     title('dy/dt [rad/s^2]');
%     plot(dy./dt,'r');
%     subplot(2,1,2);
%     title('du/dt [us/s]');
%     plot(du./dt,'r');
% end

disp('Ts from data');
mean(dt(1))
Ts = dt(1);

% Create Id data
% zt  Training set
zt = iddata(yIde,uIde,Ts);
zv = iddata(yVal,uVal,Ts);

z = iddata(y,z,Ts);

[ztDetrend, Ttest ]= detrend(zt,0);
[zvDetrend, Tvalidation ]= detrend(zv,0);
[zDetrend, T ]= detrend(z,0);

delay = delayest(zvDetrend)
% estimated delay changes as a function of the model
delay4 = delayest(zvDetrend,4,4)

%% 

V = arxstruc(ztDetrend,zvDetrend,struc(2,2,1:10));
[nn,Vm] = selstruc(V,0);

FIRModel = impulseest(ztDetrend);
clf
h = impulseplot(FIRModel);
%showConfidence(h,3)
 
V = arxstruc(ztDetrend,zvDetrend,struc(1:10,1:10,delay));
nns = selstruc(V)

%% Identifiy linear discrete time model with arx

th2 = arx(ztDetrend,nns);
[ th4] = arx(ztDetrend,nns);
compare(zvDetrend(1:end),th2,th4);

%% Identifiy linear discrete time model with n4sid

% Training
[m, x0m] = n4sid(ztDetrend,2,'InputDelay',delay,'Ts',Ts);
[ms,x0] = n4sid(ztDetrend,1:10,'InputDelay',delay,'Ts',Ts);

% Validation

[mv, x0mv] = n4sid(zvDetrend,2,'InputDelay',delay,'Ts',Ts);
[msv,x0v] = n4sid(zvDetrend,1:10,'InputDelay',delay,'Ts',Ts);


% Complete

[mc, x0mc] = n4sid(zDetrend,2,'InputDelay',delay,'Ts',Ts);
[msc,x0c] = n4sid(zDetrend,1:10,'InputDelay',delay,'Ts',Ts);
%% Compare n4Sid trained with TestSet

disp('Comparing ms and arx')
compare(zvDetrend,ms,m)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,ms,m)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,ms,m)
%% Compare n4Sid trained with validation set

disp('Comparing ms and arx')
compare(zvDetrend,msv,mv)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,msv,mv)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,msv,mv)
%% Compare n4Sid trained with Full set

disp('Comparing ms and arx')
compare(zvDetrend,msc,mc)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,msc,mc)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,msc,mc)

plot(zDetrend)
plot(z)
%% Manual compare

t = 0:Ts:Ts*size(yOriginal,1)-1;
y= lsim(m,yOriginal,t);

t = 0:Ts:Ts*size(z,1)-1;
y= lsim(m,yOriginal,t);

% Create iddata Time domain signal
simulatedDData = iddata(yOriginal,y,Ts)
plot(simulatedDData)
%%
mTrend = retrend(simulatedDData,T)
%hold on
%plot(zDetrend,'g')
hold on
plot(z,'r-')
reT = retrend(zDetrend,T)
hold on
plot(reT,'b--')
%%
%plot(t,zOriginal,'r');
hold on
plot(t,mTrend.u,'b');
