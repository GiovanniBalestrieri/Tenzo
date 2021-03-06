%% UserK 14-01-2017
% Paola got married 
clc
clear all
plotDiff = false;

%% Identification Data
dir .
load('samples.mat')

%% PreProcess

%correct data
% z(1:69)replace with zero
z(1:70) = zeros(70,1)

% rename data
time = x(62:end);
time = time*10^(-3);
input = y(62:end);
output = z(62:end);


timeOriginal = time(:);
inputOriginal = input(:);
outputOriginal = output(:);

disp('Number of Samples');
disp(size(time,1))

iter = ceil(size(time,1)*0.2);

tIde = time(200:iter);
uIde = input(200:iter);
yIde = output(200:iter);

tVal = time(iter:end);
uVal = input(iter:end);
yVal = output(iter:end);

figure(10)
subplot(2,1,1);
plot(time,input,'r');
title('Input - pwm [us]');
subplot(2,1,2);
plot(time,output,'r');
title('Output - Shaft angular velocity [rev/s]');

figure(1)
subplot(2,1,1);
plot(tIde,uIde,'r');
title('Input - pwm [us]');
subplot(2,1,2);
plot(tIde,yIde,'r');
title('Output - Shaft angular velocity [rev/s]');

% %% Designs a second order filter using a butterworth design guidelines
% [b a] = butter(2,0.045,'low');
% 
% % Plot the frequency response (normalized frequency)
% figure(13)
% z_filtered = filter(b,a,yIdeDetrend);
% plot(xIde,z_filtered,'r');

%% Diff
dt = tIde(2:end) - tIde(1:end-1);
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

z = iddata(output,input,Ts);

[ztDetrend, Ttest ]= detrend(zt,0);
[zvDetrend, Tvalidation ]= detrend(zv,0);
[zDetrend, T ]= detrend(z,0);

delay = delayest(zvDetrend)
% estimated delay changes as a function of the model 2nd order
delay4 = delayest(zvDetrend,2,2)

%% 

V = arxstruc(ztDetrend,zvDetrend,struc(2,2,1:10));
[nn,Vm] = selstruc(V,0);

FIRModel = impulseest(ztDetrend);
%clf
%h = impulseplot(FIRModel);
%showConfidence(h,3)
 
V = arxstruc(ztDetrend,zvDetrend,struc(1:10,1:10,delay));
nns = selstruc(V)

%% Identifiy linear discrete time model with arx

th2 = arx(ztDetrend,nns);
th4 = arx(ztDetrend,nn);
compare(zvDetrend(1:end),th2,th4);
%% ARX results
compare(zDetrend(1:end),th2,th4);
grid on

[motor_num_tf_arx , motor_den_tf_arx] = tfdata(th2,'v')
th5 = tf(motor_num_tf_discrete , motor_den_tf_discrete,0.009)
compare(zDetrend(1:end),th5,th2)
th5ss = canon(th5,'modal')
th5ss2 = reduce(th5ss,2)
th5ss2tf = tf(th5ss2)
th5ss1 = reduce(th5ss,1)
th5ss1tf = tf(th5ss1)
th5ss3 = reduce(th5ss,3)
th5ss3tf = tf(th5ss3)
th5ss4 = reduce(th5ss,4)
th5ss4tf = tf(th5ss4)

figure(21)
compare(zDetrend(1:end),th5,th5ss1,th5ss2,th5ss4)

figure(22)
bode(th5,'r',th5ss2,'b',th5ss1,'c',th5ss4,'k')
legend('6th order','reduction 2nd','reduction 1st','red 4th')
%% Identifiy linear discrete time model with n4sid

% Training
[m, x0m] = n4sid(ztDetrend,2,'InputDelay',delay,'Ts',Ts);
[ms,x0] = n4sid(ztDetrend,1:10,'InputDelay',delay,'Ts',Ts);

% Validation

[mv, x0mv] = n4sid(zvDetrend,2,'InputDelay',delay,'Ts',Ts);
[msv,x0v] = n4sid(zvDetrend,1:10,'InputDelay',delay,'Ts',Ts);


% Complete

%[mc, x0mc] = n4sid(zDetrend,2,'InputDelay',delay,'Ts',Ts);
[msc,x0c] = n4sid(zDetrend,1:10,'InputDelay',delay,'Ts',Ts);
%% Compare n4Sid trained with TestSet

disp('Comparing ms and arx')
compare(zvDetrend,ms)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,ms)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,ms)
%% Compare n4Sid trained with validation set

disp('Comparing ms and arx')
compare(zvDetrend,msv)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,msv)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,msv)
%% Compare n4Sid trained with Full set

disp('Comparing ms and arx')
compare(zvDetrend,msc)
% Test set
disp('Comparing ms1 and ms')
compare(ztDetrend,msc)
% Complete set
disp('Comparing th2 and ms')
compare(zDetrend,msc)

%% Manual compare

t = 0:Ts:Ts*(size(inputOriginal,1)-1);
size(t)
size(inputOriginal)

tf(msv)
[motor_num_tf_discrete , motor_den_tf_discrete] = tfdata(msv,'v')
tfM = tf(motor_num_tf_discrete*1,motor_den_tf_discrete,0.009)

y= lsim(tfM,inputOriginal,t);
%y= lsim(msv,inputOriginal,t);

plot(y)
% Create iddata Time domain signal
simulatedDData = iddata(y,inputOriginal,Ts)
simulatedDDataDetrend = retrend(simulatedDData, T)

% Plot results

figure(10)
plot(simulatedDData,'c')
grid on
hold on
plot(z,'r-')
tf(msv)


legend('Simulated','measured')
% Problem encountered real and simulated outputs are differents. Delta 43,2

save('discreteMotortf.mat','mv')
%%
figure(11)
plot(simulatedDData,'c')
grid on
hold on

plot(t,outputOriginal,'r');

hold on
plot(zDetrend,'g')
legend('Simulated','measured','simulated detrend')

