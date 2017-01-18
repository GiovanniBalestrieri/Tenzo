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

disp('Number of Samples');
disp(size(x,1))

iter = ceil(size(x,1)*0.2);

xIde = x(4:iter);
uIde = y(4:iter);
yIde = z(4:iter);

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

%figure(2)
%plot(x,z,'r');


%% Detrend

% Removes the mean value from data
yIdeDetrend = detrend(yIde,'constant');
uIdeDetrend = detrend(uIde,'constant');

figure(2)
subplot(2,1,1);
% show results
title('pwm [us]');
plot(xIde,yIdeDetrend,'r');
subplot(2,1,2);
title('Shaft angular velocity [rad/s]');
plot(xIde,uIdeDetrend,'r');



% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.045,'low');

% Plot the frequency response (normalized frequency)
figure(13)
z_filtered = filter(b,a,yIdeDetrend);
plot(xIde,z_filtered,'r');

%% Diff
dt = xIde(2:end) - xIde(1:end-1);
dy = yIde(2:end) - yIde(1:end-1);
du = uIde(2:end) - uIde(1:end-1);

if (plotDiff) 
    figure(3)
    subplot(2,1,1);
    % show results
    title('dy/dt [rad/s^2]');
    plot(dy./dt,'r');
    subplot(2,1,2);
    title('du/dt [us/s]');
    plot(du./dt,'r');
end

disp('Ts from data');
mean(dt(1))
Ts = dt(1);

% Create Id data
% zt  Training set
zt = iddata(yIde,uIde,Ts);

% zr Validation set
zv = iddata(yVal,uVal,Ts);
plot(zt)

delay = delayest(zv)
delay = delayest(zv,4,4)

%% 

V = arxstruc(zt,zv,struc(2,2,1:10));
[nn,Vm] = selstruc(V,0);

FIRModel = impulseest(zt);
clf
h = impulseplot(FIRModel);
showConfidence(h,3)

 m = n4sid(zt,1:10);
 
V = arxstruc(zt,zv,struc(1:10,1:10,delay));

nns = selstruc(V)

%% Test arx e n4sid

th2 = arx(zt,[10 7 delay]);
[ th4] = arx(zt,[2 2 delay]);
compare(zt(1:end),th2,th4);

pause()
[ms,x0] = n4sid(zt,3,'InputDelay',delay);
disp('Comparing ms and arx')
compare(zt,ms,th2)
disp('Comparing ms1 and ms')
compare(zt,ms,ms1)
%% Manual compare

t = 0:Ts:Ts*size(z,1)-1;
lsim(th2,z,t,x0)
hold
plot(t,y,'r')
