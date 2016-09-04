%% Plot performance of the angular position estimation
clc
clear all

%% load data
%load('../errorTenzo')
load('../estCtrl55PowerDynamic')

% First 100 data are the encoder values
enc = combo(1:100);

% the other are the estimated values
est = combo(101:200);

% Apply a median filter window size 3
estF3 = medfilt1(est,3);

% Apply a median fiter windows size 4
estF4 = medfilt1(est,4);

%  3 Hz acquisition
t=linspace(10,100,91);
figure
title('Roll angle estimation')
xlabel('samples @ 3Hz ')
ylabel('Angle °')
plot(t,enc(10:100),'b',t,est(10:100),'r',t,estF3(10:100),'c',t,estF4(10:100),'b')
legend('encoder','Complementary Filter','CF + Median(3)','CF + Median(4)')
grid on

%% Compute Root and Min Square Error
disp('Root Mean Square Error: Complementary Filter')
errEst = sqrt(immse(enc(10:100),est(10:100)))
% The same as above
%err = enc(10:100)- est(10:100);
%err2 = sqrt(mean((err.^2)))

disp('Root Mean Square Error: Complementary Filter + Median Filter 3 sized windows')
errEstM3 = sqrt(immse(enc(10:100),estF3(10:100)))
disp('Root Mean Square Error: Complementary Filter + Median Filter 4 sized windows')
errEstM4 = sqrt(immse(enc(10:100),estF4(10:100)))
