% This script loads and plots data from a .mat file
clc
clear all
% gxFdata = 0;
% gyFdata = 0;
% gzFdata = 0;


% Loads File 1
dat = load('SNZ95UDUD.mat');
% Loads File 2
%dat = load('SNZ95Hor.mat');
% Loads File 3
%dat = load('W95UD.mat');
% Loads File 4
%dat = load('W95Hor.mat');
% Loads File 5
%dat = load('W85UD.mat');
% Loads File 6
%dat = load('W85Hor.mat');

%% Plots Raw and Filtered data

figure(10);
title('Acceleration along X axis');
xlabel('Time [us]');
ylabel('Amplitude [m*s^-2]');
plot(dat.time,dat.x);
hold on
%plot(dat.time,dat.y,'g','LineWidth',2);
grid on
grid minor

%% Plots raw data in time domain

figure(2);
title('Noisy acceleration along X axis');
xlabel('Samples [s]');
ylabel('Amplitude [m*s^-2]');
plot(dat.x);
hold on
plot(dat.y,'g');
grid on
grid minor

%% Spectral analysis

% Plots magnitude spectrum of the signal
X_mags=abs(fft(dat.x));
figure(3)
plot(X_mags);
grid on
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
figure(3)
num_bins = length(X_mags);
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
grid on
xlabel('Normalized frequency [\pi rads/samples]');
ylabel('Magnitude');

%% Filter Design 1

% Designs a third order butterworth filter 
% plots frequency response (normalized frequency)
[b a] = butter(3,0.04,'low');
H = freqz(b,a,floor(num_bins/2));
figure(4)
hold on
plot([0:1/(num_bins/2 -1):1], abs(H), 'r');

% Filters the signal 
x_filtered = filter(b,a,dat.x);

% Plots the filtered signal
figure(4)
plot(x_filtered,'r')
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');


%% Filter Design 2 

% Redesign the filter using a different cutting freq
[b2 a2] = butter(3,0.1,'low');
% Plots the magnitude spectrum and compare with lower order
H2 = freqz(b2,a2,floor(num_bins/2));
figure(5);
hold on
plot([0:1/(num_bins/2 -1):1], abs(H2), 'r');
%filter the noisy signal
x_filtered2 = filter(b2,a2,dat.x);
figure(5)
plot(x_filtered2,'g');
title('Filtered Signal - 3 Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

%% Filter Design 3 

% Redesign the filter using a different cutting freq
[b3 a3] = butter(20,0.2,'low');
% Plots the magnitude spectrum and compare with lower order
H2 = freqz(b2,a2,floor(num_bins/2));
figure(5);
hold on
plot([0:1/(num_bins/2 -1):1], abs(H2), 'r');
%filter the noisy signal
x_filtered3 = filter(b3,a3,dat.x);
figure(5)
plot(x_filtered2,'g');
title('Filtered Signal - 20 Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

%% Comparison between filters: Digital Arduino Vs Matlab

figure(7)
% Plots Raw acc data
plot(dat.time,dat.x);
hold on
% Arduino digital filter
plot(dat.time,dat.y,'k','LineWidth',2);
hold on
% Matlab filter
plot(dat.time,x_filtered,'r-','LineWidth',2);
hold on
% Plots cF = 0.10 Matlab filtered, order 3
plot(dat.time,x_filtered2,'g-','LineWidth',2);
hold on
% Plots cf = 0.3 Matlab filtered order 30
%plot(dat.time,x_filtered3,'m-','LineWidth',2);
legend('raw acceleration','first order low pass filter',' 2nd order Butterworth','3rd order Butterworth')
title('Filter Comparison');
xlabel('Samples [us]');
ylabel('Amplitude [m*s^-2]');
grid on
grid minor

%% Plots estimated angle

figure(11)
plot(dat.time,dat.z)
grid on
grid minor

% How the digital filter has been implemented 
%
% alpha = 0.05;
% axF = 0;
% axF2 = 0;
% axF2m1 = 0;
% gxFdata = zeros(num_bins,1);
% gxFdata2 = zeros(num_bins,1);
%
% for i = 1:num_bins
%     axF = (1 - alpha)*axF + alpha*dat.x(i);
%     if (i==1)
%         axF2 = b(1)*dat.x(i);
%     end
%     if (i==2)
%         axF2 = b(1)*dat.x(i) +b(2)*dat.x(i-1)...
%             - a(2)*gxFdata2(num_bins)
%     end
%     if (i>2 && i<num_bins-2)
%         axF2 = b(1)*dat.x(i)+b(2)*dat.x(i-1)...
%             + b(3)*dat.x(i-2)...
%             - a(2)*gxFdata2(num_bins) ...
%             - a(3)*gxFdata2(num_bins-1); 
%     end
%     gxFdata = [ gxFdata(2:end) ; axF ];
%     gxFdata2 = [ gxFdata2(2:end) ; axF2 ];
% end