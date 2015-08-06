%% Quick HP Filter
%  02/08/15 -> More info @ userk.co.uk

clear all;
clc;

%% IDEAL Signal definition

% Sampling Frequency Fs
Fs = 350;
% Sinusoid frequency
freq = 30;
bias = 15
t = 0:1/Fs:1;

% Generate random values [min,max] = [xm,xM]
xm = 2;
xM = 8;
amp = xm+ (xM-xm).*rand(1,1);

% Use random value for the amplitude
y = amp*sin(2*pi*freq*t) + bias;

yEasy = y - mean(y);

y0 = 0;
y15 = 15;
% Plots raw data vs samples
figure(1);
plot(t,y);
hold on
plot(t,y0,'k','LineWidth',2);
hold on
plot(t,y15,'k');
hold on
plot(t,yEasy,'r');
title('Sinusoid ');
xlabel('Time [s]');
ylabel('Amplitude of f');
grid on
grid minor

%% Analyze signal

% Plots magnitude spectrum of the signal
X_mags=abs(fft(y));
figure(2)
plot(X_mags);
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
num_bins = length(X_mags);
plot(0:1/(num_bins/2 -1):1,X_mags(1:num_bins/2));
grid on
xlabel('Normalized frequency [\pi rads/samples]');
ylabel('Magnitude');

%% Normalize X_mags

X_magsNorm = (X_mags - min(X_mags)) / ( max(X_mags) - min(X_mags) )

%% Filter Design

% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.2,'high');

% Plot the frequency response (normalized frequency)
figure(3)
H = freqz(b,a,floor(num_bins/2));
plot(0:1/(num_bins/2 -1):1, abs(H), 'r');
hold on
grid on
plot(0:1/(num_bins/2 -1):1,X_magsNorm(1:num_bins/2),'g')

%% Filters the signal using coefficients obtained by the butter filter
% design
x_filtered = filter(b,a,y);

% Plots the filtered signal
figure(4)
plot(t,x_filtered,'r')
hold on 
plot(t,y0,'k','LineWidth',2)
hold on 
plot(t,15,'k','LineWidth',2)
grid on
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

%% Real Signal definition

% Sampling Frequency Fs
Fs = 350;
% Sinusoid frequency
freq = 30;
bias = 1
maxTime = 2;
t = 0:1/Fs:maxTime;

% Generate random values [min,max] = [xm,xM]
xm = -0.1;
xM = 0.1;
amp = xm+ (xM-xm).*rand(1,Fs*maxTime+1);

% Use random value for the amplitude
y = bias + amp;
y0 = 0;
ybias = bias;
y2 = 2;
% Plots raw data vs samples
figure(5);
plot(t,y);
hold on
plot(t,y0,'k','LineWidth',2);
hold on
plot(t,y2,'k');
hold on
plot(t,ybias,'r');
title('Sinusoid');
xlabel('Time [s]');
ylabel('Amplitude of f');
grid on
grid minor

%% Analyze signal

% Plots magnitude spectrum of the signal
X_mags=abs(fft(y));
figure(6)
plot(X_mags);
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
num_bins = length(X_mags);
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
grid on
xlabel('Normalized frequency [\pi rads/samples]');
ylabel('Magnitude');

%% Normalize X_mags

X_magsNorm = (X_mags - min(X_mags)) / ( max(X_mags) - min(X_mags) )

%% Filter Design

% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.2,'high');

% Plot the frequency response (normalized frequency)
figure(7)
H = freqz(b,a,floor(num_bins/2));
plot(0:1/(num_bins/2 -1):1, abs(H), 'r');
hold on
grid on
plot(0:1/(num_bins/2 -1):1,X_magsNorm(1:num_bins/2),'g')

% Filters the signal using coefficients obtained by the butter filter
% design
x_filtered = filter(b,a,y);

% Plots the filtered signal
figure(8)
plot(t,x_filtered,'r')
hold on 
plot(t,y0,'k','LineWidth',2)
hold on 
plot(t,ybias,'k','LineWidth',2)
grid on
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');


