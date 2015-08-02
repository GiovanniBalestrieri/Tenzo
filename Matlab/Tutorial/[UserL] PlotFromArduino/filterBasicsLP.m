%% Quick HP Filter
%  02/08/15 -> More info @ userk.co.uk

clear all;
clc;

%% Signal definition

% Sampling Frequency Fs
Fs = 2000;
% Sinusoid frequency
NoiseFreq = 100;
freq = 9;
t = 0:1/Fs:1;

% Generate random values [min,max] = [xm,xM]
xm = [ 1 0.04];
xM = [ 10 0.4];
amp = xm+ (xM-xm).*rand(1,2);

y = amp(1)*sin(2*pi*freq*t) + amp(2)*sin(2*pi*NoiseFreq*t);

% Plots raw data vs samples
figure(1);
plot(t,y);
title('Sinusoid');
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
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
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
plot(0:1/(num_bins/2 -1):1,X_magsNorm(1:num_bins/2),'g')

% Filters the signal using coefficients obtained by the butter filter
% design
x_filtered = filter(b,a,y);

% Plots the filtered signal
figure(4)
plot(x_filtered,'r')
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');


