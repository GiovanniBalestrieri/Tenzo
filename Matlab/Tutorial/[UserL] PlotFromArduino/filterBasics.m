% Sampling Frequency Fs
Fs = 100;
% Sinusoid frequency
freq = 11;
t = 0:1/Fs:1;

% Generate random values [min,max] = [xm,xM]
xm=1;
xM=10;
amp = xm+ (xM-xm).*rand(1,1);

y = amp*sin(2*pi*freq*t)+15;


% Plots raw data vs samples
figure(1);
plot(t,y);
title('Sinusoid');
xlabel('Time [s]');
ylabel('Amplitude [m*s^-2]');
grid on
grid minor


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
hold on
plot([0:1/(num_bins/2 -1):1], abs(H), 'r');
plot([0:1/(num_bins/2 -1):1],X_magsNorm(1:num_bins/2),'g')

% Filters the signal using coefficients obtained by the butter filter
% design
x_filtered = filter(b,a,y);

% Plots the filtered signal
figure(4)
plot(x_filtered,'r')
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');


