% This scipt loads and plots data from a .mat file
clc
gxFdata = 0;
gyFdata = 0;
gzFdata = 0;


% Loads samples from file
dat = load('GyroSamples.mat');

% Plots raw data vs samples
figure(2);
plot(dat.x);
title('Noisy acceleration along X axis');
xlabel('Samples [s]');
ylabel('Amplitude [m*s^-2]');
grid on
grid minor


% Plots magnitude spectrum of the signal
X_mags=abs(fft(dat.x));
figure(3)
plot(X_mags);
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
num_bins = length(X_mags);
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
xlabel('Normalized frequency [\pi rads/samples]');
ylabel('Magnitude');

% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.5,'high');

% Plot the frequency response (normalized frequency)
H = freqz(b,a,floor(num_bins/2));
hold on
plot([0:1/(num_bins/2 -1):1], abs(H), 'r');

% Filters the signal using coefficients obtained by the butter filter
% design
x_filtered = filter(b,a,dat.x);

% Plots the filtered signal
figure(4)
plot(x_filtered,'r')
title('Filtered Signal - Second Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

% Redesign the filter using a higher order filter
[b2 a2] = butter(3,0.5,'high');

% Plots the magnitude spectrum and compare with lower order
H2 = freqz(b2,a2,floor(num_bins/2));
figure(3);
hold on
plot([0:1/(num_bins/2 -1):1], abs(H2), 'r');

%filter the noisy signal and plots the result
x_filtered2 = filter(b2,a2,dat.x);
figure(5)
plot(x_filtered2,'g');
title('Filtered Signal - 20 Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

%% Comparison between filters: first order low pass Vs second order

axF2 = 0;
axF3 = 0;
gxFdata = zeros(num_bins,1);
gxFdata2 = zeros(num_bins,1);
gxFdata3 = zeros(num_bins,1);
% gyFdata = zeros(num_bins,1);
% gzFdata = zeros(num_bins,1);

for i = 1:num_bins
    axF = dat.x(i) - mean(dat.x);
    
    if (i>2 && i<num_bins-2)
        axF2 = b(1)*dat.x(i-2)+(b(2)-b(1))*dat.x(i-1)+(b(3)-b(2))*dat.x(i)...
            + (a(1)-a(2))*gxFdata2(num_bins-(i+2)) + (a(2)-a(3))*gxFdata2(num_bins-(i+1)); 
        %axF2 = (1 - 2*a(2)*a(3))*gxFdata2(num_bins-(i+2)) + (2*alpha - alpha^2)*gxFdata2(num_bins-(i+1)) + alpha^2*dat.x(i);
    end
    
    if (i>2 && i<num_bins-2)
        axF3 = b2(1)*dat.x(i-2)+(b2(2)-b2(1))*dat.x(i-1)+(b2(3)-b2(2))*dat.x(i)...
            + (a2(1)-a2(2))*gxFdata3(num_bins-(i+2)) + (a2(2)-a2(3))*gxFdata3(num_bins-(i+1)); 
        %axF2 = (1 - 2*a(2)*a(3))*gxFdata2(num_bins-(i+2)) + (2*alpha - alpha^2)*gxFdata2(num_bins-(i+1)) + alpha^2*dat.x(i);
    end
    gxFdata = [ gxFdata(2:end) ; axF ];
    gxFdata2 = [ gxFdata2(2:end) ; axF2 ];
    gxFdata3 = [ gxFdata3(2:end) ; axF3 ];
%     gyFdata = [ gyFdata(2:end) ; gyFilt ];
%     gzFdata = [ gzFdata(2:end) ; gzFilt ];
end

figure(6)
plot(dat.x,'r--*');
hold on
plot(gxFdata,'b');
hold on
plot(gxFdata2,'g');
hold on
plot(gxFdata3,'k');
%hold on
%plot(x_filtered,'g*');
%hold on
%plot(x_filtered2,'k*');
title('Filter Comparison');
xlabel('Samples [s]');
ylabel('Amplitude [m*s^-2]');
grid on
grid minor

