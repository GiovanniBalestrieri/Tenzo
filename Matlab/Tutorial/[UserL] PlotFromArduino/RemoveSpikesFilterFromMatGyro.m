% This scipt loads and plots data from a .mat file
clc
gxFdata = 0;
gyFdata = 0;
gzFdata = 0;


% Loads samples from file
dat = load('GyroSamples1.mat');
%dat = load('samples.mat');
%datX100 = dat.x(1:100)
%plot(datX100)

% Plots raw data vs samples
figure(2);
plot(dat.x);
title('Angular velocity along X axis');
xlabel('Samples');
ylabel('Amplitude [c*deg*s^-1]');
grid on
grid minor


%% Plots magnitude spectrum of the signal
X_mags=abs(fft(dat.x));
figure(3)
plot(X_mags);
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
num_bins = length(X_mags);
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
grid on
xlabel('Normalized frequency [\pi rads/samples]');
ylabel('Magnitude');

% Normalize X_mags

X_magsNorm = (X_mags - min(X_mags)) / ( max(X_mags) - min(X_mags) )

%% Designs a second order filter using a butterworth design guidelines
[b a] = butter(2,0.25,'low');

figure(4);
% Plot the frequency response (normalized frequency)
H = freqz(b,a,floor(num_bins/2));
plot([0:1/(num_bins/2 -1):1], abs(H), 'r');
hold on
grid on
plot(0:1/(num_bins/2 -1):1,X_magsNorm(1:num_bins/2),'g')

% Filters the signal using coefficients obtained by the butter filter
% design
datX = dat.x';
x_filtered = filter(b,a,dat.x);
dt = 0.01;
x_moving = smooth(dat.x,dt,'rloess')
n=3;
x_median = medfilt1(dat.x,n)
% Plots the filtered signal
figure(5)
plot(x_filtered,'r')
hold on
plot(dat.x,'b')
hold on
plot(x_moving,'k');
hold on
plot(x_median,'g-');
title('Filtered Signal - Second Order Butterworth');
legend('Filtered','real','moving','median');
grid on;
xlabel('Samples');
ylabel('Amplitude');

%% Redesign the filter using a higher order filter
[b2 a2] = butter(3,0.5,'high');

% Plots the magnitude spectrum and compare with lower order
H2 = freqz(b2,a2,floor(num_bins/2));
figure(6);
hold on
plot([0:1/(num_bins/2 -1):1], abs(H2), 'r');
hold on
grid on
plot(0:1/(num_bins/2 -1):1,X_magsNorm(1:num_bins/2),'g')

%filter the noisy signal and plots the result
x_filtered2 = filter(b2,a2,dat.x);
figure(7)
plot(x_filtered2,'g');
hold on
plot(dat.x,'b')
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

figure(8)
plot(dat.x,'r--');
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

