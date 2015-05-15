fid = fopen('DATA0001.csv');
M = textscan(fid,'%f%f%s','delimiter',',');
fclose(fid);

%Time M{1}; % Values M{2}; % Type M{3};

numberOfSamples = size(M{1},1);
% Detect acc type
contX = 1;
contY = 1;
contZ = 1;
for i=1:numberOfSamples
    % Save Xaxis' values
    if strcmp(M{3}(i,1),'ACC1X')
        accXIndex(1,contX)=i;
        contX = contX+1;
        %M{3}(i,1)
    end
    
    % Save Yaxis' values
    if strcmp(M{3}(i,1),'ACC1Y')
        accYIndex(1,contY)=i;
        contY= contY+1;
        %M{3}(i,1)
    end
    
    % Save Zaxis' values
    if strcmp(M{3}(i,1),'ACC1Z')
        accZIndex(1,contZ)=i;
        contZ = contZ + 1;
        %M{3}(i,1)
    end
end
numberOfAccSamples = size(accXIndex,2);

for j=1:(numberOfAccSamples-1)
    accXRaw(1,j) = M{2}(accXIndex(1,j),1);
    accYRaw(1,j) = M{2}(accYIndex(1,j),1);
    accZRaw(1,j) = M{2}(accZIndex(1,j),1);
    time(1,j) = M{1}(accXIndex(1,j),1);
end

%% Plot Raw Values

% X Axis
figure(1)
plot(time,accXRaw,'--rs','LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',10);
title('Raw X Acc');
xlabel('Acc');
ylabel('Time [ds]');

% Y Axis
figure;
plot(time,accYRaw,'--rs','LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',10);
title('Raw Y Acc');
xlabel('Acc');
ylabel('Time [ds]');

% Z Axis
figure;
plot(time,accZRaw,'--rs','LineWidth',2,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'MarkerSize',10);
title('Raw Z Acc');
xlabel('Acc');
ylabel('Time [ds]');

%% Spectral analysis

% Plots magnitude spectrum of the signal
X_mags=abs(fft(accXRaw));
figure(3)
plot(X_mags);
xlabel('DFT Bins');
ylabel('Magnitude');

% Plots first half of DFT (normalized frequency)
figure(3)
num_bins = length(X_mags);
plot([0:1/(num_bins/2 -1):1],X_mags(1:num_bins/2))
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
x_filtered = filter(b,a,accXRaw);

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
x_filtered2 = filter(b2,a2,accXRaw);
figure(5)
plot(x_filtered2,'g');
title('Filtered Signal - 20 Order Butterworth');
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
x_filtered3 = filter(b3,a3,accXRaw);
figure(5)
plot(x_filtered2,'g');
title('Filtered Signal - 20 Order Butterworth');
xlabel('Samples');
ylabel('Amplitude');

%% Comparison between filters: Digital Arduino Vs Matlab

figure(7)
% Plots Raw acc data
plot(time,accXRaw,'k');
hold on
% Arduino digital filter
%plot(dat.time,dat.y,'b','LineWidth',2);
%hold on
% Matlab filter
plot(time,x_filtered,'r','LineWidth',2);
hold on
% Plots cF = 0.10 Matlab filtered, order 3
plot(time,x_filtered2,'g-','LineWidth',2);
hold on
% Plots cf = 0.3 Matlab filtered order 30
plot(time,x_filtered3,'c-','LineWidth',2);
title('Filter Comparison');
xlabel('Samples [us]');
ylabel('Amplitude [m*s^-2]');
grid on
grid minor


