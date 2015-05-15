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


%% Filtering

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
