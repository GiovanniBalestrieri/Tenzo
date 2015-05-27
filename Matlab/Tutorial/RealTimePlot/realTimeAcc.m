%% Arduino Accelerometer real time plot

%  The sensor used in this project is the MMA7260Q
%  !!Caution!! operative voltage: 3.3 V
%  Wiring instrucitons:
%  Z -> A0
%  Y -> A1
%  X -> A2
%     G-Range | GS1 | GS2 | Sensitivity
%      1.5 G   GND    GND     800mV/g
%      2 G     GND    3.3     600mV/g
%      4 G     3.3    GND     300mV/g
%      6 G     3.3    3.3     200mV/g

% Connect the sleep pin to 3.3V. For power saving connect 
% the sleep pin to GND
clear all;
clc;

% declaring the arduino varialble and establishing connection
a = arduino('Com13');
modeG = 0.800
midV = 1.65;
Xacc = 0, Yacc = 0, Zacc=0;
% Figure options
figure;
hold on;
grid on;
ax = axes('box','on');
% Defines pin to AnalogInputs
% a.pinMode(A0,'input');
% a.pinMode(A1,'input');
% a.pinMode(A2,'input');

%% Initializinig the rolling plot

buf_len = 100;
index = 1:buf_len;

% create variables for the Xaxis
gxdata = zeros(buf_len,1);
gydata = zeros(buf_len,1);
gzdata = zeros(buf_len,1);

%% Data collection and Plotting
while abs(Xacc)< 3.5
    
% Reads raw data
ZVal = a.analogRead(0);
YVal = a.analogRead(1);
XVal = a.analogRead(2);

%Convertion
Zav = ZVal*0.005;
Yav = YVal*0.005;
Xav = XVal*0.005;
Zacc = -(Zav-midV)/modeG;
Yacc = -(Yav-midV)/modeG;
Xacc = -(Xav-midV)/modeG;

    %[gx, gy, gz] = [Xacc, Yacc, Zacc];
    % Update the rolling plot. Append the new reading to the end of the
    % rolling plot data
    gxdata = [ gxdata(2:end) ; Xacc ];
    gydata = [ gydata(2:end) ; Yacc ];
    gzdata = [ gzdata(2:end) ; Zacc ];    
    %Plot the X magnitude
    subplot(3,1,1);
    title('X Axis Acceleration in G');
    plot(index,gxdata,'r','LineWidth',1);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
    xlabel('Time');
    ylabel('X Acc (G)');
    axis([1 buf_len -3.5 3.5]);
    %hold on;
    subplot(3,1,2);
    title('Y Axis Acceleration in G');
    plot(index,gydata,'b','LineWidth',1);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
    xlabel('Time');
    ylabel('Y Acc');
    axis([1 buf_len -3.5 3.5]);
    subplot(3,1,3);
    title('Z Axis Acceleration in G');
    %hold on;
    plot(index,gzdata,'g','LineWidth',1);%,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',5);
    axis([1 buf_len -3.5 3.5]);
    xlabel('Time');
    ylabel('Z Acc');
    drawnow;
end

