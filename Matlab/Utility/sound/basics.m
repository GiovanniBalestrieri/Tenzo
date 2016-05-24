fs = 8000;

%Generate time values from 0 to T seconds at the desired rate.
T = 5; % 2 seconds duration
t = 0:(1/fs):T;

%Generate a sine wave of the desired frequency f at those times.
f1 = 1300.81;
f2 = 270;
a = 0.5;
b = 0.1;

y = a*sin(2*pi*f1*t);

y2 = b*sin(2*pi*f2*t);
yDoub = a*sin(2*pi*f1*t)+b*sin(2*pi*f2*t);
plot(t,y2);
sound(y2, fs);

%%
% Record your voice for 5 seconds.
recObj = audiorecorder;
disp('Start speaking.')
T =  1;
recordblocking(recObj, T);
disp('End of Recording.');

% Play back the recording.
play(recObj);

% Store data in double-precision array.
myRecording = getaudiodata(recObj);
plot(myRecording)
%%
size(myRecording)
t=0:1/fs:T;
t = t(1:size(t,2)-1);
size(t)

% Plot the waveform.
y2 = y2(1:size(y2,2)-1);
size(myRecording')
y = myRecording' + y2(1,:);
size(t)
size(y)
plot(t,y);
sound(y,fs)

y = myRecording'

size(fs)
y1 = y(1,13000:end);
t1= t(1,13000:end);
plot(y,t)


%% 
size(y)
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
