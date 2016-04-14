fs = 8000;

%Generate time values from 0 to T seconds at the desired rate.
T = 5; % 2 seconds duration
t = 0:(1/fs):T;

%Generate a sine wave of the desired frequency f at those times.
f1 = 1300.81;
f2 = 3800;
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
T =  5;
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

