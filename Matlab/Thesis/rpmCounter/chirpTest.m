t = linspace(0,100)

pwm = 650*sin(exp(t))+700
plot(t,pwm)

%%

t = 0:1:100

pwm = sin(exp(t))
plot(t,pwm)

%%
 Fs=1000; % sample rate 
tf=2; % 2 seconds
t=0:1/Fs:tf-1/Fs;
f1=100;
f1=f1*t %+(sl.*t/2);
%f2=f1(end)+f2*semi_t-sl.*semi_t/2;
%f=[f1 f2];
f = f1
y=650*cos(2*pi*f.*t)+ 1300;
plot(t,y)
