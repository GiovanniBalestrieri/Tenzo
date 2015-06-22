%% Tenzo Control System
%  gen-23-2015 UserK
%  info: balestrieri.gepp@gmail.com

%% Physical properties of the quadrotor

g=9.81; %Acceleration of gravity (m)
rho=1.2; %Density of air (m^3.kg^-1)

% Total mass of the quadrotor [Kg]
mq=0.82;
% Mass of a motor (kg). All motors have equal mass.
mm=0.068; 
% Motor length along x-axis (m). All motors have equal sizes.
lx=28.8e-3;
% Motor length along y-axis (m)
ly=28.8e-3;
% Motor length along z-axis (m)
lz=26e-3;

% Distance from the center of gravity to the center of a motor (m).
% The quadrotor is symmetric regarding the XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

% Moment of inertia (x-axis) for motors 1 and 3
% (kg.m^2).
Ix1=(1/12)*mm*(ly^2+lz^2); 
% Moment of inertia (x-axis) for motors
% 2 and 4 (kg.m^2).
Ix2=(1/12)*mm*(ly^2+lz^2)+mm*dcg^2;
% Total moment of inertia along the x-axis (kg.m^2)
Ixx=2*Ix1+2*Ix2; 
% Moment of inertia (y-axis) for motors
% 1 and 3 (kg.m^2).
Iy1=(1/12)*mm*(lx^2+lz^2)+mm*dcg^2; 
% Moment of inertia (y-axis) for motors 2 and 4
% (kg.m^2).
Iy2=(1/12)*mm*(lx^2+lz^2); 
% Total moment of inertia along the y-axis (kg.m^2)
Iyy=2*Iy1+2*Iy2; 
% Moment of inertia (z-axis) for motors
% 1 and 3 (kg.m^2).
Iz1=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Moment of inertia (z-axis) for motors
% 2 and 4 (kg.m^2).
Iz2=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Total moment of inertia along the z-axis (kg.m^2)
Izz=2*Iz1+2*Iz2;
% Inertia matrix
II=diag([Ixx Iyy Izz]); 

% Inflow coefficient
If = -0.3559;

% Thrust coefficient of the propeller and Power coefficient
cp=0.0743; 
ct=0.1154;

% Propeller radius (m)
rp=25.4e-3; 
% Constant value to calculate the moment provided
% by a propeller given its angular speed (kg.m^2.rad^-1)
Km=cp*4*rho*rp^5/pi()^3; 
% Constant value to calculate the thrust provided
% by a propeller given its angular speed (kg.m.rad^-1) 
%Kt=ct*4*rho*rp^4/pi()^2; 
Kt=ct*rho*rp^4*pi();
% Constant that relates thrust and moment of a propeller.
Ktm=Km/Kt;
k1=2.028; %Transfer-function gain of motor 1
k2=1.869; %Transfer-function gain of motor 2
k3=2.002; %Transfer-function gain of motor 3
k4=1.996; %Transfer-function gain of motor 4
tc=0.436; %Time-constant of the motors (assumed equal for all motors)

dz1=1247.4; %PWM dead-zone of motor 1 given its first-order transfer
% function (micro-seconds)
dz2=1247.8; % PWM dead-zone of motor 2 given its first-order transfer
%function (micro-seconds)
dz3=1246.4; % PWM dead-zone of motor 3 given its first-order transfer
%function (micro-seconds)
dz4=1247.9; % PWM dead-zone of motor 4 given its first-order transfer

g_m1=tf(k1,[tc 1]); % First-order transfer-function of motor 1
g_m2=tf(k2,[tc 1]); % First-order transfer-function of motor 2
g_m3=tf(k3,[tc 1]); % First-order transfer-function of motor 3
g_m4=tf(k4,[tc 1]); % First-order transfer-function of motor 4

%vNote: If the behavior of the motors is not linear, linearization around
% an operating point is required to attain these transfer-functions.
PWM_max=2200; %Upper limit of the PWM signal provided to
%the motors (micro-seconds)
w0c=sqrt(g*mq/Kt);
%X0c=w0c;
Ft0=mq*g;

U0=[1,1];
plusConfig = 1;

%% Initial Conditions
ze0=0;
phi0 = 0;
theta0 = 0;
psi0 = 0;
tc=1/20;

%% Path
X = [ 0, 1, 0,-1, 0, 1, 0, 0]; % meters
Y = [ 0, 0, 1, 0,-1, 0, 0, 0]; % meters
Z = [ 3, 3, 3, 3, 3, 3, 3, 1.5]; % meters
t = [ 0, 5,10,15,20,25,30, 35]; % seconds
Psi = [0,90*pi/180,0,180*pi/180,0,0,0,90*pi/180]; % radians
path.x = timeseries(X,t);
path.y = timeseries(Y,t);
path.z = timeseries(Z,t);
path.psi = timeseries(Psi,t);



disp('avvio simulazione 1');
pause;
open('PidNL.mdl')
sim('PidNL.mdl')