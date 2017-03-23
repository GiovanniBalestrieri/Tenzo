%% Roll Dynamics 22 mar 2017
% Giovanni Balestrieri 
% Info @ userk.co.uk

clc

%% Create 

Whovering = 1420;
Ts = 0.021;

%% Loading identified motor dynamics
motorDynamics = load('discreteMotortf.mat')
motorDynamics = motorDynamics.mv



disp('Evaluating step response for motor dynamics. Press X');
pause()
opt = stepDataOptions('InputOffset',0,'StepAmplitude',1420);
step(motorDynamics,opt)
motorDynamics = d2d(motorDynamics,0.021)

% get numerator and denominator Roll
[motor_num_tf_discrete , motor_den_tf_discrete] = tfdata(motorDynamics,'v')

% Computing observator canonical form
motorC = canon(motorDynamics,'companion');
motorC1 = motorC;
motorC2 = motorC;

%% Loading identified Roll dynamics

rollDynamics = load('discreteDynamicTenzo.mat');
rollDynamics = rollDynamics.mts

disp('Evaluating step response for roll dynamics. Press X');
pause()
step(rollDynamics)


% Computing observator canonical form
rollC = canon(rollDynamics,'companion');

%% Computing linearized 
% Computing Thrust force. It is the result of vertical forces acting on all
% blade elements of one propeller
Radius = 0.115; % m
Radius_in = 9; % in
Ct = 0.18;
rho = 1.225; % kg/m^3
Aprop = pi*Radius^2;

% Convert to RPM
Kforce = Ct*rho*Aprop*2*Whovering*Radius^2;
%Thrust_newton = rpm*Kforce;
%Thrust_kg = Thrust_newton/9.81



% get numerator and denominator Roll
[roll_num_tf_discrete , roll_den_tf_discrete] = tfdata(rollDynamics,'v');


%% LQ regulator




%% Constant definition and simulation


armLength = 0.23;

%Saturation
pwmUpperBound = 1800;
pwmLowerBound = 1000;

dmUpperBound = 300;
dmLowerBound = -300;

rpmUpperBound = 90;
rpmLowerBound = 0;

% saturations 
enablePwmSaturation = -1;
enableRpmSaturation = -1;

% Measurement Error
enableMisErr = -1;

% linearized
mode = 1;

% nonlinear
%mode = -1;

open('testRollContSolo');


%% Computing augmentedsystem

Brall = rollC.b*armLength*Kforce
Br_segn = Brall*[1 -1]

Br_segn11 = Br_segn(1,1)
Br_segn12 = Br_segn(1,2)
Br_segn21 = Br_segn(2,1)
Br_segn22 = Br_segn(2,2)

Br_segn1 = Br_segn(1:2,1)
Br_segn2 = Br_segn(1:2,2)


states = {'x1m1','x2m1','x1m2','x2m2','xr1','xr2'}
output = {'phi'}
input={'deltaPwm'}

Acomplete = [ motorC.a zeros(2,4) ; zeros(2,2) motorC.a zeros(2,2); (Br_segn1)*motorC.c (Br_segn2)*motorC.c rollC.a ]

Bcomplete = [ motorC.b ; -motorC.b; zeros(2,1)]

Ccomplete = [ zeros(1,4) rollC.c ]

Dcomplete = [ zeros(1,1) ]

rollComplete = ss(Acomplete,Bcomplete,Ccomplete,Dcomplete,Ts,'statename',states,'inputname',input,'outputname',output)

step(rollComplete)

tfRollComplete = tf(rollComplete)

% get numerator and denominator Roll
[roll_c_num , roll_c_den] = tfdata(tfRollComplete,'v');
