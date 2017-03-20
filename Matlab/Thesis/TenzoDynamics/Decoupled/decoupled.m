%% Tenzo Decoupled dynamics
% mar 20 2017
clc 
clear all
%%  Roll

% retrieve identified System
rollSys = open('transferFunctions/discreteDynamicTenzo.mat');

% get transfer function
rollSysTfDisc = tf(rollSys.mts)

% convert to continuous
rollSysTfCont = d2c(rollSysTfDisc)

% Check step response discrete Vs Continuous
figure(1)
step(rollSysTfDisc,'b')
hold on 
step(rollSysTfCont,'r')

% get numerator and denominator 
[num_tf_discrete , den_tf_discrete] = tfdata(rollSysTfDisc,'v')
[num_tf_cont , den_tf_cont] = tfdata(rollSysTfCont,'v')


% Set input saturation
satThrust = 10000;
open('PhiTheta.slx');
