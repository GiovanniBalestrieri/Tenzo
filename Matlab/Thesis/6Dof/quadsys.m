aa[function [Xout] = quadsys(Xin,m,I,wm,Kt,Ktm,dcg)
%Function to simulate the quadrotor dynamics and outputs its current
%state space vector.
% Xin - Initial state vector of the quadrotor:
% Xin = [U V W P Q R X Y Z q1 q2 q3 q4]
% U - Linear velocity in the x-axis direction of the inertial
%
% reference frame (m.s^-1);
% V - Linear velocity in the y-axis direction of the inertial
% reference frame (m.s^-1);
% W - Linear velocity in the z-axis direction of the inertial
% reference frame (m.s^-1);
% P - Angular speed around the x-axis of the quadrotor (rad.s^-1);
% Q - Angular speed around the y-axis of the quadrotor (rad.s^-1);
% R - Angular speed around the z-axis of the quadrotor (rad.s^-1);
% X - Position of body frame of the quadrotor along the x-axis of
% the inertial reference frame (m);
% Y - Position of body frame of the quadrotor along the x-axis of
% the inertial reference frame (m);
% Z - Position of body frame of the quadrotor along the x-axis of
% the inertial reference frame (m);
% q1...q4 - Quaternion containing the quadrotor's attitude
% (conversion made from Euler angles).
% m - Quadrotor's mass (kg).
%
% I - Inertia matrix assuming the quadrotor is a rigid body with
% constant mass of which its axis are aligned with the principal
% axis of inertia. The matrix I becomes a diagonal matrix
% containing only the principal moments of inertia:
%
% I = [I1 0 0;0 I2 0;0 0 I3]
% I1 - Principal moment of inertia along the x-axis of the
%
% quadrotor's body frame (kg.m^2);
% I2 - Principal moment of inertia along the y-axis of the
%
% quadrotor's body frame (kg.m^2);
% I3 - Principal moment of inertia along the z-axis of the
% quadrotor's body frame (kg.m^2).
% wm - Angular speed array of the propellers:
% wm = [w1 w2 w3 w4]
% w1 - Current angular speed of propeller 1 (North propeller)
% (rad.s-1);
% w2 - Current angular speed of propeller 2 (East propeller)
% (rad.s-1);
% w3 - Current angular speed of propeller 3 (South propeller)
%
%(rad.s-1);
% w4 - Current angular speed of propeller 4 (West propeller)
% (rad.s-1).
%
% Kt - Constant used to express the thrust and prop. angular speed
%
% Ktm - Constant used to express the thrust with prop. momentum
%
% dcg - Distance from the quadrotor's COG to the motor
%
% Outputs:
%
%
% Xout - Current system states:
% Xout = [U V W P Q R X Y Z q1 q2 q3 q4]
%
% Example:
% Xin = [0 0 0 0.1 0 0 0 0 0 1 0 0 0];
% wm = [0 0 0 0];
% m = 1;
% I = [0.1 0 0;0 0.1 0;0 0 0.1];
% Kt=0.1;

% Ktm=0.01;
% dcg=0.5;
%
% Xout = quadsys(Xin,m,I,wm,Kt,Ktm,dcg) outputs:
%
% Xout = [0 0 9.8100 0 0 0 0 0 0 0 0.0500 0 0]
print('ok')
U=Xin(1); %Linear velocity x-axis (m)
V=Xin(2); %Linear velocity y-axis (m)
W=Xin(3); %Linear velocity z-axis (m)
P=Xin(4); %Angular velocity around the x-axis (rad.s^-1)
Q=Xin(5); %Angular velocity around the y-axis (rad.s^-1)
R=Xin(6); %Angular velocity around the z-axis (rad.s^-1)
X=Xin(7); %Position of the body axes frame along the x-axis of the inertial
%reference system (m)
Y=Xin(8); %Position of the body axes frame along the y-axis of the inertial
%reference system (m)
Z=Xin(9); %Position of the body axes frame along the z-axis of the inertial
%reference system (m)
q0=Xin(10); %Quaternion element 1 calculated from Euler angle conversion
q1=Xin(11); %Quaternion element 2 calculated from Euler angle conversion
q2=Xin(12); %Quaternion element 3 calculated from Euler angle conversion
q3=Xin(13); %Quaternion element 4 calculated from Euler angle conversion

% Rotation matrix R using quaternions 
Rm=[q0^2+q1^2-q2^2-q3^2 2*(q1*q2+q0*q2) 2*(q1*q3-q0*q2);
2*(q1*q2-q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3+q0*q1);
2*(q1*q3+q0*q2) 2*(q2*q3-q0*q1) q0^2-q1^2-q2^2+q3^2]; 

% Calculate thrust produced by each propeller (N)
Tm(1:4)=Kt.*(wm(1:4).^2);
Fx=0; %x-axis force acting on the quadrotor while hovering steady at
%constant altitude
Fy=0; %y-axis force acting on the quadrotor while hovering steady at
%constant altitude
Fz=-(Tm(1)+Tm(2)+Tm(3)+Tm(4)); %z-axis force acting on the quadrotor while
%hovering steady at constant altitude
Mx=(Tm(4)-Tm(2))*dcg; %Momentum responsible for rotation around the x-axis
%(N.m)
My=(Tm(1)-Tm(3))*dcg; %Momentum responsible for rotation around the y-axis
%(N.m)
Mz=Ktm*(Tm(1)+Tm(3)-Tm(2)-Tm(4)); %Momentum z-axis responsible for rotat
g=9.81; %Acceleration of gravity (m.s^-2)


Lin_a=(1/m).*[Fx Fy Fz]'+Rm*[0 0 g]'-([Q*W-R*V R*U-P*W P*V-Q*U])'; %Calc.
%the linear acceleration of the quadrotor
AngP=(Mx/I(1,1))-(I(3,3)-I(2,2))*Q*R/I(1,1); %Angular acceleration around
%the x-axis of the body reference frame
AngQ=(My/I(2,2))-(I(1,1)-I(3,3))*R*P/I(2,2); %Angular acceleration around
%the y-axis of the body reference frame
AngR=(Mz/I(3,3))-(I(2,2)-I(1,1))*P*Q/I(3,3); %Angular acceleration around
%the z-axis of the body reference frame

%Calculate the position of the quadrotor
Pos=Rm'*[U V W]'; 

Omq=[0 -P -Q -R; 
     P  0  R -Q; 
     Q -R  0  P; 
     R  Q -P  0];
 
%Rotation matrix to calculate the time drivative of the rotation quaternion 
epsilon=1-[q0 q1 q2 q3]*[q0 q1 q2 q3]';

%Time derivative of the %rotation quaternion
dq=(1/2)*Omq*[q0 q1 q2 q3]'+epsilon*[q0 q1 q2 q3]'; 

%Output state vector
Xout=[Lin_a(1) Lin_a(2) Lin_a(3) AngP AngQ AngR Pos(1) Pos(2) Pos(3) dq(1) dq(2) dq(3) dq(4)]; 

end