%function f = codio(x)
pi = 3.1415
x_0 = 0;
v_0 = 11.5;
z_0 = 0;
X = linspace(0,pi,100);
Y = sin(X).^2 + cos(X).^2;
Z = cumtrapz(X,Y);
plot(X , [Y; Z])
%f = 100*(x(2) - x(1)^2)^2 + (1 - x(1))^2;

g = 9.81;
alpha = 86*pi/180;

A = [0 1 0 0 0; ...
     0 0 0 0 0;  ...
     0 0 0 1 0; ...
     0 0 0 0 -1; ...
     0 0 0 0 0]
B = [ 0; 0 ; 0 ;0 ; 0]
C = eye(5)
D = zeros(5,1)

ball  = ss(A,B,C,D)
x0 = [ x_0 ; v_0*cos(alpha) ; z_0  ; v_0*sin(alpha) ; g];


[y,t] = initial(ball,x0,2)

x = y(:,1)'
z = y(:,3)'
plot3(t,x,z,'ob')
grid on
