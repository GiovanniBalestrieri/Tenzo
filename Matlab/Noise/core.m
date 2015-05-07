%% Radial basis function
t=-0:0.1:10;
% variance
sigma=1; 
esp=1/(2*(sigma)^2);
center=5
F=exp(-(t-center).^2*esp.^2);
plot(t,F);
% 3D function RBF
t1=-0:0.1:10;
t2=-0:0.1:10;
t = [t1; t2];
% Or: 
[X,Y] = meshgrid(0:.1:10);
center=5*ones(101,202);
Z = exp(-norm([X;Y]'-center).^2*esp.^2);
surf(X,Y,Z)