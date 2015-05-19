XMin = -10;
XMax = 10;
YMin = -10;
YMax = 10;

%% Radial basis function
disp('2D case:');
t=-0:0.1:10;
% variance
sigma=0.8; 
esp=1/(2*(sigma)^2);
center=5;
F=exp(-(t-center).^2*esp.^2);
plot(t,F);
pause();
%% 3D function RBF
disp('3D case:');
[X,Y] = meshgrid(0:.1:10);
center
Xc1=center*ones(101,202);
Yc1=center*ones(101,202);
Z = exp(-((X-Xc1(:,1:101)).^2+(Y-Yc1(:,102:202)).^2)*esp.^2);

% sigma1=.8; 
% esp1=1/(2*(sigma1)^2);
% center1=8;
% Xc2=center1*ones(101,202);
% Yc2=center*ones(101,202);
% Z = Z + exp(-((X-Xc2(:,1:101)).^2+(Y-Yc2(:,102:202)).^2)*esp1.^2);
surf(X,Y,Z)
pause();
%% Random Radial Basis functions in space 
disp('3D case with Random path - 3 functions:');
N = 8 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)

%pause();
%% Random Radial Basis functions in space 
disp('3D case with random path - 10 rbf:');
N = 10 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)

pause();
%% Random Radial Basis functions in space 
disp('3D case with random path - 500 RBF:');
N = 500 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
surf(X,Y,Z)
pause();

%% Plane in 3D space
[X,Y] = meshgrid(95:0.1:105);
a=0.1;
b=0.2;
c=1;
d=0;
Z=(-a * X - b * Y)/c;
surf(X,Y,Z)
shading flat
xlabel('x')
ylabel('y')
zlabel('z')
pause();

%% TEst
fig = figure;
[X,Y] = meshgrid(-8:.5:8);
R = sqrt(X.^2 + Y.^2) + eps;
Z = sin(R)./R;
surf(Z)
%p = fig2plotly(fig);
disp('End');