%% 
% Plot 3d functions
%
%% Ex1 20-feb2015

[X,Y] = meshgrid(-5:.2:5);
Z = (2*X.^2+3*Y.^2+2*X+3*Y);
dZ = diff(Z)/X
figure(1)
title('Function');
surf(X,Y,Z)
shading interp
figure(2)
title('Function');
mesh(X,Y,Z)
shading interp
%% 
banana = @(x) (2*x(1)^2+3*x(2)^2+2*x(1)+3*x(2))^2
fminsearch(banana,[-1.4,-1.4])
feval(banana,[-0.5,-0.5])
feval(banana,[0,1])

feval(banana,[-1,-1])

%%
% Hessian
syms x y

banana = (2*x^2+3*y^2+2*x+3*y)^2
disp('Display Gradien of f:');
graBanana = gradient(banana,[x y])
disp('Display Hessian of f:');
HessBanana = hessian(banana,[x y])
solGrad = solve(graBanana==0,[x y]);
solGrad.x
solGrad.y




feval(HessBanana,