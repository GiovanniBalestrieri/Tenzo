%% 
% Plot 3d functions
%


[X,Y] = meshgrid(-5:.1:5);
%Z = (2*X.^2+3*Y.^2+2*X+3*Y);
Z = 1/2*(6*X.^2+4*X*Y+Y.^2)-4*X-Y;

%% Ex1 20-feb2015
figure(1)
title('Function');
surf(X,Y,Z)
shading interp
figure(2)
title('Function');
mesh(X,Y,Z)
shading interp
%% 
banana = @(x) (3*x(1)^2+1/2*x(2)^2+2*x(1)*x(2))-4*x(1)-x(2)

sol = fminsearch(banana,[-1.4,-1.4])
disp('Solution of MinSearch')
feval(banana,[sol(1),sol(2)])

%feval(banana,[-1,-1])

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




