%% Matilde prépa bac Math terminale S 
% 19/05/2016 -> compleanno papà


%% Initialization
Z0 = 1 + 2i
disp('Initial Conditions:');
disp('Real Part:');
Z0r = real(Z)
disp(Z0r);
disp('Imm Part:');
Z0i = imag(Z)
Z = Z0;
figure(1)
plot(Z0r,Z0i,'xr')
hold on

for i=1:10
    ZM = (Z + abs(Z))*1/4
    Zr = real(Z)
    Zi = imag(Z)
    plot(Zr,Zi,'+g')
    hold on
    axis on
    Z = ZM
end