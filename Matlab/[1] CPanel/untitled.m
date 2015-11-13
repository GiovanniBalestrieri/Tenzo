P = zpk([],[-220 -20],1000);
sysM = canon(P,'modal');

disp('Realization of the following Transfer Function:');
disp(P);
disp('X');
sysM.a

C = pid(10,5000,20)

H=series(C,P)
CL=feedback(H,1)

figure(1)
step(CL)