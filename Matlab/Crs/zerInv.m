c1 = 3;
c2 = 1;
c3 = 1;

A=[ -1 0 ; 2 -4];
B=[ 1 ;2];
C= [1 0 ; 0 1];
D= zeros(2,1);

sys=ss(A,B,C,D)

eig(A)
zeroT = tzero(sys)

Ps=tf(sys)

%% Simulink 

sys = 'zerInvS';
open_system(sys)
SimOut = sim(sys);