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

%% Unstable plant
A=[1 1; 0 -2];
B = [ 1; 1];
C = eye(2);
D = zeros(2,1)
uPlant = ss(A,B,C,D)
Pou=tf(uPlant)
step(uPlant);
Ssys=[A-eye(size(A,2)) B ; C D];

% bode(po)
%[sysr,u] = minreal(po,0.01) 

%% Stable plant
A=[-1 1; 0 -2];
sPlant = ss(A,B,C,D);
% p1 = zpk([],[-1 -2] ,1)
Pos = tf(sPlant);
step(sPlant);
Ssys=[A-eye(size(A,2)) B ; C D];
step(Pos);

%% Singular Values
K = 1;
Cs = zpk([],[-1 -2],K);
po = tf([2 1],[1 0]);
bode(Cs*po)
step(Cs*po)
pg=Cs*po;
step(pg)
cloop = inv(1+Cs*po) * Cs;


% 
% Ps = tf([1 1],[ 1 5 6])
% [sysr,U] = minreal(Ps,0.1);
% 
% KalmanA = U*A*U';
% KalmanB = U*B;
% KalmanC = Clocal*U';

% sysr