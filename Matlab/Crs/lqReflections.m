%% Il sys non è osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile
disp('Semplificazione del modello. Press X per mostrare il modello semplificato');
  
  AMin = [
        0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  BMin = [
    zeros(1,4);
    0 0 0 1/mq; 
    zeros(3,4);
    1/Ixx 0 0 0;
    0 1/Iyy 0 0;
    0 0 1/Izz 0];
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};

tenzoMin=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);

tenzoRetro=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
%step(tenzoRetro)


% Transfer function
Ps = tf(tenzoMin);
N_phi = Ps(1,1).num{1};
D_phi = Ps(1,1).den{1};

N_theta = Ps(2,2).num{1};
D_theta = Ps(2,2).den{1};

N_psi = Ps(3,3).num{1};
D_psi = Ps(3,3).den{1};

N_thrust = Ps(4,4).num{1};
D_thrust = Ps(4,4).den{1};

%Ps(1,1)

% invarianzZero
disp('Invariant Zeros:');
tzero(A,B,Clocal,D,eye(12))

disp('Transmission Zeros');
tzero(Ps)

% Sottosistema ragg e oss con decomposizione di Kalman
% Check wether the solution is still valid with this subsys
[sysr,U] = minreal(tenzo,0.1);

KalmanA = U*A*U';
KalmanB = U*B;
KalmanC = Clocal*U';

pause();
sysr

%% LQR matlab style
Q=eye(size(AMin));
R=eye(size(BMin,2));
[K,S,e] = lqr(tenzoMin,Q,R);
tenzoLQR=ss(AMin-BMin*K,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoLQR);
tzero(tenzoLQR);

%% Kalman
[kest,L,P,M,Z] = kalm(tenzoLQR,Qn,R)

