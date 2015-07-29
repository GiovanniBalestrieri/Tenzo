%% Tenzo Control System
%  gen-23-2015 UserK
%  info: balestrieri.gepp@gmail.com

%% Tenzo's Variables

% Inflow coefficient
If = -0.3559;

% Acceleration of gravity [m*s^-2]
g=9.81; 
% Density of air [m^3.kg^-1]
rho=1.2;

% Total mass of the quadrotor [Kg]
mq=1.20;
% Mass of a motor [kg]. All motors have equal mass.
mm=0.068; 
% Motor length along x,y,z-axis [m] 
lx=28.8e-3;
ly=28.8e-3;
lz=26e-3;

% Distance from the center of gravity to the center of a motor [m].
% The quadrotor is symmetric regarding to XZ and YZ planes, so
% dcg is the same for all motors.
dcg=0.288; 

% Moment of inertia (x-axis) for motors 1 and 3 [kg.m^2].
Ix1=(1/12)*mm*(ly^2+lz^2); 
% Moment of inertia (x-axis) for motors 2 and 4 [kg.m^2].
Ix2=(1/12)*mm*(ly^2+lz^2)+mm*dcg^2;
% Total moment of inertia along the x-axis [kg.m^2]
Ixx=2*Ix1+2*Ix2; 
% Moment of inertia (y-axis) for motors 1 and 3 [kg.m^2].
Iy1=(1/12)*mm*(lx^2+lz^2)+mm*dcg^2; 
% Moment of inertia (y-axis) for motors 2 and 4 [kg.m^2].
Iy2=(1/12)*mm*(lx^2+lz^2); 
% Total moment of inertia along the y-axis [kg.m^2]
Iyy=2*Iy1+2*Iy2; 
% Moment of inertia (z-axis) for motors 1 and 3 [kg.m^2]
Iz1=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Moment of inertia (z-axis) for motors 2 and 4 [kg.m^2]
Iz2=(1/12)*mm*(lx^2+ly^2)+mm*dcg^2; 
% Total moment of inertia along the z-axis [kg.m^2]
Izz=2*Iz1+2*Iz2;
% Inertia matrix
II=diag([Ixx Iyy Izz]); 

%% Linearized Model

states = {'xe','ye','ze','vxe','vye','vze','phi','theta','psi','wxb','wyb','wzb'};

A = [ 0 0 0 1 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 0 0;
      0 0 0 0 0 1 0 0 0 0 0 0;
      0 0 0 0 0 0 0 -g 0 0 0 0;
      0 0 0 0 0 0 g 0 0 0 0 0;
      0 0 0 0 0 If 0 0 0 0 0 0;
      0 0 0 0 0 0 0 0 0 1 0 0; 
      0 0 0 0 0 0 0 0 0 0 1 0; 
      0 0 0 0 0 0 0 0 0 0 0 1; 
      zeros(3,12)];

B = [zeros(5,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz];
 
outputsLocal = {'phi'; 'theta';'psi';'ze'};
Clocal = [ 0 0 0 0 0 0 1 0 0 0 0 0; 
           0 0 0 0 0 0 0 1 0 0 0 0; 
           0 0 0 0 0 0 0 0 1 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0];
       
outputsRemote = {'xw'; 'ye';'ze';'psi'};
Cremote = [ 1 0 0 0 0 0 0 0 0 0 0 0;
           0 1 0 0 0 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 1 0 0 0];
       
CFull = eye(size(A));       
DFull = zeros(size(A),4);
inputs = {'Thrust','TauPhi','TauTheta','TauPsi'};
D = zeros(4,4);


tenzo=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
%tenzo=ss(A,B,CFull,DFull);

%disp('Transfer matrix of the model')
modello_tf=tf(tenzo)

pause;
%% Eigenvalues of the system

disp('specifica 1) Verifichare che esiste un cotrollore che soddisfa le specifiche A1,B,C1 (teorema 5.4.2):');
disp('verifica preliminare, autovalori del processo [eig(A)]:')
stab=1;
eOp = eig(A);
[dn,dm]=size(eOp);
  for i=1:dn,
      if (real(eOp(i))>0 ) 
          stab=0; 
          disp('elemento a parte reale positiva:'); 
          disp(eOp(i));
      end    
  end
if (stab==0) 
    disp('Sistema instabile: gli autovalori a ciclo aperto sono: [comando eig(A)]'); 
end
if (stab==1) 
    disp('Sistema stabile: gli autovalori a ciclo aperto sono: [comando eig(A)]'); 
end
disp(eOp);

% Verifica Raggiungibilità

disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
disp(rank(ctrb(A,B)));


disp('Verifica osservabilità. Rango della matrice di osservabilità:')
disp(rank(obsv(A,Clocal)));

pause();

%% Il sys non è osservabile. 
% Definiamo il sottosistema osservabile e raggiungibile

AMin = [ 0 0 0 0 0 0 0  0;
       0 0 0 -g 0 0 0 0;
       0 1 g 0 0 0 0 0;
       0 If 0 0 0 0 0 0;
       0 0 0 0 0 1 0 0; 
       0 0 0 0 0 0 1 0; 
       0 0 0 0 0 0 0 1; 
      zeros(1,8)];
  
  AMin = [ 0 1 0 0 0 0 0 0;
        0 If 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0; 
        0 0 0 0 0 0 1 0; 
        0 0 0 0 0 0 0 1; 
        zeros(3,8)];

  
  BMin = [zeros(1,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz];
  
outputsLocal = {'phi'; 'theta';'psi';'ze'};
ClocalMin = [  
            0 0 1 0 0 0 0 0; 
            0 0 0 1 0 0 0 0; 
            0 0 0 0 1 0 0 0;
            1 0 0 0 0 0 0 0];
        
statesMin = {'ze','vze','phi','theta','psi','wxb','wyb','wzb'};

tenzoMin=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);

tenzoRetro=ss(AMin,BMin,ClocalMin,D,'statename',statesMin,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro)

% Check del sottosistema ragg e oss con decomposizione di Kalman
[sysr,U] = minreal(tenzoMin,0.1)

KalmanA = U*AMin*U'
KalmanB =U*BMin
KalmanC =ClocalMin*U'

% Verifica Raggiungibilità
disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
disp(rank(ctrb(AMin,BMin)));


disp('Verifica osservabilità. Rango della matrice di osservabilità:')
disp(rank(obsv(AMin,ClocalMin)));

disp('End');
pause();