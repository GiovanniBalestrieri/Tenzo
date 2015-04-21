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
      zeros(3,12)]

B = [zeros(5,4);
    1/mq 0 0 0; 
    zeros(3,4);
    0 1/Ixx 0 0;
    0 0 1/Iyy 0;
    0 0 0 1/Izz]
 
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
%modello_tf=tf(tenzo)

pause;
%% Eigenvalues of the system

disp('specifica 1) Verifichare che esiste un cotrollore che soddisfa le specifiche A1,B,C1 (teorema 5.4.2):');
disp('verifica preliminare, autovalori del processo [eig(A)]:')
stab=1;
eOp = eig(A);
[dn,dm]=size(eOp);
moltZero = 0;
for i=1:dn,
  if (real(eOp(i))>0) 
      stab=0; 
      disp('elemento a parte reale positiva:');
      disp(eOp(i)); 
  elseif (real(eOp(i))==0 && moltZero == 0) 
      moltZero =+ 1;
  elseif (real(eOp(i))==0 && moltZero > 0)
      stab=2;
  end
end
if (stab==0) disp('Sistema instabile! Gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
if (stab==1) disp('Sistema stabile! OLE!! Gli autovalori a ciclo aperto sono: [comando eig(A)]'); end
if (stab==2) disp('Sistema instabile! Sono presenti autovalori pari a zero con molteplicità > 1'); end
disp(eOp);

%% Analisi risposta a gradino
tenzoRetro=ss(A,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);


%% Proprietà strutturali:
% Verifica Raggiungibilità e Osservabilità

disp('Verifica raggiungibilà: rank([A-gI,B]) : per tutti g € spec(A)')
if (rank(ctrb(A,B))==size(A,1))
    disp('Sistema raggiungibile');
    disp(rank(ctrb(A,B)));
else
    disp('Sistema Irraggiungibile');
end


disp('Verifica osservabilità. Rango della matrice di osservabilità:')
if (rank(obsv(A,Clocal))==size(A,1))
    disp('Sistema controllabile');
    disp(rank(ctrb(A,B)));
else    
    disp('Sistema Non controllabile');
end
disp(rank(obsv(A,Clocal)));

pause();

%% Stabilizzazione mediante retroazione dallo stato
disp('Stabilizzazione mediante retroazione dallo stato');

poles = [-1 -2 -3 -4 -5 -6 -7 -8 -9 -10 -11 -11];
K11=place(A,B,poles);
K11=0*K11;
disp('Eigenvalues of the closed loop sys');
eig(A-B*K11)


tenzoRetro=ss(A-B*K11,B,Clocal,D,'statename',states,'inputname',inputs,'outputname',outputsLocal);
step(tenzoRetro);


% disp('Transfer matrix of the model')
% modello_tf=tf(tenzoRetro)

pause;
%% Proprietà strutturali sys Ciclo Chiuso

if (rank(ctrb(A-B*K11,B))==size(A)) 
    disp('a1) verificata, la coppia A,B � raggiungibile, rank(matrice controllabilit� �)');
    disp(rank(ctrb(A-B*K11,B))); 
end

% if (rank(obsv(A,CFull))==size(A)) 
%     disp('a1) verificata, la coppia A,C � osservabile, rank(matrice osservabilit� �)'); 
%     disp(rank(obsv(A,CFull))); 
% end

if (rank(obsv(A-B*K11,Clocal))==size(A)) 
    disp('a1) verificata, la coppia A,C � osservabile, rank(matrice osservabilit� �)'); 
    disp(rank(obsv(A-B*K11,Clocal))); 
else
    disp('b1) ATTENZIONE Verifica fallita!!');    
    disp(rank(obsv(A-B*K11,Clocal))); 
    disp(size(A,1));
end


% figure
% sigma(tenzoRetro)
% grid on

% verifica minimum phase
% disp('Transmission zeros')
% tzero(tenzoRetro)


disp('Premere un tasto per continuare...');
pause;


clc;

%% Astatismo

alpha=0;
omega=4;
%f=omega/(2*pi);
gamma1=1;
k1=1;
h1=1;
h2=1;
gamma2=complex(0,omega);
disp('definizione segnali esogeni');
disp('gamma1='); disp(gamma1);
disp('gamma2='); disp(gamma2);

disp('Verifica condizione di astatismo:');

R1=[ A-B*K11-alpha*eye(size(A)) B ; Clocal D];
disp('Rank R1:');
disp(rank(R1));
if (rank(R1)==size(A,1)+size(Clocal,1))  
    disp('b) verificata ,rango della matrice 5.4.23 per gamma1 �:');
    disp(rank(R1));
else
    disp('test fallito per gamma1');
end
R2=[ A-B*K11-gamma2*eye(size(A)) B ; Clocal D];
disp('Rank R2:');
disp(rank(R2)); 
if (rank(R2)==size(A,1)+size(Clocal,1)) 
    disp('b) verificata ,rango della matrice 5.4.23 per gamma2 �:');
    disp(rank(R2)); 
else
    disp('Test Fallito per gamma2');
end

disp('Premere un tasto per continuare...');
pause;
clc;
sys = 'tenzo1_05';
open_system(sys)
SimOut = sim(sys);