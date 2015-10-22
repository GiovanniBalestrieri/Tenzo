%% Esempio di stabilizzazione in C_buono mediante LQR/LQG
clear all;
clc;

disp('Definisco i dati del problema: sistema a tempo continuo con');
A=[0 1 0;0 0 1;1 2 2]
B=[0 0; 0 1;1 0]
C=[1 1 1;0 1 0]
D=zeros(2,2)

disp('Premere un tasto per continuare...');
pause;


%% Verifiche preliminari
clc;
disp('Sistema instabile: gli autovalori a ciclo aperto sono: [comando eig(A)]')
eOp = eig(A);
disp(eOp);
disp('Verifica raggiungibiltï¿½: la matrice di raggiungibiltï¿½ ha rango: [comando rank(ctrb(A,B))]')
disp(rank(ctrb(A,B)));
disp('Verifica osservabiltï¿½: la matrice di osservabiltï¿½ ha rango: [comando rank(obsv(A,C))]')
disp(rank(obsv(A,C)));

disp('Premere un tasto per continuare...')
pause;


%% Calcolo di un compensatore stabilizzante (1/3)
clc;
%Calcolo retroazione dallo stato
disp('Uso la funzione lqr per calcolare il guadagno K (la retroazione dallo stato u=Kx)')
Q=eye(3)
R=eye(2)
K = lqr(A,B,Q,R);
disp('Matrice di guadagno K: [comando K = lqr(A,B,Q,R)]');
disp(K);
disp('Autovalori a ciclo chiuso: [comando eig(A-B*K)]');
eK = eig(A-B*K);
disp(eK);
disp('[Nota: retroazione negativa!]')

disp('Premere un tasto per continuare...')
pause;

%% Calcolo di un compensatore stabilizzante (2/3)
clc;
%Calcolo guadagno del filtro di Kalman
disp('Uso la funzione lqr per calcolare il guadagno L del filtro di Kalman')
W=eye(3)
V=eye(2)
L= lqr(A',C',W,V)';
disp('Matrice di guadagno L: [comando L = lqr(A'',C'',W,V)'']');
disp(L);
disp('Autovalori del filtro di Kalman: [comando eig(A-L*C)]');
eL = eig(A-L*C);
disp(eL);

disp('Premere un tasto per continuare...')
pause;

%% Calcolo di un compensatore stabilizzante (3/3)
clc;
%Costruisco il regolatore
disp('Costruisco il regolatore come filtro di Kalman + retroazione dallo stato stimato')
Ac = A-B*K-L*C+L*D*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));
disp('Ac = A-B*K-L*C+L*D*K');
disp(Ac);
disp('Bc = L, Cc = -K, Dc = zeros(size(K,1),size(L,2))')
disp('Autovalori a ciclo chiuso:');
sistema = ss(A,B,C,D);
controllore = ss(Ac,Bc,Cc,Dc);
CicloChiuso = feedback(series(controllore, sistema),eye(2), 1); %Retroazione positiva
[Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
eCl = eig(Acl);
disp(eCl);

disp('La figura mostra la risposta al gradino del sistema di controllo');

step(CicloChiuso);
disp('Premere un tasto per continuare...')
pause;

%% Provo una sintesi piu' veloce modificando i pesi
clc;
disp('Cambio i pesi per rendere piu'' rapida la convergenza')
disp('riducendo il peso relativo di R=V rispetto a Q=W')
Q=eye(3)
R=0.001*eye(2)
W=eye(3);
V=0.001*eye(2);
K = lqr(A,B,Q,R);
L= lqr(A',C',W,V)';
disp('Matrici di guadagno K ed L:');
disp(K);
disp(L);
eK1 = eig(A-B*K);
eL1 = eig(A-L*C);
disp('Autovalori a ciclo chiuso con le due sintesi:');
disp('   K                 K1                 L                 L1:');
disp([eK eK1 eL eL1]);

Ac = A-B*K-L*C+L*D*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));

sistema = ss(A,B,C,D);
controllore1 = ss(Ac,Bc,Cc,Dc);
CicloChiuso1 = feedback(series(controllore1, sistema),eye(2), 1); %Retroazione positiva
% [Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
% eCl = eig(Acl);
% disp(eCl);

disp('La figura mostra la risposta al gradino del nuovo sistema di controllo');

hold on;
step(CicloChiuso1);

disp('Premere un tasto per continuare...')
pause;

%% Motivazione della traslazione
clc;
disp('La seconda sintesi ha reso la convergenza piu'' rapida ma ha ');
disp('modificato solo di poco gli autovalori a ciclo chiuso piu'' lenti!');
disp(' ');
disp('Parte reale degli autovalori a ciclo chiuso con le due sintesi:');
disp('   K        K1         L         L1:');
disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eL),'descend') sort(real(eL1),'descend')]);

disp('Per rendere tutti gli autovalori a ciclo chiuso piu'' rapidi');
disp('(uniformemente), posso usare la traslazione');
disp('[comando K=lqr(A+alfa*I,B,Q,R) con alfa > 0]');
disp('che garantisce che la parte reale di eig(A-B*K)<-alfa ');

disp(' ');
disp(' ');
disp('Premere un tasto per continuare...');
pause;

%% Provo una sintesi piu' veloce aggiungendo un multiplo dell'identita' alla A
clc;
disp('Con i pesi originali, rendo piu'' rapida la convergenza')
disp('traslando la A con una traslazione alfa=5')
Q=eye(3)
R=eye(2)
W=eye(3);
V=eye(2);
alfa=5;
K = lqr(A+alfa*eye(size(A)),B,Q,R);
L= lqr(A'+alfa*eye(size(A)),C',W,V)';
disp('Matrici di guadagno K ed L:');
disp(K);
disp(L);
disp('Autovalori a ciclo chiuso:');
eK2 = eig(A-B*K);
eL2 = eig(A-L*C);
disp('Parte reale degli autovalori a ciclo chiuso con le tre sintesi:');
disp('   K        K1         K2        L         L1         L2:');
disp([sort(real(eK),'descend') sort(real(eK1),'descend') sort(real(eK2),'descend') sort(real(eL),'descend') sort(real(eL1),'descend') sort(real(eL2),'descend')]);

Ac = A-B*K-L*C+L*D*K;
Bc = L;
Cc = -K;
Dc = zeros(size(K,1),size(L,2));

disp('Autovalori a ciclo chiuso:');
sistema = ss(A,B,C,D);
controllore2 = ss(Ac,Bc,Cc,Dc);
CicloChiuso2 = feedback(series(controllore2, sistema),eye(2), 1); %Retroazione positiva
% [Acl,Bcl,Ccl,Dcl] = ssdata(CicloChiuso);
% eCl = eig(Acl);
% disp(eCl);

step(CicloChiuso2);

