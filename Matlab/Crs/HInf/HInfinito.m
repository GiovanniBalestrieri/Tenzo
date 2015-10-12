%% Esempio di progetto Hinfty per robustezza
clear all;
clc;

%Carica il modello di due massa-molla-smorzatore connessi in serie
%e il modello delle incerteze sull'attuatore
%Modello di due massa-molla-smorzatore connessi in serie

m1=1;       %Massa
m2=1;
k1=0;       %Costante elastica
k2=0;
b1=1;       %Attrito viscoso
b2=1;
a1=1;       %Guadagno statico dell'attuatore
a2=1;

%% Definizione matrici del modello
A=[zeros(2,2) eye(2); -(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1;...
    k2/m2 -k2/m2 b1/m2 -b2/m2];
B=[zeros(2,2); a1/m1 0;0 a2/m2];
C=[eye(2) zeros(2,2)];
D=zeros(2,2);

modello_ss=ss(A,B,C,D);
disp([A,B;C,D])

%% Eigenvalues of matrix A
disp(eig(A))

%% Model transfer matrix
modello_tf=tf(modello_ss)

%% Verifica di stabilizzabilita'
disp(ctrb(A,B))
disp(rank(ctrb(A,B)))

%% Verifica di rilevabilita'
disp(ctrb(A',C'))
disp(rank(ctrb(A',C')))

%% Verifica zeri di trasmissione
tzero(modello_tf)

% valori singolari 
figure
sigma(modello_tf,'o-')
grid on

%% definizione dei bounds
gamma1 = 1;
gamma2 = 1;
%gamma3 = 1;  
gamma3 = 1/1.3; 
ps = gamma1*tf(1,[1 1])
la = gamma2*0.1*tf([0.5 1],[5 1])
lm = gamma3*tf([1 1], 1.1*[0.01, 1]);

%%
% ************* NOTA IMPORTANTE ***************
% Se gamma3 viene lasciato a 1 e non a 1/1.3, il controllore generato 
% dalla routine matlab ritorna un "GAM = 1.4" e non con "GAM <= 1", come
% dovrebbe essere. Subito dopo vengono generati i grafici dei bounds con 
% le funzioni di trasferimento relative 
% e si puo' fa apprezzare agli studenti che uno
% solo dei tre grafici "non rispetta" il bound, e precisamente quello delle
% variazioni moltiplicative lm(s). Allascando quindi solo questo 
% bound (e non tutti!),
% dividendo per 1.3 gamma3, e facendo rigirare tutta la procedura, il 
% controllore generato rispetta tutti i bounds, 
% come si vede dai nuovi grafici...
% *********************************************

%%
figure
sigma(lm,'ro-')
grid on
hold on
sigma(la,'go-')
sigma(ps,'bo-')
legend('lm','la','ps')

%% definizione di Cb
%epsilon = 0;
epsilon = 0.2;

% ************* NOTA IMPORTANTE ***************
% Se epsilon viene messo a zero, la verifica delle condizioni per 
% il funzionamento della routine dell'Hinfinito trova che una delle
% condizioni non e' rispettata (precisamente uno dei due casi dove si
% controlla che non ci sia uno zero di trasmissione sul percorso di Nyquist. 
% Questo fatto puo' essere utile per spiegare l'importanza della verifica 
% delle condizioni: 
% 1. si verificano le condizioni e una di queste fallisce,
% 2. si lancia comunque la routine per la generazione del controllore, 
% 3. la routine torna errore!. 
% Si puo' allora tornare indietro e cambiare il parametro
% epsilon. Mettendolo a 0.2 si fa la stabilizzazione in Cb 
% e non ci sono zeri sul percorso di Nyquist.
% *******************************************

%% definizione del Plant in Cb
plant = ss(A+epsilon*eye(length(A)),B,C,D)
eig(plant.a)

%% calcolo matrici W e sistema aumentato
W1 = ps*eye(2);
W2 = la*eye(2);
W3 = lm*eye(2);

P = augw(plant,W1,W2,W3);

%% calcolo (selezione) matrici sistema esteso 

[rB,cB] = size(P.B);
B1 = P.B(:,1:cB-2);
B2 = P.B(:,cB-1:cB);

[rC,cC] = size(P.C);
C1 = P.C(1:rC-2,:);
C2 = P.C(rC-1:rC,:);

D11 = P.D(1:rC-2,1:cB-2);
D12 = P.D(1:rC-2,cB-1:cB);
D21 = P.D(rC-1:rC,1:cB-2);
D22 = P.D(rC-1:rC,cB-1:cB);

%% Verifiche condizioni applicabilita' Hinf

%D22 = 0
D22

%D11 = 0 --> W1 strictly proper 
D11
figure;
sigma(W1,'o-');
title('Singular values of W1');
grid on;

%rg(D12) pieno colonna
D12
rank(D12)

%rg(D21) pieno riga
D21
rank(D21)

%% no null real part zeros of [A-sI,B2; C1, D12]
disp(tzero(ss(P.A,B2,C1,D12)))

%% no null real part zeros of [A-sI,B1; C2, D21]
disp(tzero(ss(P.A,B1,C2,D21)))

%% sintesi del controllore
[K,CL,GAM] = hinfsyn(P);
GAM

% definizione della catena diretta
F0 = series(K,modello_ss);

%% costruzione di S0 e min(poli)
S0 = feedback(eye(2),F0);
min(pole(S0))

%valori singolari
figure
sigma(S0,'ro-')
hold on
grid on
sigma(inv(W1),'go-')
legend('S0','1/W1')

%% costruzione di V0
V0 = feedback(K,modello_ss);

%valori singolari
figure
sigma(V0,'ro-')
hold on
grid on
sigma(inv(W2),'go-')
legend('V0','1/W2')

%% costruzione di T0
T0 = feedback(F0,eye(2));

%valori singolari
figure
sigma(T0,'ro-')
hold on
grid on
sigma(inv(W3),'go-')
legend('T0','1/W3')


%% plot step nominale
figure
step(T0,'r-')
grid on 
hold on
step(S0,'g-')
legend('step(T0)','step(S0)')

%% introduzione delle perturbazioni

pert1 = 0.5*tf([5 1],5)*tf(1,[1 1])*tf([1 1], 1.1*[0.01, 1])*tf(1,[0.01, 1]);
pert2 = 2*pert1
pert3 = 5*tf([1 1], 1.1*[0.01, 1])*tf(1,[0.01, 1]);

%% visualizzazione valori singolari
figure
sigma(W3,'ro-')
grid on
hold on
sigma(pert1,'go-')
sigma(pert2,'bo-')
sigma(pert3,'mo-')
legend('W3','pert1','pert2','pert3')

%% costruzione di Tpert
F1 = series(F0,eye(2)+pert1);
T1 = feedback(F1,eye(2));
F2 = series(F0,eye(2)+pert2);
T2 = feedback(F2,eye(2));
F3 = series(F0,eye(2)+pert3);
T3 = feedback(F3,eye(2));

%% confronto step response 
figure
step(T0,'ro-')
grid on 
hold on
step(T1,'go-')
step(T2,'bo-')
legend('step(T0)','step(T1)','step(T2)')

%% Confronto poli
pole(T1)
pole(T2)
pole(T3)


%% valori singolari
figure
sigma(T0,'ro-')
hold on
grid on
sigma(inv(W3),'go-')
legend('T0','1/W3')