%% Esempio di deduzione di bounds moltiplicativi lm

clear all;
clc;
close all;

%Carica il modello di due massa-molla-smorzatore connessi in serie
%e il modello delle incerteze sull'attuatore
%Modello di due massa-molla-smorzatore connessi in serie
%

m1=1;       %Massa
m2=1;
a1=1;       %Guadagno statico dell'attuatore
a2=1;

% k1 = ureal('k1',0.5,'Range',[0.1 0.9]); %Costante elastica
% k2 = ureal('k2',0.5,'Range',[0.1 0.9]); 
% b1 = ureal('b1',1,'Plusminus',.2); %Attrito viscoso
% b2 = ureal('b2',1,'Plusminus',.2); 

k1 = 0.1; %Costante elastica
k2 = ureal('k2',0.1,'Range',[0.0 0.9]);
b1 = .1; %Attrito viscoso
b2 = ureal('b2',.1,'Plusminus',.05);

A=[zeros(2,2) eye(2); -(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1; k2/m2 -k2/m2 b1/m2 -b2/m2];
B=[zeros(2,2); a1/m1 0;0 a2/m2];
C=[eye(2) zeros(2,2)];
D=zeros(2,2);

modello_unc = uss(A,B,C,D);
modello_nominale=modello_unc.NominalValue


%% Prende alcuni campioni del sistema incerto e calcola bound su incertezze 
N=10

for i=1:1:N
sys{i} = usample(modello_unc);
% Additive
deltaA_sys{i} = tf(sys{i}) - tf(modello_nominale);
% Moltiplicative riportate sull'Ingresso
deltaMin_sys{i} = (inv(tf(modello_nominale))) * deltaA_sys{i};
% Moltiplicative riportate sull'Usicta
deltaMout_sys{i} = deltaA_sys{i} * (inv(tf(modello_nominale)));
end

% generates 150 points between decades 10^( -2 ) and 10^( 1 ).
omega = logspace(-2,1,150);

% Plots all bode magnitude of all Ps
% modNomTf = tf(modello_nominale);
% aa = modNomTf(1,1)
% figure
% bodemag(aa)
% 
% bb = modNomTf(1,2)
% figure
% bodemag(bb)
% cc = modNomTf(2,1)
% figure
% bodemag(cc)
% dd = modNomTf(2,2)
% figure
% bodemag(dd)


% Plots the singular values of the frequency response of a model nominale
% specifies the frequency range or frequency points to be used for the plot
temp = sigma(modello_nominale,omega);
max_sig_nom = temp(1,:);

for i=1:1:N
  temp = sigma(sys{i},omega);
  max_sig_unc(i,:) = temp(1,:);
  temp = sigma(deltaA_sys{i},omega);
  max_sig_dA(i,:) = temp(1,:);
  temp = sigma(deltaMin_sys{i},omega);
  max_sig_dMin(i,:) = temp(1,:);
  temp = sigma(deltaMout_sys{i},omega);
  max_sig_dMout(i,:) = temp(1,:);
end

max_sig_unc
% Returns a row vector containing the maximum element from each column.
top_unc = max(max_sig_unc);
top_dA = max(max_sig_dA);
top_dMin = max(max_sig_dMin);
top_dMout = max(max_sig_dMout);
% 
% subplot(211)
% sigma(sys{i},[],1)
% subplot(212)
% sigma(temp,[],2)

%%
figure(1);
semilogx(omega,mag2db(max_sig_nom),'k--','LineWidth',2)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_unc(i,:)),'r:','LineWidth',2)
end
semilogx(omega,mag2db(top_unc),'b','LineWidth',5)
semilogx(omega,mag2db(max_sig_nom),'k','LineWidth',5)
title('Max sing values: Nominal model (black), perturbed models (red), bound (blue)')

%%
figure(2);
semilogx(omega,mag2db(top_dA),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dA(i,:)),'r:','LineWidth',2)
end
title('Max sing values: additive uncertainties (red), bound (blue)')

%%
figure(3);
semilogx(omega,mag2db(top_dMin),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dMin(i,:)),'r:','LineWidth',2)
end
title('Max sing values: input multiplicative uncertainties (red), bound (blue)')

%%
figure(4);
semilogx(omega,mag2db(top_dMout),'b','LineWidth',5)
grid on;
hold on;
for i=1:1:N
  semilogx(omega,mag2db(max_sig_dMout(i,:)),'r:','LineWidth',2)
end
title('Max sing values: output multiplicative uncertainties (red), bound (blue)')

%% Upper bound razionale stabile e fase minima
pre_bound_dA = frd(top_dA,omega);

% fit razionale e min phase per ricavare il bound
ord = 2; %Ordine della funzione di fitting 
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA2 = sigma(bound_dA,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA5 = sigma(bound_dA,omega);
ord = 7; %Ordine della funzione di fitting 
bound_dA = fitmagfrd(pre_bound_dA,ord,[],[],1); 
bb_dA7 = sigma(bound_dA,omega);

figure(5);
semilogx(omega,mag2db(top_dA),'b','LineWidth',2)
grid on;
hold on;
semilogx(omega,mag2db(bb_dA2(1,:)),'r','LineWidth',2)
semilogx(omega,mag2db(bb_dA5(1,:)),'k','LineWidth',2)
semilogx(omega,mag2db(bb_dA7(1,:)),'m','LineWidth',2)
title('Bound on additive uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
  'Location','SouthWest');

%% Upper bound razionale stabile e fase minima
pre_bound_dMin = frd(top_dMin,omega);

% fit razionale e min phase per ricavare il bound
ord = 2; %Ordine della funzione di fitting 
bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
bb_dMin2 = sigma(bound_dM,omega);
ord = 5; %Ordine della funzione di fitting 
bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
bb_dMin5 = sigma(bound_dM,omega);
ord = 7; %Ordine della funzione di fitting 
bound_dM = fitmagfrd(pre_bound_dMin,ord,[],[],1); 
bb_dMin7 = sigma(bound_dM,omega);

figure(6);
semilogx(omega,mag2db(top_dMin),'b','LineWidth',2)
grid on;
hold on;
semilogx(omega,mag2db(bb_dMin2(1,:)),'r','LineWidth',2)
semilogx(omega,mag2db(bb_dMin5(1,:)),'k','LineWidth',2)
semilogx(omega,mag2db(bb_dMin7(1,:)),'m','LineWidth',2)
title('Bound on additive uncertainties');
legend('strict bound', 'Rational stable min phase, order 2',...
  'Rational stable min phase, order 5', 'Rational stable min phase, order 7',...
  'Location','SouthWest');
