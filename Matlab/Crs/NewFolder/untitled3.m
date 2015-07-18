%% Nyquist plot
figure(1); clf reset
G=tf(3,[1 3 2])
nyquist(G)

%% Different Plant tf and Nyquist plots
figure(2);

G1 = tf(3*[3 1],[1 3 2]);
G2 = tf(3*[2/3 1],[1 3 2]);
G3 = tf(3*[0.1 1],[1 3 2]);

bode(G1,G2,G3);
nyquist(G1,G2,G3)
a = legend('1','2','3')

