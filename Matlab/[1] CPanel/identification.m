clc

a = load('ideTest1.mat')
a = load('errorTenzo.mat')

figure(11)
plot(a.IdeDataTimeTenzo,a.IdeDataEstRoll);

figure(13)
plot(a.IdeDataTimeVitruvio,a.IdeDataRoll);

figure(12)
plot(a.IdeDataTimeTenzo,a.IdeDataDelta);