clc

a = load('errorTenzo.mat')

figure(11)
l = size(a.IdeDataEstRoll,1)
l = size(a.IdeDataRoll,1)

size(a.IdeDataTimeTenzo)
size(a.IdeDataEstRoll)

%t=linspace(a.IdeDataEstRoll);%a.IdeDataTimeVitruvio,size(a.IdeDataRoll,1));
size(t)
plot(a.IdeDataTimeTenzo,a.IdeDataEstRoll);
figure(13)
plot(a.IdeDataTimeVitruvio,a.IdeDataRoll);

figure(12)
plot(a.IdeDataTimeTenzo,a.IdeDataDelta);