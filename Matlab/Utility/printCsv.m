fileID = fopen('~/acc11Raw.csv');
C = textscan(fileID, '%s %f %f %f %d %s','delimiter', ',', 'EmptyValue', -Inf)


N=200;

acc.X = zeros(N,1);
acc.Y = zeros(N,1);
acc.Z = zeros(N,1);
acc.t = zeros(N,1);

for i=1:N
    if strcmp(C{1}(1),'A')
       acc.X(i) = double(C{2}(i)); 
       acc.Y(i) = double(C{3}(i)); 
       acc.Z(i) = double(C{4}(i));
       acc.t(i) = C{5}(i);
    end
end
figure(1)
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % bottom subplot
ax3 = subplot(3,1,3); % bottom subplot

plot(ax1,acc.t,acc.X)
title(ax1,'Acc along X axis');
ylabel(ax1,'Raw X');
xlabel(ax1,'Sample');
grid on

plot(ax2,acc.t,acc.Y)
title(ax2,'Acc along Y axis');
ylabel(ax2,'Raw Y');
xlabel(ax2,'Sample');
grid on

plot(ax3,acc.t,acc.Z)
title(ax3,'Acc along Z axis');
ylabel(ax3,'Raw Z');
xlabel(ax3,'Sample');

figure(2)
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % bottom subplot
ax3 = subplot(3,1,3); % bottom subplot


maxX = max(acc.X)
minX = min(acc.X)
tolX = abs(maxX-minX)/2
plot(ax1,acc.t,unwrap(acc.X,tolX))
title(ax1,'Acc along X axis');
ylabel(ax1,'Raw X');
xlabel(ax1,'Sample');


maxY = max(acc.Y)
minY = min(acc.Y)
tolY = abs(maxY-minY)/2
plot(ax2,acc.t,unwrap(acc.Y,tolY))
title(ax2,'Acc along Y axis');
ylabel(ax2,'Raw Y');
xlabel(ax2,'Sample');


maxZ = max(acc.Z)
minZ = min(acc.Z)
tolZ = abs(maxZ-minZ)/2
plot(ax3,acc.t,unwrap(acc.Z,tolZ))
title(ax3,'Acc along Z axis');
ylabel(ax3,'Raw Z');
xlabel(ax3,'Sample');