% Sctipt to test voronoisphere
% Random data
n = 1000;
xyz = randn(3,n);
xyz = bsxfun(@rdivide, xyz, sqrt(sum(xyz.^2,1)));

[P, K, voronoiboundary] = voronoisphere(xyz);

%% Graphic
f = figure(1);
clf(f);
set(f,'Renderer','zbuffer');
ax = axes('Parent', f);
hold(ax, 'on');
axis(ax,'equal');

plot3(ax, xyz(1,:),xyz(2,:),xyz(3,:),'ro');
clmap = cool();
ncl = size(clmap,1);
for k = 1:n
    X = voronoiboundary{k};
    cl = clmap(mod(k,ncl)+1,:);
    fill3(X(1,:),X(2,:),X(3,:),cl,'Parent',ax,'EdgeColor','w');
end
grid on
axis(ax,'equal');
axis(ax,[-1 1 -1 1 -1 1]);

f = figure(2);
clf(f);
set(f,'Renderer','zbuffer');
ax = axes('Parent', f);
hold(ax, 'on');
%input('Type <CR>: ', 's');
cla(ax);
plot3(ax, P(1,:),P(2,:),P(3,:), 'ro');
for k = 1:n
    V = P(:,K{k});
    V = V(:,[1:end 1]);
    for i=1:length(V)-1
        plot3(ax, V(1,i:i+1),V(2,i:i+1),V(3,i:i+1), 'r', 'Linewidth', 1);
    end
end
axis(ax,'equal');
axis(ax,[-1 1 -1 1 -1 1]);
