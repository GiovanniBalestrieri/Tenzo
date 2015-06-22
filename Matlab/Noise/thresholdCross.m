XMin = -10;
XMax = 10;
YMin = -10;
YMax = 10;
addpath('Reference/c2xyz/');
addpath('Reference/export/');

%% Random Radial Basis functions in space 
disp('3D case with random path - 10 rbf:');
N = 10 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.3;1.5]
variances = 0.3 + (1.5-0.3).*rand(N,1);
% random amplitude [0.1;1]
amplitudes = 0.1 + (1-0.1).*rand(N,1);
% Random Xcenters in [-XMin;xMax]
Xcenters = XMin+ (XMax-XMin).*rand(N,1);
Ycenters = YMin+ (YMax-YMin).*rand(N,1);

esp=zeros(N,1);
esp=1./(2*(variances).^2);
for i=1:1:N
    disp('step:')
    disp(i)
    Xci=Xcenters(i,1)*ones((XMax-XMin)/stepMesh+1,((XMax-XMin)/stepMesh+1)*2);
    Yci=Ycenters(i,1)*ones((YMax-YMin)/stepMesh+1,((YMax-YMin)/stepMesh+1)*2);
    disp('Radial Basis Function: [amplitude,variance,center]');
    disp(amplitudes(i,1))
    disp(variances(i,1))
    disp(Xcenters(i,1))
    disp(Ycenters(i,1))
    Z = Z + 1*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
figure(1);
surf(X,Y,Z)

%% Post elaboration

figure(2);
grid on
axis equal
axis tight
contour(X,Y,Z);
cl = contour(X,Y,Z);
[x1,y1,z1] = C2xyz(cl);

% countours at 0.8
h=[0.8,0.8]
contour(X,Y,Z,h)


figure(3);
hold on; % Allows plotting atop the preexisting peaks plot. 
threshold = 0.8;
sector.numberOfZones = 1;
% analyze all the level curves
for n = find(z1==threshold); 
   n
   %xTh(1,n) = x1{n};
   %yTh(1,n) = y1{n};
   hold on;
   plot(x1{n},y1{n},'k-','linewidth',3);
   sector.zones(sector.numberOfZones).x(1,1:size(x1{n},2)) = x1{n}(1,:);
   sector.zones(sector.numberOfZones).y(1,1:size(x1{n},2)) = y1{n}(1,:);
   sector.numberOfZones = sector.numberOfZones + 1;
end
disp('Number of zones founded:');
disp(sector.numberOfZones);


%% Convert to BW

%Remove frames
set(gca, 'visible', 'off')
set(gcf, 'color', 'w');

%%// Get the figure as a uint8 variable
im = export_fig;

%//  Output binary image
BW = ~im2bw(uint8(255.*im2double(im)),0.99);

%%// Remove
BW = bwmorph(BW,'skel',Inf);
figure(4);
imshow(BW)
% Save it
saveas(4, 'sectors1', 'png');

%% Centroids

s = regionprops(BW,'centroid');
centroids = cat(1, s.Centroid);
imshow(BW)
hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off

stats = regionprops(BW,'Centroid',...
    'MajorAxisLength','MinorAxisLength','Extrema','BoundingBox','ConvexArea','ConvexHull','ConvexImage')

% just a test to see the results of regionprops
% centers = stats.Centroid;
% diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
% radii = diameters/2;

figure(4)
subplot(1,3,1);
%BW = imread('sectors1.png');
imshow(BW);
title('Binary');
    
% subplot(2,2,2);
% BW1 = BW > 10;
% imshow(BW1);
% title('Binary');
    
subplot(1,3,2);
CH = bwconvhull(BW);
imshow(CH);
title('Union Convex Hull');
    
subplot(1,3,3);
CH_objects = bwconvhull(BW,'objects',8);
imshow(CH_objects);
title('Objects Convex Hull');