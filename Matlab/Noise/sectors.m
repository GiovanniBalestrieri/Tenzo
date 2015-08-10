clc 
clear all

XMin = -10;
XMax = 10;
YMin = -10;
YMax = 10;

MaxAmpli = 18;
MinAmpli = 3;

addpath('Reference/c2xyz/');
addpath('Reference/export/');

%% Random Radial Basis functions in space 
disp('3D case with random path - 10 rbf:');
N = 20 % number of random functions
stepMesh = 0.1;
Z = zeros((XMax-XMin)/stepMesh+1,(XMax-XMin)/stepMesh+1);
[X,Y] = meshgrid(XMin:stepMesh:XMax);
% random variance in [a;b] = [0.2;1.5]
variances = 0.2 + (1.5-0.2).*rand(N,1);
% random amplitude 
amplitudes = MinAmpli + (MaxAmpli-MinAmpli).*rand(N,1);
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
    Z = Z + amplitudes(i,1)*exp(-((X-Xci(:,1:((XMax-XMin)/stepMesh+1))).^2+(Y-Yci(:,((XMax-XMin)/stepMesh+2):((YMax-YMin)/stepMesh+1)*2)).^2)*esp(i,1).^2);
end
figure(1);
start = surf(X,Y,Z,'LineStyle','none','FaceLighting','phong');

%% Analyze data -  Post elaboration 
% Contour[Required]
% 
% figure(2)
% grid on
% axis equal
% % axis tight
% % contour(X,Y,Z);
figure(2)
 cl = contour(X,Y,Z,100);
[x1,y1,z1] = C2xyz(cl);
% 
% % countours at 0.8
% h=[0.8,0.8]
% contour(X,Y,Z,h)
%% isaolate regions of interest
% Same ad the commented code above + more options
figure(3)
hold on; % Allows plotting atop the preexisting peaks plot. 
threshold = MaxAmpli*0.6;
sector.numberOfZones = 1;
% analyze all the level curves
for n = find(floor(z1)==floor(threshold)); 
   %n
   %xTh(1,n) = x1{n};
   %yTh(1,n) = y1{n};
   hold on;
   plot(x1{n},y1{n},'k-','linewidth',3);
   sector.zones(sector.numberOfZones).x(1,1:size(x1{n},2)) = x1{n}(1,:);
   sector.zones(sector.numberOfZones).y(1,1:size(x1{n},2)) = y1{n}(1,:);
   sector.numberOfZones = sector.numberOfZones + 1;
end
title('Contours of interest');
hold off
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
title('BW sectors');
% Save it
saveas(4, 'sectors1', 'png');

%% Centroids

stats = regionprops(BW,'Centroid','MajorAxisLength','MinorAxisLength',...
    'Orientation','Eccentricity','Extrema','BoundingBox','ConvexArea',...
    'ConvexHull','ConvexImage')

% Plot centroids in negative image
centroids = cat(1, stats.Centroid);
imshow(BW)
hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off

% Plot centroids in convex sectors
h = figure(5)
CH_objects = bwconvhull(BW,'objects',8);
    imshow(CH_objects);
hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off
title('Objects Convex Hull + centroids');
    
% Plot elliptic approximation

%imshow(BW)
hold on

phi = linspace(0,2*pi,50);
cosphi = cos(phi);
sinphi = sin(phi);

for k = 1:length(stats)
    xbar = stats(k).Centroid(1);
    ybar = stats(k).Centroid(2);

    a = stats(k).MajorAxisLength/2;
    b = stats(k).MinorAxisLength/2;

    theta = pi*stats(k).Orientation/180;
    R = [ cos(theta)   sin(theta)
         -sin(theta)   cos(theta)];

    xy = [a*cosphi; b*sinphi];
    xy = R*xy;

    x = xy(1,:) + xbar;
    y = xy(2,:) + ybar;

    plot(x,y,'r','LineWidth',2);

end
hold off

% Plot and save extrema
figure(5)
hold on
for secteur = 1:size(stats,1)    
    for i = 1:8
        plot(stats(secteur,1).Extrema(i,1),stats(secteur,1).Extrema(i,2),'gO')
        hold on
    end
    hold on
end


%% Scaling sectors up by a factor of K

% k=10
% %figure(9)
% % For each sector
% figure(4)
% for i=1:sector.numberOfZones-3
%     
%     % for each point of the sector
%     for j=1:size(sector.zones(i).x,2)-2    
%         
%         plot(sector.zones(i).x(1,j),sector.zones(i).y(1,j),'-k','LineWidth',5)
%         v=[sector.zones(i).x(1,j) - stats(i).Centroid(1); sector.zones(i).y(1,j) - stats(i).Centroid(2)];
%         sector.zone(i).x(1,j) = sector.zones(i).x(1,j) + k*v(1) ;
%         sector.zone(i).y(1,j) = sector.zones(i).y(1,j) + k*v(2) ;
%         hold on
%         %plot(sector.zone(i).x(1,j),sector.zone(i).y(1,j),'-r','LineWidth',3)
%     end
%     
%     %plot(centroids(i,1),centroids(i,2), 'r*')
%     %hold on
% end
% hold off


%% Plot bounding box
% figure(5)
% hold on
% for secteur = 1:size(stats,1)    
%         rectangle('Position',stats(secteur,1).BoundingBox,'EdgeColor','r','LineWidth',10,'LineStyle','--');
% end

%% Plots Global convexHull figure

% figure(6)
% title('Union Convex Hull');
% CH = bwconvhull(BW);
% imshow(CH);

%% Calculate sectors

set(0,'CurrentFigure',h);
voronoi(centroids(:,1),centroids(:,2))
