function [point,bounds] = GEcamera(ge,focusPoint,camera,speed)
% Change GoogleEarth camera position and orientation.
%
% This will set the Google Earth camera parameters and fly the 
% camera to the specified camera view at the specified speed. 
% The camera view is defined by a focus point, (what the camera
% is looking at) and the camera offset, tilt and azimuth.
%
% The Google Earth server structure (ge) returned from the 
% GEserver command is needed. 
%
% [point] = GEcamera(ge) will return the focus point and view 
% bounds of the current camera position. point is a 3 element 
% vector of latitude longitude and altitude where altitude is 
% the local terrain level. If the focus point is not on the 
% terrain then the projected point to the globe's horizon is 
% returned. If the terrain is not active in Google Earth then 
% zero is returned.
%
% [point, bounds] = GEcamera(ge) will return the view bounds 
% of the current camera position as a minimum and maximum
% latitude and longitude value.
%
% [point, bounds] = GEcamera(ge,focusPoint) moves the camera so 
% that it is looking at the focus point defined. The focus point
% can be specified as either a 2 element vector of latitude and 
% longitude or a 3 element vector including altitude. If altitude
% is zero or negative then the value will be interpreted as above 
% ground level, otherwise altitude will be above mean sea level.
%
% [point, bounds] = GEcamera(ge,focusPoint,camera) sets the camera
% offset, tilt and azimuth. Offset is measured as meters from the
% focus point, tilt is in degrees where 0 is a view straight down 
% and 90 is horizontal. Azimuth is in degrees where 0 is North.
%
% [point, bounds] = GEcamera(ge,focusPoint,camera,speed) sets the 
% speed at which the camera will fly to the defined point. Values 
% can be from 0 to 5 where 5 is instantaneous. The default is 2. 
%
% Examples:
%  ge = GEserver;
%  [point, bounds] = GEcamera(ge)
%  GEcamera(ge,[-25.7450 28.2788])
%  GEcamera(ge,[-25.7 28.3 1500])
%  GEcamera(ge,[-25.7 28.3 1500],[100 60 90])
%  [point] = GEcamera(ge,[-33.85 18.48 500],[3000 80 -155],5)
%
% See also: GEserver, GEaddKmlFeature, actxserver

if (nargin<2),  
  moveCamera = 0;  
else
  moveCamera = 1;
  if (length(focusPoint)<=2),
    focusPoint(3) = 0;
  end;
  if (length(focusPoint)==3),
    if (focusPoint(3)<=0),
      focusPoint(3) = abs(focusPoint(3));
      altmode = 1; % RelativeToGroundAltitudeGE
    else
      altmode = 2; % AbsoluteAltitudeGE
    end;
  else
    altmode = focusPoint(4);
  end;
end;  
% if no camera parameters set then use current camera
cam  = ge.GetCamera(1);
if (nargin<3),  
  camera = [cam.Range cam.Tilt cam.Azimuth];
end;  
if (nargin<4),  speed = 2;  end;  

if (moveCamera),
  % set camera parameters
  cam.FocusPointLatitude = focusPoint(1); % Focus point Latitude -90 (South Pole) and 90 (North Pole).
  cam.FocusPointLongitude = focusPoint(2);% Focus point Longitude -180 and 180, 0 being the longitude of the Prime Meridian
  cam.FocusPointAltitude = focusPoint(3); % See altitude mode
  cam.FocusPointAltitudeMode = altmode;   % mode : 1 = RelativeToGroundAltitudeGE, 2 =  AbsoluteAltitudeGE
  cam.Range = camera(1);                  % Distance between camera and focus point in meters.
  cam.Tilt = camera(2);                   % 0-90 deg. A tilt angle of 0 means the camera is directly on top of the focus point and looking straight down.
  cam.Azimuth = camera(3);                % Camera's azimuth in degrees (between -180 and 180). 0 means North.
  % activate camera 
  ge.SetCamera(cam,speed);    % Speed 0-5 (5=instant)
end;

% Get point on terrain information
if (nargout>=1),
  point_raw = ge.GetPointOnTerrainFromScreenCoords(0,0).get;
  point = [point_raw.Latitude point_raw.Longitude point_raw.Altitude];
end;  

% Get information about the screen bounds
if (nargout>=2),
  for jj = 1:2,
    for ii = 1:2,
      tmp = ge.GetPointOnTerrainFromScreenCoords(2*ii-3,3-2*jj).get;
      bounds_raw.Latitude(jj,ii) = tmp.Latitude;
      bounds_raw.Longitude(jj,ii) = tmp.Longitude;
      bounds_raw.Altitude(jj,ii) = tmp.Altitude;
      bounds_raw.ProjectedOntoGlobe(jj,ii) = tmp.ProjectedOntoGlobe;
      bounds_raw.ZeroElevationExaggeration(jj,ii) = tmp.ZeroElevationExaggeration;
    end;
  end;
  bounds = [min(bounds_raw.Latitude(:)) min(bounds_raw.Longitude(:)); max(bounds_raw.Latitude(:)) max(bounds_raw.Longitude(:))];
end;
