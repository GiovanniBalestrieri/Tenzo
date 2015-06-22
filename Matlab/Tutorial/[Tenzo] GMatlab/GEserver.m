function ge_out = GEserver(ge_in)
% Start Google Earth COM server
%
% Creates a local automation server connection to Google Earth 
% and returns links to the IApplicationGE interface.
%
% The interface has the following properties: 
%       StreamingProgressPercentage, AutoPilotSpeed, 
%       VersionMajor, VersionMinor, VersionBuild, 
%       VersionAppType, ElevationExaggeration
% 
% the following sub interfaces:
%         ViewExtents 
%             Properties: North, South, East & West
%      TourController
%             Properties: speed, PauseDelay, Cycles
%             Functions : PlayOrPause, Stop
%    SearchController
%             Functions : Search(searchString), 
%                         IsSearchInProgress = IsSearchInProgress
%                         Results = GetResults
%                         ClearResults 
% AnimationController 
%             Functions : Play, Pause
%             Interfaces: CurrentTimeInterval, SliderTimeInterval
%                         The above two interfaces have the following 
%                         sub interfaces
%             Interfaces: BeginTime & EndTime
%                         and the above two interfaces both have the 
%                         following  
%             Properties: Type, Year, Month, Day, Hour, Minute, 
%                         Second, TimeZone
%             Functions : handle = Clone,  
%                         time = ConvertToZone(timeZone)
%
% and the following functions
%     Camera = GetCamera(useTerrain)
%     SetCamera(camera, speed)
%     SetCameraParams(lat,lon,alt,altMode,range,tilt,azimuth, speed)
%     SaveScreenShot(fileName, quality)
%     OpenKmlFile(fileName, suppressMessages)
%     LoadKmlData(KMLstr)
%     Feature = GetFeatureByName(featureName)
%     Feature = GetFeatureByHref("KMLfileName#id")
%     SetFeatureView(Feature, speed)
%     point = GetPointOnTerrainFromScreenCoords(screen_x, screen_y)
%     ans = IsInitialized
%     ans = IsOnline
%     Login
%     Logout
%     ShowDescriptionBalloon(Feature)
%     HideDescriptionBalloons
%     Feature = GetHighlightedFeature
%     Places = GetMyPlaces
%     Places = GetTemporaryPlaces
%     Layers = GetLayersDatabases
%     Hwnd = GetMainHwnd
%     Hwnd = GetRenderHwnd
%
%  Go to <a href="http://earth.google.com/comapi/annotated.html">Google Earth COM API Documentation</a> for more information on
%  how to use the above properties and functions.
% 
% Examples:
%
%  % Start up the server
%  ge = GEserver; 
%
%  % get the camera properties
%  cam = ge.GetCamera(1).get;
%
%  % fetch the view extents
%  view = [ge.ViewExtents.South ge.ViewExtents.West; ge.ViewExtents.North ge.ViewExtents.East]
%
%  % Move the camera
%  ge.SetCameraParams(-33.85,18.48,500,1,500,70,205,2);
%
%  % Get a list of the properties and functions
%  ge.get
%  ge.invoke
%
%  % Release the interface and all its resources
%  ge.release
%
% See also: GEcamera, GEaddKmlFeature, actxserver, COM/release, COM/invoke, COM/get,
%           COM/set


persistent ge;

if nargin>=1,
  ge = ge_in;
end;

% Startup and Connect to Google Earth
% IApplicationGE This is the main entry point to the Google Earth COM API  
% ge = actxserver('GoogleEarth.ApplicationGE');

% check if there is an existing Google Earth server connection
if (~exist('ge','var') || isempty(ge)),
  ge = actxserver('GoogleEarth.ApplicationGE');
end;

% Check if Google Earth server is connected and running
try
  if (~ge.IsInitialized),
    warning('GoogleEarth:Init','Google Earth is not initialized. Performing Login...');
    ge.Login;
  end;  
catch % Server is no longer connected
  e = lasterror;
  warning('GoogleEarth:Init','%sRestarting Google Earth...',e.message);
  ge = actxserver('GoogleEarth.ApplicationGE');
end;

% Wait for GoogleEarth to start up
nn = now;
msgshown = 0;
while ((now-nn)<10/(24*60*60) && ~ge.IsInitialized),
  if ((now-nn)>1/(24*60*60)&&~msgshown),
    disp('Waiting for GoogeEarth to start...');
    msgshown = 1;
  end;  
  pause(0.1);
end;
% Double check if Google Earth is alive
if (ge.IsInitialized),
  disp('Startup successful.');
else
  error('Failed to Start GoogleEarth!');
end;

% check if online 
count = 0;
while(~ge.IsOnline && count<1),
  count = count + 1;
  warning('GoogleEarth:Init','Google Earth is not online, trying to Login...');
  ge.Login;
  if (ge.IsOnline),
    disp('Login successful.');
  end;
end;

ge_out = ge;
