function [feature] = GEaddKmlFeature(ge,kml,speed)
% Add a KML feature and return a handle to it.
%
% Inserts a KML feature into GoogleEarth. The 
% feature is also highlighted and handles to 
% all the features in the Temporary Places 
% folder are returned.
%
% [feature] = GEaddKmlFeature(ge,kml)
% Adds the feature defined by the KML markup text.
% Returns an interface to the feature.
%
% The features interface has the following 
% properties...
%   Name: Name as defined in kml text
%   Visibility: 0 or 1, is being drawn
%   HasView:  0 or 1, has a defined camera view
%   Highlighted: 0 or 1, is selected
% and the following interface
%   TimeInterval
%
% The features have the following functions...
% feature.GetParent, returns parent folder
% feature.GetChildren, 
%
% Examples:
%  % This requires some kml code! See the <a href="http://www.mathworks.com/matlabcentral/fileexchange/loadFile.do?objectId=12954&objectType=file">Google Earth Toolbox</a> for help.
%  f = GEaddKmlFeature(ge,kml)
%  disp(f.HasView);
%  % will return an error if there are no children
%  f(end).GetParent.Name
%  children = f(end).GetChildren
%  disp(children(1).Count)
%
% See also: GEserver, GEcamera, actxserver

if ~exist('speed','var'),
  speed = 3;
end;

ge.LoadKmlData(kml);

numitems = ge.GetTemporaryPlaces.GetChildren.Count;

for ii = 1:numitems
  feature(ii) = ge.GetTemporaryPlaces.GetChildren.Item(ii);
end;

feature(end).Highlight;

if feature(end).HasView,
  ge.SetFeatureView(feature(end),speed);
end;
