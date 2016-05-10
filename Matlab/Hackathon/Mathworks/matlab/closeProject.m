% Copyright (c) 2016, The MathWorks, Inc.

function closeProject()
% CLOSEPROJECT Clean up MATLAB path from project folders
%
%
%

% Determine the complete path of project folder
root_dir = fileparts(fileparts(mfilename('fullpath')));

% Remove library subfolders from path
addLibrariesInPath('clean')

% Remove project directories from path
rmFromPath(fullfile(root_dir,'data')) 
rmFromPath(fullfile(root_dir,'data', 'images')) 
rmFromPath(fullfile(root_dir,'data', 'datatypes')) 
rmFromPath(fullfile(root_dir,'lib'))
rmFromPath(fullfile(root_dir,'matlab'))
rmFromPath(fullfile(root_dir,'matlab','judge'))
rmFromPath(fullfile(root_dir,'matlab','judge','common'))
rmFromPath(fullfile(root_dir,'matlab','trackingApp'))
rmFromPath(fullfile(root_dir,'matlab','utilities'))
rmFromPath(fullfile(root_dir,'matlab','calibration'))
rmFromPath(fullfile(root_dir,'model'))
rmFromPath(fullfile(root_dir,'work'))

Simulink.fileGenControl('reset')

function rmFromPath(folder)

if isdir(folder)
    rmpath(folder)
end
