% Copyright (c) 2016, The MathWorks, Inc.

function openProject
% OPENPROJECT Initialize MATLAB path to work on project
%
%
%

% Check that version of MATLAB is 8.6 (R2015b)
if verLessThan('matlab', '8.6')
    error('MATLAB 8.6 (R2015b) or higher is required.');
elseif ~verLessThan('matlab', '8.7')
    warning('It is recommended to work with MATLAB 8.6 (R2015b).')
end

% Determine the complete path of project folder
root_dir = fileparts(fileparts(mfilename('fullpath')));

% Add to path all needed directories to work
addInPath(fullfile(root_dir,'data')) % Parameters
addInPath(fullfile(root_dir,'data', 'images')) % Images
addInPath(fullfile(root_dir,'data', 'datatypes')) % Custom data types
addInPath(fullfile(root_dir,'lib')) % Library of drivers for Robot
addInPath(fullfile(root_dir,'matlab'))
addInPath(fullfile(root_dir,'matlab','judge'))
addInPath(fullfile(root_dir,'matlab','judge','common'))
addInPath(fullfile(root_dir,'matlab','trackingApp')) % Graphical Robot representation
addInPath(fullfile(root_dir,'matlab','utilities'))
addInPath(fullfile(root_dir,'matlab','calibration'))
addInPath(fullfile(root_dir,'model')) % SLX files

% Add librairies in path
addLibrariesInPath('sim')

% Create work directory if it doesn't already exist
if ~isdir(fullfile(root_dir,'work'))
    mkdir(fullfile(root_dir,'work'));
end

% Add work directory in path and set it as destination for all generated
% files from Simulink (for simulation and code generation)
addInPath(fullfile(root_dir,'work'))
Simulink.fileGenControl('set', ...
    'CacheFolder',fullfile(root_dir,'work'), ...
    'CodeGenFolder',fullfile(root_dir,'work'))
     
% Display a message with a hyperlink to open model for simulation
disp('Project initialization is completed.')
disp(' ')

if exist(fullfile(root_dir,'model','SimulationModel.slx'),'file')
    disp('For simulation:')
    disp('	You can open <a href="matlab:SimulationModel">simulation model</a> (model/SimulationModel.slx)')
    disp(' ')
end

if exist(fullfile(root_dir,'model','ArduinoModel.slx'),'file')
    disp('For robot:')
    disp('	You can open <a href="matlab:ArduinoModel">robot model</a> (model/ArduinoModel.slx)')
    disp(' ')
end

disp('To setup arena (change sites and obstacles position):')
disp('Use function <a href="matlab:setupArena">setupArena</a> (matlab/utilities/setupArena.m)')
disp('Or use the project shortcut (Project shortcuts tab) "setupArena"')
disp(' ')

function addInPath(folder)

if isdir(folder)
    addpath(folder)
end

