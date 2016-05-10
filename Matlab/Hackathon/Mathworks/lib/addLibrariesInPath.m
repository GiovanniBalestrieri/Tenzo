% Copyright 2016 The MathWorks, Inc.
function addLibrariesInPath(varargin)

root_dir = fileparts(mfilename('fullpath'));

% % Clean path, remove all children, keep only current directory
if nargin >= 1 && strcmp(varargin{1}, 'clean')
    warning('off','MATLAB:rmpath:DirNotFound')
    rmpath(genpath(root_dir))
    warning('on','MATLAB:rmpath:DirNotFound')   
    addpath(root_dir)
end

if nargin == 0 || strcmp(varargin{1}, 'arduino')    
    addpath(fullfile(root_dir,'arduino_DigitalInput'))
    addpath(fullfile(root_dir,'arduino_Encoder'))
    addpath(fullfile(root_dir,'arduino_I2C'))
    addpath(fullfile(root_dir,'arduino_PWM'))
end

if nargin == 0 || strcmp(varargin{1}, 'rpi')
    addpath(fullfile(root_dir,'rpi_I2C'))
    addpath(fullfile(root_dir,'rpi_RWcalibration'))
end

if nargin == 0 || strcmp(varargin{1}, 'sim')
    addpath(fullfile(root_dir,'sim_RealTimePacer'))
end
