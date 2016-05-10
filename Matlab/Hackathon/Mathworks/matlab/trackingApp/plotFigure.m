% Copyright 2014 - 2016 The MathWorks, Inc.
function plotFigure(X, T, T_found, t_last, t, T_located,pcam,lcam,robotheta,...
    ArmAngleEstimated, ArmAngleCommand, DisplayPositions)
% internal function 
% Process robot localisation information from the simulation model

simApp = SimDisplay.getInstance;

% extract located targets
T_located = T_located(T_located(:,1)>0,:);

idx = (T(:,1) >= 0); %debuffer the list of target positions
update(simApp, X, T(idx,:), T_found(idx), t_last, t, T_located,pcam,lcam,...
    robotheta, false, false, ArmAngleEstimated, ArmAngleCommand, DisplayPositions)

