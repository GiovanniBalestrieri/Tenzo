% Copyright 2016 The MathWorks, Inc.
%% setupArena: Fonction de positionnement des cibles et des obstacles

%% Introduction
% La fonction *setupArena* permet de g�rer des configurations d'ar�ne
% (position des cibles et des obstacles), afin de pouvoir les utiliser lors
% de simulations.

%% Importer la configuration courante
% L'importation de la configuration courante de l'ar�ne peut se faire de
% deux fa�ons :
%
% * En allant dans le menu *Import* $\rightarrow$ *From Workspace*
% * En cliquant sur le bouton correspondant dans la barre d'outils
%
% <<setupArena_resources/ImportFromWorkspace.PNG>>

%% D�placement de cibles/obstacles
% Le d�placement des cibles et des obstacles se fait tout simplement en
% cliquant-glissant les �l�ments dans l'ar�ne. Il est important de ne pas
% laisser d'�l�ments superpos�s, ce qui pourrait entra�ner des
% comportements non pr�vus lors de la simulation.
%
% <<setupArena_resources/moveObstacle.GIF>>


%% Ajout de cibles/obstacles
% L'ajout de cibles et d'obstacles peut se faire de deux fa�ons :
%
% * En allant dans le menu *Edit* $\rightarrow$ *Add Target* ou *Add Obstacle*
% * En acc�dant au menu contextuel de l'ar�ne (clic-droit sur l'image de
% fond) et en s�lectionnant *Add Target* ou *Add Obstacle*
%
% <<setupArena_resources/AddTargetObstacle.PNG>>

%% Suppression de cibles/obstacles
% La suppression de cibles et d'obstacles peut se faire de deux fa�ons :
%
% * En s�lectionnant une cible ou un obstacle (clic souris), puis an allant
% dans le menu *Edit* $\rightarrow$ *Delete Target/Obstacle*
% * En acc�dant au menu contextuel de la cible ou de l'obstacle
% (clic-droit) et en s�lectionnant *Delete Target* ou *Delete Obstacle*
% respectivement
%
% <<setupArena_resources/DeleteTargetObstacle.PNG>>

%% Exporter une configuration comme sc�nario courant
% Exporter la configuration actuelle comme sc�nario courant (c'est-�-dire
% pour l'utiliser en simulation) peut se faire de deux fa�ons :
% 
% * En allant dans le menu *Export* $\rightarrow$ *To Workspace*
% * En cliquant sur le bouton correspondant dans la barre d'outils
%
% <<setupArena_resources/ExportToWorkspace.PNG>>

%% Importer un sc�nario de base
% Importer un des sc�narios de base fournis avec le package se fait
% simplement en allant dans le menu *Import* $\rightarrow$ *Scenario X* :
%
% <<setupArena_resources/ImportScenario.PNG>>

%% Cr�er un nouveau sc�nario
% Cr�er un nouveau sc�nario � partir de rien se fait de deux fa�ons :
%
% * En allant dans le menu *File* $\rightarrow$ *New*
% * En cliquant sur le bouton correspondant dans la barre d'outils
%
% Dans tous les cas une nouvelle fen�tre s'ouvre demandant le nombre de
% cibles et d'obstacles � ajouter
%
% <<setupArena_resources/NewArena.PNG>>

%% Sauvegarder une configuration dans un fichier
% Sauvegarder une configuration dans un fichier afin de pouvoir la
% r�utiliser se fait de deux fa�ons :
%
% * En allant dans le menu *File* $\rightarrow$ *Save* ou *SaveAs...*
% * En cliquant sur le bouton correspondant dans la barre d'outils
%
% <<setupArena_resources/SaveConfiguration.PNG>>

%% Charger un fichier personnel
% Charger une configuration pr�c�demment enregistr�e peut se faire de
% deux fa�ons :
%
% * En allant dans le menu *File* $\rightarrow$ *Open*
% * En cliquant sur le bouton correspondant dans la barre d'outils
%
% <<setupArena_resources/OpenConfiguration.PNG>>

%% Ouverture du mod�le Simulink
% Il est possible d'ouvrir le mod�le Simulink de simulation
% (SimulationModel.slx) :
%
% <<setupArena_resources/OpenModel.PNG>>

function setupArena()

scrSz = get(groot, 'ScreenSize');
screenCenter = scrSz(3:4)/2;

f = figure('DockControls', 'off',...
    'MenuBar', 'none',...
    'Name', 'Arena Setup',...
    'NumberTitle', 'off', ...
    'ToolBar', 'none', ...
    'Position', [[-250 -250]+screenCenter 500 500],...
    'HandleVisibility', 'off');

model = ArenaModel();
view = ArenaView(model, 'Parent', f);
ArenaController(model, view);