function [master_settings, group_settings_master, preset_list_cell] = ...
    config_settings(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PROGRAM SETTINGS
%
% Brent A. Wallace
%
% 2023-03-29
%
% This program performs the master config of this entire MATLAB code suite.
% The initialization process includes:
%
%   * Configure relative paths to programs, data
%   * Configure algorithm settings
%   * Configure master plot formatting
%   * Configure frequency response plot settings
%   * System initialization
%       * Modeling error parameter values \nu
%       * Integral augmentation settings
%       * Configure misc system settings -- config_sys.m
%   * Configure relative paths to data for selected system
%   * Configure controllers for selected system
%   * Configure individual preset group settings 
%       -- config_preset_group_cell.m  
%       -- config_preset_group.m 
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
%
% UNPACK USER-SET SETTINGS
% 
% *************************************************************************
% ************************************************************************* 

% System names
sysnames = master_settings.sysnames;

% System tag
systag = master_settings.systag;

% Model/controller init
init1_load0_model = master_settings.init1_load0_model;

% Preset group
preset_group = master_settings.preset_group;

% Number of preset groups executed
numgroups = master_settings.numgroups;


%%
% *************************************************************************
% *************************************************************************
%
% INCLUDE FILE PATHS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%
% INCLUDE UTILITY FOLDER
%
addpath('util');
addpath('util/plot');
addpath('util/DIRL');

% ***********************
%
% INCLUDE AIRCRAFT UTILITY FOLDERS
%
relpath_data_main = '01 data/';

% ***********************
%
% INCLUDE SYSTEM UTILITY FOLDERS
%
addpath([relpath_data_main 'ddmr/']);
addpath([relpath_data_main 'pendulum/']);
addpath([relpath_data_main 'businjet/']);

% ***********************
%
% INCLUDE CONFIG FUNCTIONS FOLDER
%
addpath('config');

% ***********************
%
% INCLUDE EVALUATION FUNCTIONS FOLDER
%
addpath('eval_functs');

% ***********************
%
% INCLUDE ALGORITHM FOLDER
%
addpath('algs');

% ***********************
%
% INCLUDE PLOT FUNCTIONS FOLDER
%
addpath('plot');


% ***********************
%
% RELATIVE PATHS TO PROGRAM DATA
%

relpath_data_root = '01 data/';

%%
% *************************************************************************
% *************************************************************************
%
% ALGORITHM INIT
% 
% *************************************************************************
% *************************************************************************

% Algorithm names
algnames.rci = 'RCI';
algnames.lq = 'Nom LQ';
algnames.lq_opt_nu = 'Opt LQ';
algnames.cfvi = 'cFVI';
algnames.rfvi = 'rFVI';
master_settings.algnames = algnames;

%%
% *************************************************************************
% *************************************************************************
%
% PLOT FORMATTING
% 
% *************************************************************************
% *************************************************************************


% Master plot settings
psett_master = init_psett();

% SET MASTER PLOT SETTINGS
master_settings.psett_master = psett_master;




%%
% *************************************************************************
% *************************************************************************
%
% SYSTEM INIT
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% RELATIVE PATH TO CURRENT MODEL
%

% Root path
relpath_model = [relpath_data_root systag '/'];

% Path to model data
relpath_model_data = [relpath_model 'model/'];

% Filename of model data
filename_model = 'model.mat';

% ***********************
%
% SETTINGS -- MODELING ERROR PARAMETERS \nu USED FOR EVALUATIONS
%

switch systag

    % ***********************
    %
    % PENDULUM
    %

    case sysnames.pendulum

        % Perturbation parms
        nuvec = (1:0.01:1.25)';
%         nuvec = [1; 1.1; 1.25];  

        % Which perturbations to plot data for
        nuvecplot = [1; 1.1; nuvec(end)];
%         nuvecplot = nuvec(end);
%         nuvecplot = 1.1;
        
        % Which single perturbation value to use for modeling error
        % evaluation studies
        nueval = nuvec(end);
%         nueval = 1.1;

        % Name of model initialization program
        model_init_program = 'pendulum_init';

        % System name (for displaying on plots)
        systag_disp = 'Pendulum';

    % ***********************
    %
    % JET
    %

    case sysnames.businjet

        % Perturbation parms 
        nuvec = (1:-0.025:0.75)';
%         nuvec = [1; 0.9; 0.75];

        % Which perturbations to plot data for
        nuvecplot = [1; 0.9; nuvec(end)];
        
        % Which single perturbation value to use for modeling error
        % evaluation studies
        nueval = nuvec(end);
%         nueval = 0.9;

        % Name of model initialization program
        model_init_program = 'businjet_init';

        % System name (for displaying on plots)
        systag_disp = 'Jet Aircraft';

    % ***********************
    %
    % DDMR
    %

    case sysnames.ddmr

        % Perturbation parms
        nuvec = (1:0.025:1.25)';
%         nuvec = [1; 1.1; 1.25];

        % Which perturbations to plot data for
        nuvecplot = [1; 1.1; nuvec(end)];
%         nuvecplot = nuvec(end);
%         nuvecplot = 1.1;
        
        % Which single perturbation value to use for modeling error
        % evaluation studies
        nueval = nuvec(end);
%         nueval = 1.1;
 
        % Name of model initialization program
        model_init_program = 'ddmr_init';        

        % System name (for displaying on plots)
        systag_disp = 'DDMR';


end

% ***********************
%
% MODEL INDICES -- NOMINAL MODEL
%

% Index of nominal model
indnom = find(nuvec == 1);

% Indices of non-nominal models
indsnotnom = setdiff((1:size(nuvec,1)),indnom);

% ***********************
%
% MODEL INDICES -- MODELS TO PLOT DATA FOR
%        

indsmodelplot = [];

for mcnt = 1:size(nuvecplot,1)          
    ind = find(nuvec == nuvecplot(mcnt));
    if ~isempty(ind)
        indsmodelplot = [indsmodelplot; ind];
    end
end

nummodelsplot = size(indsmodelplot,1);


% ***********************
%
% MODEL INDICES -- MODEL TO PERFORM MODELING ERROR EVALUATIONS FOR
%      

indnueval = find(nuvec == nueval);
nueval_str = num2filename(nueval);

% ***********************
%       
% CONTROLLER HAS INTEGRAL AUGMENTATION (=1) OR NOT (=0)
% 

switch systag
    case sysnames.pendulum
        hasintaug = 0; 
    otherwise
        hasintaug = 1; 
end

% Set
master_settings.hasintaug = hasintaug;

% *************************************************************************
%
% INITIALIZE/LOAD MODEL
%
% *************************************************************************

% ***********************
%
% MISC SETTINGS
%

% Initialize model if desired
if init1_load0_model

    % Settings
    setts.nuvec = nuvec;
    setts.indnom = indnom;
    setts.relpath_data = relpath_model_data;
    setts.filename = filename_model;   
    % Init
    eval([model_init_program '(setts)'])
    clear setts;

end

% Load model parameters
models = load([relpath_model_data filename_model]);
models = models.model_struct;
model_cell = models.model_cell; 

% Configure system settings
sys.tag = systag;
sys.tag_disp = systag_disp;
sys.relpath_data = relpath_model_data;

% Store model cell arrays, perturbation params
sys.model_cell = model_cell;   
sys.nummodels = size(model_cell,1);
sys.indnom = indnom;
sys.indsnotnom = indsnotnom;
sys.nuvec = nuvec;
sys.nuvecplot = nuvecplot;

% Initialize system
[sys, sys_plot_settings] = config_sys(sys);

% Store system parameters
master_settings.sys = sys;
master_settings.systag_disp = systag_disp;

% Store plot settings
master_settings.sys_plot_settings = sys_plot_settings;

%%
% *************************************************************************
% *************************************************************************
%
% RELATIVE PATHS TO DATA FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% RELATIVE PATHS TO RCI DATA
%

% Root path
relpath_dirl = [relpath_model 'RCI/'];

% Relative path to nominal model
relpath_dirl_nom = [relpath_dirl 'nu_1/'];

% Path to this value of \nu for evaluation
relpath_dirl_error = [relpath_dirl 'nu_' nueval_str '/'];

% File name of RCI data
filename_data_dirl = 'out_data_cell_master';

% ***********************
%
% RELATIVE PATHS TO RCI DATA -- x_0 SWEEP TRAINING DATA
%

relpath_dirl_sweep = [relpath_model 'dirl_sweep/'];


% ***********************
%
% RELATIVE PATHS TO EVALUATION DATA
%

relpath_data_eval = [relpath_model 'eval_data/'];


% ***********************
%
% RELATIVE PATHS TO LQ SERVO DATA
%

relpath_lq = [relpath_model 'lq_servo/'];

% Store relative paths to LQ servo, FBL data in master settings
master_settings.relpath_lq = relpath_lq;

% ***********************
%
% RELATIVE PATHS TO PYTHON DATA, FILE NAMES
%

% Relative path
relpath_py = [relpath_model 'python/'];

% File names
switch systag
    case sysnames.ddmr
        filename_py_base = '_ddmrintaug_quad';
    case sysnames.pendulum
        filename_py_base = '_Pendulum_QuadCost';          
    case sysnames.businjet
        filename_py_base = '_businjetintaug_quad';         
end

filename_py = [filename_py_base '_tbl_data.mat'];
filename_py_2d = [filename_py_base '_tbl_data_2D.mat'];

%%
% *************************************************************************
% *************************************************************************
%
% INITIALIZE/LAOD CONTROLLERS FOR THIS MODEL
% 
% *************************************************************************
% *************************************************************************

% Initalize/load controllers
master_settings = eval(['config_controllers_' systag '(master_settings)']);


%%
% *************************************************************************
% *************************************************************************
%
% SAVE SETTINGS
%
% *************************************************************************
% *************************************************************************

% System indices
master_settings.indsmodelplot = indsmodelplot;
master_settings.nummodelsplot = nummodelsplot;
master_settings.indnueval = indnueval;

% Store relative paths to RCI data
master_settings.relpath_dirl = relpath_dirl;
master_settings.relpath_dirl_nom = relpath_dirl_nom;
master_settings.relpath_dirl_error = relpath_dirl_error;
master_settings.filename_data_dirl = filename_data_dirl;

% Store relative paths to RCI x_0 sweep data in master settings
master_settings.relpath_dirl_sweep = relpath_dirl_sweep;

% Store relative path and filename to Python data in master settings
master_settings.relpath_py = relpath_py;
master_settings.filename_py_base = filename_py_base;
master_settings.filename_py = filename_py;
master_settings.filename_py_2d = filename_py_2d;

% Store relative path and filename to evaluation data in master settings
master_settings.relpath_data_eval = relpath_data_eval;



%%
% *************************************************************************
% *************************************************************************
%
% METHOD/SYSTEM/DESIGN PRESET LIST AND SETTINGS
%
% *************************************************************************
% *************************************************************************


% Preset group cell
group_settings_master = ...
    config_preset_group_cell(master_settings);

% Each entry contains the presets executed for the respective group
preset_list_cell = cell(numgroups, 1);

% Initialize 
for i = 1:numgroups

    preset_groupi.group_settings = group_settings_master{i};
    preset_groupi.tag = preset_group;

    [preset_list_cell{i}, group_settings_master{i}] = ...
                config_preset_group(preset_groupi, master_settings);

end

% Get total number of presets executed -- including sweeps
numpresets_tot = 0;
for i = 1:numgroups

    % Group settings
    group_settingsi = group_settings_master{i};

    % Number of presets in this group
    numpresetsi = group_settingsi.numpresets;

    % Extract sweep settings
    sweepsettsi = group_settingsi.sweepsetts;

    % Is a sweep preset group (=1) or not (=0)
    issweep = sweepsettsi.issweep;

    % Total number of sweep iterations
    if issweep
        numsweepits = prod(sweepsettsi.sweepsizevec);
    else
        numsweepits = 1;
    end

    % Increment total
    numpresets_tot = numpresets_tot + numsweepits * numpresetsi;

end

% Store
master_settings.numpresets_tot = numpresets_tot;
