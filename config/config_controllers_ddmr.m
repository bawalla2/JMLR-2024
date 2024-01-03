function master_settings = config_controllers_ddmr(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG LQ SERVO CONTROLLERS
%
% Brent A. Wallace 
%
% 2023-04-13
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

% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************

% System
sys = master_settings.sys;
model_cell = sys.model_cell;
nummodels = sys.nummodels;
indnom = sys.indnom;

% Current system being executed
systag = master_settings.systag;

% Controller initialization controls
init1_load0_lq = master_settings.init1_load0_lq;
% init1_load0_fbl = master_settings.init1_load0_fbl;

% Relative path to controller data
relpath_lq = master_settings.relpath_lq;
% relpath_fbl = master_settings.relpath_fbl;

% String with system specifier
relpath_sys = [systag '/'];

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE CONTROLLERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% LQ SERVO
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS
%
% NOTE: State partitioned as (see below)
%
%      = [  z
%           y
%           x_r ]
%


Q1 = diag([10 10]);
R1 = 0.75;
Q2 = diag([25 7.5]);
R2 = 1;


% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS -- INITIAL STABILIZING CONTROLLER
%


Q10 = diag([5 5]);
R10 = 1;

Q20 = diag([7.5 2.5]);
R20 = 0.1;



% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER
%   

% Relative path to controller params
relpath_ctrl_tmp = [relpath_lq relpath_sys];

% File name to save to
filename_tmp = 'lq_data_cell.mat';

% Cell array to contain LQ data
lq_data_cell = cell(nummodels,1);

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q1;
    alg_settings.R1 = R1;
    alg_settings.Q2 = Q2;
    alg_settings.R2 = R2;
    alg_settings.hasintaug = hasintaug;

    % Loop over model cell array
    for i = 1:nummodels

        % Get current model
        alg_settings.model_d = model_cell{i};
 
        % Initialize controller 
        lq_datai = config_lq_servo_tito(alg_settings);

        % Store lq_data in array
        lq_data_cell{i} = lq_datai;

    end

    % Make directory to save lq_data cell to
    mkdir(relpath_ctrl_tmp);

    % Save data 
    varname = 'lq_data_cell';
    save([relpath_ctrl_tmp filename_tmp], varname);

end

% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER -- INITIAL STABILIZING CONTROLLER
%   

% File name to save to
filename_tmp0 = 'lq_data_0.mat';

if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.Q1 = Q10;
    alg_settings.R1 = R10;
    alg_settings.Q2 = Q20;
    alg_settings.R2 = R20;
    alg_settings.hasintaug = hasintaug;

    % Get current model -- nominal model
    alg_settings.model_d = model_cell{indnom};

    % Initialize controller 
    lq_data_0 = config_lq_servo_tito(alg_settings);

    % Save data 
    varname = 'lq_data_0';
    save([relpath_ctrl_tmp filename_tmp0], varname);

end

% Load controllers
data = load([relpath_ctrl_tmp filename_tmp]);
lq_data_cell = data.lq_data_cell;
      
% Load initial stabilizing controllers
data = load([relpath_ctrl_tmp filename_tmp0]);
lq_data_0 = data.lq_data_0;




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE DATA
% 
% *************************************************************************
% *************************************************************************       
% *************************************************************************

% ***********************
%
% LQ SERVO
%   

master_settings.lq_data_cell = lq_data_cell;
master_settings.pfavec = lq_data_cell{indnom}.pfavec;

% MANUALLY OVERRIDE PREFILTER ZERO LOCATION
master_settings.pfavec(2) = 1;

% ***********************
%
% LQ SERVO -- INITIAL STABILIZING CONTROLLER
%   

master_settings.lq_data_0 = lq_data_0;


