% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% RE-PLOT PREVIOUSLY GENERATED PRESET GROUP DATA
%
% Brent A. Wallace
%
% 2022-12-12
%
% This program, given previously generated preset group settings and output
% data, re-calls the main plot function plot_main.m to generate new plots
% for the old data. See re_plot.m for further documentation.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
%
% PROGRAM CONFIG
% 
% *************************************************************************
% *************************************************************************


% ***********************
%
% FIGURE CONTROLS
%

close all

% Save figures control
savefigs = 1; 
% savefigs = 0; 

% Save data control
savedata = 1;
% savedata = 0;

% Relative file path for saved figures
relpath = '00 figures/'; 	

% % Do plots for each preset in the group
% do_individual_plots = 1;   

% Tag for group of plots to execute
% group_tag = 'ES_ddmr_sweep';
% group_tag = 'ES_pendulum_sweep';
group_tag = 'ES_businjet_sweep';

% *************************************************************************
% *************************************************************************
%
% INITIALIZE PROGRAM BASED ON DESIRED PLOT DATA
% 
% *************************************************************************
% *************************************************************************

switch group_tag


    % ***********************
    %
    % SWEEP -- DDMR
    %    
    case 'ES_ddmr_sweep'

        % Relative path to data
        relpath_data = '01 data/ddmr/dirl_sweep/';

    % ***********************
    %
    % SWEEP -- PENDULUM
    %    
    case 'ES_pendulum_sweep'

        % Relative path to data
        relpath_data = '01 data/pendulum/dirl_sweep/';

    % ***********************
    %
    % SWEEP -- BUSINESS JET
    %    
    case 'ES_businjet_sweep'

        % Relative path to data
        relpath_data = '01 data/businjet/dirl_sweep/';

end


% *************************************************************************
% *************************************************************************
%
% RE-PLOT THE DATA
% 
% *************************************************************************
% *************************************************************************

% Pack input arguments
plot_settings.savefigs = savefigs;    
plot_settings.savedata = savedata;
plot_settings.relpath = relpath; 	
plot_settings.relpath_data = relpath_data;

% Call function
re_plot(plot_settings);

