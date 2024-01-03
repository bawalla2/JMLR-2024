function [out_data_cell, group_settings]  = plot_preset_group(...
    alg_settings_cell, out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAIN PLOT FUNCTION -- PRESET GROUP
%
% Brent A. Wallace
%
% 2021-11-06
%
% This program, given a specified preset group, generates all plots for the
% group and saves the plots.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Unpack plot settings
savefigs = master_settings.savefigs;
savedata = master_settings.savedata;
relpath = group_settings.relpath;
dolegend = group_settings.dolegend;
do_individual_plots = group_settings.do_individual_plots;
sys_plot_settings = master_settings.sys_plot_settings;

% Number of designs to plot for
numpresets = size(alg_settings_cell,1);


% Is a sweep (=1) or not (=0)
issweep = group_settings.issweep;

% Is cFVI comparison (=1) or not (=0)
iseval = group_settings.iseval;

% Initialize figure counter
figcount = group_settings.figcount;


% Create save directory if figures are to be saved
if savefigs
    mkdir(relpath);                   % Create directory for relative path
end

% Update relpath field of group_settings struct to include the time stamp
% (if it was added, otherwise this line is redundant)
group_settings.relpath = relpath;



% ***********************
%
% MAKE LEGEND IF USER SPECIFIED
%  

if dolegend
   
    % Initialize legend
    lgnd = cell(numpresets,1);
    
    % Fill legend entries
    for i = 1:numpresets
       
        % Extract preset legend entry
        lgnd{i} = alg_settings_cell{i}.plot_settings.legend_entry;
        
    end
    
    % Store legend
    group_settings.lgnd = lgnd;
    
end



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
% *************************************************************************
%
% STATE TRAJECTORY, CONTROL SIGNAL PLOTS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if ~issweep

    figcount = plot_x_u(alg_settings_cell,...
        out_data_cell, group_settings, master_settings);

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ALGORITHM-SPECIFIC PLOTS
%
% E.g., NN weight parameters with respect to iteration count, etc. See
% respective algorithm plot function (e.g. plot_rci.m) for further details.
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

if do_individual_plots

    for i = 1:numpresets
        
        % Store figure counter in plot settings to pass to function
        group_settings.figcount = figcount;

        % Extract algorithm settings and output data for current preset
        alg_settings = alg_settings_cell{i};
        out_data = out_data_cell{i};

        % ***********************
        %
        % EXECUTE APPROPRIATE ALGORITHM PLOT FUNCTION
        %
        switch alg_settings.alg

            % ***********************
            %
            % RCI
            %

            case 'rci'

                figcount = ...
                    plot_rci(alg_settings, out_data, ...
                    group_settings, master_settings); 


            otherwise

                % NOTHING            

        end

    end

end







%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ADDITIONAL PLOTS IF IS AN IC SWEEP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if issweep
% if 0
    figcount = ...
        plot_x0_sweep(alg_settings_cell, out_data_cell,...
        group_settings, master_settings);
end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% ADDITIONAL PLOTS IF IS cFVI COMPARISON
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings to pass to function
group_settings.figcount = figcount;

if iseval
% if 0
    [figcount, eval_data] = ...
        plot_val_pol(group_settings, master_settings);
end




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SAVE PRESET GROUP DATA TO DIRECTORY
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Store figure counter in plot settings
group_settings.figcount = figcount;

data_folder = 'data/';

if savedata
    
    % Make directory to save data to
    relpath_data = [relpath data_folder];
    group_settings.relpath_data = relpath_data;
    mkdir(relpath_data)
    
    % Save data -- alg_settings_cell struct
    varname = 'alg_settings_cell';
    save([relpath_data varname], varname);
    
    % Save data -- out_data_cell struct
    varname = 'out_data_cell';
    save([relpath_data varname], varname);
    
    % Save data -- group_settings struct
    varname = 'group_settings';
    save([relpath_data varname], varname);
  
    % Save data -- eval_data struct
    if iseval
        varname = 'eval_data';
        save([relpath_data varname], varname)
    end

end

