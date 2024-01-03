function group_settings_master = config_preset_group_cell(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET GROUP
%
% Brent A. Wallace 
%
% 2022-02-16
%
% This program, given a list of desired preset groups to execute,
% initializes each of the preset groups. Most of the execution in this
% program is configuring automatic settings. However, to change which
% algorithms are executed for a given system, see the variable
% 'alg_list_default'.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% master_settings       (Struct) Master settings as initialized by main.m
%                       and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% group_settings_master     ('numgroups' x 1 Cell) The i-th entry of this   
%                           cell array is itself a struct containing all of
%                           the settings for the corresponding preset
%                           group.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Save figures
savefigs = master_settings.savefigs;

% Save data
savedata = master_settings.savedata;

% Relative path
relpath = master_settings.relpath;

% Number of preset groups executed
numgroups = master_settings.numgroups;

% Preset group list
preset_group_list = master_settings.preset_group_list; 

% Algorithm names
algnames = master_settings.algnames;

% System names
sysnames = master_settings.sysnames;

% Current system being executed
systag = master_settings.systag;

% Master plot settings
psett_master = master_settings.psett_master;


% ***********************
%
% GET SYSTEM DATA
%   

% System
sys = master_settings.sys;
model_cell = sys.model_cell;
indnom = sys.indnom;
model = model_cell{indnom};
nummodels = size(sys.model_cell,1);

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SETTINGS TAGS, DEFAULT SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Algorithm list to execute -- default
switch systag

   
    case sysnames.pendulum
        alg_list_default = {
                                algnames.rci
                                algnames.cfvi
                                algnames.rfvi
                                algnames.lq_opt_nu                                
                                algnames.lq    
                                        };      

    case sysnames.ddmr
        alg_list_default = {                                                                  
                                algnames.rci
                                algnames.cfvi
                                algnames.rfvi
                                algnames.lq_opt_nu                                
                                algnames.lq                                  
                                        }; 
      
    case sysnames.businjet
        alg_list_default = {
                                algnames.rci
                                algnames.cfvi
                                algnames.rfvi
                                algnames.lq_opt_nu                                
                                algnames.lq   
                                        };     


end


% Algorithm list to execute -- RCI only
alg_list_rci_only = { algnames.rci };


% Algorithm list to execute -- RCI and FVIs
% alg_list_rci_cfvi = { algnames.rci; algnames.cfvi };
% alg_list_rci_cfvi = { algnames.rci; algnames.rfvi };
alg_list_rci_cfvi = { algnames.rci; algnames.cfvi; algnames.rfvi };
% alg_list_rci_cfvi = { algnames.rci };
% alg_list_rci_cfvi = { algnames.cfvi };
% alg_list_rci_cfvi = { algnames.rfvi };



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP CELL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Number of preset groups executed
group_settings_master = cell(numgroups, 1);


% ***********************
%       
% PRESET GROUP NAME, ALGORITHM NAMES, SYSTEM NAMES
%     

for i = 1:numgroups

    % Group name
    groupnamei = preset_group_list{i};
    group_settings_master{i}.groupname = groupnamei;

    % Algorithm names
    group_settings_master{i}.algnames = algnames;

    % System tag
    group_settings_master{i}.systag = systag;

    % System names
    group_settings_master{i}.sysnames = sysnames;    

    % Master plot settings
    group_settings_master{i}.psett_master = psett_master;    

end

% ***********************
%       
% DETERMINE IF IS A SWEEP PRESET GROUP OR NOT, OTHER GROUP TRAITS BASED ON
% GROUP NAME
%     

for i = 1:numgroups

    % Group name
    groupnamei = group_settings_master{i}.groupname;

    % Check group name for '_sweep'
    strg = '_sweep';
    issweep = contains(groupnamei,strg);

    % Check group name for '_nom': Is on the nominal model (=1) or
    % perturbed model (=0)
    strg = '_nom';
    nom1error0 = contains(groupnamei,strg); 

    % Check group name for '_eval_val_pol'
    strg = '_eval_val_pol';
    iseval = contains(groupnamei,strg);

    % Set flags
    group_settings_master{i}.issweep = issweep;
    group_settings_master{i}.nom1error0 = nom1error0;
    group_settings_master{i}.iseval = iseval;

    % Sweep state variable indices
    switch systag
        case sysnames.pendulum
            group_settings_master{i}.inds_x_sweep = ...
            [model.indth; model.indthd];            
        otherwise
            group_settings_master{i}.inds_x_sweep = model.inds_xr;
    end

end


% ***********************
%       
% DETERMINE IF IS A TRAINING PRESET GROUP OR NOT
%
% DETERMINE IF IS A RCI TRAINING PRESET GROUP OR NOT
%  

for i = 1:numgroups

    % Group name
    groupnamei = group_settings_master{i}.groupname; 

    % Is training (=1) or not (=0)
    group_settings_master{i}.istraining = contains(groupnamei,'training');
    

end

% ***********************
%       
% DETERMINE WHETHER OR NOT TO SAVE TRAINING DATA IF IS A TRAINING GROUP
%  

for i = 1:numgroups

    % Is training (=1) or not (=0)
    if group_settings_master{i}.istraining
        group_settings_master{i}.savetrainingdata = ...
            ~group_settings_master{i}.issweep;
    end
    
end

% ***********************
%       
% DETERMINE WHETHER OR NOT TO RUN ALGORITHMS
% 

for i = 1:numgroups

    group_settings_master{i}.runpresets = ...
 ~(group_settings_master{i}.iseval );
    
end


% ***********************
%       
% ALGORITHMS EXECUTED, NUMBER OF ALGORITHMS EXECUTED
%     

for i = 1:numgroups

    % If is a training preset, only execute RCI. Else, execute all
    % algorithms
    if group_settings_master{i}.istraining
       
        group_settings_master{i}.alg_list = alg_list_rci_only;

    else
        if ~group_settings_master{i}.iseval
            group_settings_master{i}.alg_list = alg_list_default;
        else
            group_settings_master{i}.alg_list = ...
                        alg_list_rci_cfvi;
        end
    end

    % Number of presets executed
    group_settings_master{i}.numpresets =...
    size(group_settings_master{i}.alg_list,1);

end

% ***********************
%       
% DETERMINE WHETHER TO SAVE PRESET GROUP DATA OR NOT
%    

for i = 1:numgroups
    
    % Store system parameters
    group_settings_master{i}.savedata = savedata;    

end


% ***********************
%       
% CONFIGURE NOMINAL, SIMULATION MODEL, SET SYSTEM
%    

for i = 1:numgroups

    % Store system parameters
    group_settings_master{i}.sys = master_settings.sys;
    
    % Store plot settings
    group_settings_master{i}.sys_plot_settings =...
        master_settings.sys_plot_settings;

    % Check if to use the nominal model (=1) or perturbed model (=0)
    if group_settings_master{i}.nom1error0

        % ***********************
        %       
        % NOMINAL MODEL
        %     

            % Nominal model
            model_nom_tag = 'default';     
    
            % Simulation model
            model_sim_tag = 'default';
             

    else

        % ***********************
        %       
        % PERTURBED MODEL
        %             

            % Nominal model
            model_nom_tag = 'default';      
    
            % Simulation model
            model_sim_tag = 'perturbed';
    
    end

    % Set value in the struct
    group_settings_master{i}.model_nom_tag = model_nom_tag;
    group_settings_master{i}.model_sim_tag = model_sim_tag;

end


% ***********************
%       
% REFERENCE COMMAND SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % RCI TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining ...
            || group_settings_master{i}.iseval ...

        % Reference command tag
        group_settings_master{i}.refcmd = 'training';

        % Reference command type
        group_settings_master{i}.refcmdtype = 'training';
    
    % ***********************
    %       
    % NON-RCI TRAINING PRESET GROUPS
    %             
    else

        % Group name
        groupnamei = group_settings_master{i}.groupname;


        % ***********************
        %       
        % STEP V PRESET GROUPS
        %          
        if contains(groupnamei,'step_V')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_V';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            

        % ***********************
        %       
        % STEP \gamma PRESET GROUPS
        %     
        elseif contains(groupnamei,'step_g')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_g';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            

        % ***********************
        %       
        % STEP \omega PRESET GROUPS (DDMR)
        %    
        elseif contains(groupnamei,'step_w')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_w';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';               

        % ***********************
        %       
        % STEP \theta PRESET GROUPS (PENDULUM)
        %    
        elseif contains(groupnamei,'step_th')

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_th';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';        

    end       

    end
                
end

% ***********************
%       
% DO PREFILTER OR NOT
% 

for i = 1:numgroups

    switch group_settings_master{i}.refcmdtype

        % ***********************
        %       
        % TRAINING PRESET GROUPS
        %     
        case 'training'
        
            group_settings_master{i}.pf1nopf0 = 0;
        

        % ***********************
        %       
        % STEP PRESET GROUPS
        %          
        case 'step'

            switch systag
                case sysnames.ddmr
                    group_settings_master{i}.pf1nopf0 = 1;   
                case sysnames.businjet
                    group_settings_master{i}.pf1nopf0 = 1;                     
                otherwise
                    group_settings_master{i}.pf1nopf0 = 0; 
            end

       
                       

    end

end


% ***********************
%       
% RCI LEARNING SETTINGS -- WHICH LOOPS TO EXECUTE
% 

for i = 1:numgroups

    % Number of presets
    numpresets = group_settings_master{i}.numpresets;

    % Holds which loops to execute for each preset in the group
    irl_setts_cell = cell(numpresets,1);

    for j = 1:numpresets

        % Determine number of loops to execute
        curralg = group_settings_master{i}.alg_list{j};
        switch systag
            case sysnames.pendulum
                numloops = 1; 
            otherwise
                switch curralg
                    case algnames.rci
                        numloops = 2;
                    otherwise
                        numloops = 1;                
                end
        end
        % Set number of loops
        irl_setts.numloops = numloops;

        % Which loops to execute
        irl_setts.doloopvec = ones(numloops,1);

        % Store IRL settings for this preset
        irl_setts_cell{j} = irl_setts;

    end

    % Store IRL settings for this group
    group_settings_master{i}.irl_setts_cell = irl_setts_cell;

end

% ***********************
%       
% RCI LEARNING SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 1;
        
        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots =...
            ~ group_settings_master{i}.issweep;
    
        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 0;            

    
    % ***********************
    %       
    % NON TRAINING PRESET GROUPS
    %             
    else

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 0;

        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots = 0;

        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 1;


    end

end


% ***********************
%       
% RCI LEARNING SETTINGS -- DO REFERENCE r(t) OR NOT
% 

for i = 1:numgroups

    % ***********************
    %       
    % RCI TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        group_settings_master{i}.dort = 1;

    else

        group_settings_master{i}.dort = 0;

    end

end

      
% ***********************
%       
% RCI DATA RELATIVE PATH AND FILE NAME
%     

% Relative path
relpath_dirl_nom = master_settings.relpath_dirl_nom;
relpath_dirl_error = master_settings.relpath_dirl_error;
filename_data_dirl = master_settings.filename_data_dirl;


for i = 1:numgroups

    % File name
    group_settings_master{i}.filename_data_dirl = filename_data_dirl;

    % Relative path to RCI data
    if group_settings_master{i}.istraining 

        switch group_settings_master{i}.model_sim_tag
        
                case 'default'
                
                    group_settings_master{i}.relpath_data_dirl = ...
                          relpath_dirl_nom;
        
                case 'perturbed'
        
                    group_settings_master{i}.relpath_data_dirl = ...
                        relpath_dirl_error;
        end

    else

        group_settings_master{i}.relpath_data_dirl = ...
                    master_settings.relpath_dirl_sweep;
    
    
    end

end


% ***********************
%       
% PLOT SETTINGS
%    

for i = 1:numgroups

    % Save figures
    group_settings_master{i}.savefigs = savefigs;

    % Relative path
    group_settings_master{i}.relpath = relpath;
    

end






