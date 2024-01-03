function alg_settings = config_preset(preset, group_settings, ...
    master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SELECT ALGORITHM, SYSTEM, DESIGN PARAMETERS BASED ON PRESET
%
% Brent A. Wallace
%
% 2021-11-06
%
% This program, given a desired preset, handles all algorithm
% initialization/configuration. This is the main program for configuring
% specific algorithm hyperparameters (e.g., sample rates, excitations,
% etc.)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset            (String) Algorithm/system preset for desired example.
% group_settings    (Struct) Contains system/design parameters to
%                   be shared across all presets in the desired group.
%                   E.g., if for this preset group all designs share the
%                   same Q, R matrices, those fields may be included in
%                   this struct. This is initialized in
%                   config_preset_group.m.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% alg_settings  (Struct) Algorithm settings/parameters for subsequent
%               execution according to desired preset (see respective
%               algorithm .m-file for details). 
%               
%
% NOTE: These are initialized automatically by settings in
% config_preset_group, but they nevertheless may be manually overriden.
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
sys = group_settings.sys;
m = sys.m;

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


% Tag of system being executed
systag = master_settings.systag;
% List of system names
sysnames = master_settings.sysnames;

% % Extract algorithm names
% algnames = master_settings.algnames;

% Is a sweep (=1) or not (=0)
issweep = group_settings.issweep;

% ICs
ICs = group_settings.ICs;

% ***********************
%
% GET SWEEP ITERATION VECTOR, PARAMETERS
%    

% Sweep iteration vector
sweepindvec = group_settings.sweepindvec;

% IC indices 
if issweep
    indICs = sweepindvec(1:2);
else
    indICs = [1;1];
end

% Model index
if issweep
    indmodel = sweepindvec(3);
else
    indmodel = master_settings.indnueval;
end        

% ***********************
%
% GET ICs FOR THIS SWEEP ITERATION
%    

% Initial condition
x0 = ICs.x0mat(indICs(1),indICs(2),:);  
x0 = x0(:);

% ***********************
%
% GET NOMINAL, PERTURBED MODELS FOR THIS ITERATION
%    

% Nominal, perturbed models
model_cell = sys.model_cell;
indnom = sys.indnom;
model = model_cell{indnom};
model_nu = model_cell{indmodel};

% Indices of output variables
inds_xr = model.inds_xr;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

switch preset

    %%    
    % *********************************************************************
    % *********************************************************************
    %
    % RCI
    %      
    % *********************************************************************
    % *********************************************************************

    
    case 'rci'

        % ***********************
        %
        % ALGORITHM
        %        

%         alg = strrep(preset,['_' systag],'');
        alg = preset;
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        

        % Get preset count
        presetcount = group_settings.presetcount;

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end
 
       
        % Time to simululate for 
        tsim = group_settings.tsim;

        % IRL settings
        irl_setts = group_settings.irl_setts_cell{presetcount};

        % Number of loops executed for this preset
        numloops = irl_setts.numloops;

        % Which loops to train
        doloopvec = irl_setts.doloopvec;

        % Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
        sim_w = 0;
        
        % Nominal model
        model_nom_tag = group_settings.model_nom_tag;
        switch model_nom_tag
            case 'default'
                model_nom_ind = indnom;
            case 'perturbed'
                model_nom_ind = indmodel;
        end 

        % Simulation model
        model_sim_tag = group_settings.model_sim_tag;
        switch model_sim_tag
            case 'default'
                model_sim_ind = indnom;
            case 'perturbed'
                model_sim_ind = indmodel;
        end       

        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data_0 = master_settings.lq_data_0;
        lq_data = lq_data_cell{indnom};

        % ***********************
        %
        % LOOP SETTINGS
        %   

        % Holds loop settings
        loop_cell = cell(numloops,1);
        
        % Initialize loop settings
        switch numloops

            % ***********************
            %
            % SINGLE-LOOP
            %   

            case 1

            % System-specific settings
            switch systag                   
                case sysnames.pendulum
                    % Sample period
                    Ts = 1;
                    % x, y indices
                    tmp.indsx = (1:2)';
                    tmp.indsy = (1)';
                    % Learning settings
                    tmp.istar = 5;
                    tmp.nTs = 1;    
                    tmp.l = 15; 
            end


            tmp.Q = lq_data.Qirl;
            tmp.R = lq_data.R;
            % Initial stabilizing design on nominal model
            tmp.K0 = lq_data_0.Kdirl;   
                      

            % Set current loop settings
            loop_cell{1} = tmp;

            % ***********************
            %
            % DOUBLE-LOOP
            %   

            case 2

                % Sample period
                % System-specific settings
                switch systag
                    case sysnames.ddmr
                        Ts = 1;   
                    case sysnames.businjet
                        Ts = 0.5;                        
                end                


            for k = 1:numloops
                switch k
    
                    % Loop j = 1
                    case 1
                        % System-specific settings
                        switch systag                             
                            case sysnames.ddmr
                                % x, y indices
                                tmp.indsx = 1;
                                tmp.indsy = 1;   
                                % Learning settings
                                tmp.istar = 5;
                                tmp.nTs = 4 / Ts;  
                                tmp.l = 20;
                            case sysnames.businjet
                                % x, y indices
                                tmp.indsx = 1;
                                tmp.indsy = 1;   
                                % Learning settings
                                tmp.istar = 5; 
                                tmp.nTs = 4 / Ts;  
                                tmp.l = 15;            
                        end  

                        tmp.Q = lq_data.lq_data_11.Q;
                        tmp.R = lq_data.lq_data_11.R;

%                         % Nominal design on nominal model
                        % Initial stabilizing design on nominal model
                        tmp.K0 = lq_data_0.lq_data_11.K;   
                        

    
                    % Loop j = 2   
                    case 2

                        % System-specific settings
                        switch systag                        
                            case sysnames.ddmr
                                % x, y indices
                                tmp.indsx = 2;
                                tmp.indsy = 2;  
                                % Learning settings
                                tmp.istar = 5;
                                tmp.nTs = 1;
                                tmp.l = 15;
                            case sysnames.businjet
                                % x, y indices
                                tmp.indsx = (2:4)';
                                tmp.indsy = 2; 
                                % Learning settings
                                tmp.istar = 5;
                                tmp.nTs = 1;
                                tmp.l = 30;                               
                        end  

                        tmp.Q = lq_data.lq_data_22.Q;
                        tmp.R = lq_data.lq_data_22.R;

                        % Initial stabilizing design on nominal model
                        tmp.K0 = lq_data_0.lq_data_22.K;                         
  
    
    
                end
    
                % Set current loop settings
                loop_cell{k} = tmp;
            end

        end




        % ***********************
        %
        % REFERENCE COMMANDS -- FOR TRAINING PHASE
        %


        % Determine whether or not to insert r(t)
        hasdort = isfield(group_settings, 'dort');
        if hasdort
            dort = group_settings.dort;
        else
            dort = 1;
        end

        % Proceed to insert r(t) if desired 
        if dort
            refcmdl = 'sum_sin';
        else
            refcmdl = 'zero';
        end


        % Trim
        x_e = model.trimconds.xe;

        switch m
            case 1
                x1_e = x_e(inds_xr(1));
                x1_m = x1_e; 
                biasvec = x1_m;
                nderivvec = 0;                
            case 2
                x1_e = x_e(inds_xr(1));
                x2_e = x_e(inds_xr(2));
                x1_m = x1_e;
                x2_m = x2_e;   
                biasvec = [x1_m; x2_m];
                nderivvec = [0; 0];
        end

        switch refcmdl

            % NO r(t)
            case 'zero'

                cos1_sin0 = zeros(m,1);
                Amat = zeros(m,1);
                Tmat = ones(m,1);

            % INSERT r(t)
            case 'sum_sin'
                
                switch systag




                    case sysnames.ddmr


        % %                 % ORIGINAL
                        cos1_sin0 = [   0   0  0   
                                        0   0   0   ];
                        Amat = [        2  1  0  
                                        5 5   5 ];
                        Tmat = [        10 5  1
                                        5  2.5 50      ];
        
                        % Convert \omega amplitudes to rad/s
                        Amat(2,:) = D2R * Amat(2,:);


                    case sysnames.pendulum

                        % ORIGINAL 
                        cos1_sin0 = [   0   0     ];
                        Amat = [        10  5    ];
                        Tmat = [        10 5  ];
           
        
                        % Perform deg -> rad conversions
                        Amat(1,:) = D2R * Amat(1,:);

                    case sysnames.businjet

                        % ORIGINAL
                        cos1_sin0 = [   0   0    
                                        0   0     ];
                        Amat = [        5  10     
                                        0.1   0.1   ];
                        Tmat = [        50 25    
                                        2.5  1.5        ]; 



                        % Convert \gamma amplitudes to rad
                        Amat(2,:) = D2R * Amat(2,:);

                end


        end


        % ***********************
        %
        % TRANINING SETTINGS
        %


        % DEBUGGING: Use the nominal linearization A for w = f(x) - A x at
        % nominal trim (=1) or simulation trim (=0)
        % NOTE: Not implemented -- keep =1
        wnom1sim0 = 1;
%         wnom1sim0 = 0;


        % Reference command settings
        switch refcmdl
            case 'zero'
                r_sett_train.tag = 'sum_sin';   
            case 'sum_sin'
                r_sett_train.tag = 'sum_sin'; 
        end
        switch refcmdl
            case {'sum_sin';'zero'}
                r_sett_train.biasvec = biasvec;
                r_sett_train.nderivvec = nderivvec;
                r_sett_train.dorefvec = ones(m,1);
                r_sett_train.cos1_sin0 = cos1_sin0;
                r_sett_train.Amat = Amat;
                r_sett_train.Tmat = Tmat;
        end
        



        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;     



    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case [{'lq_servo_inout'}]
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'lq_servo_inout';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       


        % Get preset count
        presetcount = group_settings.presetcount;

        % Reference command
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);


        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);


        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end        

        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Time to simululate for 
        tsim = group_settings.tsim;


        % Nominal, simulation model
        switch curralg
            case algnames.lq_opt_nu
                model_nom_tag = group_settings.model_sim_tag;
                model_sim_tag = group_settings.model_sim_tag;
            otherwise
                model_nom_tag = group_settings.model_nom_tag;
                model_sim_tag = group_settings.model_sim_tag;                
        end

        % Nominal model
        switch model_nom_tag
            case 'default'
                model_nom_ind = indnom;
            case 'perturbed'
                model_nom_ind = indmodel;
        end 

        % Simulation model
        switch model_sim_tag
            case 'default'
                model_sim_ind = indnom;
            case 'perturbed'
                model_sim_ind = indmodel;
        end     


        % ***********************
        %
        % GET CONTROLLER SETTINGS FOR THIS SWEEP ITERATION
        %   

        lq_data_cell = master_settings.lq_data_cell;
        lq_data = lq_data_cell{indnom};

        % ***********************
        %
        % GET CONTROLLER SETTINGS
        %   

        switch curralg
            case algnames.lq
                K = lq_data.K;
            case algnames.lq_opt_nu
                K = lq_data_cell{model_sim_ind}.K;             
        end

        

        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           


    % *********************************************************************
    %
    % cFVI TRACKING
    %
    
    case [{'cfvi_tracking'}]
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'cfvi_tracking';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       


        % Get preset count
        presetcount = group_settings.presetcount;

        % Reference command
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);


        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);


        % Get algorithm names
        algnames = master_settings.algnames;

        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Time to simululate for 
        tsim = group_settings.tsim;


        % Nominal, simulation model
        model_nom_tag = group_settings.model_nom_tag;
        model_sim_tag = group_settings.model_sim_tag; 

        % Nominal model
        switch model_nom_tag
            case 'default'
                model_nom_ind = indnom;
            case 'perturbed'
                model_nom_ind = indmodel;
        end 

        % Simulation model
        switch model_sim_tag
            case 'default'
                model_sim_ind = indnom;
            case 'perturbed'
                model_sim_ind = indmodel;
        end     



        % Relative path to cFVI data
        relpath_py = master_settings.relpath_py;
        filename_py =  master_settings.filename_py;

        % Append 'cFVI' or 'rFVI' tag to file path, name
        relpath_py = [relpath_py curralg '/'];
        filename_py = [curralg filename_py];
        
        % Load cFVI data -- n-D
        cfvi_data = load([relpath_py filename_py]);


        % ***********************
        %
        % n-D GRID DATA
        %  
        
        cfvi_x_tbl_min = cfvi_data.x_tbl_min;
        cfvi_x_tbl_max = cfvi_data.x_tbl_max;
        cfvi_x_tbl_nxpts = cfvi_data.x_tbl_nxpts;
        cfvi_u_tbl =  cfvi_data.u_tbl;


        % ***********************
        %
        % cFVI GRID VECTORS -- n-D
        %  
        
        % Get number of states in cFVI plant
        n_cfvi = length(cfvi_x_tbl_min);
        
        % Cell array containing grid vectors
        cfvi_xgridvec_cell = cell(n_cfvi, 1);
        
        % Initialize grid vectors
        for i = 1:n_cfvi
            cfvi_xgridvec_cell{i} = ...
                linspace(cfvi_x_tbl_min(i), cfvi_x_tbl_max(i),...
                cfvi_x_tbl_nxpts(i))';
        end

        

        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           



    % *********************************************************************    
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: PRESET TAG NOT RECOGNIZED ***');  
       
end





%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE ALGORITHM SETTINGS/DESIGN PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% GENERAL SETTINGS
%
% *************************************************************************

% Preset tag
alg_settings.preset = preset;

% Plot settings -- general
plot_settings.legend_entry = legend_entry;
plot_settings.plotfolder = plotfolder;
% plot_settings.sys_settings = sys_settings;

% Write plot settings
alg_settings.plot_settings = plot_settings;


% *************************************************************************
%
% ALGORITHM-SPECIFIC SETTINGS
%
% *************************************************************************

switch alg

    % *********************************************************************
    %
    % RCI
    %
    
    case 'rci'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.indmodel = indmodel;

        % Whether or not to perform learning in each loop
        alg_settings.doloopvec = irl_setts.doloopvec;
       

        % Simulate w = f(x) - Ax for simulation model (=1) or not (=0)
        alg_settings.sim_w = sim_w;

  
        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;


        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;



        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = ...
            group_settings.lin1nonlin0vec(presetcount);



        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = group_settings.pf1nopf0vec(presetcount);
   

        % Reference command settings -- training phase
        alg_settings.r_sett_train = r_sett_train;

        % Reference command settings
        alg_settings.r_sett = group_settings.r_sett;

        % Loop settings
        alg_settings.loop_cell = loop_cell;

        % Sample period
        alg_settings.Ts = Ts;

        % Do x_3 loop
        alg_settings.dox3 = 0;
        alg_settings.lenx3 = 0;


        % Is training preset (=1) or not (=0)
        istraining = group_settings.istraining;
        alg_settings.istraining = istraining;
        
        % Do reference command r(t) injection (=1) or not (=0)
        alg_settings.dort = group_settings.dort;

        % Overwrite current controller (=1) or not (=0)
        alg_settings.updatecontroller = istraining && ~issweep;

  
        % Use the nominal linearization A for w = f(x) - A x at nominal
        % trim (=1) or simulation trim (=0)
        alg_settings.wnom1sim0 = wnom1sim0;


        % ICs, simulation time
        alg_settings.x0 = x0;  
        alg_settings.tsim = group_settings.tsim; 


    % *********************************************************************
    %
    % cFVI
    %
    
    case 'cfvi'

    % *********************************************************************
    %
    % cFVI TRACKING
    %
    
    case 'cfvi_tracking'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.model = model;
        alg_settings.model_nu = model_nu;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;


        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;


        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            pfavec = master_settings.pfavec;   
        end

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Policy lookup table, grid vectors
        alg_settings.xgridvec_cell = cfvi_xgridvec_cell;
        alg_settings.u_tbl = cfvi_u_tbl;

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 


    % *********************************************************************
    %
    % LQ SERVO INNER/OUTER
    %
    
    case 'lq_servo_inout'

        alg_settings.alg = alg;

        % Nominal, perturbed models
        alg_settings.model = model;
        alg_settings.model_nu = model_nu;

        % Nominal model
        alg_settings.model_nom_tag = model_nom_tag;
        alg_settings.model_nom_ind = model_nom_ind;

        % Simulation model
        alg_settings.model_sim_tag = model_sim_tag;
        alg_settings.model_sim_ind = model_sim_ind;


        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;


        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = pfavec;   
        end

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Controller
        alg_settings.K = K;

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 
  



        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('**** ERROR: ALGORITHM TAG NOT RECOGNIZED ***');  
       
end
