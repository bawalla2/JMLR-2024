function [preset_list, group_settings] = config_preset_group( ...
    preset_group, master_settings)
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
% This program, given a desired preset group, initializes each of the
% presets to run in the group.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset_group      (String) Tag corresponding to the preset group to run.
% master_settings   (Struct) Master settings as initialized by main.m
%                   and config_settings.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% preset_list       ('numpresets' x 1 Cell) The i-th entry of this cell
%                   array is a string which is the tag of the desired
%                   preset to execute as the i-th preset in the group. 
% group_settings    (Struct) This is where any group-shared settings (e.g.,
%                   system, initial conditions, etc.) are stored for
%                   subsequent initialization of each of the presets. All
%                   of these settings are user-configurable, and each are
%                   well-commented below.
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
% *************************************************************************
%
% PLOT FORMATTING OPTIONS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Check if pre-existing preset group settings have been initialized
if isfield(preset_group, 'group_settings')
    group_settings = preset_group.group_settings;
    preset_group = preset_group.tag;
end

% Store the preset group tag in the 'group_settings' struct
group_settings.preset_group = preset_group;

% Master plot settings
psett_master = group_settings.psett_master;

% Plot formatting -- line width (wide)
psett_linewidthwide = psett_master.psett_linewidthwide;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- dotted line
psett_dot = psett_master.psett_dot;


% Plot formatting -- colors
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;

% Extract system names
sysnames = master_settings.sysnames;

% Extract algorithm names
algnames = master_settings.algnames;

switch preset_group


      
    % *********************************************************************
    %
    % MAIN PRESET GROUP
    % 
    %
    
    case 'main'

        % *****************************************************************
        %
        % PRESET GROUP SHARED SETTINGS      
        %        

        % Tag of system being executed
        systag = group_settings.systag;
        
        % Extract algorithm list
        alg_list = group_settings.alg_list;

        % Number of algorithms executed
        numalgs = size(alg_list,1);

        % Is a sweep preset group (=1) or not (=0)
        issweep = group_settings.issweep;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0vec = zeros(numalgs,1);

        % Do prefilter (=1) or not (=0)
        pf1nopf0vec = group_settings.pf1nopf0 * ones(numalgs,1);


        % Legend, formatting cells, algorithm settings
        preset_list = cell(numalgs,1);
        lgd_p = alg_list;
        indiv_sett_cell = cell(numalgs,1);
        color_sett_cell = cell(numalgs,1);
        for i = 1:numalgs
            switch alg_list{i}
                case algnames.rci
                    color_sett_cell{i} = psett_matlabblue;
                    indiv_sett_cell{i} = ...
                        [psett_linewidthwide; color_sett_cell{i}];
                    lin1nonlin0vec(i) = 0;                        
                    preset_list{i} = 'rci';                           
                case algnames.lq
                    color_sett_cell{i} = psett_matlabyellow;  
                    indiv_sett_cell{i} = ...
                        [psett_dot; color_sett_cell{i}];
                    lin1nonlin0vec(i) = 0;
                preset_list{i} = 'lq_servo_inout';    
                case algnames.lq_opt_nu
                    color_sett_cell{i} = psett_matlaborange;                    
                    indiv_sett_cell{i} = ...
                        [psett_dash; color_sett_cell{i}];
%                         indiv_sett_cell{i} = {};
                    lin1nonlin0vec(i) = 0;   
                preset_list{i} = 'lq_servo_inout';                   
                case algnames.cfvi
                    color_sett_cell{i} = psett_matlabgreen;
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                       
                    preset_list{i} = 'cfvi_tracking';  
                case algnames.rfvi
                    color_sett_cell{i} = psett_matlabpurple;
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;                       
                    preset_list{i} = 'cfvi_tracking';                                          
            end

        end


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


        % Trim
        x_e = model.trimconds.xe;

        % State dimension n
        n = size(x_e, 1);

        % Input dimension m
        m = model.m;

        % Sweep variable indices
        inds_x_sweep = group_settings.inds_x_sweep;

        switch systag
            case sysnames.ddmr
                % Trim speed (m/s)
                x1_e = x_e(model.indV);  
                % Trim angular velocity (rad/s)
                x2_e = x_e(model.indw);
            case sysnames.pendulum
                % Trim pendulum angle (rad)
                x1_e = x_e(model.indth);  
                % Trim pendulum angular velocity (rad/s)
                x2_e = x_e(model.indthd);     
            case sysnames.businjet
                % Trim speed (m/s)
                x1_e = x_e(model.indV);  
                % Trim FPA (rad)
                x2_e = x_e(model.indg);                   
        end

        % Degree/radian conversions
        D2R =   pi/180;
        R2D =   180/pi;

        % ***********************
        %
        % ICS
        %   

        % Vectors of initial values (x_{1}(0), x_{2}(0)) to test in the IC
        % sweep -- before trim applied
        switch systag
            case sysnames.ddmr
                % x_{1}(0)
                tx10vec_sweep = (-0.5:0.125:0.5)';    % FINAL
%                 tx10vec_sweep = (-0.5:0.5:0.5)';        % TESTING
                % x_{2}(0) -- in deg/s
                    tx20vec_sweep = (-30:5:30)';        % FINAL
%                 tx20vec_sweep = (-30:30:30)';       % TESTING
            case sysnames.pendulum
                % x_{1}(0) -- in deg
                tx10vec_sweep = (-60:15:60)';       % FINAL
%                 tx10vec_sweep = (-60:60:60)';   % TESTING
                % x_{2}(0) -- in deg/s
                tx20vec_sweep = (-60:15:60)';       % FINAL
%                 tx20vec_sweep = (-60:60:60)';     % TESTING
            case sysnames.businjet
                % x_{1}(0) -- in m
                tx10vec_sweep = (-10:2:10)';       % FINAL
%                 tx10vec_sweep = (-10:10:10)';   % TESTING
                % x_{2}(0) -- in deg
                tx20vec_sweep = (-2:0.5:2)';       % FINAL
%                 tx20vec_sweep = (-2:2:2)';     % TESTING                
        end

        % If current preset group is a sweep, then take sweep ICs. Else,
        % use single IC
        if issweep
           
            tx10vec = tx10vec_sweep;
            tx20vec = tx20vec_sweep;

        else

            switch systag               
                case sysnames.ddmr
                    tx10vec = 0;
                    tx20vec = 0;
                case sysnames.pendulum                  
                    tx10vec = 180;
                    tx20vec = 0;     
                case sysnames.businjet                  
                    tx10vec = 0;
                    tx20vec = 0;                 
            end

        end

        
        % Number of ICs tested in each variable
        numx10 = size(tx10vec,1);
        numx20 = size(tx20vec,1);
        numICs = numx10 * numx20;

        % Apply deg -> rad, deg/s -> rad/s conversion for ICs x_{0}
        switch systag          
            case sysnames.ddmr
                tx20vec = tx20vec * D2R;
                tx20vec_sweep = tx20vec_sweep * D2R;
            case sysnames.pendulum
                tx10vec = tx10vec * D2R;
                tx10vec_sweep = tx10vec_sweep * D2R;
                tx20vec = tx20vec * D2R;
                tx20vec_sweep = tx20vec_sweep * D2R;        
            case sysnames.businjet
                tx20vec = tx20vec * D2R;
                tx20vec_sweep = tx20vec_sweep * D2R;                    
        end

        % Apply shift by trim
        x10vec = tx10vec + x1_e;
        x10vec_sweep = tx10vec_sweep + x1_e;
        x20vec = tx20vec + x2_e;
        x20vec_sweep = tx20vec_sweep + x2_e;

        % Initialize IC matrix: entry (i,j,:) contains the total IC vector
        % with x10vec(i), x20vec(j) in their appropriate place
        x0mat = zeros(numx10,numx20,n);
        for i = 1:numx10
            for j = 1:numx20
                x0mat(i,j,:) = x_e;
                x0mat(i,j,inds_x_sweep(1)) = x10vec(i);
                x0mat(i,j,inds_x_sweep(2)) = x20vec(j);
            end
        end

        % Find indices of trim states in sweep
        indsxe = zeros(2,1);
        for i = 1:2
            ind_xri = inds_x_sweep(i);
            xei = x_e(ind_xri);
            switch i
                case 1
                    xi0vec = x10vec_sweep;
                case 2
                    xi0vec = x20vec_sweep;
            end
            indsxe(i) = find(xi0vec == xei);
        end


        % ***********************
        %
        % VALUE FUNCTION, POLICY EVALUATION (POST-TRAINING) SETTINGS
        %   

        % Initialize evaluation data (=1) or load previous (=0)
%         init1load0_eval_data = 1;
        init1load0_eval_data = 0;     

        % Do plots for evaluations (=1) or not (=0)
        doplots_eval = 1;
%         doplots_eval = 0;

        % Do policy plots for evaluations (=1) or not (=0) 
%         doplots_eval_pol = 1;
        doplots_eval_pol = 0;

        % Use uniform color limits on countour plots (=1) or not (=0)
        doClim = 1;

        % Evaluation grid
        switch systag           
            case sysnames.ddmr
                % x_{1}(0)
%                 tx10vecp = (-1:0.1:1)';         % TESTING
%                 tx10vecp = (-0.5:0.05:0.5)';         % TESTING
                tx10vecp = (-0.5:0.0125:0.5)';      % FINAL
                % x_{2}(0) -- in deg/s
%                 tx20vecp = (-30:3:30)';        % TESTING   
                tx20vecp = (-30:0.25:30)';      % FINAL
            case sysnames.pendulum
                 % x_{1}(0) -- in deg
%                 tx10vecp = (-60:60:60)';         % TESTING
                tx10vecp = (-60:0.5:60)';
                % x_{2}(0) -- in deg/s
%                 tx20vecp = (-60:60:60)';        % TESTING    
                tx20vecp = (-60:0.5:60)';          
            case sysnames.businjet
                % x_{1}(0) -- in m/s
                tx10vecp = (-10:0.5:10)';       % FINAL
%                 tx10vecp = (-10:10:10)';       % TESTING
                % x_{2}(0) -- in deg 
                tx20vecp = (-2:0.1:2)';      % FINAL         
%                 tx20vecp = (-2:2:2)';      % TESTING 
        end

        % Number of ICs tested in each variable
        numx10p = size(tx10vecp,1);
        numx20p = size(tx20vecp,1);
        numICsp = numx10p * numx20p;

        % Apply deg -> rad, deg/s -> rad/s conversion for ICs x_{0}
        switch systag
            case sysnames.ddmr
                tx20vecp = tx20vecp * D2R;
            case sysnames.pendulum
                tx10vecp = tx10vecp * D2R;
                tx20vecp = tx20vecp * D2R;    
            case sysnames.businjet
                tx20vecp = tx20vecp * D2R;                
        end

        % Apply shift by trim
        x10vecp = tx10vecp + x1_e;
        x20vecp = tx20vecp + x2_e;

        % ***********************
        %
        % COST J(x) EVALUATION SETTINGS
        %   

        % Do cost J(x) evaluations (=1) or not (=0)
        Jx_sett.doJx = 1;
%         Jx_sett.doJx = 0;
        
        % Integration horizon length (sec)
        switch systag          
            case sysnames.ddmr
                Jx_sett.T = 10;
            case sysnames.pendulum
                Jx_sett.T = 3; 
            case sysnames.businjet
                Jx_sett.T = 25;                    
        end

        % Threshold \epsilon > 0 such that simulation terminates when ||x||
        % < \epsilon -- NOT USED. KEEP donormeps = 0
        Jx_sett.donormeps = 0;
        Jx_sett.normeps = 5e-2;       

        % ***********************
        %
        % EXPECTED RETURN EVALUATION SETTINGS
        %   

        ER_sett.doER = 1;
%         ER_sett.doER = 0;

        % Integration horizon length (sec)
        switch systag           
            case sysnames.ddmr
                ER_sett.T = 5;
            case sysnames.pendulum
                ER_sett.T = 5;    
            case sysnames.businjet
                ER_sett.T = 20;                    
        end

        % Number of seeds
        ER_sett.n_seed = 20;

        % Number of simulations for evaluating ER
        switch systag
            case sysnames.ddmr
                ER_sett.nsim = 100;
            case sysnames.pendulum
                ER_sett.nsim = 100;    
            case sysnames.businjet
                ER_sett.nsim = 100; 
            otherwise
                ER_sett.nsim = 100;
        end        

        % Threshold \epsilon > 0 such that simulation terminates when ||x||
        % < \epsilon -- NOT USED. KEEP donormeps = 0
        ER_sett.donormeps = 0;
        ER_sett.normeps = Jx_sett.normeps;  


        % ***********************
        %
        % REFERENCE COMMAND SETTINGS
        %
        refcmd = group_settings.refcmd;
        refcmdtype = group_settings.refcmdtype;

        switch refcmd

            case 'training'

                % Simulation, plot time
                tsim = 0;          
                tsim_plot = 0;   

            case 'step_V'

                switch systag

                    case sysnames.ddmr
                        % Simulation, plot time
                        tsim = 10;          
                        tsim_plot = 10;    
        
                        % 1 m/s step-velocity command
                        x1r = 1;
                        x2r = 0;

                    case sysnames.businjet

                        % Simulation, plot time
                        tsim = 100;          
                        tsim_plot = 50;    
        
                        % 10 m/s step-velocity command
                        x1r = 10;
                        x2r = 0;

                end

            case 'step_g'

                switch systag


                    case sysnames.businjet

                        % Simulation, plot time
                        tsim = 30;          
                        tsim_plot = 20;  
        
                        % 1 deg FPA command
                        x1r = 0;
                        x2r = 1*D2R;  

                end

            case 'step_w'

                % Simulation, plot time
                tsim = 25;          
                tsim_plot = 10;  

                % 30 deg/s \omega command
                x1r = 0;  
                x2r = 30*D2R;

            case 'step_th'

                switch systag
                    case sysnames.pendulum

                        % Simulation, plot time
                        tsim = 10;          
                        tsim_plot = 2.5;  
            
                        % 0 deg \theta command
                        x1r = 0;     

                end

        end

        % ***********************
        %
        % REFERENCE COMMANDS
        %

        switch refcmdtype

            case 'step'
                
                switch m
                    case 1
                        % Set params
                        x1_m = x1_e + x1r;
        
                        biasvec = x1_m;
                        nderivvec = 0;
                        cos1_sin0 = 0;
                        Amat = 0;
                        Tmat = 1;  
                    otherwise
                        % Set params
                        x1_m = x1_e + x1r;
                        x2_m = x2_e + x2r;
        
                        biasvec = [x1_m; x2_m];
                        nderivvec = [0; 0];
                        cos1_sin0 = zeros(2,1);
                        Amat = zeros(2,1);
                        Tmat = ones(2,1);  
                end

        end


        % ***********************
        %
        % PLOT SETTINGS
        %


        % Include legend in plots
        dolegend = 1;        

        % ***********************
        %
        % STORE SETTINGS
        %

        % Number of presets
        group_settings.numpresets = numalgs;

        % Legend entries
        group_settings.lgd_p = lgd_p;

        % Individual plot settings
        group_settings.color_sett_cell = color_sett_cell;        
        group_settings.indiv_sett_cell = indiv_sett_cell;

        group_settings.tsim = tsim;
        group_settings.tsim_plot = tsim_plot;

        % ICs
        ICs.numx10 = numx10;
        ICs.numx20 = numx20;
        ICs.numICs = numICs;
        ICs.x10vec = x10vec;
        ICs.x20vec = x20vec;
        ICs.x0mat = x0mat;
        ICs.indsxe = indsxe;
        group_settings.ICs = ICs;

        % Initialize evaluation data (=1) or load previous (=0)
        group_settings.init1load0_eval_data = init1load0_eval_data;

        % Do plots for evaluations (=1) or not (=0)
        group_settings.doplots_eval = doplots_eval;

        % Do policy plots for evaluations (=1) or not (=0) 
        group_settings.doplots_eval_pol = doplots_eval_pol;

        % Evaluation points
        xplot.numx10p = numx10p;
        xplot.numx20p = numx20p;
        xplot.numICsp = numICsp;
        xplot.tx10vecp = tx10vecp;
        xplot.tx20vecp = tx20vecp; 
        xplot.x10vecp = x10vecp;
        xplot.x20vecp = x20vecp;
        group_settings.xplot = xplot;

        % Use uniform color limits on countour plots (=1) or not (=0)
        group_settings.doClim = doClim;

        % Cost J(x) evaluation settings
        group_settings.Jx_sett = Jx_sett;

        % Expected return evaluation settings
        group_settings.ER_sett = ER_sett;

        % Model linear (=1) or nonlinear (=0)
        group_settings.lin1nonlin0vec = lin1nonlin0vec;

        % Do prefilter (=1) or not (=0)
        group_settings.pf1nopf0vec = pf1nopf0vec;

        % Command r(t)
        switch refcmdtype
            case 'step'
                r_sett.tag = 'sum_sin';
                switch m
                    case 1
                        r_sett.Arvec = x1r;
                    case 2
                        r_sett.Arvec = [x1r; x2r];
                end               
        end
        switch refcmdtype
            case 'training'
                r_sett = [];
            case 'step'   
                r_sett.biasvec = biasvec;
                r_sett.nderivvec = nderivvec;
                r_sett.dorefvec = ones(2,1);
                r_sett.cos1_sin0 = cos1_sin0;
                r_sett.Amat = Amat;
                r_sett.Tmat = Tmat;                
        end
        group_settings.r_sett = r_sett;

      
        % ***********************
        %
        % SWEEP SETTINGS
        %          

        sweepsetts.issweep = issweep;
        if issweep
            % Vector, each entry of which determines the number of sweep
            % parameters in the respective variable (e.g., number of ICs)
            sweepsizevec = [numx10; numx20; nummodels]; 
            sweepsetts.sweepsizevec = sweepsizevec;
            sweepsetts.sweepdim = size(sweepsizevec,1);
        end

        % Store sweep settings
        group_settings.sweepsetts = sweepsetts;        





        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('*** ERROR: PRESET GROUP TAG NOT RECOGNIZED ***');  
       
end    




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE GROUP SETTINGS WHICH ARE ALWAYS DECLARED
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Preset group
group_settings.preset_group = preset_group;


% Include legend in plots
group_settings.dolegend = dolegend; 
