function [figcount, eval_data] = plot_val_pol(group_settings, ...
    master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOTS VALUE AND POLICY FUNCTIONS
%
% Brent A. Wallace
%
% 2023-03-08
%
% Given algorithm learning data, this program evaluates and plots 
%
%   Critic network V
%   Policy cost J (evaluated numerically by this program)
%   Critic network error J - V
%   Policy \mu
%   Learning curves (in the case of FVI)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% group_settings    (Struct) Contains the preset group settings for this
%                   program. This contains settings which are auto-set by
%                   previously-called programs, as well as the following
%                   user-configurable settings. NOTE: See
%                   config_preset_group.m to configure these settings:
%   init1load0_eval_data    (Bool) Initialize new evaluation (i.e., of the
%                   policy cost J) (=1) or load previous evaluation from
%                   data (=0).
%   xplot           (Struct) Contains data for the 2D surface plot
%                   settings. Has the following fields:
%       x10vecp     (Vector) Contains a vector of evaluations points for
%                   the first state variable x_1 (i.e., the variable
%                   plotted on the x-axis of the surface plots).
%       x20vecp     (Vector) Contains a vector of evaluations points for
%                   the first state variable x_2 (i.e., the variable
%                   plotted on the y-axis of the surface plots).
%   doplots_eval    (Bool) Do plots after calculating data (=1) or not
%                   (=0).
%   doplots_eval_pol    (Bool) Do policy plots (=1) or not (=0).
%   doClim          (Bool) Normalize color maps for contour plots to be the
%                   on the same scale (=1) or on an individual scale for
%                   each plot (=0).
%   Jx_sett         (Struct) Contains settings for evaluating the policy
%                   cost J(x). Has the following fields:
%       doJx        (Bool) Evaluate policy cost (=1) or not (=0).
%       T           (Double) Policy evaluation horizon (s).
%       normeps     (Double, NOT USED) This setting was developed for a
%                   stop criteria: ||x|| < normeps to terminate the policy
%                   evaluation simulations early. In principle, this
%                   reduces evaluation time. However, we found it to result
%                   in discontinuities in the policy cost plots, so we DO
%                   NOT use it in our evaluations.
%       donormeps   (Bool, SET =0) This bool controls whether or not to use
%                   the above stopping criteria (=1) or not (=0). Since
%                   this feature doesn't function as intended, it is set
%                   LOW in all evaluations.
% master_settings   (Struct) Contains master program settings. Has the
%                   following fields relevant to this program:
%   savefigs        (Bool) Save figures to PDF (=1) or not (=0).
%   relpath         (String) Relative path to figures (for saving only).
%                   NOTE: This string is auto-set. See '/00 figures'.
%   indsmodelplot   (Vector) This contains the indices in the system
%                   modeling error vector 'nuvec' (see config_settings.m)
%                   for which to plot evaluation data for.
%   
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% figcount          (Integer) Cumulative figure count after all plots in
%                   this function have been created.
% eval_data         (Struct) Contains all of the evaluation data output by
%                   this program. Has the following fields:
%   V_x_cell        (Cell) This contains the critic NN V(x) evaluated over
%                   the constructed state grid.
%   u_x_cell        (Cell) This contains the policies \mu(x) evaluated over
%                   the constructed state grid.
%   J_x_cell        (Cell) This contains the policy cost J(x) evaluated 
%                   over the constructed state grid.
%   sys             (Struct, BACKEND ONLY) Contains system model info. Is 
%                   saved so that this evaluation may be re-plotted after
%                   the system parameters may have changed.
%   lq_data_cell    (Cell, BACKEND ONLY) Contains LQ data used in this
%                   evaluation.
%   alg_list        (Cell, BACKEND ONLY) Contains the list of algorithm
%                   names tested in this evaluation for later indexing.
%   x_plot          (Cell, BACKEND ONLY) Contains the (x_1, x_2) grid
%                   values evaluated in this evaluation.
%   cfvi_data_cell  (Cell, BACKEND ONLY) Contains misc cFVI/rFVI algorithm
%                   data for evaluation generation.
%   indsmodelplot   (Vector, BACKEND ONLY) Contains the indices of the
%                   modeling error perturbations \nu evaluated.
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

% Unpack plot settings
savefigs = master_settings.savefigs;
if savefigs
    relpath = group_settings.relpath;
end

% Initialize figure counter
figcount = group_settings.figcount;

% Algorithm names
algnames = master_settings.algnames;

% Current system being executed
systag_disp = master_settings.systag_disp;

% Use uniform color limits on countour plots (=1) or not (=0)
doClim = group_settings.doClim;

% Cost J(x) evaluation settings
Jx_sett = group_settings.Jx_sett;

% Do cost J(x) evaluations (=1) or not (=0)
doJx = Jx_sett.doJx;

% Expected return evaluation settings
ER_sett = group_settings.ER_sett;

% Do ER evaluations (=1) or not (=0)
doER = ER_sett.doER;

% Do plots (=1) or not (=0)
doplots = group_settings.doplots_eval;

% Do policy plots for evaluations (=1) or not (=0) 
doplots_eval_pol = group_settings.doplots_eval_pol;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% ***********************
%
% SURFACE PLOT SETTINGS -- WEIGHT VALUES
%        

% Master plot formatting settings
psett_master = master_settings.psett_master;



% ***********************
%
% SYSTEM PLOT SETTINGS
%

% System names
sysnames = master_settings.sysnames;

% System tag
systag = master_settings.systag;

% Extract system plot settings
sys_plot_settings = master_settings.sys_plot_settings;

% Properties of output variables
y_propts_cell = sys_plot_settings.y_propts_cell;

% State trajectory x(t) unit scaling. E.g., if x_1(t) is angular
% displacement but is desired in degrees, declare the field 'sclvec' and
% put sclvec(1) = 180/pi.
x_sclvec = sys_plot_settings.x_sclvec;

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
% UNPACK ALGORITHM PARAMETERS
%
% *************************************************************************

% Initialize evaluation data (=1) or load previous (=0)
init1load0_eval_data = group_settings.init1load0_eval_data;

% ***********************
%
% LOAD EVALUATION DATA IF USER DESIRED
%   

if ~init1load0_eval_data

    % Relative path, file name of evaluation data
    relpath_data_eval = master_settings.relpath_data_eval;
    
    % Load
    data = load([relpath_data_eval 'eval_data.mat']);
    eval_data = data.eval_data;

end

% Sweep variable indices
inds_x_sweep = group_settings.inds_x_sweep;

% Algorithm list
alg_list = group_settings.alg_list;
if init1load0_eval_data
    alg_list_prev = alg_list;
else
    alg_list_prev = eval_data.alg_list;
end

% Number of algorithms tested
numalgs = size(alg_list,1);

% Get index of MI-DIRL
indmidirl = find(strcmp(alg_list, algnames.rci));
indsnotmidirl = setdiff((1:numalgs)',indmidirl);
if ~isempty(indsnotmidirl)
    indsnotmidirl = indsnotmidirl';
end
hasmidirl = ~isempty(indmidirl);

% x grid data for evaluation
if init1load0_eval_data
    xplot = group_settings.xplot;
else
    xplot = eval_data.xplot;
end

numx10p = xplot.numx10p;
numx20p = xplot.numx20p;
x10vecp = xplot.x10vecp;
x20vecp = xplot.x20vecp;


% Scale these IC vectors to desired units
for i = 1:2
    ind_xri = inds_x_sweep(i);
    currscl = x_sclvec(ind_xri);
    switch i
        case 1
            sx10vecp = x10vecp * currscl;
        case 2
            sx20vecp = x20vecp * currscl;
    end
end

% Create meshgrid of IC values
[Xp, Yp] = meshgrid(sx10vecp, sx20vecp);
       
% Get verticies of IC sweep
x1min = min(sx10vecp);
x1max = max(sx10vecp);
x2min = min(sx20vecp);
x2max = max(sx20vecp);
x1vert = [x1min x1max x1max x1min];
x2vert = [x2min x2min x2max x2max];

% ***********************
%
% FIND INDICES OF CURRENT ALGORITHM LIST IN PREVIOUSLY-CREATED ALGORITHM
% LIST
%   

% Indices of the current algorithm list specified by the user in the
% previously-run algorithm list
algindsprev = zeros(numalgs,1);

% Keeps track of if current algorithm is in the previous list (=1) or not
% (=0)
hasalg = zeros(numalgs,1);

for algcnt = 1:numalgs

    % Current algorithm name
    curralg = alg_list{algcnt};

    % Find where this name is in the list
    indprev = find(strcmp(curralg,alg_list_prev));

    % See if current algorithm is in previous list
    hasalg(algcnt) = ~ isempty(indprev);

    % If current algorithm not present, then leave zero
    if hasalg(algcnt)
        algindsprev(algcnt) = indprev;
    end

end


% ***********************
%
% PULL SYSTEM DATA
%   

% System, lq_data
sys = master_settings.sys;
lq_data_cell = master_settings.lq_data_cell; 
n = sys.n;                      % Number of states
m = sys.m;                      % Number of inputs
model_cell = sys.model_cell;
nummodels = sys.nummodels;
indnom = sys.indnom;

% Previous system
if init1load0_eval_data
    sys_prev = sys; 
else
    sys_prev = eval_data.sys; 
end

% Current values of \nu to plot for (must be a subset of previous, if
% loading data)
nuvec = sys.nuvec;
indsmodelplot = master_settings.indsmodelplot;
nuvec_plot = nuvec(indsmodelplot);

% Previous list of which values of \nu to plot for
% Current indices of \nu to plot for 
if init1load0_eval_data
    nuvec_prev = nuvec;
    indsmodelplot_prev = indsmodelplot;
    nuvec_plot_prev = nuvec_plot;
else
    nuvec_prev = sys_prev.nuvec;
    indsmodelplot_prev = eval_data.indsmodelplot;
    nuvec_plot_prev = nuvec_prev(indsmodelplot_prev);    
end

% Number of \nu values to plot for
nummodelsplot = size(indsmodelplot,1);

% Previous values of \nu to plot for -- index in CURRENT list
indsmodelplotprev = zeros(nummodelsplot,1);

% Keeps track of if current \nu value is in the previous list (=1) or not
% (=0)
hasnu = zeros(nummodelsplot,1);

for mcnt = 1:nummodelsplot

    % Current \nu value
    numcnt = nuvec_plot(mcnt);

    % Find where this \nu is in the previous list
    indprev = find(numcnt == nuvec_plot_prev);

    % See if current \nu is in previous list
    hasnu(mcnt) = ~ isempty(indprev);

    % If current \nu not present, then leave zero
    if hasnu(mcnt)
        indsmodelplotprev(mcnt) = indprev;
    end

end


% Add \nu = [...] at the end of plot titles (=1) or not (=0)
% addnu = nummodelsplot > 1;
addnu = 1;

% Trim data
xe = model_cell{indnom}.trimconds.xe;

% Get Q, R -- LQ servo coords
Q = lq_data_cell{indnom}.Q;
R = lq_data_cell{indnom}.R;

% *************************************************************************
% 
% PULL SWEEP DATA
%
% *************************************************************************

% Holds cFVI/rFVI tabular data (as-relevant)
cfvi_data_cell = cell(numalgs, 1);
cfvi_sweep_data_cell = cell(numalgs, 1);
cfvi_data2_cell = cell(numalgs, 1);

% ***********************
%
% DIRL
%   

% Relative path to DIRL x_0 sweep data
relpath_dirl_sweep = master_settings.relpath_dirl_sweep;

% Load DIRL sweep data
data = load([relpath_dirl_sweep 'alg_settings_cell_master.mat']);
data = data.alg_settings_cell_master;
alg_settings_dirl_sweep = data{1};
data = load([relpath_dirl_sweep 'out_data_cell_master.mat']);
data = data.out_data_cell_master;
out_data_dirl_sweep = data{1};
data = load([relpath_dirl_sweep 'group_settings_master.mat']);
data = data.group_settings_master;
group_settings_dirl_sweep = data{1};

% IC sweep data
ICs = group_settings_dirl_sweep.ICs;
numx10 = ICs.numx10;
numx20 = ICs.numx20;
numICs = ICs.numICs;
x10vec = ICs.x10vec;
x20vec = ICs.x20vec;
x0mat = ICs.x0mat;
indsxe = ICs.indsxe;

% DIRL loop_cell data
loop_cell = alg_settings_dirl_sweep{1}.loop_cell;

% DIRL alg_settings data
alg_settings_dirl = alg_settings_dirl_sweep{1};

% DIRL state index data
inds_dirl = out_data_dirl_sweep{1}.inds;

% ***********************
%
% OTHER
%  

for algcnt = 1:numalgs
  
    % Index of current algorithm in previous list
    indcurprev = algindsprev(algcnt);

switch alg_list{algcnt}


    % ***********************
    %
    % PULL DIRL SWEEP TRAINING DATA
    %   
    case algnames.rci

    % Done already

    % ***********************
    %
    % PULL cFVI DATA
    %   
    case {algnames.cfvi;  algnames.rfvi}
    
        % Relative path to cFVI data
        relpath_py = master_settings.relpath_py;
        filename_py_base = master_settings.filename_py_base;
        filename_py = master_settings.filename_py;
        filename_py2 = master_settings.filename_py_2d;


        % Append 'cFVI' or 'rFVI' tag to file path, name
        relpath_py = [relpath_py alg_list{algcnt} '/'];
        filename_py_base = [alg_list{algcnt} filename_py_base];        
        filename_py = [alg_list{algcnt} filename_py];
        filename_py_sweep = [filename_py_base '_sweep.mat'];
        filename_py2 = [alg_list{algcnt} filename_py2];

        % Load cFVI/rFVI data
        if init1load0_eval_data
%         if init1load0_eval_data || strcmp(alg_list{algcnt}, algnames.rfvi)    
            % Load cFVI data -- n-D
            cfvi_data_cell{algcnt} = load([relpath_py filename_py]);
            % Load cFVI data -- 2D
            cfvi_data2_cell{algcnt} = load([relpath_py filename_py2]);
            % Load cFVI data -- sweep
            cfvi_sweep_data_cell{algcnt} = ...
                load([relpath_py filename_py_sweep]);
        else
            if hasalg(algcnt)               
            % Load cFVI data -- n-D
            cfvi_data_cell{algcnt} = eval_data.cfvi_data_cell{indcurprev};
            % Load cFVI data -- 2D
            cfvi_data2_cell{algcnt} = eval_data.cfvi_data2_cell{indcurprev};
            % Load cFVI data -- sweep
     cfvi_sweep_data_cell{algcnt} = load([relpath_py filename_py_sweep]);
%  cfvi_sweep_data_cell{algcnt} = eval_data.cfvi_sweep_data_cell{indcurprev};
            end
        end     

end

end

% ***********************
%
% PULL OPTIMAL LQ DATA
%   

% Number of loops executed
numloops = size(alg_settings_dirl_sweep{1}.loop_cell,1);


% ***********************
%
% PLOT SETTINGS -- x, y AXIS LABELS FOR IC SWEEP PLOTS
%  

x0surf_labels = cell(2,1);

for i = 1:2

    y_propts = y_propts_cell{i};
    currtexname = y_propts.texname;
    currunits = y_propts.units;
    x0surf_labels{i} = ['$' currtexname '$ (' currunits ')'];

end

% ***********************
%
% PLOT SETTINGS -- STANDARD COLOR COUNTOUR PLOT SETTINGS
%  

psett_contourf.alg_list = alg_list;
psett_contourf.indmidirl = indmidirl;
psett_contourf.indsnotmidirl = indsnotmidirl;
psett_contourf.nuvec = nuvec;
psett_contourf.indsmodelplot = indsmodelplot;
psett_contourf.Xp = Xp;
psett_contourf.Yp = Yp;
psett_contourf.psett_master = psett_master;
psett_contourf.doClim = doClim;
psett_contourf.x1min = x1min;
psett_contourf.x1max = x1max;
psett_contourf.x2min = x2min;
psett_contourf.x2max = x2max;
psett_contourf.x0surf_labels = x0surf_labels;
psett_contourf.savefigs = savefigs;
if savefigs
    psett_contourf.relpath = relpath;
end
psett_contourf.addnu = addnu;

% ***********************
%
% PLOT SETTINGS -- LEARNING CURVE PLOTS
%  

psett_lc.psett_master = psett_master;
psett_lc.alg_list = alg_list;
psett_lc.savefigs = savefigs;
if savefigs
    psett_lc.relpath = relpath;
end
psett_lc.filename = 'learning_curves';
psett_lc.color_sett_cell = group_settings.color_sett_cell;
psett_lc.ttl = [systag_disp];


% ***********************
%
% PLOT SETTINGS -- TITLES FOR VALUE FUNCTION, POLICY PLOTS
%  

% Value function
ttl_V = 'Critic NN $V(x)$';

% Cost function
ttl_J = 'Cost $J(x)$';

% Policy (before enumeration)
ttl_u = 'Policy $\mu_{';

% ***********************
%
% MISC PLOT SETTINGS
%   

% Number of contours for contour plots
ncontour = 50;

% *************************************************************************
% 
% PULL, CALCULATE SWEEP DATA
%
% *************************************************************************

% ***********************
%
% PULL SWEEP DATA
%  

if hasmidirl

sweep_lq_data_cell = cell(numx10,numx20,nummodels);

for x1c = 1:numx10
    for x2c = 1:numx20
        for mcnt = 1:nummodels

            % Extract current out_data struct
            outdata = out_data_dirl_sweep{x1c,x2c,mcnt};

            % Extract lq_data
            lqdata = outdata.dirl_data.lq_data;

            % Store lq_data
            sweep_lq_data_cell{x1c,x2c,mcnt} = lqdata;


        end
    end
end

end


% *************************************************************************
% 
% CALCULATE RUNTIME DATA OVER SWEEP
%
% *************************************************************************

% Contains averages taken over IC sweep for each modeling error
avg_runtime_model = zeros(numalgs,nummodels);

% Contains total average over all IC sweeps, all modeling errors
avg_runtime = zeros(numalgs,1);

for algcnt = 1:numalgs

switch alg_list{algcnt}

    % ***********************
    %
    % MI-DIRL
    %  
    case algnames.rci

        avg_runtime_midirl_model = zeros(nummodels,1);
        avg_runtime_midirl = 0;
        
        for x1c = 1:numx10
            for x2c = 1:numx20
                for mcnt = 1:nummodels
        
                    % Extract current out_data struct
                    outdata = out_data_dirl_sweep{x1c,x2c,mcnt};
        
                    % Extract runtime data
                    runtime = outdata.runtime;
        
                    % Update cumulative sums
                    avg_runtime_midirl_model(mcnt) = ...
                        avg_runtime_midirl_model(mcnt) + runtime;
                    avg_runtime_midirl = avg_runtime_midirl + runtime;
                
        
                end
            end
        end
        
        % Divide by sample sizes
        avg_runtime_model(algcnt,:) = ...
            avg_runtime_midirl_model / (numx10*numx20);
        avg_runtime(algcnt) = ...
            avg_runtime_midirl / (numx10*numx20*nummodels);

    % ***********************
    %
    % cFVI
    %  
    case {algnames.cfvi; algnames.rfvi}
        
        cfvi_data = cfvi_data_cell{algcnt};
        runtime_vec_cfvi = cfvi_data.runtime_vec;
        total_runtime_cfvi = sum(runtime_vec_cfvi); 
        avg_runtime_cfvi = mean(runtime_vec_cfvi);
        avg_runtime_model(algcnt,:) = avg_runtime_cfvi;
        avg_runtime(algcnt) = avg_runtime_cfvi;

end


end


% *************************************************************************
%
% PULL CONTROLLERS, RICCATI EQUATION SOLUTIONS AND CONTROLLERS
%
% Corresponding to the MI-DIRL algorithm data run at IC x_0 = x_e
%  
% *************************************************************************

if hasmidirl


% Stores composite controllers for each model (in DIRL coords)
Kirl_cell = cell(nummodels, 1);

% Stores composite controllers for each model (in LQ servo coords)
Klq_cell = cell(nummodels, 1);

% Stores Riccati equation solutions for each model
Pj_cell = cell(numloops,nummodels);     % Stores for each loop
% Stores composite (in DIRL coords)
P_cell = cell(nummodels, 1);               


for mcnt = 1:nummodels

    % Extract current out_data struct
    outdata = out_data_dirl_sweep{indsxe(1),indsxe(2),mcnt};

    % Extract current lq_data struct
    lqdata = sweep_lq_data_cell{indsxe(1),indsxe(2),mcnt};

    % Init composite P
    Pmcnt = [];

    % For each of the loops j, get P_{i^*, j}
    for j = 1:numloops
        
        % Get final Riccati equation solution for this model
        Pistarj = outdata.P_cell{j}(:,:,end);   

        % Store
        Pj_cell{j,mcnt} = Pistarj;

        % Make block-diagonal composite P
        Pmcnt = blkdiag(Pmcnt,Pistarj);
                       
    end
    
    % Store P
    P_cell{mcnt} = Pmcnt;

    % Store K -- DIRL coords
    Kirl_cell{mcnt} = lqdata.Kirl;

    % Store K -- LQ servo coords
    Klq_cell{mcnt} = lqdata.K;

end

end

% *************************************************************************
%
% INITIALIZE GRID OF EVALUATION VECTORS
%
% In both: 
%   Plant state x_p (pre-transformation, NOT shifted by trim)
%   and MI-DIRL coords (pre-transformation, shifted by trim)
%  
% *************************************************************************

x0mat_dirl = zeros(numx10p, numx20p, m + n);
xp0mat = zeros(numx10p, numx20p, n);

for x1c = 1:numx10p

    % Current evaluation point in state x_1
    x10 = x10vecp(x1c);

    % Current evaluation point in state x_1 (trim = 0)
    tx10 = x10 - xe(inds_x_sweep(1));

    for x2c = 1:numx20p

        % Current evaluation point in state x_1
        x20 = x20vecp(x2c);

        % Current evaluation point in state x_2 (trim = 0)
        tx20 = x20 - xe(inds_x_sweep(2));

        % Current evaluation point (trim = 0)
        tx0 = [tx10; tx20];

        % Current evaluation point: integrator states zero (in DIRL coords,
        % shifted by trim)
        txirl = [];
        for j = 1:numloops
            nxj = size(loop_cell{j}.indsx, 1);
            nyj = size(loop_cell{j}.indsy, 1);
            txj = [zeros(nyj,1); tx0(j); zeros(nxj-1,1)];
            txirl = [txirl; txj];
        end    

        % Plant state x_p (pre-trans, NOT shifted by trim)
        xp = zeros(n, 1);
        xp(inds_x_sweep(1)) = x10;
        xp(inds_x_sweep(2)) = x20;

        % Store
        x0mat_dirl(x1c, x2c, :) = txirl;
        xp0mat(x1c, x2c, :) = xp;

    end
end

% *************************************************************************
% *************************************************************************
%
% CALCULATE VALUE FUNCTION, POLICY DATA OVER THE PLOT SWEEP
%  
% *************************************************************************
% *************************************************************************


% Stores value function data over the sweep for each algorithm
V_x_cell = cell(numalgs,1);

% Stores value policy data over the sweep for each algorithm
u_x_cell = cell(numalgs,1);

% Cell array containing grid vectors for each cFVI/rFVI algorithm
cfvi_xgridvec_cell = cell(numalgs, 1);


% *************************************************************************
%
% OPTION 1: CALCULATE
%  
% *************************************************************************

if init1load0_eval_data

for algcnt = 1:numalgs

switch alg_list{algcnt}

   
% *************************************************************************
%
% CALCULATE MI-DIRL VALUE FUNCTION, POLICY OVER THE PLOT SWEEP
%  
% *************************************************************************

    case algnames.rci    

V_x_midirl = zeros(numx10p,numx20p,nummodelsplot);
u_x_midirl = zeros(numx10p,numx20p,m,nummodelsplot);

for x1c = 1:numx10p

    % Current evaluation point in state x_1 (trim = 0)
    tx10 = x10vecp(x1c) - xe(inds_x_sweep(1));

    for x2c = 1:numx20p

        % Current evaluation point in state x_2 (trim = 0)
        tx20 = x20vecp(x2c) - xe(inds_x_sweep(2));

        % Current evaluation point (trim = 0)
        tx0 = [tx10; tx20];

        % Current evaluation point: integrator states zero (in DIRL coords)
        txirl = [];
        for j = 1:numloops
            nxj = size(loop_cell{j}.indsx, 1);
            nyj = size(loop_cell{j}.indsy, 1);
            switch numloops
                case 1
                    if hasintaug
                        txj = [zeros(nyj,1); tx0(:); zeros(nxj-2,1)];
                    else
                        txj = [tx0(:); zeros(nxj-2,1)];
                    end 
                case 2 
                    if hasintaug
                        txj = [zeros(nyj,1); tx0(j); zeros(nxj-1,1)];
                    else
                        txj = [tx0(j); zeros(nxj-1,1)];
                    end                    
            end
            txirl = [txirl; txj];
        end    

        for mcnt = 1:nummodelsplot

            % Currrent model index
            imcnt = indsmodelplot(mcnt);

            % Extract P-matrix for this model
            Pmcnt = P_cell{imcnt};

            % Extract K-matrix for this model
            Kmcnt = Kirl_cell{imcnt};

            % Evaluate value function V(x) = x^T P x
            V_x_midirl(x1c,x2c,mcnt) = txirl' * Pmcnt * txirl;

            % Evaluate policy u = - K x
            u_x_midirl(x1c,x2c,:,mcnt) = - Kmcnt * txirl;

        end
    end
end

% STORE
V_x_cell{algcnt} = V_x_midirl;
u_x_cell{algcnt} = u_x_midirl;

% *************************************************************************
% 
% EXTRACT cFVI GRID DATA
%
% NOTE: 
%       The 2D grid data (not the n-D grid data) is used for plotting the
%       value function estimate and policy.
%       The n-D grid data is used for the policy lookup in the simulations
%       performed when calculating the cost J(x)
%
% *************************************************************************

    case {algnames.cfvi; algnames.rfvi}

% Current cFVI/rFVI data
cfvi_data = cfvi_data_cell{algcnt};
cfvi_data2 = cfvi_data2_cell{algcnt};

% ***********************
%
% n-D GRID DATA
%  

cfvi_x_tbl_min = cfvi_data.x_tbl_min;
cfvi_x_tbl_max = cfvi_data.x_tbl_max;
cfvi_x_tbl_nxpts = cfvi_data.x_tbl_nxpts;
cfvi_V_tbl =  cfvi_data.V_tbl;
cfvi_u_tbl =  cfvi_data.u_tbl;

% NOTE: Need to negate the V(x) data for cFVI
cfvi_V_tbl = - cfvi_V_tbl;

% ***********************
%
% cFVI GRID VECTORS -- n-D
%  

% Get number of states in cFVI plant
n_cfvi = length(cfvi_x_tbl_min);

% Cell array containing grid vectors
cfvi_xgridvec_cell_curr = cell(n_cfvi, 1);

% Initialize grid vectors
for i = 1:n_cfvi
    cfvi_xgridvec_cell_curr{i} = ...
      linspace(cfvi_x_tbl_min(i), cfvi_x_tbl_max(i), cfvi_x_tbl_nxpts(i))';
end

% Store grid vectors in master cell array
cfvi_xgridvec_cell{algcnt} = cfvi_xgridvec_cell_curr;

% ***********************
%
% 2D GRID DATA
%  

cfvi_x_tbl_min2 = cfvi_data2.x_tbl_min;
cfvi_x_tbl_max2 = cfvi_data2.x_tbl_max;
cfvi_x_tbl_nxpts2 = cfvi_data2.x_tbl_nxpts;
cfvi_V_tbl2 =  cfvi_data2.V_tbl;
cfvi_u_tbl2 =  cfvi_data2.u_tbl;

% NOTE: Need to negate the V(x) data for cFVI/rFVI
cfvi_V_tbl2 = - cfvi_V_tbl2;

% ***********************
%
% cFVI GRID VECTORS -- 2D
%  

% Grid coordinate point vectors
X1 = linspace(cfvi_x_tbl_min2(1), cfvi_x_tbl_max2(1), cfvi_x_tbl_nxpts2(1))';
X2 = linspace(cfvi_x_tbl_min2(2), cfvi_x_tbl_max2(2), cfvi_x_tbl_nxpts2(2))';

% Cell array containing grid vectors
cfvi_xgridvec_cell2 = cell(2, 1);
cfvi_xgridvec_cell2{1} = X1;
cfvi_xgridvec_cell2{2} = X2;

% ***********************
%
% INTERPOLATE 2D cFVI VALUE FUNCTION, POLICY DATA ON MI-DIRL GRID
%  

cfvi_V_tbl2_p = zeros(numx10p,numx20p);
cfvi_u_tbl2_p = zeros(numx10p,numx20p,m);

for x1c = 1:numx10p

    % Current evaluation point in state x_1 (trim = 0)
    tx10 = x10vecp(x1c) - xe(inds_x_sweep(1));

    for x2c = 1:numx20p

        % Current evaluation point in state x_2 (trim = 0)
        tx20 = x20vecp(x2c) - xe(inds_x_sweep(2));

        % Current evaluation point (trim = 0)
        tx0 = [tx10; tx20];

        % Call interpolation function -- value function
        cfvi_V_tbl2_p(x1c, x2c) = ...
            interpn_cell(cfvi_xgridvec_cell2, cfvi_V_tbl2, tx0);

        % Call interpolation function -- policy
        cfvi_u_tbl2_p(x1c, x2c, :) = ...
            interpn_cell(cfvi_xgridvec_cell2, cfvi_u_tbl2, tx0);        

    end
end

% STORE
cfvi_V_tbl2_p_nu = zeros(numx10p,numx20p,nummodelsplot);
cfvi_u_tbl2_p_nu = zeros(numx10p,numx20p,m,nummodelsplot);
for mcnt = 1:nummodelsplot

   cfvi_V_tbl2_p_nu(:,:,mcnt) = cfvi_V_tbl2_p;
   cfvi_u_tbl2_p_nu(:,:,:,mcnt) = cfvi_u_tbl2_p;

end
V_x_cell{algcnt} = cfvi_V_tbl2_p_nu;
u_x_cell{algcnt} = cfvi_u_tbl2_p_nu;

end             % END switch alg_list{algcnt}

end             % END for algcnt = 1:numalgs

else            % ELSE if init1load0

% *************************************************************************
%
% OPTION 2: LOAD FROM PREVIOUS
%  
% *************************************************************************

% Pull from previous eval
V_x_cell_prev = eval_data.V_x_cell;
u_x_cell_prev = eval_data.u_x_cell;
cfvi_xgridvec_cell_prev = eval_data.cfvi_xgridvec_cell;

% Re-order to match current list
for algcnt = 1:numalgs

    % Index of current algorithm in previous list
    indcurprev = algindsprev(algcnt);

    % Initialize temp variables
    Vxtmp = zeros(numx10p,numx20p,nummodelsplot);
    uxtmp = zeros(numx10p,numx20p,m,nummodelsplot);

    % If current algorithm is in previous list, store data
    if hasalg(algcnt)

        for mcnt = 1:nummodelsplot

            % Index of current model in previous data
            indmcntprev = indsmodelplotprev(mcnt);

            % Fill data
            Vxtmp(:,:,mcnt) = V_x_cell_prev{indcurprev}(:,:,indmcntprev);
            uxtmp(:,:,:,mcnt) = ...
                u_x_cell_prev{indcurprev}(:,:,:,indmcntprev);

        end

        % Store
        V_x_cell{algcnt} = Vxtmp;
        u_x_cell{algcnt} = uxtmp;
        cfvi_xgridvec_cell{algcnt} = cfvi_xgridvec_cell_prev{indcurprev};

    end

end

end             % END if init1load0

  

%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE POLICY COST J(x)
%
% ************************************************************************* 
% *************************************************************************

% *************************************************************************
%
% OPTION 1: CALCULATE MANUALLY
%
% ************************************************************************* 

% Storage
J_x_cell = cell(numalgs,1);

if doJx
if init1load0_eval_data


% ***********************
%
% INITIALIZE ALGORITHM SETTINGS
%  

% System
algsettmp.sys = sys;
algsettmp.lin1nonlin0 = 0;      % Nonlinear simulation

% Integration horizon length
algsettmp.T = Jx_sett.T;

% Q, R
algsettmp.Q = Q;
algsettmp.R = R;

% Termination threshold
algsettmp.donormeps = Jx_sett.donormeps;
algsettmp.normeps = Jx_sett.normeps;

% Index of nominal model
algsettmp.model_nom_ind = indnom;

% IC matrix
algsettmp.x0mat = xp0mat;


% ***********************
%
% EVALUATE COST
%  

for algcnt = 1:numalgs

    % Current algorithm
    algi = alg_list{algcnt};

    % Set algorithm-specific settings
    alg_setts = algsettmp;
    alg_setts.algtag = algi;
    switch algi
        case algnames.rci
            alg_setts.utag = 'lq_servo';            
        case {algnames.cfvi; algnames.rfvi}
            alg_setts.utag = 'cfvi_lookup';
            alg_setts.xgridvec_cell = cfvi_xgridvec_cell{algcnt};
            alg_setts.u_tbl = cfvi_data_cell{algcnt}.u_tbl;
    end

    % Temp storage
    Jxtmp = zeros(numx10p,numx20p,nummodelsplot);

    % Evaluate cost for each model
    for mcnt = 1:nummodelsplot

        % Index of current model
        imcnt = indsmodelplot(mcnt);

        % DEBUGGING: Display current algorithm, model
        disp('% *********************************************************')
        disp('% *********************************************************')
        disp('%')
        disp(['% EVALUATING COST J(x) FOR ALGORITHM -- ' algi])
        disp(['% AND MODEL    ' num2str(mcnt) '   OUT OF  ' ...
            num2str(nummodelsplot)])
        disp(['% \nu =     ' num2str(nuvec(imcnt))])
        disp('%')
        disp('% *********************************************************')
        disp('% *********************************************************')

        % Set simulation model index
        alg_setts.model_sim_ind = imcnt;

        % Set model-specific settings for this algorithm
        switch algi
            case {algnames.cfvi; algnames.rfvi}
                % Evaluate cost over IC matrix          
                Jx =...
             alg_eval_cost(alg_setts, group_settings, master_settings);
                % Store
                Jxtmp(:,:,mcnt) = Jx;                                                  
            case algnames.rci
                % Set current model controller
                alg_setts.K = Klq_cell{imcnt};
                % Evaluate cost over IC matrix
                Jx = ...
                 alg_eval_cost(alg_setts, group_settings, master_settings);
                % Store
                Jxtmp(:,:,mcnt) = Jx;
        end

    end

    % Store cost data for this algorithm
    J_x_cell{algcnt} = Jxtmp;

end

% *************************************************************************
%
% OPTION 2: LOAD FROM PREVIOUS
%
% ************************************************************************* 

else

    % Pull from evaluation data
    J_x_cell_prev = eval_data.J_x_cell;

    % Re-order to match current list
    for algcnt = 1:numalgs

        % Index of current algorithm in previous list
        indcurprev = algindsprev(algcnt);

        % Previus J(x) data for this alg
        if hasalg(algcnt)
            Jxprev = J_x_cell_prev{indcurprev};
        else
            Jxprev = nan * J_x_cell_prev{1};
        end

        % Get current J(x) data
        Jxtmp = zeros(numx10p,numx20p,nummodelsplot);

        for mcnt = 1:nummodelsplot

            % Get current J(x) data for this model
            Jxtmp(:,:,mcnt) = Jxprev(:,:,indsmodelplotprev(mcnt));

        end

        % Set
        J_x_cell{algcnt} = Jxtmp;

    end

end 
end


%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE EXPECTED RETURN
%
% ************************************************************************* 
% *************************************************************************

% *************************************************************************
%
% OPTION 1: CALCULATE MANUALLY
%
% ************************************************************************* 

if doER
if init1load0_eval_data

% ***********************
%
% INITIALIZE SEEDS
%  

% Number of seeds
n_seed = ER_sett.n_seed;

% Number of simulations for evaluating ER
nsim_ER = ER_sett.nsim;

% ***********************
%
% INITIALIZE ALGORITHM SETTINGS
%  

% System
alg_setts.sys = sys;
alg_setts.lin1nonlin0 = 0;      % Nonlinear simulation

% Integration horizon length
alg_setts.T = Jx_sett.T;

% Q, R
alg_setts.Q = Q;
alg_setts.R = R;

% Termination threshold
alg_setts.donormeps = Jx_sett.donormeps;
alg_setts.normeps = Jx_sett.normeps;

% Index of nominal model
alg_setts.model_nom_ind = indnom;

% Number of seeds, simulations
alg_setts.n_seed = n_seed;
alg_setts.nsim = nsim_ER;

% Set algorithm-specific settings
alg_setts.algtag = algnames.rci;
alg_setts.utag = 'lq_servo';  
alg_setts.model_sim_ind = indnom;

% Algorithm hyperparameters
algsetttmp = alg_settings_dirl;
% Group settings
tmpgroupset = group_settings;
tmpgroupset.dolearning = 1;
tmpgroupset.savetrainingdata = 0;
tmpgroupset.dopostlearning = 0;
tmpgroupset.clearglobal = 0;

switch systag
    case sysnames.pendulum
        loop_cell{1}.istar = 150;
        algsetttmp.Uc0vec = 10;
    case sysnames.businjet
        loop_cell{1}.istar = 100;
        loop_cell{2}.istar = 100;     
        algsetttmp.Uc0vec = [25; 1];
    case sysnames.ddmr
        loop_cell{1}.istar = 100;
        loop_cell{2}.istar = 100;   
        algsetttmp.Uc0vec = [25; 10];
end
alg_setts.loop_cell = loop_cell;
algsetttmp.loop_cell = loop_cell;
alg_setts.alg_hyperparams = algsetttmp;

% Indexing
alg_setts.inds_dirl = inds_dirl;

% Get max number of iterations
istarmax = -1;
for j = 1:m
    istarmax = max([istarmax loop_cell{j}.istar]);
end
alg_setts.istarmax = istarmax;



% ***********************
%
% EVALUATE EXPECTED RETURN
%  

% Evaluate cost for each model
[meanJxmat, outdata_ER] = ...
    alg_eval_ER(alg_setts, tmpgroupset, master_settings);

% Extract data
ERmat = - meanJxmat;
ERruntimevec = outdata_ER.runtimevec;

% *************************************************************************
%
% OPTION 2: LOAD FROM PREVIOUS
%
% ************************************************************************* 

else

    ERmat = eval_data.ERmat;
    ERruntimevec = eval_data.ERruntimevec;

end

% Storage
cfvi_sweep_data_cell{indmidirl}.meanreturn = ERmat;
cfvi_sweep_data_cell{indmidirl}.runtime = ERruntimevec;

end




%%
% *************************************************************************
% *************************************************************************
% 
% CALCULATE DIFFERENCE DATA
%
% *************************************************************************
% *************************************************************************

% ***********************
%
% VALUE FUNCTION DIFFERENCE: (cFVI) - (MI-DIRL)
%  

V_x_diff_cell = cell(numalgs,1);

if hasmidirl

for algcnt = indsnotmidirl

    % Temp
    Vxdifftmp = zeros(numx10p, numx20p, nummodelsplot);

    % Calculate difference for each model
    for mcnt = 1:nummodelsplot
                 
        % Get data
        Vxtmp_midirl = V_x_cell{indmidirl}(:,:,mcnt);
        Vxtmp = V_x_cell{algcnt}(:,:,mcnt);
    
        % Calculate difference
        Vxdifftmp(:,:,mcnt) = (Vxtmp - Vxtmp_midirl);     
    
    end

    % Store
    V_x_diff_cell{algcnt} = Vxdifftmp;

end

end

% ***********************
%
% COST FUNCTION DIFFERENCE: (cFVI) - (MI-DIRL)
%  

if doJx && hasmidirl

    J_x_diff_cell = cell(numalgs,1);

    for algcnt = indsnotmidirl

        % Temp
        Jxdifftmp = zeros(numx10p, numx20p, nummodelsplot);
        
        % Calculate difference for each model
        for mcnt = 1:nummodelsplot      
        
            % Get data
            Jxtmp_midirl = J_x_cell{indmidirl}(:,:,mcnt);
            Jxtmp = J_x_cell{algcnt}(:,:,mcnt);
        
            % Calculate difference
            Jxdifftmp(:,:,mcnt) = (Jxtmp - Jxtmp_midirl);
                
        end

        % Store
        J_x_diff_cell{algcnt} = Jxdifftmp;

    end

end

% ***********************
%
% COST/VALUE FUNCTION DIFFERENCE J(x) - V(x)
%  

if doJx
    
    JmV_x_cell = cell(numalgs,1);

    for algcnt = 1:numalgs

        % Temp
        JmV_x_tmp = zeros(numx10p, numx20p, nummodelsplot);

        % Calculate data for each model
        for mcnt = 1:nummodelsplot     

            % Get value data
            Vxtmp = V_x_cell{algcnt}(:,:,mcnt);
        
            % Get cost data
            Jxtmp = J_x_cell{algcnt}(:,:,mcnt);
        
            % Calculate J - V
            JmV_x_tmp(:,:,mcnt) = (Jxtmp - Vxtmp);
          
        
        end

        % Store
        JmV_x_cell{algcnt} = JmV_x_tmp;

    end

end

% ***********************
%
% COST/VALUE FUNCTION DIFFERENCE J(x) - V(x): (cFVI) - (MI-DIRL) 
%  

if doJx && hasmidirl
    
    JmV_x_diff_cell = cell(numalgs,1);

    for algcnt = indsnotmidirl

        % Temp
        JmV_x_diff_tmp = zeros(numx10p, numx20p, nummodelsplot);

        % Calculate data for each model
        for mcnt = 1:nummodelsplot     
        
            % Get MI-DIRL data
            JmVxtmp_midirl = JmV_x_cell{indmidirl}(:,:,mcnt);
        
            % Get other alg data
            JmVxtmp = JmV_x_cell{algcnt}(:,:,mcnt);
        
            % Calculate difference
            JmV_x_diff_tmp(:,:,mcnt) = (JmVxtmp - JmVxtmp_midirl);  
        
        end

        % Store
        JmV_x_diff_cell{algcnt} = JmV_x_diff_tmp;

    end

end


%%
% *************************************************************************
% *************************************************************************
% 
% SET COLORMAP RANGES AND CONTOUR VALUES BASED ON SWEEP DATA
%
% *************************************************************************
% *************************************************************************

% ***********************
%
% VALUE FUNCTION
%  

% Color range normalization
tmp = [];
for algcnt = 1:numalgs
    tmp = [tmp; V_x_cell{algcnt}(:)];
end

CLim_V = [min(tmp) max(tmp)];

% Contour values
cvals_V = linspace(CLim_V(1), CLim_V(2), ncontour);

% ***********************
%
% COST FUNCTION
%  

% Color range normalization
if doJx

    tmp = [];
    for algcnt = 1:numalgs
        tmp = [tmp; J_x_cell{algcnt}(:)];
    end

    CLim_J = [min(tmp) max(tmp)];

    % Contour values
    cvals_J = linspace(CLim_J(1), CLim_J(2), ncontour);
end



% ***********************
%
% VALUE FUNCTION DIFFERENCE PLOTS: (cFVI) - (MI-DIRL)
%  

CLim_V_diff = zeros(2,1);
CLim_V_diff(1) = inf;
CLim_V_diff(2) = -inf;

if hasmidirl

% Include data for each non-MI-DIRL algorithm
for algcnt = indsnotmidirl

    for mcnt = 1:nummodelsplot
    
        % Get data
        Vxdifftmp = V_x_diff_cell{algcnt}(:,:,mcnt);
        
        % Update color limits
        CLim_V_diff(1) = min([CLim_V_diff(1); Vxdifftmp(:)]);
        CLim_V_diff(2) = max([CLim_V_diff(2); Vxdifftmp(:)]);        
    
    end

end

% Contour values
cvals_V_diff = linspace(CLim_V_diff(1), CLim_V_diff(2), ncontour);

end


% ***********************
%
% COST FUNCTION DIFFERENCE PLOTS: (cFVI) - (MI-DIRL)
%  

if doJx && hasmidirl

    CLim_J_diff = zeros(2,1);
    CLim_J_diff(1) = inf;
    CLim_J_diff(2) = -inf;

    % Include data for each non-MI-DIRL algorithm
    for algcnt = indsnotmidirl
    
        for mcnt = 1:nummodelsplot     
            
            % Get data
            Jxdifftmp = J_x_diff_cell{algcnt}(:,:,mcnt);
            
            % Update color limits    
            CLim_J_diff(1) = min([CLim_J_diff(1); Jxdifftmp(:)]);
            CLim_J_diff(2) = max([CLim_J_diff(2); Jxdifftmp(:)]);  
        
        end

    end
    
    % Contour values
    cvals_J_diff = linspace(CLim_J_diff(1), CLim_J_diff(2), ncontour);

end

% ***********************
%
% COST/VALUE FUNCTION DIFFERENCE J(x) - V(x) PLOTS
%  

if doJx

    CLim_JmV = zeros(2,1);
    CLim_JmV(1) = inf;
    CLim_JmV(2) = -inf;
    
    for algcnt = 1:numalgs
    
        for mcnt = 1:nummodelsplot
        
            % Get J - V
            JxmVxtmp = JmV_x_cell{algcnt}(:,:,mcnt);
 
            % Update color limits    
            CLim_JmV(1) = min([CLim_JmV(1); JxmVxtmp(:)]);
            CLim_JmV(2) = max([CLim_JmV(2); JxmVxtmp(:)]);  
        
        end

    end
    
    % Contour values
    cvals_JmV = linspace(CLim_JmV(1), CLim_JmV(2), ncontour);

end


% ***********************
%
% COST/VALUE FUNCTION DIFFERENCE J(x) - V(x) PLOTS -- ALGORITHM DIFFERENCE
%  

if doJx && hasmidirl

    CLim_JmV_diff = zeros(2,1);
    CLim_JmV_diff(1) = inf;
    CLim_JmV_diff(2) = -inf;
    
    for algcnt = indsnotmidirl
    
        for mcnt = 1:nummodelsplot
        
            % Get J - V
            tmp = JmV_x_diff_cell{algcnt}(:,:,mcnt);
 
            % Update color limits    
            CLim_JmV_diff(1) = min([CLim_JmV_diff(1); tmp(:)]);
            CLim_JmV_diff(2) = max([CLim_JmV_diff(2); tmp(:)]);  
        
        end

    end
    
    % Contour values
    cvals_JmV_diff = ...
        linspace(CLim_JmV_diff(1), CLim_JmV_diff(2), ncontour);

end


% ***********************
%
% POLICY
%  

CLim_u = zeros(m,2);
cvals_u = zeros(m,ncontour);

for i = 1:m

    % Initialize min, max
    minui = inf;
    maxui = -inf;

    % Include policy data for each algorithm
    for algcnt = 1:numalgs

        switch alg_list{algcnt}

            case algnames.cfvi

                % Get \mu_i data -- cFVI
                uitmp = u_x_cell{algcnt}(:,:,i);
            
                % Update color limits
                minui = min([minui; uitmp(:)]);
                maxui = max([maxui; uitmp(:)]);
        
            otherwise

                % Include data for each model
                for mcnt = 1:nummodelsplot     
                    
                    % Get \mu_i data -- MI-DIRL at this modeling error
                    uitmp = u_x_cell{algcnt}(:,:,i,mcnt);
            
                    % Update color limits
                    minui = min([minui; uitmp(:)]);
                    maxui = max([maxui; uitmp(:)]);        
                
                end
        
        end

    end

    % Set color limits for this control
    CLim_u(i,1) = minui;
    CLim_u(i,2) = maxui;

    % Set color values for this control
    cvals_u(i,:) = linspace(CLim_u(i,1), CLim_u(i,2), ncontour);


end


if doplots

%%
% *************************************************************************
% *************************************************************************
%
% PLOT: VALUE FUNCTION
%
% ************************************************************************* 
% *************************************************************************

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 0;
settstmp.data_cell = V_x_cell;
settstmp.cvals = cvals_V;
settstmp.Clim = CLim_V;
settstmp.ttl_base = ttl_V;
settstmp.fnmp = 'Vx';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: VALUE FUNCTION -- ALGORITHM DIFFERENCE
%
% ************************************************************************* 
% *************************************************************************

if hasmidirl

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 1;
settstmp.data_cell = V_x_diff_cell;
settstmp.cvals = cvals_V_diff;
settstmp.Clim = CLim_V_diff;
settstmp.ttl_base = 'V';
settstmp.fnmp = 'Vx_diff';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);

end

%%
% *************************************************************************
% *************************************************************************
%
% PLOT: COST FUNCTION J(x)
%
% ************************************************************************* 
% *************************************************************************

if doJx

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 0;
settstmp.data_cell = J_x_cell;
settstmp.cvals = cvals_J;
settstmp.Clim = CLim_J;
settstmp.ttl_base = ttl_J;
settstmp.fnmp = 'Jx';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);

end


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: COST FUNCTION -- ALGORITHM DIFFERENCE
%
% ************************************************************************* 
% *************************************************************************

if doJx && hasmidirl

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 1;
settstmp.data_cell = J_x_diff_cell;
settstmp.cvals = cvals_J_diff;
settstmp.Clim = CLim_J_diff;
settstmp.ttl_base = 'J';
settstmp.fnmp = 'Jx_diff';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);

end


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: COST/VALUE FUNCTION DIFFERENCE J(x) - V(x)
%
% ************************************************************************* 
% *************************************************************************

if doJx

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 0;
settstmp.data_cell = JmV_x_cell;
settstmp.cvals = cvals_JmV;
settstmp.Clim = CLim_JmV;
settstmp.ttl_base = '$J(x) - V(x)$';
settstmp.fnmp = 'JxmVx';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);

end



%%
% *************************************************************************
% *************************************************************************
%
% PLOT: COST/VALUE FUNCTION DIFFERENCE J(x) - V(x) -- ALGORITHM DIFFERENCE
%
% ************************************************************************* 
% *************************************************************************

if 0
% if doJx && hasmidirl

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 1;
settstmp.data_cell = JmV_x_diff_cell;
settstmp.cvals = cvals_JmV_diff;
settstmp.Clim = CLim_JmV_diff;
settstmp.ttl_base = '(J - V)';
settstmp.fnmp = 'JxmVx_diff';
settstmp.figcount = figcount;

% Plot 
figcount = plot_contourf_multi(settstmp);

end




%%
% *************************************************************************
% *************************************************************************
%
% PLOT: POLICY
%
% ************************************************************************* 
% *************************************************************************

if doplots_eval_pol

% Initialize contour plot settings
settstmp = psett_contourf;
settstmp.isdiffplot = 0;


for i = 1:m

    % Initialize cell for \mu_i data
    ui_cell = cell(numalgs,1);

    % Pull \mu_i data for this i
    for algcnt = 1:numalgs
        ui_cell{algcnt} = u_x_cell{algcnt}(:,:,i,:);
    end

    % Set plot settings
    settstmp.data_cell = ui_cell;
    settstmp.cvals = cvals_u(i,:);
    settstmp.Clim = CLim_u(i,:);
    settstmp.ttl_base = ['$\mu_{' num2str(i) '}$'];
    settstmp.fnmp = ['u' num2str(i) 'x'];
    settstmp.figcount = figcount;

    % Plot 
    figcount = plot_contourf_multi(settstmp);

end

end         % END if doplots_eval_pol

end         % END if doplots_eval


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: LEARNING CURVES
%
% ************************************************************************* 
% *************************************************************************


% Initialize contour plot settings
settstmp = psett_lc;
settstmp.data_cell = cfvi_sweep_data_cell;
settstmp.figcount = figcount;
settstmp.indmidirl = indmidirl;
settstmp.systag = systag;
settstmp.sysnames = sysnames;

% Plot 
[figcount, avgruntimevec_cfvi, meanreturnvec, ...
    stdreturnvec] = plot_lc(settstmp);

% *************************************************************************
% 
% UPDATE FVI RUNTIME DATA 
%
% *************************************************************************

for algcnt = 1:numalgs

switch alg_list{algcnt}


    % ***********************
    %
    % cFVI
    %  
    case {algnames.cfvi; algnames.rfvi}
        
        avg_runtime_model(algcnt,:) = avgruntimevec_cfvi(algcnt);
        avg_runtime(algcnt) = avgruntimevec_cfvi(algcnt);

    otherwise

        % Nothing

end

end




%%
% *************************************************************************
% *************************************************************************
%
% PRINT SWEEP METRICS
%
% ************************************************************************* 
% *************************************************************************

% ***********************
%
% STORAGE
%
% NOTE: Indexed by row: MI-DIRL, cFVI, (MI-DIRL)-(cFVI)
%       Indexed by column: \nu
%  

% V(x) data
minVxmat = zeros(numalgs,nummodelsplot);
maxVxmat = zeros(numalgs,nummodelsplot);
meanVxmat = zeros(numalgs,nummodelsplot);
stdVxmat = zeros(numalgs,nummodelsplot);

% J(x) data
minJxmat = zeros(numalgs,nummodelsplot);
maxJxmat = zeros(numalgs,nummodelsplot);
meanJxmat = zeros(numalgs,nummodelsplot);
stdJxmat = zeros(numalgs,nummodelsplot);

% J(x) - V(x) data
minJmVxmat = zeros(numalgs,nummodelsplot);
maxJmVxmat = zeros(numalgs,nummodelsplot);
meanJmVxmat = zeros(numalgs,nummodelsplot);
stdJmVxmat = zeros(numalgs,nummodelsplot);

% V(x) data -- algorithm difference
minVxdiffmat = zeros(numalgs,nummodelsplot);
maxVxdiffmat = zeros(numalgs,nummodelsplot);
meanVxdiffmat = zeros(numalgs,nummodelsplot);
stdVxdiffmat = zeros(numalgs,nummodelsplot);

% J(x) data -- algorithm difference
minJxdiffmat = zeros(numalgs,nummodelsplot);
maxJxdiffmat = zeros(numalgs,nummodelsplot);
meanJxdiffmat = zeros(numalgs,nummodelsplot);
stdJxdiffmat = zeros(numalgs,nummodelsplot);

% J(x) - V(x) data -- algorithm difference
minJmVxdiffmat = zeros(numalgs,nummodelsplot);
maxJmVxdiffmat = zeros(numalgs,nummodelsplot);
meanJmVxdiffmat = zeros(numalgs,nummodelsplot);
stdJmVxdiffmat = zeros(numalgs,nummodelsplot);


% ***********************
%
% LOOP OVER MODELS TESTED
%  

for mcnt = 1:nummodelsplot

    % Current model index
    imcnt = indsmodelplot(mcnt);

    % Current \nu value
    nuimcnt = nuvec(imcnt);

    disp('***************************************************************')
    disp('*')
    disp(['* DISPLAYING SWEEP PERFORMANCE METRICS FOR \nu = ' ...
        num2str(nuimcnt)])
    disp('*')
    disp('***************************************************************')
    
    % ***********************
    %
    % LOOP OVER DATA FOR EACH INDIVIDUAL ALGORITHM
    %    

    for algcnt = 1:numalgs

        % Current algorithm
        algi = alg_list{algcnt};

        disp('***********************')
        disp('*')
        disp(['* ' algi ':'])
        disp('*') 

        % Display runtime data
        runtimetmp = avg_runtime_model(algcnt,imcnt);
        disp(['* AVG RUNTIME FOR \nu = ' ...
        num2str(nuimcnt) ':          ' num2str(runtimetmp) ' s'])  

        Vxtmp = V_x_cell{algcnt}(:,:,mcnt);
        if doJx
            Jxtmp = J_x_cell{algcnt}(:,:,mcnt);
            JmVxtmp = JmV_x_cell{algcnt}(:,:,mcnt); 
        else
            Jxtmp = nan;
            JmVxtmp = nan;
        end        


        % Display data
        [mind, maxd, meand, stdd] = disp_JV_metr(Vxtmp, Jxtmp, JmVxtmp);
     
        % V(x) data
        minVxmat(algcnt,mcnt) = mind.V;
        maxVxmat(algcnt,mcnt) = maxd.V;
        meanVxmat(algcnt,mcnt) = meand.V;
        stdVxmat(algcnt,mcnt) = stdd.V;
        
        % J(x) data
        minJxmat(algcnt,mcnt) = mind.J;
        maxJxmat(algcnt,mcnt) = maxd.J;
        meanJxmat(algcnt,mcnt) = meand.J;
        stdJxmat(algcnt,mcnt) = stdd.J;
        
        % J(x) - V(x) data
        minJmVxmat(algcnt,mcnt) = mind.JmV;
        maxJmVxmat(algcnt,mcnt) = maxd.JmV;
        meanJmVxmat(algcnt,mcnt) = meand.JmV;
        stdJmVxmat(algcnt,mcnt) = stdd.JmV;

    end


    % ***********************
    %
    % LOOP OVER ALGORITHM DIFFERENCE DATA
    %    

    if hasmidirl

    for algcnt = indsnotmidirl

        % Current algorithm
        algi = alg_list{algcnt};

        disp('***********************')
        disp('*')
        disp(['* (' algi ' - ' algnames.rci '):'])
        disp('*') 

        Vxtmp = V_x_diff_cell{algcnt}(:,:,mcnt);
        if doJx
            Jxtmp = J_x_diff_cell{algcnt}(:,:,mcnt);
            JmVxtmp = JmV_x_diff_cell{algcnt}(:,:,mcnt); 
        else
            Jxtmp = nan;
            JmVxtmp = nan;
        end        


        % Display data
        [mind, maxd, meand, stdd] = disp_JV_metr(Vxtmp, Jxtmp, JmVxtmp);
     
        % V(x) data -- algorithm difference
        minVxmat(algcnt,mcnt) = mind.V;
        maxVxmat(algcnt,mcnt) = maxd.V;
        meanVxmat(algcnt,mcnt) = meand.V;
        stdVxmat(algcnt,mcnt) = stdd.V;
        
        % J(x) data -- algorithm difference
        minJxmat(algcnt,mcnt) = mind.J;
        maxJxmat(algcnt,mcnt) = maxd.J;
        meanJxmat(algcnt,mcnt) = meand.J;
        stdJxmat(algcnt,mcnt) = stdd.J;
        
        % J(x) - V(x) data -- algorithm difference
        minJmVxmat(algcnt,mcnt) = mind.JmV;
        maxJmVxmat(algcnt,mcnt) = maxd.JmV;
        meanJmVxmat(algcnt,mcnt) = meand.JmV;
        stdJmVxmat(algcnt,mcnt) = stdd.JmV;

    end

    end

end

% ***********************
%
% ER DATA
%    

if doER

disp('***************************************************************')
disp('*')
disp(['* DISPLAYING EXPECTED RETURN DATA'])
disp('*')
disp('***************************************************************')

for algcnt = 1:numalgs

    % Current algorithm
    algi = alg_list{algcnt};

    meanER = meanreturnvec(algcnt);
    stdER = stdreturnvec(algcnt);

    disp('*****')
    disp('*')
    disp(['* ' algi ':  MEAN RETURN = avg +/- 2 \sigma =     '...
        num2str(meanER) '   +/-  '...
        num2str(1.96*stdER)])
    disp('*') 
    disp('*****')

end

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PACK EVALUATION DATA
%
% ************************************************************************* 
% *************************************************************************
% *************************************************************************

% System 
eval_data.sys = sys;

% lq_data
eval_data.lq_data_cell = lq_data_cell;

% Algorithm list
eval_data.alg_list = alg_list;

% x grid data for evaluation
eval_data.xplot = xplot;

% cFVI data -- n-D
eval_data.cfvi_data_cell = cfvi_data_cell;

% cFVI data -- 2D
eval_data.cfvi_data2_cell = cfvi_data2_cell;

% Which values of \nu to plot for
eval_data.indsmodelplot = indsmodelplot;
% eval_data.nummodelsplot = nummodelsplot;  

% Critic NN V(x), policy \mu(x)
eval_data.V_x_cell = V_x_cell;
eval_data.u_x_cell = u_x_cell;

% cFVI/rFVI gridpoint vectors
eval_data.cfvi_xgridvec_cell = cfvi_xgridvec_cell;

% Cost data J(x)
if doJx
    eval_data.J_x_cell = J_x_cell;
end


% V(x) data
eval_data.minVxmat = minVxmat;
eval_data.maxVxmat = maxVxmat;
eval_data.meanVxmat = meanVxmat;
eval_data.stdVxmat = stdVxmat;

% J(x) data
if doJx
    eval_data.minJxmat = minJxmat;
    eval_data.maxJxmat = maxJxmat;
    eval_data.meanJxmat = meanJxmat;
    eval_data.stdJxmat = stdJxmat;
end

% J(x) - V(x) data
if doJx
    eval_data.minJmVxmat = minJmVxmat;
    eval_data.maxJmVxmat = maxJmVxmat;
    eval_data.meanJmVxmat = meanJmVxmat;
    eval_data.stdJmVxmat = stdJmVxmat;
end

% ER data
if doER
    eval_data.ERmat = ERmat;
    eval_data.ERruntimevec = ERruntimevec;
    eval_data.meanreturnvec = meanreturnvec;
    eval_data.stdreturnvec = stdreturnvec;
end


% V(x) data -- algorithm difference
eval_data.minVxdiffmat = minVxdiffmat;
eval_data.maxVxdiffmat = maxVxdiffmat;
eval_data.meanVxdiffmat = meanVxdiffmat;
eval_data.stdVxdiffmat = stdVxdiffmat;

% J(x) data -- algorithm difference
if doJx
    eval_data.minJxdiffmat = minJxdiffmat;
    eval_data.maxJxdiffmat = maxJxdiffmat;
    eval_data.meanJxdiffmat = meanJxdiffmat;
    eval_data.stdJxdiffmat = stdJxdiffmat;
end

% J(x) - V(x) data -- algorithm difference
if doJx
    eval_data.minJmVxdiffmat = minJmVxdiffmat;
    eval_data.maxJmVxdiffmat = maxJmVxdiffmat;
    eval_data.meanJmVxdiffmat = meanJmVxdiffmat;
    eval_data.stdJmVxdiffmat = stdJmVxdiffmat;
end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% DISPLAY EVALUATION DATA
%
% ************************************************************************* 
% *************************************************************************
% *************************************************************************

function [mind, maxd, meand, stdd] = disp_JV_metr(Vx, Jx, JmVx)


% ***********************
%
% FORMATTING
%  

% One tab
onetab = '  ';
twotab = [onetab onetab];
fourtab = [twotab twotab];

% "&" with tab on either side
tabatab = [onetab '&' onetab];

% String to print out 
strng_collabels = ...
    [fourtab twotab 'V(x)' fourtab 'J(x)' fourtab 'J(x)-V(x)'];


% ***********************
%
% CALCULATE
%  

% V(x) data
tmpd = Vx(:);
minVx = min(tmpd);
maxVx = max(tmpd);
meanVx = mean(tmpd);
stdVx = std(tmpd);

% J(x) data
tmpd = Jx(:);
minJx = min(tmpd);
maxJx = max(tmpd);
meanJx = mean(tmpd);
stdJx = std(tmpd);

% J(x) - V(x) data
tmpd = JmVx(:);
minJmVx = min(tmpd);
maxJmVx = max(tmpd);
meanJmVx = mean(tmpd);
stdJmVx = std(tmpd);

% ***********************
%
% DISPLAY
%  

% Column labels
disp(strng_collabels)

% Min data
disp(['MIN: ' twotab num2str(minVx) tabatab num2str(minJx) ...
    tabatab num2str(minJmVx)])

% Max data
disp(['MAX: ' twotab num2str(maxVx) tabatab num2str(maxJx) ...
    tabatab num2str(maxJmVx)])

% Mean data
disp(['MEAN:' twotab num2str(meanVx) tabatab num2str(meanJx) ...
    tabatab num2str(meanJmVx)])

% Std data
disp(['STD: ' twotab num2str(stdVx) tabatab num2str(stdJx) ...
    tabatab num2str(stdJmVx)])



% ***********************
%
% STORE
%  

% Min data
mind.V = minVx;
mind.J = minJx;
mind.JmV = minJmVx;

% Max data
maxd.V = maxVx;
maxd.J = maxJx;
maxd.JmV = maxJmVx;

% Mean data
meand.V = meanVx;
meand.J = meanJx;
meand.JmV = meanJmVx;

% Std data
stdd.V = stdVx;
stdd.J = stdJx;
stdd.JmV = stdJmVx;


%%
% *************************************************************************
% *************************************************************************
%
% PLOT COLOR COUNTOUR PLOT -- MULTI-PLOT
%
% ************************************************************************* 
% *************************************************************************

function figcount = plot_contourf_multi(setts)

% Extract settings
ttl_base = setts.ttl_base;
fnmp = setts.fnmp;
data_cell = setts.data_cell;
alg_list = setts.alg_list;
isdiffplot = setts.isdiffplot;
figcount = setts.figcount;
nuvec = setts.nuvec;
indsmodelplot = setts.indsmodelplot;

% Number of models plotted
nummodelsplot = size(indsmodelplot,1);

% Depending on if this is a difference plot or not, get the proper
% algorithm indices
if isdiffplot
    alginds = setts.indsnotmidirl;
    indmidirl = setts.indmidirl;
else
    alginds = 1:size(alg_list,1);
end

% Plot data for each algorithm
for algcnt = alginds

    % Current algorithm
    algi = alg_list{algcnt};

    % Current algorithm data
    datai = data_cell{algcnt};

    % Current title
    if isdiffplot
        ttl = ['$' ttl_base '_{' algi '} - '...
                    ttl_base '_{' alg_list{indmidirl} '}$'];        
    else
        ttl = [ttl_base ' $\mid$ ' algi];
    end

    % Current file name
    filename = [fnmp '_' algi];

    % Set current settings
    setts.ttl = ttl;

    % Loop over each model
    for mcnt = 1:nummodelsplot
    
        % Current model index, \nu value
        imcnt = indsmodelplot(mcnt);
        numcnt = nuvec(imcnt);
        
        % Set \nu value
        setts.nu = numcnt;
    
        % Get data
        dataitmp = datai(:,:,mcnt);
    
        % Set current settings            
        setts.figcount = figcount;
        setts.pdata = dataitmp;
        setts.filename = [filename '_nu_' num2filename(numcnt)];
    
        % Plot
        figcount = plot_contourf(setts);

    end

end




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT COLOR COUNTOUR PLOT 
%
% ************************************************************************* 
% *************************************************************************
% *************************************************************************

function figcount = plot_contourf(setts)

% Unpack settings
figcount = setts.figcount;
Xp = setts.Xp;
Yp = setts.Yp;
pdata = setts.pdata;
cvals = setts.cvals;
psett_master = setts.psett_master;
doClim = setts.doClim;
ttl = setts.ttl;
x1min = setts.x1min;
x1max = setts.x1max;
x2min = setts.x2min;
x2max = setts.x2max;
x0surf_labels = setts.x0surf_labels;
savefigs = setts.savefigs;
if savefigs
    relpath = setts.relpath;
end
filename = setts.filename;
addnu = setts.addnu;

% PLOT
figure(figcount)
contourf(Xp, Yp, pdata', cvals, ...
    'EdgeAlpha', psett_master.contour.edgealpha);
if doClim
    clim(setts.Clim);
end
colormap jet
colorbar;

title(ttl);
xlim([x1min-0.001*abs(x1min) x1max+0.001*abs(x1max)]);
ylim([x2min-0.001*abs(x2min) x2max+0.001*abs(x2max)]);
xlabel(x0surf_labels{1});
ylabel(x0surf_labels{2});

% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    savepdf(figcount, relpath, filename); 
end

% Add \nu = [...]  to plot title
if addnu    
    ttl = [ttl ' $(\nu = ' num2str(setts.nu) ')$'];
    title(ttl);

    % SAVE PLOT
    if savefigs
        filename = [filename '_nu_ttl'];
        savepdf(figcount, relpath, filename); 
    end
end

% Increment figure counter
figcount = figcount + 1; 



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT COLOR LEARNING CURVES 
%
% ************************************************************************* 
% *************************************************************************
% *************************************************************************

function [figcount, meanruntimevec, meanreturnvec, ...
    stdreturnvec] = plot_lc(setts)

% Unpack settings
figcount = setts.figcount;
alg_list = setts.alg_list;
psett_master = setts.psett_master;
savefigs = setts.savefigs;
if savefigs
    relpath = setts.relpath;
end
filename = setts.filename;
data_cell = setts.data_cell;
ttl = setts.ttl;
color_sett_cell = setts.color_sett_cell;
indmidirl = setts.indmidirl;
systag = setts.systag;
sysnames = setts.sysnames;

% Number of algorithms
numalgs = size(data_cell,1);

% Init output
meanruntimevec = zeros(numalgs,1);

% Line width
linewidth = psett_master.linewidth;

% *************************************************************************
%
% PLOT
%
% ************************************************************************* 

% Storage
meanreturnvec = zeros(numalgs,1);
stdreturnvec = zeros(numalgs,1);

% PLOT
figure(figcount);
hold on;

lgd = [];

for algcnt = 1:numalgs

    % Current algorithm name
    curralg = alg_list{algcnt};
    
    % Pull data
    cfvidata = data_cell{algcnt};
    meanreturn = cfvidata.meanreturn;
    runtime = cfvidata.runtime;
    n_iter = size(meanreturn, 2);
    
    % Find number of seeds not empty
    numseeds = min(find(meanreturn(:,1) == 0));
    if ~isempty(numseeds)
        numseeds = numseeds - 1;
    else
        numseeds = size(meanreturn, 1);
    end
    
    % Get the mean, min, max of the mean return data
    meanmeanreturn = zeros(1,n_iter);
    minmeanreturn = zeros(1,n_iter);
    maxmeanreturn = zeros(1,n_iter);
    stdmeanreturn = zeros(1,n_iter);
    
    % Get mean runtime data
    runtime = runtime(:);
    runtime = runtime(runtime ~= 0);
    meanruntimevec(algcnt) = mean(runtime);
    
    
    for i = 1:n_iter
    
        % Current data
        datai = meanreturn(:,i);
    
        % Get current nonzero data
        datai = datai(datai ~= 0);
    
        % Store stats
        meanmeanreturn(i) = mean(datai);
        minmeanreturn(i) = min(datai);
        maxmeanreturn(i) = max(datai);
        stdmeanreturn(i) = std(datai);
       
    end
    
    % Current color
    colori = color_sett_cell{algcnt}{2};
    
    % Add to legend
    lgd = [ lgd
            {''}
            {curralg} ];
    
    % Format shaded area
    % See: https://www.mathworks.com/matlabcentral/
    %               answers/494515-plot-standard-deviation-as-a-shaded-area
    x = (1:n_iter) - 1;
    x2 = [x, fliplr(x)];
    inbetween = [maxmeanreturn fliplr(minmeanreturn)];
    
    % Plot
    fill(x2, inbetween, colori, ...
        'EdgeAlpha', 0, 'FaceAlpha', 0.5);
    plot(x, meanmeanreturn, 'color', colori, 'LineWidth', linewidth)

    % Store data
    meanreturnvec(algcnt) = meanmeanreturn(end);
    stdreturnvec(algcnt) = stdmeanreturn(end);

end


title(ttl);
xlabel('Iteration $i$');
ylabel('Average Return');

% % Set y limits
switch systag
    case sysnames.pendulum
        ylim([-50 0]);
    case sysnames.businjet
        ylim([-500 0]);
    case sysnames.ddmr
        ylim([-50 0]);
    otherwise
        % Nothing
end

legend(lgd);

% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);   
clear p_sett;

% SAVE PLOT
if savefigs
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 




