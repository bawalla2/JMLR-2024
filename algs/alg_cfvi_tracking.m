function out_data = alg_cfvi_tracking(alg_settings,...
    group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% FVI NONLINEAR TRACKING
%
% Brent A. Wallace
%
% 2023-03-15
%
% This program, given a trained FVI controller, implements the controller
% in a tracking control scheme.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings              (Struct) contains algortithm settings
%                           configured in config_preset.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       tvec                ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%       xmat                ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       umat                ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       inds                (Struct) Contains the state vector partition
%                           for the numerical simulation conducted by
%                           ode45. E.g., to find where the plant states are
%                           in the ode45 state vector, query inds.indsx.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% 
% GLOBAL VARIABLES
% 
% *************************************************************************

% Clear global variables
clear global

global sys;


% Reference signal r(t) settings
global r_sett;

% Do nominal min-phase model (=1) or nonmin-phase model (=0)
global u_sett;

        

% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
sys = master_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension

% System cell array
model_cell = sys.model_cell;

% Indices of nominal, perturbed models
indnom = sys.indnom;
% indmodel = alg_settings.indmodel;

% Nominal, perturbed models
model = model_cell{indnom};                  % System model
% model_nu = model_cell{indmodel};            % System model -- perturbed

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;

% Simulation length
tsim = alg_settings.tsim;

% ***********************
%       
% TRIM CONDITIONS
%    

% Nominal model
model_nom_tag = alg_settings.model_nom_tag;
model_nom_ind = alg_settings.model_nom_ind;
model_nom = model_cell{model_nom_ind};
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Simulation model
model_sim_tag = alg_settings.model_sim_tag;
model_sim_ind = alg_settings.model_sim_ind;
model_sim = model_cell{model_sim_ind};
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;


% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;


% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = master_settings.pfavec;   
end


% ***********************
%       
% CONTROLLER
%

% Policy lookup table, grid vectors
u_sett.xgridvec_cell = alg_settings.xgridvec_cell;
u_sett.u_tbl = alg_settings.u_tbl;

% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initial condition
x0 = alg_settings.x0;  

x0_sim = x0;

% Append integrator ICs
if hasintaug
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% ***********************
%       
% STATE VECTOR PARTITION INDICES
%   

% Indices of how state vector is partitioned
indsxr = model.inds_xr;

% Vehicle states
cnt = 1;
len = n;
indsx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Integral augmentation states
if hasintaug
    len = m;
    indsz = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Pre-filter states
if pf1nopf0
    len = m;
    indspf = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
if hasintaug
    inds.indsz = indsz;
end
if pf1nopf0
    inds.indspf = indspf;
end
u_sett.inds = inds;

% ***********************
%       
% CONTROL SETTINGS
%   

% Store control tag
u_sett.tag = 'cfvi_lookup';

% System dimensions
u_sett.n = n;
u_sett.m = m;

% Has integral augmentation (=1) or not (=0)
u_sett.hasintaug = hasintaug;

% Reference command settings
u_sett.r_sett = r_sett;

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

% Store trim state, control for control calculations
u_sett.xe_ctrl = xe_nom;
u_sett.ue_ctrl = ue_nom;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Indices of state variables to be tracked
inds_xr = model.inds_xr;
u_sett.inds_xr = inds_xr;

% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
u_sett.sy = model.lin.io.syd;


% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end


% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tvec = [];
xmat = [];
umat = [];


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
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% RUN SIMULATION
%

% Time span for simulation
tspan = [0, tsim];

% Run simulation
[t, x] = ode45(@odefunct, tspan, x0_sim);

% % DEBUGGING: Plot states while calling ode45
% options = odeset('OutputFcn',@odeplot);
% figure(100);
% [t, x] = ode45(@odefunct, tspan, x0_sim, options); 


% ***********************
%       
% STORE DATA
%

% Store time data
tvec = [    tvec
            t       ];


% Store system state data
xmat = [    xmat
            x       ];


% DEBUGGING: Print done simulating
disp('***** SIMULATION COMPLETE *****')



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PREPARE OUTPUT DATA
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), m);

% Calculate control
for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    u = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

end

% Store control signal
out_data.umat = umat;

% *************************************************************************
%
% TIME, STATE
% 
% *************************************************************************

% Time vector
out_data.tvec = tvec;

% Rescale integrator states to reflect original units
if hasintaug
    xmat(:,indsz) = (inv(u_sett.sy) * xmat(:,indsz)')';
end

% If a prefilter was used, rescale and shifted filtered reference command
% responses to reflect trim (pre-transformation)
if pf1nopf0
   xmat(:,indspf) = (inv(u_sett.sy) * xmat(:,indspf)')' ...
       +  [xe_sim(inds_xr(1)); xe_sim(inds_xr(2))]';
   out_data.xmat = xmat;
end

% Store modified state vector
out_data.xmat = xmat;

% Indices of how state vector is partitioned
out_data.inds = inds;

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMICS 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get indices of state partition
inds = u_sett.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;

% Initialize empty state derivative vector
xdot = zeros(u_sett.n_sim, 1);

% Get coordinate transformations
% sx = u_sett.sx;
% su = u_sett.su;
sy = u_sett.sy;

% Extract plant states
xp = x(inds.indsx);


% Get equilibrium point x_e (pre-transformation) -- nominal system
xe = u_sett.xe_nom;

% Get equilibrium control u_e (pre-transformation) -- nominal system
ue = u_sett.ue_nom;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_sim.lin.Ap * tx;
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett.lin1nonlin0
    % System linear
    g_x = model_sim.lin.Bp;
else
    % System nonlinear
    g_x = model_sim.gx(xp);
end

% Calculate control signal
u = uxt_alg(x, t);

% If model is linear, apply \tilde{u} = u - u_{e} (pre-transformation)
if u_sett.lin1nonlin0
    tu = u - ue;
end

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (post-transformation)
yr = rt(:,1);
yrp = sy * yr;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(inds.indspf);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if hasintaug
    if u_sett.pf1nopf0
        zdot = -(trfp - typ);
    else
        zdot = -(trp - typ);
    end
    % Append integral augmentation state derivatives
    xdot(inds.indsz) = zdot;
end

% State derivatives
if u_sett.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;


% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot(inds.indspf) = pfdot;
end       



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function u = uxt_alg(x, t)

% % Global variables
% global sys;
% n = sys.n;
% m = sys.m;

% Control settings
% global r_sett;
global u_sett;


% Evaluate control
u = eval_u(x, t, u_sett);



