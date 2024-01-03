function [Jxmat, out_data] = alg_eval_cost(alg_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE INFINITE HORIZON COST OF CONTROL ALGORITHM
%
% Brent A. Wallace
%
% 2022-11-14
%
% This program, given a system, controller, initial condition x_0,
% termination time T > 0, and cost structure Q, R, integrates the cost
%
%   J(x_0) = \int_{0}^{T} x^T Q x + u^T R u dt
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

global normeps;

% % Reference signal r(t) settings
% global r_sett;

% Control settings
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

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% Nominal, perturbed models
model = model_cell{indnom};                  % System model

% Integration time interval T
T = alg_settings.T;

% Has integral augmentation (=1) or not (=0)
hasintaug = master_settings.hasintaug;

% Threshold \epsilon > 0 such that simulation terminates when ||x|| <
% \epsilon
donormeps = alg_settings.donormeps;
normeps0 = alg_settings.normeps;

% Algorithm tag
algtag = alg_settings.algtag;

% Extract algorithm names
algnames = master_settings.algnames;


% Nominal model
model_nom_ind = alg_settings.model_nom_ind;
model_nom = model_cell{model_nom_ind};
xe_nom = model_nom.trimconds.xe;
ue_nom = model_nom.trimconds.ue;

% Simulation model
model_sim_ind = alg_settings.model_sim_ind;
model_sim = model_cell{model_sim_ind};
xe_sim = model_sim.trimconds.xe;
ue_sim = model_sim.trimconds.ue;


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

% Cost integral J(x)
len = 1;
indsJ = (cnt:cnt+len-1)';
cnt = cnt + len;

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
if hasintaug
    inds.indsz = indsz;
end
inds.indsJ = indsJ;
u_sett.inds = inds;



% % DEBUGGING: Print done loading
% disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% Time span for simulation
tspan = [0, T];

% Options -- terminate algorithm when norm of state goes below threshold
if donormeps
    odeopt = odeset('Events', @normevent);
end

% ***********************
%       
% CONTROL SETTINGS
%   

% Tag
u_sett.tag = alg_settings.utag;

% System dimensions
u_sett.n = n;
u_sett.m = m;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% Has integral augmentation (=1) or not (=0)
u_sett.hasintaug = hasintaug;

% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
% u_sett.sxh = model.lin.io.sxdh;
u_sett.sy = model.lin.io.syd;

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

% Q, R
u_sett.Q = alg_settings.Q;
u_sett.R = alg_settings.R;

% ***********************
%       
% ALGORITHM-SPECIFIC CONTROL SETTINGS
%  

switch algtag

    % RCI
    case algnames.rci

        % Controller
        u_sett.K = alg_settings.K;

        % Equilibrium state, control for control calculations
        u_sett.xe_ctrl = xe_sim;
        u_sett.ue_ctrl = ue_sim;

    % Nominal LQ
    case algnames.lq

        % Controller
        u_sett.K = alg_settings.K;

        % Equilibrium state, control for control calculations
        u_sett.xe_ctrl = xe_nom;
        u_sett.ue_ctrl = ue_nom;        

    % cFVI/rFVI
    case {algnames.cfvi; algnames.rfvi}

        % Policy lookup table, grid vectors
        u_sett.xgridvec_cell = alg_settings.xgridvec_cell;
        u_sett.u_tbl = alg_settings.u_tbl;

        % Equilibrium state, control for control calculations
        u_sett.xe_ctrl = xe_nom;
        u_sett.ue_ctrl = ue_nom;

    % THROW ERROR OTHERWISE
    otherwise

        error(['EVALUATION OF COST J(X) -- ALG TAG     ' algtag ...
            '     NOT RECOGNIZED']);
end


% ***********************
%       
% ICS
%

% Initial condition matrix
x0mat = alg_settings.x0mat;  

% Size of IC matrix
nx0r = size(x0mat, 1);
nx0c = size(x0mat, 2);



% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Stores integrated cost J(x_0) at each IC x_0
Jxmat = zeros(nx0r, nx0c);

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
% *************************************************************************
%
% RUN SIMULATION
% 
% *************************************************************************
% *************************************************************************

for i = 1:nx0r 

    % Initialize empty vector to store mean final state norm over this
    % value of x_1(0) swept
    normxtfvec = zeros(nx0c,1);

    % Start timer
    t0 = tic;

    for j = 1:nx0c

        % Initialize empty IC vector
        x0_sim = zeros(n_sim,1);

        % Get current IC
        x0 = squeeze(x0mat(i,j,:));

        % Determine threshold \epsilon > 0 such that simulation terminates
        % when ||x|| < \epsilon
        if norm(x0) > 2 * normeps0
            normeps = normeps0;
        else
            normeps = norm(x0) / 5;
        end

    
        % Set current IC
        x0_sim(indsx) = x0;
        
        % Run simulation
        if donormeps
            [~, x] = ode45(@odefunct, tspan, x0_sim, odeopt);
        else
            [~, x] = ode45(@odefunct, tspan, x0_sim);
%             % DEBUGGING: Plot states while calling ode45
%             options = odeset('OutputFcn',@odeplot);
%             figure(100);
%             [~, x] = ode45(@odefunct, tspan, x0_sim, options);             
        end
        

        % Final state x(t_f)
        xend = x(end,:)';

        % Store cost
        Jxmat(i,j) = xend(indsJ);

        % Store norm of final state
        if hasintaug
            xtf = [ xend(indsz)
                    xend(indsx) - xe_sim ];
        else
            xtf = xend(indsx) - xe_sim;
        end
        normxtfvec(j) = norm(xtf);

    end

    % Stop timer
    t1 = toc(t0);

    % DEBUGGING: Display current IC in x_1
    disp(['***** EVALUATING COST J(x) FOR ALG --      ' algtag ...
        '   --      IC x_1 COUNT:   ' num2str(i) '      OUT OF      ' ...
        num2str(nx0r)])   

    % DEBUGGING: Display mean cost
    disp(['     *** MEAN COST J(x)' ...
        '   =   ' num2str(mean(Jxmat(i,:)))])

    % DEBUGGING: Display mean final state norm
    disp(['     *** MEAN FINAL STATE NORM ||(z, x_p)(t_f)|| WRT TRIM' ...
        '   =   ' num2str(mean(normxtfvec))])

    % DEBUGGING: Display mean final state norm
    disp(['     *** BATCH ELAPSED TIME' ...
        '   =   ' num2str(t1) ' s'])    
end




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

% % Cost data
% out_data.Jxmat = Jxmat;

out_data = [];



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
% global r_sett;
global u_sett;

% % Extract nominal model
% model_nom = u_sett.model_nom;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get coordinate transformations
sx = u_sett.sx;
% sxh = u_sett.sxh;
su = u_sett.su;
sy = u_sett.sy;
% Sxirl_x3 = u_sett.Sxirl_x3;   % [z, y, x_r, x_3] -> [x_1, x_2, x_3]

% Get cost structure Q, R
Q = u_sett.Q;
R = u_sett.R;

% Get indices of state partition
inds = u_sett.inds;

% Has integral augmentation (=1) or not (=0)
hasintaug = u_sett.hasintaug;

% Initialize empty state derivative vector
xdot = zeros(u_sett.n_sim, 1);

% Extract plant states
xp = x(inds.indsx);


% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett.ue_sim;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Get state of linear system (post-transformation) 
% \tilde{x}^{\prime} = S_x * \tilde{x}
txp = sx * tx;

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

% Calculate \tilde{u} = u - u_{e} (pre-transformation) 
tu = u - ue;

% Calculate \tilde{u}^{\prime} = S_u * \tilde{u} (post-transformation) 
tup = su * tu;

% State derivatives
if u_sett.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
% NOTE: \tilde{r}^{\prime}(t) = \tilde{y}_r^{\prime}(t) -
% \tilde{y}_e^{\prime} = 0 here (ref cmd is just the equilibrium output)
trp = zeros(m, 1);

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% Evaluate integrator state derivative \dot{z} (post-transformation)
if hasintaug
    zdot = -(trp - typ);
    xdot(inds.indsz) = zdot;
end

% Extract the integral augmentation states z (post-transformation)
if hasintaug
    z = x(inds.indsz);
    
    % Form aggregate state x = [z^T x_p^T]^T = [z^T y^T x_r^T]^T 
    % (post-trans)
    zxp = [ z 
            txp ];
else
    zxp = xp;
end

% Calculate running cost r(x,i) = x^T Q x + u^T R u
rxu = zxp' * Q * zxp + tup' * R * tup;

% Set running cost as J(x) derivative
xdot(inds.indsJ) = rxu;



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


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVENT HANDLING -- TERMINATE ode45 WHEN STATE NORM GOES BELOW THRESHOLD
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [value, isterminal, direction] = normevent(t, x)

% Global variables
global u_sett;
global normeps;

% Get indices of state partition
inds = u_sett.inds;

% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Extract the integral augmentation states z (post-transformation)
z = x(inds.indsz);

% Concatenate (x_p, z)
txz = [tx; z];

% CONDITION CHECK
value = norm(txz) < normeps;      % Check if norm < \epsilon
isterminal = 1;                 % Stop the integration
direction  = 0;

