function businjet_init(settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE BUSINESS JET MODEL
%
% [*** ANONYMIZED ***]  
%
% 2022-09-28
%
% This program initializes model data for a linear business jet model. Data
% is from:
%
%   R. F. Stengel. Flight Dynamics. Princeton University Press, Princeton,
%   NJ, USA, 2 edition, 2022
%
% NOTE: Make sure the MATLAB path is in the folder holding main.m before
% running this program.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% % % ***********************
% % %
% % % CLEAR VARIABLES, COMMAND WINDOW, FIGURES
% % %
% % clear
% % clc
% % close all

% *************************************************************************
%
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;

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
% *************************************************************************
%
% SETTINGS
%
% *************************************************************************
% *************************************************************************

% Number of inputs
m = 2;

% Perturbation params
nuvec = settings.nuvec;

% Number of perturbed models initialized
nummodels = size(nuvec,1);

% Index of nominal model
indnom = settings.indnom;

% Relative path to save data to
relpath_data = settings.relpath_data;

% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE PARAMETERS
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% "FLIGHT DYNAMICS"
%
% Cf. Table 5.4-1, pp. 435
% 

% Trim arispeed (m/s)
Ve = 100;

% Drag aero derivative wrt speed V (N/(m/s))
DV = 0.0185;

% Ratio: (Lift aero derivative wrt speed V) / (V_e) (N/(m/s))/(m/s)
LVdVe = 0.0019;

% Ratio: (Lift aero derivative wrt AOA \alpha) / (V_e) (N/(rad))/(m/s)
LadVe = 1.279;

% Pitching derivative wrt AOA \alpha (N-m/rad)
Ma = - 7.9856;

% Pitching derivative wrt pitch rate q (N-m/rad)
Mq = - 1.2794;

% Gravitational field * cos(\theta_{e}) (m/s^2)
gcth = 9.0867;

% Thrust derivative with respect to throttle setting (N/-)
TdT = 4.6645;

% Pitching moment derivative with respect to elevator setting (N-m/rad)
MdE = -9.069;


% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE DYNAMIC FUNCTIONS
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% % x = [V, \omega, i_{a,r}, i_{a,l}] in R^{4}
% xs = sym('x', [1 4], 'real')';

% x = [V, \omega] in R^{4}
xs = sym('x', [1 4], 'real')';


% u in R^{m}
us = sym('u', [1 m], 'real')'; 


% Index variables (i.e., where each state is in x)
indV = 1;
indg = 2;
indq = 3;
inda = 4;

% Output variables to track: y = [V, \gamma]^T     
inds_xr = [indV; indg];


% ***********************
%       
% MISC
%   

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;


% ***********************
%       
% MODEL CELL ARRAY
%
% Holds model for each of the perturbations tested
%   

model_cell = cell(nummodels, 1);



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL -- DEFAULT PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************


% ***********************
%       
% SYMBOLIC VARIABLES
%      

model.xs = xs;
% model.xvs = xvs;
model.us = us; 
% model.xvus = xvus;
% model.indsz = indsz;
% model.xvs0 = xvs0;

% Degree/radian conversions
model.D2R = D2R;
model.R2D = R2D;



% ***********************
%
% VEHICLE PARAMETERS
%

model.m = m;


model.Ve = Ve;
model.DV = DV;
model.LVdVe = LVdVe;
model.LadVe = LadVe;
model.Ma = Ma;
model.Mq = Mq;
model.gcth = gcth;
model.TdT = TdT;
model.MdE = MdE;


% Index variables (i.e., where each state is in x)
model.indV = indV;
model.indg = indg;
model.indq = indq;
model.inda = inda;

% Output variables to track: y     
model.inds_xr = inds_xr;

% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model = init_model(model);
trimconds_nom = model.trimconds;

% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT NOMINAL MODEL TRIM
%

lin_nom = linearize_model(model, trimconds_nom);
model.lin = lin_nom;

% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT PERTURBED MODEL TRIM
%

% Initialize empty. Will be filled below
lin_atnu = cell(nummodels,1);


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL -- PERTURBED PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Indices not corresponding to the nominal model
inds_notnom = 1:nummodels;
inds_notnom = inds_notnom(inds_notnom ~= indnom);

for i = inds_notnom

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************

% Initialize as the default, modify perturbed functions
model_nui = model;

% Perturbations for this model
nui.La = nuvec(i);

% Store pertubation values
model_nui.nu = nui;

% Modify perturbed model params from nominal
model_nui.LadVe = nui.La * LadVe;


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model_nui = init_model(model_nui);
trimconds_nui = model_nui.trimconds;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT PERTURBED MODEL TRIM
%

lin_nui = linearize_model(model_nui, trimconds_nui);
model_nui.lin = lin_nui;


% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT PERTURBED MODEL TRIM
%

lin_nom_atnui = linearize_model(model, trimconds_nui);
lin_atnu{i} = lin_nom_atnui;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT NOMINAL MODEL TRIM
%

lin_nui_atnom = linearize_model(model_nui, trimconds_nom);
model_nui.lin_atnom = lin_nui_atnom;


% ***********************
%
% SET THIS PERTURBED MODEL IN THE ARRAY
%

model_cell{i} = model_nui;

end


% ***********************
%
% SET THE NOMINAL MODEL
%

% Perturbations for this model
nui.La = nuvec(indnom);

% Store pertubation values
model.nu = nui;

model.lin_atnu = lin_atnu;
model_cell{indnom} = model;


% % ***********************
% %
% % DEBUGGING: TEST DYNAMICAL FUNCTIONS
% %
% 
% xtst1 = [1; -10*D2R];
% xtst2 = [1; -30*D2R];
% xtst3 = [3; -10*D2R];
% xtst4 = [3; -30*D2R];
% 
% disp(['x1 = ' num2str(xtst1')])
% f1x1 = model.fx(xtst1)
% f2x1 = model_cell{end}.fx(xtst1)
% disp(['x2 = ' num2str(xtst2')])
% f1x2 = model.fx(xtst2)
% f2x2 = model_cell{end}.fx(xtst2)
% disp(['x3 = ' num2str(xtst3')])
% f1x3 = model.fx(xtst3)
% f2x3 = model_cell{end}.fx(xtst3)
% disp(['x4 = ' num2str(xtst4')])
% f1x4 = model.fx(xtst4)
% f2x4 = model_cell{end}.fx(xtst4)

% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************


% Make directory to save data to
mkdir(relpath_data)

% Initialize model struct
% model_struct.model = model;
model_struct.model_cell = model_cell;

% Save data 
varname = 'model_struct';
filename = settings.filename;
save([relpath_data filename], varname);



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Display complete
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* BUSINESS JET MODEL INITIALIZATION COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE MODEL
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function model = init_model(model)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
% xvs = model.xvs;
us = model.us; 
% xvus = model.xvus;
% xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% ***********************
%
% VEHICLE PARAMETERS
%

m = model.m;


Ve = model.Ve;
DV = model.DV;
LVdVe = model.LVdVe;
LadVe = model.LadVe;
Ma = model.Ma;
Mq = model.Mq;
gcth = model.gcth;
TdT = model.TdT;
MdE = model.MdE;


% Index variables (i.e., where each state is in x)
indV = model.indV;
indg = model.indg;
indq = model.indq;
inda = model.inda;



% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% "A" MATRIX TERMS ENTERING DIRECTLY IN THE NONLINEAR EQUATIONS
%
% Cf. K. Mondal PhD thesis, (3.80), pp. 62
%    



Af = [  -DV     -gcth   0   0
        LVdVe   0       0   LadVe
        0       0       Mq  Ma
        -LVdVe  0       1   -LadVe  ];





% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
% 
% *************************************************************************


% ***********************
%       
% f(x)
%  

% Trim state
xe = [  Ve
        zeros(3,1) ];

fxs(xs) =   Af * (xs - xe);

% ***********************
%       
% g(x)
%  

% g_1(x)
g1x =  [    TdT
            0 
            0
            0   ];
g1s = symfun(g1x,xs);   % Do for constant sym functions only
g1s(xs) = formula(g1s);

% g_2(x)
g2x =  [    0
            0
            MdE
            0       ];
g2s = symfun(g2x,xs);   % Do for constant sym functions only
g2s(xs) = formula(g2s);



% Combine
gxs(xs) = formula([g1s g2s]);




%%
% *************************************************************************
% *************************************************************************
%
% CREATE INLINE FUNCTIONS
% 
% *************************************************************************
% *************************************************************************


% f(x) -
fx = matlabFunction(fxs, 'vars', {xs});
sys.model.fx = fx;

% g(x) 
gx = matlabFunction(gxs, 'vars', {xs});
sys.model.gx = gx;



%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************

% Initialize trim parameters
Ve = 100;           % Equilibrium airspeed (m/s)

% Calculate equilibrium controls u_e = [\delta_{T,e}, \delta_{E,e}]
dTe = 0;
dEe = 0;



disp('***********************')
disp('*')
disp('* TRIM CALCULATION')
disp('*')
disp('***********************')
disp(' ')

disp(['Trim Diff. Throttle \delta_{T,e} =      ' num2str(dTe)])
disp(['Trim Diff. Elevator   \delta_{E,e} =      ' num2str(dEe)])

disp(['Trim State x_e = [V_e, \gamma_e, q_e, \alpha_e]^T = '])
xe 

% Trim control u_e 
disp(['Trim Control u_e = [\delta_{T,e}, \delta_{E,e}]^T = '])
ue = [  dTe
        dEe    ];


% Evaluate system dynamics
f_xe = fx(xe);
g_xe = gx(xe);
xdot = f_xe + g_xe * ue;

disp(['State Derivatives Evaluated at Trim: ' ...
    '\dot{x} = f(x_e) + g(x_e) * u_e ='])

xdot

disp(['||\dot{x} = f(x_e) + g(x_e) * u_e || ='])

norm(xdot)

% Store equilibrium calculations
trimconds.Ve = Ve;               
trimconds.ge = 0;                      
trimconds.xe = xe;
trimconds.ue = ue;



%%
% *************************************************************************
% *************************************************************************
%
% SYMBOLIC LINEARIZATION
% 
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% PARTIALS PERTAINING TO VEHICLE DYNAMICAL EQUATIONS
% 
% *************************************************************************

% ***********************
%       
% df / dx
%   

% (V, \omega)
dfdxs(xs) = symfun(Af,xs);


% ***********************
%       
% "A", "B", "C", "D" MATRIX
%
% NOTE: Is sum of terms from the dynamical equations + load/armature
% equations
%   


% d g_j(x) / d x 
dg1dxs = formula(jacobian(g1s));
dg2dxs = formula(jacobian(g2s));







%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************


% ***********************
%
% SYSTEM DYNAMICAL EQUATIONS
%


% f(x), g(x)
model.fxs = fxs;
model.gxs = gxs;


% ***********************
%
% INLINE FUNCTIONS
%

% f(x), g(x)
model.fx = fx;
model.gx = gx;

% ***********************
%
% VEHICLE TRIM
%

model.trimconds = trimconds;


% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

model.dfdxs = dfdxs;
model.g1s = g1s;
model.g2s = g2s;
model.dg1dxs = dg1dxs;
model.dg2dxs = dg2dxs;

%%
% *************************************************************************
% *************************************************************************
%
% LINEARIZE SYSTEM
% 
% *************************************************************************
% *************************************************************************

function lin = linearize_model(model, trimconds)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% VEHICLE PARAMETERS
%    

% Number of inputs
m = model.m;

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
% xvs = model.xvs;
% us = model.us; 
% xvus = model.xvus;
% indsz = model.indsz;
% xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% ***********************
%
% INLINE FUNCTIONS
%

% f(x), g(x)
% fx = model.fx;
gx = model.gx;

% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

dfdxs = model.dfdxs;
g1s = model.g1s;
g2s = model.g2s;
dg1dxs = model.dg1dxs;
dg2dxs = model.dg2dxs;

% ***********************
%
% VEHICLE TRIM
%

xe = trimconds.xe;
ue = trimconds.ue;


% d {g(x) u} / d x 
dgudx = double(subs(dg1dxs, xs, xe)) * ue(1) ...
    + double(subs(dg2dxs, xs, xe)) * ue(2);

% Outputs: [V \gamma] 
Cvg = [eye(2) zeros(2)]; 
Cp = Cvg;

% "D" matrix
Dp = zeros(m);

% Evaluate numerically at trim
Ap = double(subs(dfdxs, xs, xe)) + dgudx;
Bp = gx(xe);

% Number of states 
n = size(Ap,1);

% Plant y = [V, \gamma]^T
Pvg = ss(Ap,Bp,Cp,Dp);

% Plant y = x
Px = ss(Ap,Bp,eye(n),zeros(n,m));


%%
% *************************************************************************
% *************************************************************************
%
% FORM DESIGN PLANT FOR PI-PD INNER-OUTER DESIGN
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% FORM DESIGN PLANT
%
% Given an original plant of the form P = (A, B, C, D), we would like to
% form a design plant (appropriately scaled) such that the state vector
% assumes the form
%
% x' = [    y
%           x_r     ]
%
% Where y \in R^m are the outputs to be tracked, and x_r \in R^{n-m} forms
% the rest of the state.
%
% Introduce the coordinate transformations:
%
%  u' = S_u * u
%  x' = S_x * x
%  y' = S_y * y
%
%
% New Control Variables
%     u' = u        (unchanged)
%
%
% New State Variables
%       x' =  x     (unchanged)
%
%            = [    y
%                   x_r             ]
%    
% New Output Variables
%       y' = y      (unchanged)
%
% *************************************************************************


% Coordinate transformations
% sud = eye(m);
sud = diag([1 R2D]);
% sxd = eye(4);
sxd = diag([1 R2D R2D R2D]);
% syd = eye(m);
syd = diag([1 R2D]);

% Scaled linear dynamics
Ad = sxd*Ap*inv(sxd);
Bd = sxd*Bp*inv(sud);
Cd = syd*Cp*inv(sxd);
Dd = syd*Dp*inv(sud);

% Scaled design plant
Pd = ss(Ad,Bd,Cd,Dd);


% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS
%
% [                   [   x_1
%   y           ->          x_2     ]
%   x_r ]
%
%
%       x_1 = [ V   (m/s)     ]
%
%       x_2 = [ \gamma      (rad)
%               q           (rad/s)
%               \alpha      (rad)       ]      
%
% *************************************************************************

% DIRL state transformation: [y, x_r] -> [x_1 x_2]
% x_{lq servo} -> x_{dirl}
% Sxirl = eye(4);

% DIRL state transformation: [z, y, x_r] -> [x_1 x_2]
Sxirl = [   1 0 0 0 0 0
            0 0 1 0 0 0
            0 1 0 0 0 0
            0 0 0 1 0 0
            0 0 0 0 1 0
            0 0 0 0 0 1];

invSxirl = inv(Sxirl);

% *************************************************************************
%
% FORM TRANSFORMATION FROM LQ SERVO COORDS TO DIRL COORDS (WITH x_3)
%
% [ z                   [   x_1
%   y           ->          x_2     
%   x_r                     x_3     ];
%   x_3     ]
%
%       NOTE: x_3 absent for business jet       
%
% *************************************************************************

Sxirl_x3 = Sxirl;

invSxirl_x3 = inv(Sxirl_x3);

% *************************************************************************
%
% FORM PLANT FOR DIRL
%
% State: x = [  x_1
%               x_2     ]
%
%       x_1 = [ z_V (m/s -s)
%               V   (m/s)     ]
%
%       x_2 = [ z_\omega    (deg-s)
%               \omega      (deg)       ]   
%
% *************************************************************************


% % DIRL state-space matrices
% Airl = Sxirl_x3 * Ad * inv(Sxirl_x3);
% Birl = Sxirl_x3 * Bd;


% Augmented plant
% x = [z^T x_p^T]^T
Aaug = [    zeros(m)   Cd
            zeros(n,m) Ad  ];
Baug = [    zeros(m,m)
            Bd         ];



% DIRL state-space matrices
Airl = Sxirl_x3 * Aaug * inv(Sxirl_x3);
Birl = Sxirl_x3 * Baug;



% *************************************************************************
%
% FORM SIMPLIFIED DECOUPLED DESIGN PLANT FOR INNER-OUTER LOOP DESIGN
%
% *************************************************************************


% (1,1) Linearization
Ad11 = Ad(1,1);
Bd11 = Bd(1,1);
Cd11 = Cd(1,1);
Dd11 = Dd(1,1);


% (2,2) Linearization
Ad22 = Ad(2:4,2:4);
Bd22 = Bd(2:4,2);
Cd22 = Cd(2,2:4);
Dd22 = Dd(2,2);

% Plant (1,1)
Pd11 = ss(Ad11,Bd11,Cd11,Dd11);

% Plant (2,2)
Pd22 = ss(Ad22,Bd22,Cd22,Dd22);



%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************

% ***********************
%
% LINEARIZATION TERMS
%

% A, B, C, D matrix
lin.Ap = Ap;
lin.Bp = Bp;
lin.Cp = Cp;
lin.Dp = Dp;

% Plant ss objects -- nominal MP model
lin.Pvg = Pvg;          % Plant y = [V, \gamma]^T
lin.Px = Px;            % Plant y = x


% ***********************
%
% LINEARIZATION TERMS -- DESIGN PLANT FOR PD-PI INNER-OUTER
%

io.sud = sud;
io.sxd = sxd;
io.syd = syd;

io.Ad = Ad;
io.Bd = Bd;
io.Cd = Cd;
io.Dd = Dd;
io.Pd = Pd;
io.nlq = size(Ad,1);        % System order for LQ servo

% IRL linear dynamics, coord transformations
io.Airl = Airl;
io.Birl = Birl;
io.Sxirl = Sxirl;
io.invSxirl = invSxirl;
io.Sxirl_x3 = Sxirl_x3;
io.invSxirl_x3 = invSxirl_x3;


% Plant (1,1)
io.Ad11 = Ad11;
io.Bd11 = Bd11;
io.Cd11 = Cd11;
io.Dd11 = Dd11;
io.Pd11 = Pd11;

% Plant (2,2)
io.Ad22 = Ad22;
io.Bd22 = Bd22;
io.Cd22 = Cd22;
io.Dd22 = Dd22;
io.Pd22 = Pd22;


% Store all inner/outer terms
lin.io = io;


% Store linearization params
model.lin = lin;




