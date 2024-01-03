function ddmr_init(settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE DDMR MODEL
%
% [*** ANONYMIZED ***]
%
% 2022-09-28
%
% This program initializes model data for a DDMR model. Data is from:
%
%   K. Mondal, "Dynamics, directional maneuverability and optimization
%   based multivariable control of nonholonomic differential drive mobile
%   robots," Ph.D. thesis, Arizona State University, Department of
%   Electrical Engineering, Tempe, AZ, USA, Dec. 2021.
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
% PARAMETERS
% 

% Mass of robot base (excludes wheels/motors) (kg)
mc = 3963e-3;

% Mass of single wheel/motor combination (kg)
mw = 659e-3;

% Wheel+motor moment of inertia about axle (kg-m^2)
Iw = 570e-6;

% Total inertia (kg-m^2)
I = 0.224;

% Radius of wheels (m)
r = 3.85e-2;

% Length of robot chassis/base (m)
l = 44e-2;

% Width of robot chassis/base (m)
w = 34e-2;

% Distance between two wheels (at midpoint) (m)
d_w = 34e-2;

% Distance c.g. lies forward of wheel axles (m)
d = -6e-2;

% Armature inductance (H)
la = 13.2e-6;

% Armature resistance (Ohm)
ra = 3.01;

% Back EMF constant (V / (rad/s))
kb = 0.075;

% Torque constant (N-m/A)
kt = 0.075;

% Motor-wheel gear (down) ratio 
kg = 1;

% Speed damping constant (N-m-s)
beta = 7.4e-6;


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

% x = [V, \omega] in R^{2}
xs = sym('x', [1 2], 'real')';


% u in R^{m}
us = sym('u', [1 m], 'real')'; 


% Index variables (i.e., where each state is in x)
indV = 1;
indw = 2;
% indiar = 3;
% indial = 4;

% Output variables to track: y = [V, \omega]^T     
inds_xr = [indV; indw];


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


model.mc = mc;
model.mw = mw;
model.Iw =  Iw;
model.I = I;
model.r = r;
model.l = l; 
model.w = w; 
model.d_w = d_w; 
model.d = d; 
model.la = la; 
model.ra = ra; 
model.kb = kb;
model.kt = kt; 
model.kg = kg; 
model.beta = beta;


% Index variables (i.e., where each state is in x)
model.indV = indV;
model.indw = indw;
% model.indiar = indiar;
% model.indial = indial;

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
nui.d = nuvec(i);

% Store pertubation values
model_nui.nu = nui;

% Modify perturbed model params from nominal
model_nui.d = nui.d * d;


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
nui.d = nuvec(indnom);

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
disp(['* DDMR MODEL INITIALIZATION COMPLETE'])
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


mc = model.mc;
mw = model.mw;
Iw =  model.Iw;
I = model.I;
r = model.r;
l = model.l; 
w = model.w; 
d_w = model.d_w; 
d = model.d; 
la = model.la; 
ra = model.ra; 
kb = model.kb;
kt = model.kt; 
kg = model.kg; 
beta = model.beta;


% Index variables (i.e., where each state is in x)
indV = model.indV;
indw = model.indw;

% ***********************
%       
% EFFECTIVE MASS, INERTIA
%    

% Total vehicle mass
mt = mc + 2*mw;

% Effective mass
mh = mt + 2 * Iw / r^2;

% Effective moment of inertia
Ih = I + d_w^2 * Iw / (2 * r^2);

% Effective damping constant \overline{\beta}
obeta = beta + kt * kb / ra;

% ***********************
%       
% TRANSFORMATIONS
%    

% M_F: [\tau_r, \tau_l] -> [F, \tau]
M_F = [     1/r         1/r
            d_w/(2*r)   -d_w/(2*r)  ];

% M_V: [\omega_r, \omega_l] -> [V, \omega]
M_V = [     r/2   r/2
            r/d_w -r/d_w    ];
invM_V = M_F';
invM_F = M_V';

% M_{e_a}: [\overline{e_{a}}, \Delta e_{a}] -> [e_{a,r}, e_{a,l}]
M_e = [     1   0.5
            1   -0.5    ];
invM_e = [  0.5 0.5
            1   -1      ];



% ***********************
%       
% VARIABLE SHORTHANDS
%    

Vs = xs(indV);
ws = xs(indw);

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

a11f = - 2*obeta/(mh*r^2);
a12f = 0;
a21f = 0;
a22f = -obeta*d_w^2/(2*Ih*r^2);

Af = [  a11f    a12f
        a21f    a22f    ];





% ***********************
%       
% FIRST DERIVATIVES (WITHOUT CONTROL)
%    

% \dot{V} -- f(x)
% Cf. Wang, Eqn. (20)
dV0(xs) = mc * d / mh * ws^2 + Af(1,:) * xs;

% \dot{\omega} -- f(x)
dw0(xs) = - mc * d / Ih * Vs * ws + Af(2,:) * xs;



% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
% 
% *************************************************************************


% ***********************
%       
% f(x)
%  

fxs(xs) =   [   dV0
                dw0   ];

% ***********************
%       
% g(x)
%  

% g_1(x)
g1x =  [    2*kt/(mh*kg*ra*r)
            0                   ];
g1s = symfun(g1x,xs);   % Do for constant sym functions only
g1s(xs) = formula(g1s);

% g_2(x)
g2x =  [    0
            d_w*kt/(2*Ih*kg*ra*r)   ];
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
Ve = 2;             % Equilibrium speed (m/s)
we = 0;             % Equilibrium rotational velocity (rad/s)

% Calculate equilibrium controls u_e = [\bar{e}_{a,e}, \Delta e_{a,e}]
oeae = -mh*kg*ra*r/(2*kt) * (-2*obeta/(mh*r^2)*Ve + mc*d/mh*we^2);
Deae = 2*Ih*kg*ra*r/(d_w*kt) * (mc*d/Ih*Ve*we + obeta*d_w^2/(2*Ih*r^2)*we);


disp('***********************')
disp('*')
disp('* TRIM CALCULATION')
disp('*')
disp('***********************')
disp(' ')

% Trim settings -- throttle d_T, elevator d_E, AOA \alpha
disp(['Trim Forward Voltage \overline{e}_a =      ' num2str(oeae)])
disp(['Trim Diff. Voltage   \Delta e_a =      ' num2str(Deae)])

% Trim state x_e
% disp(['Trim State x_e = [V_e, \omega_e]^T = '])
disp(['Trim State x_e = [V_e, \omega_e]^T = '])
xe = [  Ve
        we  ];

% Trim control u_e 
disp(['Trim Control u_e = [\bar{e}_{a,e}, \Delta e_{a,e}]^T = '])
ue = [  oeae
        Deae    ];

				
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
trimconds.we = we;                      
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
% PARTIALS PERTAINING TO VEHICLE DYNAMICAL EQUATIONS: (V, \omega)
% 
% *************************************************************************

% ***********************
%       
% df_1 / dx
%    


df11 = Af(1,1);
df12(xs) = 2*mc*d/mh * ws + Af(1,2);


% ***********************
%       
% df_2 / dx
%   

df21(xs) = - mc*d/Ih * ws + Af(2,1);
df22(xs) = - mc*d/Ih * Vs + Af(2,2);

% ***********************
%       
% df / dx
%   

% (V, \omega)
dfdxs = [   df11    df12
            df21    df22    ];
dfdxs = formula(dfdxs);




% ***********************
%       
% "A", "B", "C", "D" MATRIX
%
% NOTE: Is sum of terms from the dynamical equations + load/armature
% equations
%   

% % d f / d x
% dfdxs(xs) = dfdxd4 + Apla;
% dfdxs = formula(dfdxs);


% d g_j(x) / d x 
dg1dxs = formula(jacobian(g1s));
dg2dxs = formula(jacobian(g2s));

% % DEBUGGING: Evaluate df/dx at trim
% dfdxe = double(subs(dfdxs, xs, xe))






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
% EFFECTIVE MASS, INERTIA
%    

% Total vehicle mass
model.mt = mt;

% Effective mass
model.mh = mh;

% Effective moment of inertia
model.Ih = Ih;

% Effective damping constant \overline{\beta}
model.obeta = obeta;

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

% Outputs: [V \omega]
Cvw = eye(2); 
% Cvw = [eye(2) zeros(2)]; 
Cp = Cvw;

% "D" matrix
Dp = zeros(m);

% Evaluate numerically at trim
Ap = double(subs(dfdxs, xs, xe)) + dgudx;
Bp = gx(xe);

% Number of states 
n = size(Ap,1);

% Plant y = [V, \emega]^T
Pvw = ss(Ap,Bp,Cp,Dp);

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
%       x' =   [    V (m/s)
%                   \omega (deg)    ]
%
%            = [    y
%                   x_r (empty)             ]
%    
% New Output Variables
%       y' =   [    V (ft/s)
%                   \omega (deg)     ] 
%          = y
%
% *************************************************************************


% Coordinate transformations
sud = eye(2);
% sud = diag([1 R2D]);
% sxd = diag([1 R2D]);
sxd = eye(2);
% sxd = [ 1   0   0   0
%         0   R2D 0   0
%         0   0   1   0
%         0   0   0   1   ];
% syd = diag([1 R2D]);
syd = eye(2);

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
% [ z                   [   x_1
%   y           ->          x_2     ]
%   x_r ]
%
%
%       x_1 = [ z_V (m/s -s)
%               V   (m/s)     ]
%
%       x_2 = [ z_\omega    (deg-s)
%               \omega      (deg)       ]      
%
% *************************************************************************

% DIRL state transformation: [z, y, x_r] -> [x_1 x_2]
% x_{lq servo} -> x_{dirl}
Sxirl = [   1 0 0 0
            0 0 1 0
            0 1 0 0
            0 0 0 1  ];


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
%       NOTE: x_3 absent for DDMR       
%
% *************************************************************************

% Sxirl_x3 = blkdiag(Sxirl, eye(2));
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
Ad22 = Ad(2,2);
Bd22 = Bd(2,2);
Cd22 = Cd(2,2);
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
lin.Pvw = Pvw;          % Plant y = [V, \omega]^T
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




