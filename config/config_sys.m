function [sys, sys_plot_settings] = config_sys(sys)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE SYSTEM PROPERTIES
%
% Brent A. Wallace
%
% 2022-02-08
%
% This program, given a system tag, initializes System dimensions such as
% system order, number of inputs, etc. 
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% sys = config_sys(sys)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% sys           (Struct) System for design. Has the following fields:
%   tag         (String) System tag
%
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% sys           (Struct) Contains System dimensions, fully initialized. Has
%               the following fields:
%   tag         (String) System tag (see above for options).
%   n           (Integer) System order.
%   m           (Integer) Number of inputs of system.
%
% sys_plot_settings    (Struct) Contains plot settings corresponding to
%                       the system designed for. Has the following fields:
%   x_sclvec    (n-dim. Vector) A vector containing the scaling desired for
%               each state variable on plots. Is a vector of ones by
%               default (i.e., no scaling). E.g., if x_2(t) is an angle in
%               radians, but it is desired to plot it in degrees, declare
%               x_sclvec(2) = 180/pi.
%   x_t_title   (n-dim. Cell) i-th entry contains the title to put
%                       on the plot of x_i(t). E.g., x_t_title{i} =
%                       'Pendulum angle \theta'.
%   x_t_xlabel  (n-dim. Cell) i-th entry contains the x-axis label
%                       to put on the plot of x_i(t). E.g., x_t_xlabel{i} =
%                       '\theta(t) (deg)'.
%   x_t_filename (n-dim. Cell) i-th entry contains the desired file
%                       name for the plot of x_i(t). E.g., x_t_filename{2}
%                       = 'x_2_t' or 'theta_t'.
%                       to put on the plot of x_i(t). E.g., x_t_xlabel{i} =
%                       '\theta(t) (deg)'.
%   u_t_title   (m-dim. Cell) i-th entry contains the title to put
%                       on the plot of u_i(t). E.g., u_t_title{i} = 'Thrust
%                       T'. For m = 1, this could simply be, e.g.,
%                       u_t_title{1} = 'Control Signal u(t)'
%   u_t_xlabel  (m-dim. Cell) i-th entry contains the x-axis label
%                       to put on the plot of u_i(t). E.g., u_t_xlabel{i} =
%                       'T(t)'. For m = 1, this could simply be, e.g.,
%                       u_t_xlabel{1} = 'u(t)'.
%           u_t_filename (m-dim. Cell) i-th entry contains the desired file
%                       name for the plot of u_i(t). E.g., u_t_filename{2}
%                       = 'u_2_t' or 'T_t'.
%   y_propts_cell (m-dim. Cell) i-th entry contains the properties of the
%                       system output y_i, and has the following fields:
%       varname     (String) Variable name for figure saving. E.g., 'g'.
%       texname     (String) LaTEX name (without $$). E.g. '\gamma'.
%       engname     (String) English name. E.g., 'FPA'.
%       units       (String) Units of variable. E.g., 'deg'.
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

switch sys.tag

   

    % ***********************
    %
    % DDMR
    %
    
    case 'ddmr'

        % System dimensions
        sys.n = 2;                  % System order
        sys.m = 2;                  % Number of inputs  
        sys.nlq = sys.model_cell{1}.lin.io.nlq; % System order for LQ servo


    % ***********************
    %
    % PENDULUM
    %
    
    case 'pendulum'

        % System dimensions
        sys.n = 2;                  % System order
        sys.m = 1;                  % Number of inputs  
        sys.nlq = sys.model_cell{1}.lin.io.nlq; % System order for LQ servo

    % ***********************
    %
    % JET
    %
    
    case 'businjet'

        % System dimensions
        sys.n = 4;                  % System order
        sys.m = 2;                  % Number of inputs  
        sys.nlq = sys.model_cell{1}.lin.io.nlq; % System order for LQ servo


    % ***********************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('*** ERROR: SYSTEM TAG NOT RECOGNIZED ***');

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE SYSTEM PLOT SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% DEFAULT SETTINGS
%
% Note: These settings will be initialized as default. If different
% settings are desired, simply overwrite the default value in the switch
% case structure below
%
% *************************************************************************

% Initialize empty structs
x_t_title = cell(sys.n, 1);
x_t_ylabel = cell(sys.n, 1);
x_t_filename = cell(sys.n, 1);
x_t_state_lgd = cell(sys.n, 1);
u_t_title = cell(sys.m, 1);
u_t_ylabel = cell(sys.m, 1); 
u_t_filename = cell(sys.m, 1);

% State variable scaling for plots (default, no scaling)
x_sclvec = ones(sys.n, 1);

% Control variable scaling for plots (default, no scaling)
u_sclvec = ones(sys.m, 1);

% Fill out state trajectory settings
if sys.n == 1
   
    % n = 1. Do not include subscripts in plots.
    x_t_title{1} = ['State Trajectory $x(t)$'];
    x_t_ylabel{1} = ['$x(t)$'];
    x_t_state_lgd{1} = x_t_ylabel{1};
    x_t_filename{1} = ['x_t'];
    
else
    
    % n > 1. Include subscripts in plots.
    for i = 1:sys.n  
        nsi = num2str(i);
        x_t_title{i} = ['State Trajectory $x_{' nsi '}(t)$'];
        x_t_ylabel{i} = ['$x_{' nsi '}(t)$'];  
        x_t_state_lgd{i} = x_t_ylabel{i};
        x_t_filename{i} = ['x_' nsi '_t'];
    end
    
end

% Fill out control signal settings
if sys.m == 1
   
    % m = 1. Do not include subscripts in plots.
    u_t_title{1} = 'Control Signal $u(t)$';
    u_t_ylabel{1} = '$u(t)$';  
    u_t_filename{1} = 'u_t';
    
else
    
    % m > 1. Include subscripts in plots.
    for i = 1:sys.m   
        nsi = num2str(i);
        u_t_title{i} = ['Control Signal $u_{' nsi '}(t)$'];
        u_t_ylabel{i} = ['$u_{' nsi '}(t)$'];  
        u_t_filename{i} = ['u_' nsi '_t'];
    end    
    
end


% *************************************************************************
%
% CUSTOM SETTINGS
%
% Note: If default settings are desired, simply leave the respective system
% case empty.
%
% *************************************************************************

switch sys.tag



    % ***********************
    %
    % DDMR
    %
    
    case 'ddmr'

        % Overwrite state trajectory plot settings
        x_t_title = cell(sys.n, 1);
        x_t_ylabel = cell(sys.n, 1);
        x_t_filename = cell(sys.n, 1);

        % Degree/radian conversions
        D2R = pi/180;
        R2D = 180/pi;

        % Display angles and rates in deg, deg/s
        deg1rad0 = 1;

        if deg1rad0
            x_sclvec([2]) = R2D;
        end
        
        % STATE VARIABLES

        % Start counter
        xcnt = 1;
        
        % V
        x_t_title{xcnt} = 'Speed $V$';
        x_t_ylabel{xcnt} = '$V(t)$ (m/s)';
        x_t_state_lgd{xcnt} = '$V(t)$';
        x_t_filename{xcnt} = 'Vt'; 
        xcnt = xcnt + 1;    % Increment counter

        % \omega
        x_t_title{xcnt} = 'Angular Velocity $\omega$';
        x_t_ylabel{xcnt} = '$\omega(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg/s)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad/s)'];
        end
        x_t_state_lgd{xcnt} = '$\omega(t)$';
        x_t_filename{xcnt} = 'wt';
        xcnt = xcnt + 1;    % Increment counter    

    

        % CONTROL VARIABLES

        % Start counter
        ucnt = 1;


        u_t_title{ucnt} = 'Avg. Voltage $\overline{e}_{a}$';
        u_t_ylabel{ucnt} = '$\overline{e}_{a}(t)$ (V)';          
        u_t_filename{ucnt} = 'oeat';
        ucnt = ucnt + 1;    % Increment counter  

        u_t_title{ucnt} = 'Diff. Voltage $\Delta e_{a}$';
        u_t_ylabel{ucnt} = '$\Delta e_{a}(t)$ (V)';  
        u_t_filename{ucnt} = 'Deat';
        ucnt = ucnt + 1;    % Increment counter          


        % ***********************
        %
        % CREATE CELL ARRAY WITH OUTPUT VARIABLE PROPERTIES
        %
        
        y_propts_cell = cell(2,1);
        for i = 1:size(y_propts_cell,1)
            switch i
        
                % y_i = V
                case 1
                    y_propts_cell{i}.varname = 'V';
                    y_propts_cell{i}.texname = 'V';
                    y_propts_cell{i}.engname = 'Vel.';
                    y_propts_cell{i}.units = 'm/s';
        
                % y_i = \gamma
                case 2
                    y_propts_cell{i}.varname = 'w';
                    y_propts_cell{i}.texname = '\omega';
                    y_propts_cell{i}.engname = 'Ang. Vel.';
                    if deg1rad0
                        y_propts_cell{i}.units = 'deg/s';
                    else
                        y_propts_cell{i}.units = 'rad/s';
                    end
        
            end
        end

        % Store output variable propts
        sys_plot_settings.y_propts_cell = y_propts_cell;

    % ***********************
    %
    % PENDULUM
    %
    
    case 'pendulum'

        % Overwrite state trajectory plot settings
        x_t_title = cell(sys.n, 1);
        x_t_ylabel = cell(sys.n, 1);
        x_t_filename = cell(sys.n, 1);

        % Degree/radian conversions
        D2R = pi/180;
        R2D = 180/pi;

        % Display angles and rates in deg, deg/s
        deg1rad0 = 1;

        if deg1rad0
            x_sclvec([1:2]) = R2D;
        end
        
        % STATE VARIABLES

        % Start counter
        xcnt = 1;
        
        % \theta
        x_t_title{xcnt} = 'Angle $\theta$';
        x_t_ylabel{xcnt} = '$\theta(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad)'];
        end        
        x_t_state_lgd{xcnt} = '$\theta(t)$';
        x_t_filename{xcnt} = 'thetat'; 
        xcnt = xcnt + 1;    % Increment counter

        % \dot{\theta}
        x_t_title{xcnt} = 'Angular Velocity $\omega$';
        x_t_ylabel{xcnt} = '$\dot{\theta}(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg/s)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad/s)'];
        end
        x_t_state_lgd{xcnt} = '$\omega(t)$';
        x_t_filename{xcnt} = 'omegat';
        xcnt = xcnt + 1;    % Increment counter    
    

        % CONTROL VARIABLES

        % Start counter
        ucnt = 1;

        u_t_title{ucnt} = 'Torque $\tau$';
        u_t_ylabel{ucnt} = '$\tau(t)$ (N-m)';          
        u_t_filename{ucnt} = 'taut';
        ucnt = ucnt + 1;    % Increment counter  
    


        % ***********************
        %
        % CREATE CELL ARRAY WITH OUTPUT VARIABLE PROPERTIES
        %

        % NOTE: \dot{\theta} is not an output variable per se, but it is
        % still swept over. So we need to define its properties
        
        y_propts_cell = cell(2,1);
        for i = 1:size(y_propts_cell,1)
            switch i
                case 1
                    y_propts_cell{i}.varname = 'th';
                    y_propts_cell{i}.texname = '\theta';
                    y_propts_cell{i}.engname = 'Angle';
                    if deg1rad0
                        y_propts_cell{i}.units = 'deg';
                    else
                        y_propts_cell{i}.units = 'rad';
                    end
                case 2
                    y_propts_cell{i}.varname = 'omega';
                    y_propts_cell{i}.texname = '\omega';
                    y_propts_cell{i}.engname = 'Ang. Vel.';
                    if deg1rad0
                        y_propts_cell{i}.units = 'deg/s';
                    else
                        y_propts_cell{i}.units = 'rad/s';
                    end
            end
        end

        % Store output variable propts
        sys_plot_settings.y_propts_cell = y_propts_cell;      


 
    % ***********************
    %
    % JET
    %
    
    case 'businjet'

        % Overwrite state trajectory plot settings
        x_t_title = cell(sys.n, 1);
        x_t_ylabel = cell(sys.n, 1);
        x_t_filename = cell(sys.n, 1);

        % Degree/radian conversions
        D2R = pi/180;
        R2D = 180/pi;

        % Display angles and rates in deg, deg/s
        deg1rad0 = 1;

        if deg1rad0
            x_sclvec([2 3 4]) = R2D;
        end

        % Display elevator deflection in deg
        u_sclvec(2) = R2D;
        
        % STATE VARIABLES

        % Start counter
        xcnt = 1;
        
        % V
        x_t_title{xcnt} = 'Airspeed $V$';
        x_t_ylabel{xcnt} = '$V(t)$ (m/s)';
        x_t_state_lgd{xcnt} = '$V(t)$';
        x_t_filename{xcnt} = 'Vt'; 
        xcnt = xcnt + 1;    % Increment counter

        % \gamma
        x_t_title{xcnt} = 'FPA $\gamma$';
        x_t_ylabel{xcnt} = '$\gamma(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad)'];
        end
        x_t_state_lgd{xcnt} = '$\gamma(t)$';
        x_t_filename{xcnt} = 'gammat';
        xcnt = xcnt + 1;    % Increment counter    
       
        % q
        x_t_title{xcnt} = 'Pitch Angular Rate $q$';
        x_t_ylabel{xcnt} = '$q(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg/s)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad/s)'];
        end    
        x_t_state_lgd{xcnt} = '$q(t)$';
        x_t_filename{xcnt} = 'qt';
        xcnt = xcnt + 1;    % Increment counter        

        % \alpha
        x_t_title{xcnt} = 'AOA $\alpha$';
        x_t_ylabel{xcnt} = '$\alpha(t)$';
        if deg1rad0
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (deg)'];
        else
            x_t_ylabel{xcnt} = [x_t_ylabel{xcnt} ' (rad)'];
        end        
        x_t_state_lgd{xcnt} = '$\alpha(t)$';
        x_t_filename{xcnt} = 'alphat';
        xcnt = xcnt + 1;    % Increment counter

        

        % CONTROL VARIABLES

        % Start counter
        ucnt = 1;

        u_t_title{ucnt} = 'Throttle $\delta_{T}$';
        u_t_ylabel{ucnt} = '$\delta_{T}(t)$';          
        u_t_filename{ucnt} = 'dTt';
        ucnt = ucnt + 1;    % Increment counter  

        u_t_title{ucnt} = 'Elevator Deflection $\delta_{E}$';
        u_t_ylabel{ucnt} = '$\delta_{E}(t)$ (deg)';  
        u_t_filename{ucnt} = 'dEt';
        ucnt = ucnt + 1;    % Increment counter  


        % ***********************
        %
        % CREATE CELL ARRAY WITH OUTPUT VARIABLE PROPERTIES
        %
        
        y_propts_cell = cell(2,1);
        for i = 1:2
            switch i
        
                % y_i = V
                case 1
                    y_propts_cell{i}.varname = 'V';
                    y_propts_cell{i}.texname = 'V';
                    y_propts_cell{i}.engname = 'Vel.';
                    y_propts_cell{i}.units = 'm/s';
        
                % y_i = \gamma
                case 2
                    y_propts_cell{i}.varname = 'g';
                    y_propts_cell{i}.texname = '\gamma';
                    y_propts_cell{i}.engname = 'FPA';
                    y_propts_cell{i}.units = 'deg';

        
            end
        end

        % Store output variable propts
        sys_plot_settings.y_propts_cell = y_propts_cell;




        
        
    % ***********************
    %
    % USE DEFAULT SETTINGS IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        % Use default settings

end

% *************************************************************************
%
% STORE SYSTEM PLOT SETTINGS
% 
% *************************************************************************

sys_plot_settings.x_sclvec = x_sclvec;
sys_plot_settings.u_sclvec = u_sclvec;

sys_plot_settings.x_t_title = x_t_title;
sys_plot_settings.x_t_ylabel = x_t_ylabel;
sys_plot_settings.x_t_state_lgd = x_t_state_lgd;
sys_plot_settings.x_t_filename = x_t_filename;
sys_plot_settings.u_t_title = u_t_title;
sys_plot_settings.u_t_ylabel = u_t_ylabel;
sys_plot_settings.u_t_filename = u_t_filename;

