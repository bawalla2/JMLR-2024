function figcount = plot_x0_sweep(alg_settings_cell,...
                        out_data_cell, group_settings, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOTS FOR INITIAL CONDITION AND MODELING ERROR SWEEP
%
% Brent A. Wallace
%
% 2022-03-11
%
% *************************************************************************
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
%   ICs             (Struct) Contains data for the 2D surface plot
%                   settings. Has the following fields:
%       x10vec      (Vector) Contains a vector of evaluations points for
%                   the first state variable x_1 (i.e., the variable
%                   plotted on the x-axis of the surface plots).
%       x20vec      (Vector) Contains a vector of evaluations points for
%                   the first state variable x_2 (i.e., the variable
%                   plotted on the y-axis of the surface plots).
% out_data_cell     (3-dim Cell) A cell array containing algorithm output
%                   data with the dimensions indexed in: (x_1, x_2, \nu),
%                   where x_1 is the first IC state variable swept, x_2 is
%                   the second IC state variable swept, and \nu is the
%                   vector of modeling error parameters swept.
% master_settings   (Struct) Contains master program settings. Has the
%                   following fields relevant to this program:
%   savefigs        (Bool) Save figures to PDF (=1) or not (=0).
%   relpath         (String) Relative path to figures (for saving only).
%                   NOTE: This string is auto-set. See '/00 figures'.
%   indsmodelplot   (Vector) This contains the indices in the system
%                   modeling error vector 'nuvec' (see config_settings.m)
%                   for which to plot sweep data for.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% figcount          (Integer) Cumulative figure count after all plots in
%                   this function have been created.
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
% dolegend = group_settings.dolegend;


% Initialize figure counter
figcount = group_settings.figcount;


% Plot nominal LQ curves in surface plots (=1) or not (=0)
plot_nom_lq = 1;

% Number of significant digits to display data with at command window
nsd = 3;

% Number of decimal points to display to at command window
fspc = '%.2f';

% System names
sysnames = master_settings.sysnames;

% Current system being executed
systag = master_settings.systag;

% ***********************
%
% SURFACE PLOT SETTINGS -- WEIGHT VALUES
%        

% Master plot formatting settings
psett_master = master_settings.psett_master;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- colors
colors = psett_master.colors;

% Plot formatting -- color commands
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;
psett_black = psett_master.psett_black;
psett_gray = psett_master.psett_gray;

% Plot formatting -- loop-wise color commands
% loop_color_sett_cell = psett_master.loop_color_sett_cell;
color_sett_cell = psett_master.color_sett_cell;


% Surface plot face transparency
facealpha = psett_master.facealpha;

% Surface plot edge transparency
edgealpha = psett_master.edgealpha;

% ***********************
%
% SURFACE PLOT SETTINGS -- OPTIMAL WEIGHT VALUE PLANES
%        

% Surface plot face color
facecolor_hl = psett_master.facecolor_hl;

% Surface plot face transparency
facealpha_hl = psett_master.facealpha_hl;

% Surface plot edge transparency
edgealpha_hl = psett_master.edgealpha_hl;    


% ***********************
%
% SYSTEM PLOT SETTINGS
%

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


% Sweep variable indices
inds_x_sweep = group_settings.inds_x_sweep;

% IC data
ICs = group_settings.ICs;
numx10 = ICs.numx10;
numx20 = ICs.numx20;
numICs = ICs.numICs;
x10vec = ICs.x10vec;
x20vec = ICs.x20vec;
x0mat = ICs.x0mat;

% Scale these IC vectors to desired units
for i = 1:2
    ind_xri = inds_x_sweep(i);
    currscl = x_sclvec(ind_xri);
    switch i
        case 1
            x10vecp = x10vec * currscl;
        case 2
            x20vecp = x20vec * currscl;
    end
end

        

% % Indices in the state vector corresponding to the state variables swept
% xinds = group_settings.xinds;
% 
% % Check whether surface plots should be plotted or not
% do_sweep_plots = (nx1 > 1) && (nx2 > 1);

% Create meshgrid of IC values
[X1, X2] = meshgrid(x10vecp, x20vecp);

% Get verticies of IC sweep
x1min = min(x10vecp);
x1max = max(x10vecp);
x2min = min(x20vecp);
x2max = max(x20vecp);
x1vert = [x1min x1max x1max x1min];
x2vert = [x2min x2min x2max x2max];


% % Matrix of ICs to plot auxiliary plots for (see below)
% aux_plot_x0s = group_settings.aux_plot_x0s;
% aux_plot_x0s_inds = group_settings.aux_plot_x0s_inds;
% num_aux_plot_x0s = size(aux_plot_x0s, 1);



% ***********************
%
% PULL SYSTEM DATA
%   

% System
sys = master_settings.sys;

model_cell = sys.model_cell;
nummodels = sys.nummodels;
indsmodelplot = master_settings.indsmodelplot;
nummodelsplot = master_settings.nummodelsplot;
indnom = sys.indnom;

% Get vector of \nu values
nuvec = sys.nuvec;
nuvec_plot = nuvec(indsmodelplot);

% LQ data
lq_data_cell = master_settings.lq_data_cell;

% Get nominal LQ data
lq_data_nom = lq_data_cell{indnom};


% ***********************
%
% PULL OPTIMAL LQ DATA
%   

% Number of loops executed
numloops = size(alg_settings_cell{1}.loop_cell,1);

% Simulation model optimal LQ data
lq_data_opt_sim_cell = cell(nummodels,1);

% Optimal controllers in each loop
Kstar_cell = cell(nummodels,numloops,1);

% Get optimal LQ data for each model
for mcnt = 1:nummodels
    model_sim_ind = alg_settings_cell{1,1,mcnt}.model_sim_ind;
    lqdataoptsim = lq_data_cell{model_sim_ind};
    lq_data_opt_sim_cell{mcnt} = lqdataoptsim;
    for j = 1:numloops
        switch numloops
            case 1
                Kstar_cell{mcnt,j} = lqdataoptsim.Kcirl;
            case 2
                switch j
                    case 1
                        Kstar_cell{mcnt,j} = lqdataoptsim.lq_data_11.K;
                    case 2
                        Kstar_cell{mcnt,j} = lqdataoptsim.lq_data_22.K;
                end
        end
    end
end




% ***********************
%
% PLOT SETTINGS -- COLORS
%  

% Loop colors
loop_color_sett_cell = color_sett_cell(1:numloops);

% List of colors
facecolor_list = cell(nummodelsplot,1);

% Counter to keep track of number of models so far that weren't the nominal
cntnotnom = 1;

for mcnt = 1:nummodelsplot
    
    % Get the index of the current model
    indmcnt = indsmodelplot(mcnt); 

    % If \nu = 1, set color to blue. Else, go down the line
    if indmcnt == indnom
        facecolor_list{mcnt} = colors.matlabblue;
    else
        switch cntnotnom
            case 1                
                facecolor_list{mcnt} = colors.matlabgreen; 
            case 2
                facecolor_list{mcnt} = colors.matlabmaroon;     
            case 3
                facecolor_list{mcnt} = colors.matlabyellow;                   
        end
        % Increment counter
        cntnotnom = cntnotnom + 1;
    end

end

% ***********************
%
% PLOT SETTINGS -- LEGEND -- \nu VALUES
%  

lgd_nu = cell(nummodelsplot,1);

for mcnt = 1:nummodels   

    % Get \nu for this model
    numcnt = nuvec(mcnt);

    % Add legend entry
    lgd_nu{mcnt} = ['$\nu = ' num2str(numcnt) '$'];

end

% Plotted values of \nu only
lgd_nu_plot = lgd_nu(indsmodelplot);

% ***********************
%
% PLOT SETTINGS -- LEGEND -- LOOP NAMES
%  

lgd_loop = cell(numloops,1);

for j = 1:numloops
    
    lgd_loop{j} = ['$j =' num2str(j) '$ $('...
        y_propts_cell{j}.texname ')$'];

end





% ***********************
%
% PLOT SETTINGS -- x, y AXIS LABELS FOR IC SWEEP PLOTS
%  

x0surf_labels = cell(2,1);

for i = 1:2

    y_propts = y_propts_cell{i};
    currtexname = y_propts.texname;
    currunits = y_propts.units;
%     x0surf_labels{i} = ['$' currtexname '(0)$ (' currunits ')'];
    x0surf_labels{i} = ['$' currtexname '_{0}$ (' currunits ')'];

end



% *************************************************************************
% 
% PULL, CALCULATE SWEEP DATA
%
% *************************************************************************

% ***********************
%
% PULL SWEEP DATA
%  

sweep_lq_data_cell = cell(numx10,numx20,nummodels);
cond_data_cell = cell(numx10,numx20,nummodels);

% Stores the iteration-max conditioning at each IC for each model
max_cond_data_mat = zeros(numx10,numx20,nummodels,numloops);

% Final controller error 
normeKjmat = zeros(numx10,numx20,nummodels,numloops);

for x1c = 1:numx10
    for x2c = 1:numx20
        for mcnt = 1:nummodels

            % Extract current out_data struct
            outdata = out_data_cell{x1c,x2c,mcnt};

            % Extract lq_data
            lqdata = outdata.dirl_data.lq_data;

            % Extract conditioning data
            conddata = outdata.cond_A_vec_cell;

            % For each of the loops j, calculate ||K_{i^*,j} - K_j^*||
            for j = 1:numloops
                
                % Get optimal LQ controller for this model, this loop
                Kstarj = Kstar_cell{mcnt,j};

                % Get final controller for this IC, this model
                Kistarj = outdata.K_cell{j}(:,:,end);

                % Calculate final controller error
                eKj = Kstarj - Kistarj;
                normeKj = norm(eKj);

                % Store error
                normeKjmat(x1c,x2c,mcnt,j) = normeKj;
                               
            end

            % For each of the loops j, calculate max conditioning
            for j = 1:numloops
                
                % Get conditioning data in this loop
                conddataj = conddata{j};

                % Get iteration-max conditioning
                maxcondj = max(conddataj);

                % Store
                max_cond_data_mat(x1c,x2c,mcnt,j) = maxcondj;

            end


            % Store lq_data
            sweep_lq_data_cell{x1c,x2c,mcnt} = lqdata;

            % Store conditioning data
            cond_data_cell{x1c,x2c,mcnt} = conddata;

        end
    end
end


% ***********************
%
% CALCULATE NOMINAL LQ CONTROLLER ERROR
%  

Kstar_nom_cell = cell(numloops,1);
normeKjnommat = zeros(nummodels,numloops);

% Get nominal LQ controllers
for j = 1:numloops
    switch numloops
        case 1
            Kstar_nom_cell{j} = lq_data_nom.Kcirl;
        case 2
            switch j
                case 1
                    Kstar_nom_cell{j} = lq_data_nom.lq_data_11.K;
                case 2
                    Kstar_nom_cell{j} = lq_data_nom.lq_data_22.K;
            end
    end
end

% For each model and loop, calculate ||K_{nom,j}^{*} - K_j^*||
for mcnt = 1:nummodels
    for j = 1:numloops
        
        % Get optimal LQ controller for this model, this loop
        Kstarj = Kstar_cell{mcnt,j};

        % Get nominal controller for this loop
        Knomj = Kstar_nom_cell{j};

        % Calculate controller error
        eKj = Kstarj - Knomj;
        normeKj = norm(eKj);

        % Store error
        normeKjnommat(mcnt,j) = normeKj;

    end
end

% ***********************
%
% CALCULATE MEAN, MAX, STD CONTROLLER ERROR, PERCENT CONTROLLER ERROR,
% CONDITIONING BASED ON SWEEP DATA
%  

% IC sweep mean, max, std controller error vs \nu
avgnormeKjmat = zeros(nummodels,numloops);
maxnormeKjmat = zeros(nummodels,numloops);
stdnormeKjmat = zeros(nummodels,numloops);

% IC sweep mean, max, std percent controller error vs \nu
avgpcteKjmat = zeros(nummodels,numloops);
maxpcteKjmat = zeros(nummodels,numloops);
stdpcteKjmat = zeros(nummodels,numloops);

% IC sweep mean, max, std conditioning vs \nu
avgcondjmat = zeros(nummodels,numloops);
maxcondjmat = zeros(nummodels,numloops);
stdcondjmat = zeros(nummodels,numloops);

for mcnt = 1:nummodels
    for j = 1:numloops

        % Get nominal LQ error
        normeKjnom = normeKjnommat(mcnt,j);
        isoptKjnom = (normeKjnom < 1e-4);

        % Pull controller error data
        eKjx0s = normeKjmat(:,:,mcnt,j);
        eKjx0s = eKjx0s(:);

        % Calculate mean, max, std controller error
        avgnormeKj = mean(eKjx0s);
        maxnormeKj = max(eKjx0s);
        stdnormeKj = std(eKjx0s);


        % Calculate controller percent error data
        if ~isoptKjnom
            pcteKjx0s = eKjx0s / normeKjnom * 100;
        end

        % Calculate mean, max, std controller percent error
        if ~isoptKjnom
            avgpcteKj = mean(pcteKjx0s);
            maxpcteKj = max(pcteKjx0s);
            stdpcteKj = std(pcteKjx0s);
        else
            avgpcteKj = -1;
            maxpcteKj = -1;
            stdpcteKj = -1;
        end

        % Pull conditioning data
        conddataj = max_cond_data_mat(:,:,mcnt,j);
        conddataj = conddataj(:);

        % Calculate mean, max, std conditioning
        avgcondj = mean(conddataj);
        maxcondj = max(conddataj);
        stdcondj = std(conddataj);

        % Store mean, max, std controller error
        avgnormeKjmat(mcnt,j) = avgnormeKj;
        maxnormeKjmat(mcnt,j) = maxnormeKj;
        stdnormeKjmat(mcnt,j) = stdnormeKj;

        % Store mean, max, std controller percent error
        avgpcteKjmat(mcnt,j) = avgpcteKj;
        maxpcteKjmat(mcnt,j) = maxpcteKj;
        stdpcteKjmat(mcnt,j) = stdpcteKj;

        % Store mean, max, std conditioning
        avgcondjmat(mcnt,j) = avgcondj;
        maxcondjmat(mcnt,j) = maxcondj;
        stdcondjmat(mcnt,j) = stdcondj;

    end
end



% ***********************
%
% CALCULATE WORST-CASE OPTIMALITY RECOVERY
%  
       
% IC sweep worst-case optimality recovery vs \nu
minerrredmat = zeros(nummodels,numloops);

for mcnt = 1:nummodels
    for j = 1:numloops
        
        % Get worst-case controller error
        maxnormeKj = maxnormeKjmat(mcnt,j);

        % Get nominal LQ error
        normeKjnom = normeKjnommat(mcnt,j);

        % Calculate worst-case optimality recovery
        minerrred = (1 - maxnormeKj / normeKjnom) * 100;

        % Store worst-case optimality recovery
        minerrredmat(mcnt,j) = minerrred;

    end
end




%%
% *************************************************************************
% *************************************************************************
%
% PLOTS: CONTROLLER ERROR VS. IC FOR VARYING MODELS
%
% ************************************************************************* 
% *************************************************************************

for j = 1:numloops 

    for mcnt = 1:nummodelsplot 
    
        % Get the index of the current model
        indmcnt = indsmodelplot(mcnt);

        % Extract data for this model, this loop
        neKjmat = normeKjmat(:,:,indmcnt,j);
        

        % PLOT
        figure(figcount)    
        h_fig = surf(X1, X2, neKjmat');
        set(h_fig, 'FaceColor', facecolor_list{mcnt});
        set(h_fig, 'FaceAlpha', facealpha);
        set(h_fig, 'EdgeAlpha', edgealpha);            
        hold on
        
    end

    % Plot the nominal LQ error
    if plot_nom_lq

        % Extract the nominal LQ error
        normeKj = normeKjnommat(end,j);

        % Extract \nu for this model
        nuend = nuvec_plot(end);

        % PLOT
        h_fig = patch('XData', x1vert, 'YData', x2vert, ...
                'ZData', normeKj * ones(1,4));
        set(h_fig, 'FaceColor', facecolor_hl);
        set(h_fig, 'FaceAlpha', facealpha_hl);
        set(h_fig, 'EdgeAlpha', edgealpha_hl);      

        % Legend entry
        eKjnom_str_nd = ['e_{K_{' num2str(j) '},nom}(' num2str(nuend) ')'];
        eKjnom_str = ['Nom'];

    end     
    

  Kerr_str = ['$||\mu_{i^{*},' num2str(j) '} - K_{' num2str(j) '}^{*}||$'];  
    ttl = ['Policy Optimality Error ' Kerr_str ' vs. $x_{0}$'];
    title(ttl)          
    xlabel(x0surf_labels{1});
    ylabel(x0surf_labels{2});
    zlabel([Kerr_str]);
    xlim([x1min x1max]);
    ylim([x2min x2max]);
    
    if plot_nom_lq
        lgd = legend([lgd_nu_plot; {eKjnom_str}]);    
    else
        lgd = legend(lgd_nu_plot);   
    end       

    % Position legend, view manually
    % Get current legend position
    currfig = gcf;
    lgdpos = currfig.CurrentAxes.Legend.Position;
    switch systag
        case sysnames.ddmr
            lgdpos(1:2) = [0.5414 0.4982];  
            p_sett.custom_sett.lgd_position = lgdpos;
            p_sett.custom_sett.axview = [-30.4919 13.8250];
    end

    % Format plot
    p_sett.figcount = figcount;
    plot_format(p_sett);   
    clear p_sett;
    
    % SAVE PLOT
    if savefigs
        jstr = strrep(num2str(j),'.','p');
        filename = ['K_err_vs_x0_j_' jstr];
        savepdf(figcount, relpath, filename); 
    end

    % Increment figure counter
    figcount = figcount + 1; 

end


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: WORST-CASE CONTROLLER ERROR VS. MODELING ERROR
%
% ************************************************************************* 
% *************************************************************************

figure(figcount)
hold on;

for j = 1:numloops 

    % PLOT
    h_fig = plot(nuvec, maxnormeKjmat(:,j)); 
    
end

eKjmax_str_nd = 'e_{K_{j},\max}';
eKjmax_str = ['$' eKjmax_str_nd '$'];

ttl = ['Worst-Case Controller Error ' eKjmax_str ' vs. $\nu$'];
title(ttl)          
xlabel('$\nu$');
ylabel(['$' eKjmax_str_nd '(\nu)$']);
xlim([min(nuvec) max(nuvec)]);

% if numloops > 1
%     lgd = legend(lgd_loop);    
% end       
lgd = legend(lgd_loop);

% Format plot
p_sett.figcount = figcount;
p_sett.indiv_sett_cell = loop_color_sett_cell;
plot_format(p_sett);     

% SAVE PLOT
if savefigs
    filename = ['max_K_err_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 


%%
% *************************************************************************
% *************************************************************************
%
% PLOT: WORST-CASE CONTROLLER ERROR VS. MODELING ERROR -- WITH NOMINAL LQ
% ERROR
%
% ************************************************************************* 
% *************************************************************************

figure(figcount)
hold on;

for j = 1:numloops 

    % PLOT
    h_fig = plot(nuvec, maxnormeKjmat(:,j)); 
    
end

% Legend entries -- nomial controller error
lgd_eKjnom = cell(numloops,1);

% Individual settings -- nominal controller error
indiv_sett_cell_nom = cell(numloops,1);
for j = 1:numloops
    switch j
        case 1
            switch systag
                case sysnames.ddmr
                    indiv_sett_cell_nom{j} = [psett_gray; psett_dash];
                otherwise
                    indiv_sett_cell_nom{j} = [psett_gray];
            end
        case 2
            indiv_sett_cell_nom{j} = [psett_gray];
    end
end


for j = 1:numloops 
    
    % Legend entry
    eKjnom_str_nd = ['e_{K_{' num2str(j) '},nom}'];
    eKjnom_str = ['$' eKjnom_str_nd '$'];
    lgd_eKjnom{j} = eKjnom_str;
    
    % PLOT
    h_fig = plot(nuvec, normeKjnommat(:,j)); 
    
end

ttl = ['Worst-Case Controller Error ' eKjmax_str ' vs. $\nu$'];
title(ttl)          
xlabel('$\nu$');
ylabel(['$' eKjmax_str_nd '(\nu)$']);
xlim([min(nuvec) max(nuvec)]);

% if numloops > 1
%     lgd = legend([lgd_loop; lgd_eKjnom]);   
% end       
lgd = legend([lgd_loop; lgd_eKjnom]);

% Format plot
p_sett.figcount = figcount;
p_sett.indiv_sett_cell = [loop_color_sett_cell; indiv_sett_cell_nom];
plot_format(p_sett);     

% SAVE PLOT
if savefigs
    filename = ['max_K_err_vs_nu_w_eKj_nom'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 



%%
% *************************************************************************
% *************************************************************************
%
% PLOT: WORST-CASE CONDITIONING VS. MODELING ERROR
%
% ************************************************************************* 
% *************************************************************************

figure(figcount)
hold on;

for j = 1:numloops 

    % PLOT
    h_fig = plot(nuvec, maxcondjmat(:,j)); 
    
end

condjmax_str_nd = '\overline{\kappa}({\bf{A}}_{i,j})_{\max}';
condjmax_str = ['$' condjmax_str_nd '$'];

ttl = ['Worst-Case Conditioning ' condjmax_str ' vs. $\nu$'];
title(ttl)          
xlabel('$\nu$');
ylabel(['$' condjmax_str_nd '(\nu)$']);
xlim([min(nuvec) max(nuvec)]);

% if numloops > 1
%     lgd = legend(lgd_loop);    
% end       
lgd = legend(lgd_loop);  

% Format plot
p_sett.figcount = figcount;
plot_format(p_sett);     

% SAVE PLOT
if savefigs
    filename = ['max_cond_vs_nu'];
    savepdf(figcount, relpath, filename); 
end

% Increment figure counter
figcount = figcount + 1; 


%%
% *************************************************************************
% *************************************************************************
%
% PRINT SWEEP METRICS
%
% ************************************************************************* 
% *************************************************************************

disp('***************************************************************')
disp('*')
disp('* DISPLAYING x_0 SWEEP PERFORMANCE METRICS')
disp('*')
disp('***************************************************************')


% ***********************
%
% FORMATTING
%  

% One tab
onetab = '  ';
twotab = [onetab onetab];
fourtab = [twotab twotab];

% String to print out \nu values
strng_j = [fourtab twotab];
for j = 1:numloops
    strng_j = [strng_j lgd_loop{j} onetab];
end

% ***********************
%
% NOMINAL CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* NOMINAL CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = normeKjnommat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% MAX CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* MAX CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = maxnormeKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% MEAN CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* MEAN CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = avgnormeKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end


% ***********************
%
% STD CONTROLLER ERROR
%  

disp('*****')
disp('*')
disp('* STD CONTROLLER ERROR:')
disp('*')

disp(strng_j);
tmpmat = stdnormeKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end


% ***********************
%
% MAX CONTROLLER PERCENT ERROR (%)
%  

disp('*****')
disp('*')
disp('* MAX CONTROLLER PERCENT ERROR (%):')
disp('*')

disp(strng_j);
tmpmat = maxpcteKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% MEAN CONTROLLER PERCENT ERROR (%)
%  

disp('*****')
disp('*')
disp('* MEAN CONTROLLER PERCENT ERROR (%):')
disp('*')

disp(strng_j);
tmpmat = avgpcteKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% STD CONTROLLER PERCENT ERROR (%)
%  

disp('*****')
disp('*')
disp('* STD CONTROLLER PERCENT ERROR (%):')
disp('*')

disp(strng_j);
tmpmat = stdpcteKjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end


% ***********************
%
% MAX CONDITIONING
%  

disp('*****')
disp('*')
disp('* MAX CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = maxcondjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% MEAN CONDITIONING
%  

disp('*****')
disp('*')
disp('* MEAN CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = avgcondjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

% ***********************
%
% STD CONDITIONING
%  

disp('*****')
disp('*')
disp('* STD CONDITIONING:')
disp('*')

disp(strng_j);
tmpmat = stdcondjmat;
for mcnt = 1:nummodels
    tmp = num2str(tmpmat(mcnt,:));
    disp([lgd_nu{mcnt} twotab tmp])
end

