%
% Tutorial: use an altered linear model to compute an LQR controller that
%           stabilizes the true system. Observe that the associated LQR
%           cost is suboptimal, and manually tune the design weights to try
%           to improve it (i.e., minimize it).
%
%           Before completing this tutorial, one needs to finish:
%               tut1_cart_pole_equations.m
%
% =========================================================================
% Automatic LQR Tuning tutorial, using a cart-pole system as example
%
% Copyright 2016 Max Planck Society. All rights reserved.
% 
% Alonso Marco
% Max Planck Institute for Intelligent Systems
% Autonomous Motion Department
% amarco(at)tuebingen.mpg.de
%
% Revision history
% First version: 24.06.2016
%

%% Initialization: 
% When running this block, all the variables in the workspace will be
% erased! Please, type help <function_name> to get more information about
% all the functions used in this tutorial
    clear all; close all;

%% Construct cart-pole system:

% Construct cart-pole system:
    cp = construct_cart_pole();
    
% Construct cart-pole system with some wrong parameters:
    cpw = construct_cart_pole_wrong(cp);

%% Get pole linear and non-linear dynamics:

% Construct simulation settings:
    sims = construct_simulator_settings();

% Get non-linear cart-pole dynamics:
    f = construct_nonlinear_dynamics(cp,sims);

% Get linearized dynamics with the wrong pole paramaters:
    [A,B] = get_linearized_dynamics(cpw,sims);

%% Tune LQR weights:

% Get empirical Q and R weight matrices: these weight matrices encode the
% desired behavior we expect on the real system, and thus, it makes sense
% to fix them.
    ew = struct(); % Empirical weights
    [ew.Q,ew.R] = get_LQR_weights([100 1 100 1]);
    
% Get design Q and R weight matrices: these weights have to be manually
% adjusted to obtain LQR controllers able to improve the performance on the
% real system. Notice that the linear system [A,B] is now computed out of
% the available model (which is a wrong model). Therefore, by manually
% tuning dw.Q, we should try to make J(dw) close to J_best
    dw = struct(); % Design weights
    [dw.Q,dw.R] = get_LQR_weights([10000 1 10000 1]); % [10000 1 10000 1]
    
%% Simulate pole dynamics:

% Design LQR control gain:
    K = -dlqr(A,B,dw.Q,dw.R);

% Simulate pole dynamics:
    [x0,F0] = initial_conditions();
    [x_simu,F_simu] = simulate_cart_pole(f,K,x0,F0,sims);
    
%% Compute LQR cost:
% This function needs the empricial weights and the time signals from the
% simulation (x_simu,F_simu). It returns the associated quadratic cost.
    J_val = get_lqr_cost(ew,x_simu,F_simu);
    
%% Compute best possible cost, just for comparison:
% The best cost possible can be obtained if the linearized model is
% computed using the correct pole parameters (cp) instead of the wrong ones
% (cpw). This controller, when used on the real system, also with the
% design weight equal to the empirical weights, retrieve the best possible
% cost for this setting. We compute just for compare with J_val.
    [Ab,Bb] = get_linearized_dynamics(cp,sims);
    Kb = -dlqr(Ab,Bb,ew.Q,ew.R);
    [x_simub,F_simub] = simulate_cart_pole(f,Kb,x0,F0,sims);
    J_best = get_lqr_cost(ew,x_simub,F_simub);

%% Display information:
    display(' ');
    display(' ** dw.Q');
    disp(dw.Q);
    display(' ** dw.R');
    disp(dw.R);
	display([' ** J(dw) = ' num2str(J_val,'%10.5f')]);
    display([' ** J_best = ' num2str(J_best,'%10.5f')]);
    
%% Plot signals:
% Comment out this cell if undesired
    [fig_hdl,hdl_splot] = initialize_plot();
    plot_cart_pole(fig_hdl,hdl_splot,x_simu,F_simu,sims);
    
%% Cart-pole visual simulation:
% Comment out this cell if undesired
    visual_simulation(x_simu,cp,sims);
