%
% Tutorial: use an altered linear model to compute an LQR controller that
%           stabilizes the true system. Observe that the associated LQR
%           cost is suboptimal, and manually tune the design weights to try
%           to improve it (i.e., minimize it).
%
%           Before completing this tutorial, one needs to finish:
%               tutorial_cart_pole_equations.m
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
    clear all; close all; clc;

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

% Get empirical Q and R weight matrices:
    ew = struct(); % Empirical weights
    [ew.Q,ew.R] = get_LQR_weights([1 1 1 1]);
    
% Get design Q and R weight matrices:
    dw = struct(); % Design weights
    [dw.Q,dw.R] = get_LQR_weights([1 2 1 1]);
    
%% Simulate pole dynamics:

% Design LQR control gain:
    K = -dlqr(A,B,dw.Q,dw.R);

% Simulate pole dynamics:
    [x0,F0] = initial_conditions();
    [x_simu,F_simu] = simulate_cart_pole(f,K,x0,F0,sims);
    
%% Compute LQR cost:
    J_val = get_lqr_cost(ew,x_simu,F_simu);
    
%% Plot signals:
    [fig_hdl,hdl_splot] = initialize_plots();
    plot_cart_pole(fig_hdl,hdl_splot,x_simu,F_simu,sims);
    
%% Cart-pole visual simulation:
    visual_simulation(x_simu,cp,sims);
    