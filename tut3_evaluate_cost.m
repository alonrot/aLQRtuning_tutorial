%
% Tutorial: select a set of parameters to tune withing the LQR design
%           weights. Propose the limits of a box domain, discretize it, and
%           compute the associated cost values. Find the optimal controller
%           using the min() function.
%
%           Before completing this tutorial, one needs to finish:
%               tut1_cart_pole_equations.m
%               tut2_manual_tuning.m
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
% First version: 28.06.2016
%

%% Initialization: 
% When running this block, all the variables in the workspace will be
% erased! Please, type help <function_name> to get more information about
% all the functions used in this tutorial
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
    
%% Construct unknown cost function:

% Empirical weights:
    ew = struct(); 
    [ew.Q,ew.R] = get_LQR_weights([100 1 100 1]);
    
% Initial conditions:
    [x0,F0] = initial_conditions();
    
% Get J_empirical: this function returns a handle to the empirical cost
% function, created with the empirical weights (ew). Type "help
% construct_unknown_cost_function" for more information.
    J_empirical = construct_unknown_cost_function(A,B,f,x0,F0,sims,ew);
    
%% Evaluate the cost function for a certain set of design weights:
% This step is meant to exemplify how to use this new function
% J_empirical(). We use the same weights as the empirical weights

% Design weights:
    dw = struct();
    [dw.Q,dw.R] = get_LQR_weights([100 1 100 1]);
    
% Evaluate the cost: this function, created in the block above, maps a set
% of design weights with a cost function value.
    Jval = J_empirical(dw);
    
%% Vary one parameter:
% Choose one parameter of the diagonal of dw.Q to tune. Choose a domain for
% this parameter, where you think you can find a controller that improves
% the system performance (i.e., decreases the cost). Discretize the domain
% with 5 points and compute for each point a cost function value calling
% the function J_empirical()
% Hint: use linspace() or logspace()

%     theta = 
%     J = 

% Find the minimum using the function min()
%   theta_min = 
%   J_min = 
    
%% Plot the cost:
% Plot the cost value for each parameter, and the minimum