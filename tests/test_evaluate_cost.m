%
% Tutorial: select a set of parameters to tune withing the LQR design
%           weights. Propose the limits of a box domain, discretize it, and
%           compute the associated cost values. Find the optimal controller
%           using the min() function.
%
%           Before completing this tutorial, one needs to finish:
%               tutorial_cart_pole_equations.m
%               test_manual_tuning.m
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
    
%% Construct cost unknown cost function:

% Empirical weights:
    ew = struct(); 
    [ew.Q,ew.R] = get_LQR_weights([1 1 1 1]);
    
% Initial conditions:
    [x0,F0] = initial_conditions();
    
% Get J_empirical:
    J_empirical = construct_unknown_cost_function(A,B,f,x0,F0,sims,ew);
    
%% Evaluate the cost function for a certain set of deisgn weights:

% Design weights:
    dw = struct();
    [dw.Q,dw.R] = get_LQR_weights([1 3 1 1]);
    
% Evaluate the cost:
    Jval = J_empirical(dw);
    
%% Vary one parameter:

    Neval = 11;
    theta_vec = logspace(-1,1,Neval);
    
    % Evaluate at multiple locations:
    fprintf(' ** Evaluating unknown cost function');
    J = zeros(Neval,1);
    for k = 1:Neval
        fprintf('.')
        [dw.Q,dw.R] = get_LQR_weights([1 theta_vec(k) 1 1]);
        J(k) = J_empirical(dw);
    end
    fprintf '\n';
    
%% Plot the cost:
    figure; 
    [J_min,ind_min] = min(J);
    semilogx(theta_vec,J,'o--',theta_vec(ind_min),J_min,'or')
    legend('empirical cost',['minimum cost at theta = ' num2str(exp(theta_vec(ind_min)))]);
    