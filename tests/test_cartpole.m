%
% Tutorial: understand each cell of the script and implement one function
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

%% Initialization:
    clear all; close all; clc;

%% Construct cart-pole system:
% Open this function and change manually the physical parameters of the
% cart-pole system.
    cp = construct_cart_pole();

%% Get pole linear and non-linear dynamics:

% Construct simulation settings: within this function, parameters like
% sampling time or simulation time can be chosen. The sampling time must be
% chosen small enough, typically such that the sampling frequency is much
% faster than the dynamics of the system.
    sims = construct_simulator_settings();

% Get non-linear cart-pole dynamics: this function uses the non-linear
% equations of the cart-pole system to generate the transition function
% f(x,u), such that x(k+1) = f(x(k),u(k)). These non-linear dynamics are
% computed using the parameters of the physical cart-pole system, passed in
% the structure cp.
    f = construct_nonlinear_dynamics(cp,sims);

% Get linearized dynamics with the wrong pole paramaters: the linearized
% system dynamics are computed using the parameters of the physical
% cart-pole system, passed in the structure cp.
    [A,B] = get_linearized_dynamics(cp,sims);

%% Simulate pole dynamics:

% Compute Q and R weight matrices:
    [Q,R] = get_LQR_weights([1 1 1 1]);

% Design LQR control gain: we use the function MATLAB function dlqr that
% computes the optimal state feedback controller that minimizes the
% associated quadratic cost (type dlqr for more information). This
% controller is optimal only when the true system is linear and behaves
% according to [A,B]. Since the true system is non-linear, this controller
% K will always be suboptimal.
    K = -dlqr(A,B,Q,R);

% Simulate pole dynamics: we obtain the time evolution of the states and
% control input when applying the controller K to the real system.
    [x0,F0] = initial_conditions();
    [x_simu,F_simu] = simulate_cart_pole(f,K,x0,F0,sims);

%% Plot results:
    [fig_hdl,hdl_splot] = initialize_plots();
    plot_cart_pole(fig_hdl,hdl_splot,x_simu,F_simu,sims);
    
%% Cart-pole visual simulation:
    visual_simulation(x_simu,cp,sims);