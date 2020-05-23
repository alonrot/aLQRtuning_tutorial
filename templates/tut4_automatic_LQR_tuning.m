%
% Tutorial: use Entropy Search to automatically tune the unknown function
%           J. This function can be constructed using
%           evaluation_functions/prepare_cart_pole_function_1D.m
%
%           Before completing this tutorial, one needs to finish:
%               tut1_cart_pole_equations.m
%               tut2_manual_tuning.m
%               tut3_evaluate_cost.m
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
% When running this block, all the variables in the workspace will be
% erased! Please, type help <function_name> to get more information about
% all the functions used in this tutorial
    close all; clear all; clc;
    
%% Prepare ES:
% Prepare input parameters for initializing Entropy Search, defined in the
% function initialize_ES(). As evaluation function use J(theta), defined in
% /evaluation_functions/prepare_cart_pole_function_1D.
    specs = initialize_ES();
    
%% Add datapoints:
    
    % Prior data:
    new_data    = get_existent_prior_data(specs);
    specs.GP    = add_initial_points(specs.GP,new_data);
    
%% Run ES:
    out = EntropySearch(specs);
    
%% Save output structure:
    save('ESout','out');