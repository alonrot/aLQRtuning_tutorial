% Copyright 2020 Max Planck Society. All rights reserved.
% 
% Author: Alonso Marco Valle (amarcovalle/alonrot) amarco(at)tuebingen.mpg.de
% Affiliation: Max Planck Institute for Intelligent Systems, Autonomous Motion
% Department / Intelligent Control Systems
% 
% This file is part of aLQRtuning_tutorial.
% 
% aLQRtuning_tutorial is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by the Free
% Software Foundation, either version 3 of the License, or (at your option) any
% later version.
% 
% aLQRtuning_tutorial is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
% FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
% details.
% 
% You should have received a copy of the GNU General Public License along with
% aLQRtuning_tutorial.  If not, see <http://www.gnu.org/licenses/>.
%
%
%% Initialization:
    close all; clear all; clc;
    
%% Prepare ES:
    specs   = initialize_ES();
    
%% Add datapoints:
    specs   = get_new_evaluation(specs);
    
%% Prepare GP:

    GP              = struct;
    GP.covfunc      = specs.covfunc;
    GP.covfunc_dx   = specs.covfunc_dx;
    GP.likfunc      = specs.likfunc;
    GP.hyp          = specs.hyp;
    GP.res          = 1;
    GP.deriv        = specs.with_deriv;
    GP.poly         = specs.poly;
    GP.log          = specs.log;
    GP.x            = specs.x;
    GP.y            = specs.y;

    % Needed for the function get_posterior():
    GP.z            = specs.z;
    GP.z_plot       = specs.z_plot;
    GP.mean         = specs.mean;
    
%% Initialize plots:

    global GP_plot;
    [fig_hdl,axes_hdl] = initialize_plots(GP,specs);
    
%% Simulate ES loop:

    numiter = 0;
    while numiter <= specs.MaxEval
        
        % Label the new measurement:
        new_meas_ind = round(rand);
        switch new_meas_ind
            case 0
                flag_new_meas = 'in_simu';
            case 1
                flag_new_meas = 'in_real';
        end
        
        % Get new data point:
        xp = specs.xmin + rand(1,1) * ( specs.xmax - specs.xmin );
        yp = specs.f(xp);
        
        % Add the new data point:
        GP = add_measurement2GP_sorted(GP,xp,yp,flag_new_meas);
        
        GP_plot.x = GP.x;
        GP_plot.y = GP.y;
        
        % Compute posterior:
        global delta_query;
        delta_query = 1;
        GP = get_posterior(GP);
        
        % Plot:
        update_plot(fig_hdl,axes_hdl);
        
        pause;
        
    end