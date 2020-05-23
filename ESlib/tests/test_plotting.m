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

%% Get data:

% Load an existent output structure from an ES run:
load('ESout');

% Plot from iteration number which_iter:
which_iter = 1;

% Get the part needed for plotting, at the last iteration:
global GP_plot; % This variable is needed inside update_plot()
GP_plot = out.GP_plots{which_iter};

%% Plot stuff:

% Initialize subplots structure:
GP = out.GPs{which_iter};
[fig_hdl,axes_hdl] = initialize_plots(GP,out.specs);

% Plot posterior GP, p_min, and E[dH]:
update_plot(fig_hdl,axes_hdl);

% Plot the location of the current global minimum:
update_plot(fig_hdl,axes_hdl,'only_global_minimum');