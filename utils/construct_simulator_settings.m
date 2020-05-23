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
function sims = construct_simulator_settings()
%CONSTRUCT_SIMULATOR_SETTINGS
%   sims = CONSTRUCT_SIMULATOR_SETTINGS() returns the structure containing
%   the settings for the cart-pole system discretization and simulation
%   time.
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_CART_POLE CONSTRUCT_CART_POLE_WRONG

    % Discretization time-step (sec):
    sims.Ts = 0.001;
    
    % Simulation time horizon (sec):
    sims.tf = 15;

    % Get number of time steps:
    sims.Kf = sims.tf/sims.Ts;
    
    % Time vector:
    sims.t = 0:sims.Ts:sims.tf;

end