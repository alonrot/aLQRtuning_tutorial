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
function J = construct_cart_pole_function_1D()

% Construct cart-pole system:
    cp = construct_cart_pole();
    
% Construct cart-pole system with some wrong parameters:
    cpw = construct_cart_pole_wrong(cp);

% Construct simulation settings:
    sims = construct_simulator_settings();

% Get non-linear cart-pole dynamics:
    f = construct_nonlinear_dynamics(cp,sims);

% Get linearized dynamics with the wrong pole paramaters:
    [A,B] = get_linearized_dynamics(cpw,sims);

% Empirical weights:
    ew = struct(); 
    [ew.Q,ew.R] = get_LQR_weights([100 1 100 1]);
    
% Initial conditions:
    [x0,F0] = initial_conditions();
    
% Get J_empirical:
    J_empirical = construct_unknown_cost_function(A,B,f,x0,F0,sims,ew);
    
% Wrap it into one dimension:
    J = wrapper_for_one_dimension(J_empirical);

end

function J = wrapper_for_one_dimension(J_empirical)

    J = @(theta) J_empirical(design_LQR_weights(theta));

end

function dw = design_LQR_weights(theta)

    % Design weights: parametrize the second element of the diagonal of Q:
    dw = struct();
    [dw.Q,dw.R] = get_LQR_weights([10^theta 1 100 1]);

end