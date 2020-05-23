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
function J = construct_unknown_cost_function(A,B,f,x0,F0,sims,ew)
%CONSTRUCT_UNKNOWN_COST_FUNCTION
%   J = CONSTRUCT_UNKNOWN_COST_FUNCTION(A,B,f,x0,F0,sims,ew) returns a
%   handle to the unknown cost function that maps each set of deisgn
%   weights to a cost function value. Given a set of design weights, and
%   the linearized system, an LQR controller is computed. This is used in
%   to stabilize the real cart-pole non-linear system; a simulation is made
%   accordingly. Then, the system states and control input are used to
%   compute the associated quadratic cost, by calling the function
%   get_lqr_cost().
%
%   The cost function is:
%
%       J = J(dw)
%
%   The design weights dw can be computed using the function
%   GET_LQR_WEIGHTS.
%
%   In order to construct J, we need as input arguments:
%       [A,B]   linear system
%       f       function f(x,u) that represents the true non-linear
%               dynamics of the cart-pole system
%       [x0,F0] initial conditions
%       sims    simulation settings
%       ew      empirical weights, used to construct the quadratic cost
%       function, by calling get_lqr_cost()
%
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_CART_POLE CONSTRUCT_CART_POLE_WRONG DLQR
% SIMULATE_CART_POLE GET_LQR_COST GET_LQR_WEIGHTS

    J = @(dw) evaluate_cost(A,B,f,x0,F0,sims,ew,dw);

end

function J_val = evaluate_cost(A,B,f,x0,F0,sims,ew,dw)

% Design LQR control gain:
    K = -dlqr(A,B,dw.Q,dw.R);

% Simulate pole dynamics:
    [x_simu,F_simu] = simulate_cart_pole(f,K,x0,F0,sims);
    
% Compute LQR cost:
    J_val = get_lqr_cost(ew,x_simu,F_simu);

end