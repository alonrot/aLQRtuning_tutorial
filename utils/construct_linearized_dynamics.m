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
function get_next_cp_state = construct_linearized_dynamics(cp,sims)
%CONSTRUCT_LINEARIZED_DYNAMICS
%   CONSTRUCT_LINEARIZED_DYNAMICS(cp,sims) returns a handle to a
%   the linearized system [A,B] that maps the current state x(k) to the
%   next state x(k+1) according to the physical laws that drive the
%   dynamics of a cart-pole system, linearized around an equilibrium
%   defined by x = [0 0 0 0]^T, F = 0. Internally, it calls to the function
%   get_linearized_dynamics(cp,sims), which computes the linearized system
%   matrices [A,B] taking into account the pole's phisical parameters
%   (described in cp), and the simulation parameters (described in sims)
%
%       x(k+1) = Ax(k) + BF(k)
%
% The returned function fl(x,u) takes as input arguments:
%       Column vector x(k), that contains the system state at time k, with
%       states x(k) = [phi(k) phid(k) s(k) sd(k)]^T, where phi is the pole
%       angle (rad), measured with respect to the vertical axis, phid the
%       pole's angluar velocity (rad/s), s is the position of the cart (m),
%       and sd is the linear velocity of the cart (m/s).
%
% As output arguments, it returns the updated vector x(k+1).
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also GET_LINEARIZED_DYNAMICS CONSTRUCT_NONLINEAR_DYNAMICS

    get_next_cp_state = @(x,u) get_next_state(x,u,cp,sims);

end

function x_next = get_next_state(x,F,cp,sims)
%GET_NEXT_STATE   Compute the next state of the cart-pole system.
%   GET_NEXT_STATE(x,u,cp)

% Linearized dynamics:
[A,B] = get_linearized_dynamics(cp,sims);

% Get next state:
x_next = A * x + B * F/cp.mc;

end