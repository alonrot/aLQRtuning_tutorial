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
function get_next_cp_state = construct_nonlinear_dynamics(cp,sims)
%CONSTRUCT_NONLINEAR_DYNAMICS
%   CONSTRUCT_NONLINEAR_DYNAMICS(cp,sims) returns a handle to a
%   the nonlinear function f that maps the current state x(k) to the next
%   state x(k+1) according to the physical laws that drive the dynamics of
%   a cart-pole system.
%
%       x(k+1) = f(x,u)
%
% The returned function f(x,u) takes as input arguments:
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
% See also GET_LINEARIZED_DYNAMICS CONSTRUCT_LINEARIZED_DYNAMICS

% Obtain f(x,u) as a handle:
    get_next_cp_state = @(x,u) get_next_state(x,u,cp,sims);

end

function x_next = get_next_state(x,F,cp,sims)

% Current states:
[phi,phid,s,sd] = get_individual_states(x);

% Integrate pole states:
phid_next   = sims.Ts * ( cp.g/cp.l * sin(phi) - 1/cp.l * cos(phi) * F/cp.mc - cp.D * phid ) + phid;
phi_next    = sims.Ts * phid + phi;

% Integrate cart states:
sd_next = sims.Ts * F/cp.mc + sd;
s_next  = sims.Ts * sd + s;

% Map back to state vector:
x_next = get_state_vector(phi_next,phid_next,s_next,sd_next);

end