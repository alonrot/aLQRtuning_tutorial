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
function [A,B] = get_linearized_dynamics(cp,sims)
%GET_LINEARIZED_DYNAMICS
%   GET_LINEARIZED_DYNAMICS(cp,sims) returns the linearized matrices [A,B]
%   of the cart-pole system, defined using the cart-pole parameters,
%   defined in cp. The linearized dynamics are governed by:
%
%       x(k+1) = Ax(k) + BF(k)
%
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_LINEARIZED_DYNAMICS

    % Model entries:
    a1 = cp.mp * cp.g * cp.l / cp.J;
    a2 = cp.mp * cp.l / cp.J;
    a3 = cp.D / cp.J;
    Ts = sims.Ts;

    % Discrete-time linearized model:
    A = [1          Ts          0 0;...
         a1*Ts      1-a3*Ts     0 0;...
         0          0           1 Ts;....
         0          0           0 1];
    B = [0; -a2*Ts; 0; Ts];

end