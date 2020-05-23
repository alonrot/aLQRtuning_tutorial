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
function cp = construct_cart_pole()
%CONSTRUCT_CART_POLE
%   cp = CONSTRUCT_CART_POLE() returns a structure containing physical
%   parameters of the cart-pole system, defined within this function.
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_CART_POLE_WRONG

    % Cart-pole structre: the pole is modeled as the typical physical
    % inverted pendulum, considering a point mass at the tip.
    cp = struct();
    
    % Pole mass:
    cp.mp = 1;
    
    % Distance to the center of mass:
    cp.l = 1;
    
    % Inertia:
    cp.J = cp.mp * cp.l^2;
    
    % Friction coefficient:
    cp.D = 0.001;
    
    % Cart mass:
    cp.mc = 1;
    
    % Gravity:
    cp.g = 9.81;

end