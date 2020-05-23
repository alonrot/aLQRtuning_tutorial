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
function [x_simu,F_simu] = simulate_cart_pole(f,K,x0,F0,sims)
%SIMULATE_CART_POLE
%   [x_simu,F_simu] = SIMULATE_CART_POLE(f,K,x0,F0,sims) returns the
%   time evolution of the system states when applying the controller K to
%   stabilize it, starting from the initial conditions [x0,F0].
%
%   As input arguments it takes:
%
%       f       transition function f(x,u) of the real system, such that 
%               x(k+1) = f(x(k),u(k))
%
%       K       state feedback controller, as a static gain. When passing K
%               = [], no controller is applied, and thus, the system will
%               evolve freely.
%
%       [x0,F0] initial conditions for the states and the control input
%
%       sims    simulation settings
%
%   As output arguments:
%
%       x_simu  matrix whose rows are each state's time signals spanned in
%       columns
%       F_simu  matrix whose rows are each control input's time signal
%       spanned in columns.
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_NONLINEAR_DYNAMICS INITIAL_CONDITIONS CONSTRUCT_SIMULATOR_SETTINGS

    % Get states dimension:
    nx = size(x0,1);
    
    % Get input dimension:
    nu = size(F0,1);
    
    % Get state feedback controller:
    K = construct_state_feedback_controller(K);
    
    % Define simulation signals:
    x_simu = zeros(nx,sims.Kf);
    F_simu = zeros(nu,sims.Kf);
    
    % Initialize simulation:
    x_simu(:,1) = x0;
    F_simu(:,1) = F0;
    
    % Simulation loop:
    for k = 1:sims.Kf
       
        F_simu(:,k+1) = K(x_simu(:,k));
        x_simu(:,k+1) = f(x_simu(:,k),F_simu(:,k));
        
    end

end

function K = construct_state_feedback_controller(Kx)

    if isempty(Kx)
        K = @(x) 0.0;
    else
        K = @(x) Kx * x;
    end

end