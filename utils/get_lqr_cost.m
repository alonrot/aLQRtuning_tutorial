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
function J_val = get_lqr_cost(ew,x,u)
%GET_LQR_COST
%   J_val = GET_LQR_COST(ew,x,u) computes the cost associated to the
%   deviation of the signals x, u from the zero equilibrium. The cost has
%   the form
%
%         J = Sum {x'Qx + u'Ru}
%
%   where x is a matrix whose rows are each state's time signals spanned in
%   columns, and same for u. Matrices Q and R are elements from the
%   structure ew (empirical weights).
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%

    % Get dimensions:
    nx = size(x,1); % Number of states
    nu = size(u,1); % Number of control inputs
    n  = size(x,2); % Number of time steps

    % Reserve memory:
    J           = zeros(nx+nu,n); 
    J_weighted  = zeros(nx+nu,n);
    
    % Initialize costs:
    J(:,1)            = [x(:,1).^2;u(:,1).^2];
    J_weighted(:,1)   = [diag(ew.Q);diag(ew.R)] .* J(:,1);
    
    % Compute accumulated cost recursively:
    for k = 1:n-1
        J(:,k+1) = (J(:,k)*k + [x(:,k+1).^2;u(1,k+1).^2])./(k+1);
        J_weighted(:,k+1) = [diag(ew.Q);diag(ew.R)] .* J(:,k+1);
    end

    % Total accumulated and weighted cost per variable:
    J_total = sum(J_weighted,1);

    % Final cost:
    J_val = J_total(end);
    
    % % 25 times more slowly way of computing it:
    % J_slow = trace(x' * ew.Q * x + F' * ew.R * F) / n;

end