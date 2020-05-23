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
function [Q,R] = get_LQR_weights(parameters)
%GET_LQR_WEIGHTS
%   [Q,R] = GET_LQR_WEIGHTS(parameters) places the input vector
%   parameters in the diagonal of matrix Q. Matrix R is set to the
%   identity without loss of generality. The rest of elements of Q are set
%   to zero.
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
    
    % Matrix Q:
    Q = get_Q(parameters);
    
    % Matrix R is set identity without loss of generality:
    R = 1;

end

function Q = get_Q(par)

    if ~isempty(find(par<=0))
        error('Entries of matrix Q must be positive non-zero');
    end

    Q = diag(par);

end