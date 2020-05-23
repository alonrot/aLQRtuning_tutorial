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
function [phi,phid,s,sd] = get_individual_states(x)
%GET_INDIVIDUAL_STATES
%   [phi,phid,s,sd] = GET_INDIVIDUAL_STATES(x) maps the state vector x,
%   passed as input argument, to its individual components.
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also GET_STATE_VECTOR

    % Current states:
    phi     = x(1,:);
    phid    = x(2,:);
    s       = x(3,:);
    sd      = x(4,:);

end