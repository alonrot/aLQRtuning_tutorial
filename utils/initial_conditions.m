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
function [x0,F0] = initial_conditions()
%INITIAL_CONDITIONS
%   [x0,F0] = INITIAL_CONDITIONS() choose the initial conditions for the
%   cart-pole system
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also GET_STATE_VECTOR GET_INDIVIDUAL_STATES

    % Initial angle (deg):
    phi0 = 15;
    phi0 = phi0/180*pi;
    
    % Initial angle (deg/s):
    phid0 = 0;
    phid0 = phid0/180*pi;
    
    % Initial cart position:
    s0  = -1;
    sd0 = 1;
    
    % Initial state:
    x0 = [phi0;phid0;s0;sd0];   
    
    % Initial force:
    F0 = 0;

end