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
% function [Xs,Xr] = get_sets(x)
% Extract the set of locations corresponding to evaluations in the
% simulator (Xs) and in the real system (Xr):
function [Xs,Xr,Ys,Yr] = get_sets(x,varargin)

    global delta;
    
    Ns = length(find(~delta));
    Xs = x(1:Ns,:);
    Xr = x(Ns+1:end,:);
    
    if nargin == 2
        y = varargin{1};
        Ys = y(1:Ns,:);
        Yr = y(Ns+1:end,:);
    else
        Ys = []; Yr = [];
    end

end