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
function noise_var = GP_noise_var(GP,y)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if ~isfield(GP,'log')
  GP.log = 0;
end

if GP.log
  noise_var = exp(2*GP.hyp.lik)*ones(numel(y),1) ./ exp(2*y);
else
  noise_var = exp(2*GP.hyp.lik)*ones(numel(y),1);
end

if GP.deriv
  noise_var = [noise_var; noise_var];
end


end

