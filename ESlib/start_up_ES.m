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
function start_up_ES()

    cur_path = [pwd '/ESlib'];

    run([cur_path '/gpml-matlab-v3.1-2010-09-27/startup.m']);
    addpath([cur_path '/utils/']);
    addpath([cur_path '/tests/']);
    addpath([cur_path '/priors/']);
    addpath([cur_path '/plotting/']);
    addpath([cur_path '/evaluation_functions/']);
    addpath([cur_path '/ES/tprod/']);
    addpath([cur_path '/ES/Logsumexp/']);
    addpath([cur_path '/ES']);
    addpath(cur_path);
    
    
end