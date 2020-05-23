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
function specs = initialize_ES()
    
    % Constraints defining search space (DIM):
        specs.xmin          = [0 0];
        specs.xmax          = [1 2];

    % Maximum number of evaluations allowed:
        specs.MaxEval = 15;    % Horizon (number of evaluations allowed)
        
    % Number of dimensions:
        specs.D = length(specs.xmin);

    % Number of representer points (typically 50 for 1D):
        specs.Nb = 75;
        
	% Have into account prior data?
        specs.use_prior_data = 0;
        
	% Ask the user to continue, at the end of each iteration:
        specs.ask_user = 0;
        
    % Load the rest of default options:
        specs = load_default_ES_options(specs);
        
    % Prepare GP:
        specs.GP = initialize_GP(specs);
        
	% Function:
        specs.f = construct_cart_pole_function_1D();

end
