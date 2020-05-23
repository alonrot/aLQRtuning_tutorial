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
function GP = initialize_GP(specs)

    % >>>>>> Relevant variables <<<< %

    % Create GP structure:
        GP = struct;
        
    % Covariance function:
        GP.covfunc       = {@kerSEard};
        GP.covfunc_dx    = {@kerSEard_dx};
        
        % Hyperparameters (DIM):
            % l_s = [2 2];
            ls = 0.5;
            ls = reshape(ls,[length(ls),1]);
            std = 400;
        
        % Cov. function hyperparameters (in this order):
            hyp.cov = log([ls;std]);
    
    % Likelihood function:
        GP.likfunc = @likGauss;
        std_n = 0.01;
        hyp.lik = log(std_n);
        
    % Mean function:
        GP.mean      = {@meanConst};
        hyp.mean     = 0;
        
    % Empty set of initial evaluations:
        GP.x          = [];
        GP.y          = [];
        
    % >>>>>> Not so relevant variables <<<< %
        
    % Include the hyperparameters:
        GP.hyp     	= hyp;
        
    % Number of dimensions:
        GP.D             = specs.D;
        
	% Add a vector of test locations to plot in:
        GP.z_plot   = specs.z_plot;
        
    % Location of representer points:
        GP.z     	= specs.z;
        
    % Other variables needed for ES:
        GP.deriv    = specs.with_deriv;
        GP.res    	= 1;
        GP.poly  	= specs.poly;
        GP.log     	= specs.log;
        
end

