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
function specs = load_default_ES_options(specs)
%LOAD_DEFAULT_ES_PARAMETERS
%   specs = LOAD_DEFAULT_ES_PARAMETERS(specs) %
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle

    % Constraints defining search space (DIM):
        if ~isfield(specs,'xmin') || ~isfield(specs,'xmax')
%             specs.xmin          = [-10 -10];
%             specs.xmax          = [10 10];
            specs.xmin          = -1;
            specs.xmax          = 2;
        end

    % Maximum number of evaluations allowed:
        if ~isfield(specs,'MaxEval')
            specs.MaxEval      = 20;    % Horizon (number of evaluations allowed)
        end
        
    % Number of dimensions:
        if ~isfield(specs,'D')
            specs.D             = size(specs.xmin,2);
        end

    % Number of representer points:
        if ~isfield(specs,'Nb')
            specs.Nb        = 50;
        end
        
    % Random set of representer points:
        if ~isfield(specs,'z')
            specs.z         = rand(specs.Nb,specs.D);
        end
        
    % Test points for obtaining the posterior belief:
        if ~isfield(specs,'Nbel')
            specs.Nbel      = 200;
        end
        
    % Random set of representer points:
        if ~isfield(specs,'z_plot')
            specs.z_plot    = define_Ndim_grid(specs);
        end
        
    % Enumerate the subplots:
        if ~isfield(specs,'subplot_struct') || ~isfield(specs,'subplot_num')
            specs.subplot_struct    = [3 1];
            specs.subplot_num.gp    = 1;
            specs.subplot_num.pmin  = 2;
            specs.subplot_num.EdH   = 3;
        end
        
    % Type of mean function:
        if ~isfield(specs,'poly')
            specs.poly = -1;
        end
        
    % With observation derivatives?:
        if ~isfield(specs,'with_deriv')
            specs.with_deriv = 0;
        end
        
	% Logarithmic transformed observations?:
        if ~isfield(specs,'log')
            specs.log        = 0;
        end
        
    % Prepare GP:
        if ~isfield(specs,'GP')
            specs.GP        = initialize_GP(specs);
        end
        
	% Function:
        if ~isfield(specs,'f')
            specs.f         = construct_holder(specs.GP.hyp);
        end
        
	% Get underlying function:
        if ~isfield(specs,'f_true')
            specs.f_true    = zeros(specs.Nbel,specs.D);
        end
        
	% Have into account prior data?
        if ~isfield(specs,'use_prior_data')
            specs.use_prior_data = 0;
        end
        
	% Ask the user to continue, at the end of each iteration:
        if ~isfield(specs,'ask_user')
            specs.ask_user       = 0;
        end
    
    % Should the hyperparameters be learned, too?
        if ~isfield(specs,'LearnHypers')
            specs.LearnHypers  = false;
        end
        
        if ~isfield(specs,'HyperPrior')
            specs.HyperPrior   = @SEGammaHyperPosterior_no_mean;
        end

    % Number of samples in entropy prediction:
        if ~isfield(specs,'T')
            specs.T         = 200;
        end
        
    % Variable W needed for Riemann integration when we compute the
    % expected change in the GP mean, for which we need an unknown new
    % measurement y, over which we marginalize:
        if ~isfield(specs,'W')
            W = sqrt(2) * erfinv(2*linspace(0,1,specs.T+2)-1);
            specs.W = W(2:end-1);
        end
        
    % Loss function:
        if ~isfield(specs,'LossFunc')
            specs.LossFunc  = {@LogLoss};
        end

    % Utility function for sampling the representers:
        if ~isfield(specs,'PropFunc')
            specs.PropFunc  = {@EI_fun};
        end
        
    % Save data at each iteration?
        if ~isfield(specs,'save_data')
            specs.save_data = 1;
        end
        
	% Numer of GP samples:
        if ~isfield(specs,'Ns')
            specs.Ns        = 3;
        end
        
    % Random number for sampling from GP:
        if ~isfield(specs,'Ws')
            specs.Ws        = randn(specs.Nbel,specs.Ns);
        end

end