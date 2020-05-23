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
function newdata = get_existent_prior_data(specs)

    % Select whether we eant to use prior existent data or not:
    use_prior_data = specs.use_prior_data;
    
    if ~use_prior_data
        newdata = [];
        return;
    end

    % Create data structure:
    newdata = struct;

    % Prior data:
    newdata.X              = [-9;-4;2];
    newdata.Y              = zeros(size(newdata.X,1),1);
    newdata.new_delta_flag = logical([0;0;1]);
    
    % We get the Y values from the true function, typically unknown:
    newdata = evaluate_from_true_function(newdata,specs.f);

end

function newdata = evaluate_from_true_function(newdata,J)

    % Labels:
    in_simu = 0; in_real = 1;
    
    % We get the Y values from the true function, typically unknown:
    for k = 1:size(newdata.X,1)
        
        switch newdata.new_delta_flag(k)
            case in_simu
                flag_new_meas = 'in_simu';
            case in_real
                flag_new_meas = 'in_real';
        end
        
        newdata.Y(k) = J(newdata.X(k,:),flag_new_meas);
        
    end

end
