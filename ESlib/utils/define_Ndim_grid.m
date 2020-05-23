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
function X_eval = define_Ndim_grid(specs)

	% Define the D-dimensional grid, with resolution Nbel per dimension:
    
        % Create an Nbel vector per dimension:
        X_div = zeros(specs.Nbel,specs.D);
        for k = 1:specs.D
            X_div(:,k) = linspace(specs.xmin(k),specs.xmax(k),specs.Nbel)';
        end
    
        % Create the string to be evaluated:
        in_str  = 'ndgrid(';
        out_str = '[';
        all_in_a_row = '[';
        for k = 1:specs.D
            
            if k == specs.D
                in_str = [in_str 'X_div(:,' num2str(k) '));'];
                out_str = [out_str 'x' num2str(k) '] = '];
                all_in_a_row = [all_in_a_row 'x' num2str(k) '(:)]'];
            else
                in_str = [in_str 'X_div(:,' num2str(k) '),'];
                out_str = [out_str 'x' num2str(k) ','];
                all_in_a_row = [all_in_a_row 'x' num2str(k) '(:),'];
            end
            
        end
        create_grid = [out_str in_str];
        
        % Evaluate the string:
        eval(create_grid);
    
    % Put all the evaluation points in a row Nbel^D x D:
    X_eval = eval(all_in_a_row);

end