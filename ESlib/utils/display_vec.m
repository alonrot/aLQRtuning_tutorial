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
function display_vec(vec,vec_name)

    D = length(vec);

    if D == 1
        disp(['    ' vec_name ' = ' num2str(vec)]);
        return;
    end

    str_cat = ['    ' vec_name ' = [ '];
    for k = 1:D

        if k < D
            str_cat = [str_cat num2str(vec(k)) ' , '];
        else
            str_cat = [str_cat num2str(vec(k))];
        end

    end

    str_cat = [str_cat ' ]'];
    
    disp(str_cat);
end