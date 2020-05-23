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
function new_cost = input_function(F_value)
    
    display(' ');
    display('Perform a new experiment with the robot and press any key to continue.');
    pause();
% % % %     valid_cost = 0;
% % % %     while valid_cost ~= 1
% % % %         load('./controller_update/new_cost.mat','J_val');
% % % %         display(strcat(['New cost value: ' num2str(J_val)]));
% % % %         valid_cost = input('Please, enter 1 if you want to feed ES with this cost value or anything else to reload it: ');
% % % %         if isempty(valid_cost)
% % % %             valid_cost = 0;
% % % %         end
% % % %     end
% % % 
% % %     
    
	new_cost = F_value;

end