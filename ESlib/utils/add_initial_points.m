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
function GP = add_initial_points(GP,new_data)

    if isempty(new_data)
        return;
    end

    % Evaluation locations:
    X_eval = new_data.X;
    
    % Values:
    Y_eval = new_data.Y;
    
    % This initial delta has to be already sorted: first simulation set,
    % and then real set. new_delta_flag = logical([0;1;0]); wouldn't be
    % accepted.
    new_delta_flag = new_data.new_delta_flag;

    % Update global delta:
    global delta;
    delta = logical([delta;new_delta_flag]);

    % Evaluations re-ordering: first those corresponding to simulations and
    % afterwards those corresponding to real experiments:
    X_eval_sorted = [X_eval(~delta,:);X_eval(delta,:)];
    Y_eval_sorted = [Y_eval(~delta);Y_eval(delta)];
        
    % Update GP structure:
    GP.x    = [GP.x;X_eval_sorted];
    GP.y    = [GP.y;Y_eval_sorted];
    
end

