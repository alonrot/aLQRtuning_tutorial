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
function A = meanConst(hyp, x, which_case)

    % Error checking:
    if nargin == 1
        
        A = '1'; 
        return;
        
    elseif nargin == 2
        
        error('Please, specify the type of input');
        
    elseif ~ischar(which_case)
        
        error('Please, specify the type of input');
            
    end 
    
    % Compute mean:
    switch which_case
        
        case 'm(X)'
            
            A = get_m(hyp,x);
             
        case 'm(z)'
            
            A = get_m(hyp,x);
    
    end
    
    
end

function A = get_m(hyp,x)

    c = hyp;
    A = c*ones(size(x,1),1);
    
end
