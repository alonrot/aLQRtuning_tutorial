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
function k_n = get_noise_cov(hyp,x,which_case)

    % Error checking:
    switch nargin
        case 1
            error('3 inputs are needed');
        case 2
            error('3 inputs are needed');
        case 3
            if ~ischar(which_case)
                error('Please, specify the type of input');
            end
    end
    
    % Compute Gramm Matrix:
    switch which_case
        
        case 'var_n(X)'
            
            k_n = get_kn(hyp,x);
             
        case 'var_n(z)'
            
            k_n = get_kn(hyp,x);
    end



end

function k_n = get_kn(hyp,x)

    % Get noise parameters:
    std_n = exp(2*hyp.lik);
    
    % Get noise covariance matrcies:
    k_n = std_n*eye(size(x,1));

end