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
function K = kerSEard(hyp, x, z, which_case)

    flag_xeqz = 0;

    % Error checking:
    if nargin == 1
        
        K = '(D+1)+(D+1)'; 
        return;
        
    elseif nargin == 2
        
        error('Please, specify the type of input');
        
    elseif nargin == 3
        
        if ~ischar(z)
            error('Please, specify the type of input');
        else
            which_case = z;
        end
        
        flag_xeqz = 1;
        
    else
        
        if ~ischar(which_case)
            error('Please, specify the type of input');
        end
            
    end 
    
    % Compute Gramm Matrix:
    switch which_case
        
        case 'k(X,X)'
            
            K = get_kXX(hyp,x);
             
        case 'k(z,z)'
            
            if flag_xeqz
                K = get_kzz(hyp,x);
            else
                K = get_kzz(hyp,x,z);
            end
            
            
        case 'k(X,z)'
            
            K = get_kzX(hyp,x,z)';
            
        case 'k(z,X)'
            
            % In this case, the order of the inputs must change:
            x_in = z;
            z_in = x;
            K = get_kzX(hyp,x_in,z_in);
    
    end
        

end

% No need ofa delta query in this function, because they are all
% measurements:
function K = get_kXX(hyp,x)

    [ell,sf2] = get_hyp(hyp,x);

	K = sf2*exp(-0.5*sq_dist(diag(1./ell)*x')); % k(X,X)

end

function K = get_kzz(hyp,x,z)

    [ell,sf2] = get_hyp(hyp,x);

    if nargin <= 2
        
        K = sf2*exp(-0.5*sq_dist(diag(1./ell)*x'));
        
    else
        
        K = sf2*exp(-0.5*sq_dist(diag(1./ell)*x',diag(1./ell)*z'));
    end

end

function K = get_kzX(hyp,x,z)

    [ell,sf2] = get_hyp(hyp,x);

	K = sf2*exp(-0.5*sq_dist(diag(1./ell)*z',diag(1./ell)*x')); % k(zbel,X)   

end

function [ell,sf2] = get_hyp(hyp,x)

    [~,D] = size(x);
    ell = exp(hyp(1:D));                               % characteristic length scale
    sf2 = exp(2*hyp(D+1));                                         % signal variance

end