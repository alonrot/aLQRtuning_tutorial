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
function K = get_der_block(x,xp,K0,ell)

    D = size(x,2);
    p = reshape(xp',[1,size(xp,2),size(xp,1)]);
    
    D_idj = bsxfun(@minus,x,p);
    T_idj = bsxfun(@rdivide,D_idj,reshape(ell.^2,[1,D,1]));
    K     = -bsxfun(@times,T_idj,reshape(K0,[size(K0,1),1,size(K0,2)]));
    K     = permute(K,[1,3,2]);

end