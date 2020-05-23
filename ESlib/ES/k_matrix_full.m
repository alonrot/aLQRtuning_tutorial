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
function K = k_matrix_full(GP,x,z)
% Kernel gram matrix, with outputs for the derivatives
switch nargin
  case 2
    N = size(x,1);
    if ~GP.deriv
      K = feval(GP.covfunc{:},GP.hyp.cov,x); % + exp(2*GP.hyp.lik)*eye(N);
    else
      K = feval(GP.covfunc{:},GP.hyp.cov,x);
      K_dx = feval(GP.covfunc_dx{:},GP.hyp.cov,x);
      K_dx2 = feval(GP.covfunc_dxdz{:},GP.hyp.cov,x);
      K = [K    -K_dx;
           K_dx K_dx2]'; % + exp(2*GP.hyp.lik)*eye(2*N);
    end
  case 3
    if isempty(z) || isempty(x)
      K = zeros(size(z,1),size(x,1));
    elseif ~GP.deriv
      K = (feval(GP.covfunc{:},GP.hyp.cov,x,z))';
    else
      K = feval(GP.covfunc{:},GP.hyp.cov,x,z);
      K_dx = feval(GP.covfunc_dx{:},GP.hyp.cov,x,z);
      K_dx2 = feval(GP.covfunc_dxdz{:},GP.hyp.cov,x,z);
      K = [K    -K_dx;  % why the minus? This is a special aspect of SE? % amarco: I agree. In Rasmussen that minus is not there.
           K_dx K_dx2]'; % why no transpose on the K? So size(K(x,y)) = [size(y),size(x)] ? That seems odd.
    end
end
end

