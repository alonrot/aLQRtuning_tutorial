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
function [Mx,Vx,Mz,Vz,Vxz] = GP_moments(GP,x,z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% 
%   Explanation completed by amarco:
%
%   x: training locations where new data is observed
%   y: values of the new observed data at the training locations x. It is obtained with
%   y_vector(GP).
%   z: test locations
%
%   Mx:  posterior mean at training locations x
%   Vx:  noise free self-variance gramm matrix for training locations x
%   Mz:  posterior mean at test locations z
%   Vz:  posterior noise free self-variance gramm matrix for test locations z
%   Vxz: posterior noise free cross-variance gramm matrix between training
%   locations x and test locations z

if ~isfield(GP,'poly')
  GP.poly = -1;
end

kx = k_matrix_full(GP,GP.x,x);
if GP.poly >= 0 && ~isempty(GP.x) % special case for no data. 
  GP.H    = poly_mat(GP.x,GP.poly,GP.deriv);
  cKH     = (GP.cK' \ GP.H');
  GP.cHKH = chol(cKH' * cKH + 1.0e-4 * eye(GP.poly + 1));
  Rx      = poly_mat(x,GP.poly,GP.deriv) - GP.H * (GP.cK \ (GP.cK' \ kx'));
else
  Rx = [];
end

Mx = GP_mean(GP,x,kx,Rx);
if nargout > 1
  Vx = GP_var(GP,x,kx,Rx);
end

if nargin > 2
  kz = k_matrix_full(GP,GP.x,z);
  if GP.poly >= 0 && ~isempty(GP.x)
    Rz = poly_mat(z,GP.poly,GP.deriv) - GP.H * (GP.cK \ (GP.cK' \ kz'));
  else
    Rz = [];
  end

  Mz = GP_mean(GP,z,kz,Rz);
  Vz = GP_var(GP,z,kz,Rz);  
  Vxz = GP_var(GP,x,kx,Rx,z,kz,Rz);
end

end

function M = GP_mean(GP,x,k,R)

  if isempty(GP.x)
    if GP.deriv
      M = zeros(size(x,1) .* 2,1);
    else
      M = zeros(size(x,1),1);
    end  
  else
    Ky = GP.cK \ (GP.cK' \ y_vector(GP));
    M = k * Ky;
    if ~isempty(R)
      M = M + R' * (GP.cHKH \ (GP.cHKH' \ (GP.H * Ky)));
    end
  end
 
end

function V = GP_var(GP,x,kx,Rx,z,kz,Rz)

  if nargin < 5
    z = x;
    kz = kx;
    Rz = Rx;
  end
  kk = k_matrix_full(GP,x,z);  
  
  if isempty(GP.x)
    V = kk;
  else
    V = kk - kz * (GP.cK \ (GP.cK' \ kx'));
    if ~isempty(Rx)
      V = V + Rz' * (GP.cHKH \ (GP.cHKH' \ Rx));
    end
  end
  
end


