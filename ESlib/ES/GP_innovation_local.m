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
function innovation_function = GP_innovation_local(GP,zbel)
% what is the belief over zbel, und how does it change when we evaluate anywhere
% in zeval?
%
% Philipp Hennig & Christian Schuler, August 2011

if GP.poly >=0 || GP.deriv
    error 'current implementation is for standard GPs only. No fancy stuff yet'.
end

% amarcovalle: commented this, because it is available at the GP structure:
% K   = feval(GP.covfunc{:},GP.hyp.cov,GP.x) + exp(2*GP.hyp.lik) * eye(size(GP.x,1));
% cK  = chol(K);
cK  = GP.cK;

if isempty(GP.x)
    kbX = [];
else
    kbX = feval(GP.covfunc{:},GP.hyp.cov,zbel,GP.x,'k(z,X)');
end

innovation_function = @(x) efficient_innovation(x,cK,kbX,GP,zbel);

end

function [Lx,dLxdx] = efficient_innovation(x,cK,kbX,GP,zbel)
if size(x,1) > 1 
    error 'this function is for only single data-point evaluations.'
end

if isempty(GP.x)
    % kernel values
    kbx  = feval(GP.covfunc{:},GP.hyp.cov,zbel,x,'k(z,z)');
    kxx  = feval(GP.covfunc{:},GP.hyp.cov,x,'k(z,z)');
    
    % derivatives of kernel values
    dkxb = feval(GP.covfunc_dx{:},GP.hyp.cov,x,zbel,'k(z,z)');
    dkxx = feval(GP.covfunc_dx{:},GP.hyp.cov,x,'k(z,z)');
    
    dkxb = reshape(dkxb,[size(dkxb,2),size(dkxb,3)]);
    dkxx = reshape(dkxx,[size(dkxx,2),size(dkxx,3)]);
    
    % terms of the innovation
    sloc   = sqrt(kxx);
    proj   = kbx;
    
    dvloc  = dkxx;
    dproj  = dkxb;
    
    % innovation, and its derivative
    Lx     = proj ./ sloc;
    dLxdx  = dproj ./ sloc - 0.5 * bsxfun(@times,proj,dvloc) ./ sloc.^3;
    return
end

% kernel values
kbx  = feval(GP.covfunc{:},GP.hyp.cov,zbel,x,'k(z,z)');
kXx  = feval(GP.covfunc{:},GP.hyp.cov,GP.x,x,'k(X,z)');
kxx  = feval(GP.covfunc{:},GP.hyp.cov,x,'k(z,z)') + get_noise_cov(GP.hyp,x,'var_n(z)');

% derivatives of kernel values
dkxb = feval(GP.covfunc_dx{:},GP.hyp.cov,x,zbel,'k(z,z)');
dkxX = feval(GP.covfunc_dx{:},GP.hyp.cov,x,GP.x,'k(z,X)');
dkxx = feval(GP.covfunc_dx{:},GP.hyp.cov,x,'k(z,z)');

dkxb = reshape(dkxb,[size(dkxb,2),size(dkxb,3)]);
dkxX = reshape(dkxX,[size(dkxX,2),size(dkxX,3)]);
dkxx = reshape(dkxx,[size(dkxx,2),size(dkxx,3)]);

% terms of the innovation
sloc   = sqrt(kxx - kXx' * (cK \ (cK' \ kXx)));
proj   = kbx - kbX * (cK \ (cK' \ kXx));

dvloc  = (dkxx' - 2 * dkxX' * (cK \ (cK' \ kXx)))';
dproj  = dkxb - kbX * (cK \ (cK' \ dkxX));

% innovation, and its derivative
Lx     = proj ./ sloc;
dLxdx  = dproj ./ sloc - 0.5 * bsxfun(@times,proj,dvloc) ./ sloc.^3;
end