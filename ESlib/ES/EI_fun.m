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
function fx = EI_fun(GP,xmin,xmax,invertsign)
% returns a function, which returns the expected improvement and its derivative,
% with respect to x. (EI has a surprisingly simple derivative).
%
% Philipp Hennig, August 2011

if nargin < 4
    invertsign = false;
end

if GP.deriv; 
    error 'this code is for GPs without derivative observations only.'; 
end

if ~isempty(GP.x)
    alpha = GP.cK \ (GP.cK' \ GP.y);
    m     = feval(GP.covfunc{:},GP.hyp.cov,GP.x,'k(X,X)') * alpha;
    fmin  = min(m);
else
    alpha = [];
    fmin  = +Inf;
end

fx    = @(x) EI_f(x,alpha,GP,fmin,xmin,xmax,invertsign);
end

function [f,df] = EI_f(x,alpha,GP,fm,xmin,xmax,invertsign)
if invertsign; sign = -1; else sign = 1; end
D = size(x,2);
if size(x,1) > 1; error 'only single inputs allowed'; end;
if any(x<xmin) || any(x>xmax)
    f  = 0;
    df = zeros(size(x,2),1);
%     fprintf 'projecting back. check that this works.'
%     df = (x < xmin) - (x > xmax); % project back towards good regions.
%     df = sign * df';
    return;
end

if isempty(GP.x)
    kxx  = feval(GP.covfunc{:},GP.hyp.cov,x);
    dkxx = feval(GP.covfunc_dx{:},GP.hyp.cov,x);
    dkxx = reshape(dkxx,[size(dkxx,2),size(dkxx,3)]);
    
    s    = sqrt(kxx);
    dsdx = 0.5 / s * dkxx';
    
    z    = fm / s;                          % assuming zero mean
    
    phi   = exp(-0.5 * z.^2) ./ sqrt(2*pi); % faster than Matlabs normpdf
    Phi   = 0.5 * erfc(-z ./ sqrt(2));      % faster than Matlabs normcdf

    
    f  = sign * (fm * Phi + s * phi);
    df = sign * (dsdx * phi);
    return;
end

% kernel values
kXx  = feval(GP.covfunc{:},GP.hyp.cov,GP.x,x,'k(X,z)');
kxx  = feval(GP.covfunc{:},GP.hyp.cov,x,'k(z,z)');

% derivatives of kernel values
dkxX = feval(GP.covfunc_dx{:},GP.hyp.cov,x,GP.x,'k(z,X)');
dkxx = feval(GP.covfunc_dx{:},GP.hyp.cov,x,'k(z,z)');

dkxX = reshape(dkxX,[size(dkxX,2),size(dkxX,3)]);
dkxx = reshape(dkxx,[size(dkxx,2),size(dkxx,3)]);

m    = kXx' * alpha;
dmdx = (dkxX' * alpha);
s    = sqrt(kxx - kXx' * (GP.cK \ (GP.cK' \ kXx)));
dsdx = zeros(D,1);
for d = 1:D
    dsdx(d) = 0.5 / s * (dkxx(1,d) - 2 * dkxX(:,d)' * (GP.cK \ (GP.cK' \ kXx)));
end

z    = (fm - m) ./ s;

phi   = exp(-0.5 * z.^2) ./ sqrt(2*pi); % faster than Matlabs normpdf
Phi   = 0.5 * erfc(-z ./ sqrt(2));      % faster than Matlabs normcdf

f  = sign * ((fm - m) * Phi + s * phi);
df = sign * (-dmdx    * Phi + dsdx * phi);

if sign * f < 0 
    f  = 0;
    df = zeros(size(x,2),1);
end
end