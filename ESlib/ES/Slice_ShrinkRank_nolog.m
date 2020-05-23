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
function xx = Slice_ShrinkRank_nolog(xx,P,s0,transpose)
% multivariate slice sampling with shrinking rank covariance adaptation.
% Implemented after Thompson and Neal, November 2010
%
% The algorithm has three inputs:
%   xx   -- the last sample of the Markov Chain
%   P    -- function returning [P*,dP*/dx], the probability and its first derivative.
%   s0   -- initial proposal width, the one free parameter.
%           Set it to the width of the prior.
%
% Philipp Hennig, August 2011
if transpose; xx = xx'; end;

D    = size(xx,1);
[f,~]= P(xx');
logf = log(f);
logy = log(rand()) + logf;

theta= 0.95;

k = 0;
s = s0;
c = zeros(D,0);
J = [];

while true
    k      = k + 1;
    c(:,k) = ProjNullSpace(J,xx + s(k) * randn(D,1));
    sx     = 1 ./ (sum(1./s));
    mx     = sx * sum(bsxfun(@times,1./s,bsxfun(@minus,c,xx)),2);
    xk     = xx + ProjNullSpace(J,mx + sx .* randn(D,1));
    
    [fk,dfk] = P(xk');
    logfk  = log(fk);
    dlogfk = dfk ./ fk;
    if logfk > logy     % accept
        xx = xk';
        break
    else                % shrink
        g  = ProjNullSpace(J,dlogfk);
        if size(J,2) < D - 1 && g' * dlogfk > 0.5 * norm(g) * norm(dlogfk)
            J      = [J,  g ./ norm(g)];                        %#ok<AGROW>
            s(k+1) = s(k);
        else
            s(k+1) = theta .* s(k); % max(theta .* s(k),s0/1000);
            if s(k+1) < eps
                %keyboard;
                fprintf 'bug found: contracted down to zero step size, still not accepted.\n'
                if transpose; xx = xx'; return; else return;
                end
            end
        end
    end
end
end

function p = ProjNullSpace(J,v)
    if size(J,2) > 0 
        p = v - J * J' * v;
    else
        p = v;
    end
end