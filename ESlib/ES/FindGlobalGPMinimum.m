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
function [x,f] = FindGlobalGPMinimum(BestGuesses,GP,xmin,xmax)

[N,D] = size(BestGuesses);

X = zeros(N+10,D);
F = zeros(N+10,1);

M = feval(GP.mean{:},GP.hyp.mean,GP.x,'m(X)');

alpha = (GP.cK \ (GP.cK' \ (GP.y - M)));

for i = 1 : size(BestGuesses,1)
    [X(i,:),F(i)]   = fmincon(@(x)GPmeanderiv(x,GP,alpha),BestGuesses(i,:),[],[],[],[],xmin,xmax,[], ...
        optimset('MaxFunEvals',100,'TolX',eps,'Display','off','GradObj','on','Algorithm','interior-point'));
    
    %[X(i,:),F(i)]   = fminbnd(@(x)GPmeanderiv(x,GP,alpha),xmin,xmax, ...
    %    optimset('MaxFunEvals',100,'TolX',eps,'Display','off'));
end
for i = size(BestGuesses,1) + 1: size(BestGuesses,1) + 10
    start = xmin + (xmax-xmin) .* rand(1,D);
    [X(i,:),F(i)]   = fmincon(@(x)GPmeanderiv(x,GP,alpha),start,[],[],[],[],xmin,xmax,[], ...
        optimset('MaxFunEvals',100,'TolX',eps,'Display','off','GradObj','on','Algorithm','interior-point'));
end

[f,xi] = min(F);
x = X(xi,:);
end

function [f,df] = GPmeanderiv(x,GP,alpha)

kx  = feval(GP.covfunc{:},GP.hyp.cov,x,GP.x,'k(z,X)');
dkx = feval(GP.covfunc_dx{:},GP.hyp.cov,x,GP.x,'k(z,X)');

f  = kx * alpha;
df = zeros(size(x,2),1);
for d = 1:size(x,2)
    df(d) = dkx(:,:,d) * alpha;
end

end