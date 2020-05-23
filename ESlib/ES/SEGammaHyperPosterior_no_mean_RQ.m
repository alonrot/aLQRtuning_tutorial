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
function [logpost,dlogpost] = SEGammaHyperPosterior_no_mean_RQ(hyp,x,y,covfunc,meanfunc,likfunc)

if nargin > 1 % data available: evaluate log likelihood
    [lik,dlik] = gp(hyp,@infExact,meanfunc,covfunc,likfunc,x,y);
else
    lik = 0; dlik.cov = []; dlik.lik = [];
end

% modif: amarcovalle
% evaluate log prior
log_ell         = hyp.cov(1:end-2);
log_the2        = 2 * hyp.cov(end-1);
log_alph        = hyp.cov(end);
log_sig         = hyp.lik;

N = size(hyp.cov,1) - 2;

    % Modifying the Gamma Prior distribution over the lengthscales, as
    % suggested by Philipp:
%     a = 0.05 * ones(N,1); % hyperhyp.cov.a_ell;
%     b = 100   * ones(N,1); % hyperhyp.cov.b_ell;
%     c = 0.01; % hyperhyp.cov.a_theta; %shape (signal variance)
%     d = 200;   % hyperhyp.cov.b_theta; %scale (signal variance)
%     s = 0.005; % hyperhyp.lik.s; %shape (noise stddev)
%     m = 10; % hyperhyp.lik.m; %scale(noise stddev)
    a = 50 * ones(N,1); % hyperhyp.cov.a_ell; %shape (length scales)
    b = 10 * ones(N,1); % hyperhyp.cov.b_ell; %scale (length scales)
    
    c = 100; % hyperhyp.cov.a_theta; %shape (signal variance)
    d = 4e-3;   % hyperhyp.cov.b_theta; %scale (signal variance)
    
    p = 10; % hyperhyp.cov.p; %shape (alpha)
    q = 1e-3; % hyperhyp.cov.q; %scale(alpha)
    
    s = 2; % hyperhyp.lik.s; %shape (noise stddev)
    m = 6e-3; % hyperhyp.lik.m; %scale(noise stddev)
    

    
% endmodif: amarcovalle

% hyperprior
logprior = sum(log_ell .* a - exp(log_ell) ./ b) ... % gamma prior over length scales
    + log_the2 .* c - exp(log_the2) ./ d ...     % gamma prior over signal variance
    + log_alph .* p - exp(log_alph) ./ q ...     % gamma prior over alpha
    + log_sig .* s - exp(log_sig) ./ m;         % gamma prior over noise stddev

% gradient
dlogprior = [a - exp(log_ell) ./ b; ...
    2 * c - 2 * exp(log_the2) ./ d; ...
    p - exp(log_alph) ./ q;...
    s - exp(log_sig) ./ m];

% log posterior is log lik + log prior
logpost         = lik       - logprior;
dlogpost.cov    = dlik.cov  - dlogprior(1:end-1);
dlogpost.lik    = dlik.lik  - dlogprior(end);


%% unit test:
% N = 3;
% x.cov = randn(N+1,1);
% x.lik = randn();
% e = 1.0e-6; dt = zeros(N+1,1);
% [f,df] = SEGammaPrior(x); 
% for i = 1:N+1
%     y = x; y.cov(i) = y.cov(i) + e; f1 = SEGammaPrior(y);
%     y = x; y.cov(i) = y.cov(i) - e; f2 = SEGammaPrior(y);
%     dt(i) = (f1 - f2) / (2*e);
% end
% y = x; y.lik = y.lik + e; f1 = SEGammaPrior(y);
% y = x; y.lik = y.lik - e; f2 = SEGammaPrior(y);
% dt(N+2) = (f1 - f2) / (2*e);
% [df dt]