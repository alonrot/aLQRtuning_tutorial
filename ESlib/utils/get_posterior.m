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
function GP = get_posterior(GP)
    
    display('    Computing GP posterior...');
    [mpost,vpost,stdpo,K,cK] = conditioning(GP);
    
    if ~isempty(find(stdpo<0))
        display('Some variances are negative. A fix might be needed');
    elseif ~isempty(find(isinf(stdpo)))
        display('Some variances are Inf');
        % Fix for plotting purposes:
        display('The Inf values are changed to non-Inf values, just for plotting purposes');
        stdpo(isinf(stdpo)) = max(stdpo(~isinf(stdpo)))*10;
    end
    
    if size(GP.z,2) == 1
        [z_sorted,z_i]= sort(GP.z);
        m_sorted = mpost(z_i);
        std_sorted = stdpo(z_i);
    else
        z_sorted    = GP.z;
        m_sorted    = mpost;
        std_sorted  = stdpo;
    end
    
    % Update GP:
    GP.z        = z_sorted;
    GP.mpost    = m_sorted;
    GP.vpost    = vpost; % No need to be sorted (...)
    GP.stdpost  = std_sorted;
    GP.K        = K;
    GP.cK       = cK;
end

function [mpost,vpost,stdpost,K,cK] = conditioning(GP)

    % Special case for no data:
    if isempty(GP.x)
        
        kzz = get_Gramm_Matrix(GP);
        mpost = feval(GP.mean{:},GP.hyp.mean,GP.z,'m(z)');
        vpost = kzz;
        stdpost = sqrt(diag(vpost));
        
        % ES meas. Gramm Matrix with noise:
        K   = [];
        cK  = [];

    else

        % Build up Gramm matrices:
        [kzz,kXX,kzX] = get_Gramm_Matrix(GP);

        % Prior mean:
        M = feval(GP.mean{:},GP.hyp.mean,GP.x,'m(X)');
        m = feval(GP.mean{:},GP.hyp.mean,GP.z,'m(z)');

        % Get noise covariance function:
        kXX_n = get_noise_cov(GP.hyp,GP.x,'var_n(X)');

        % Posterior mean and covariance:
        G_ = kXX + kXX_n;
        R_ = chol(chol_fix(G_));
        A_ = kzX / R_;
        mpost = m + A_ * (R_' \ (GP.y-M));
        vpost = kzz - A_*A_';
        stdpost = sqrt(diag(vpost));
        
        % ES meas. Gramm Matrix with noise:
        K   = G_;
        cK  = R_;
    
    end

end

function [kzz,kXX,kzX] = get_Gramm_Matrix(GP)

    % Build up Gramm matrices:
    if nargout == 1
        kzz     = feval(GP.covfunc{:},GP.hyp.cov,GP.z,'k(z,z)');
        return;
    else
        kXX     = feval(GP.covfunc{:},GP.hyp.cov,GP.x,'k(X,X)');
        kzz     = feval(GP.covfunc{:},GP.hyp.cov,GP.z,'k(z,z)');
        kzX     = feval(GP.covfunc{:},GP.hyp.cov,GP.z,GP.x,'k(z,X)');
    end

end