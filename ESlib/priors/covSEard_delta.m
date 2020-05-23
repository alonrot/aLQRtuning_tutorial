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
function K = covSEard_delta(hyp, x, z, which_case)

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

    [ell_s,sf2_s,ell_e,sf2_e] = get_hyp(hyp,x);

    [Xs,Xr] = get_sets(x);

    % Transpose because sq_dist() needs data vectors organized in
    % columns, as opposed to the ES's philosophy, which is rows.
    Xs = Xs'; Xr = Xr';

    if isempty(Xs)
        Ks_ss = []; Ks_rs = []; Ks_sr = [];
        KF_rr = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*Xr)) + sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*Xr));
    elseif  isempty(Xr)
        Ks_ss = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*Xs));
        Ks_rs = []; Ks_sr = []; KF_rr = [];
    else     
        Ks_ss = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*Xs));                      % Ks(Xs,Xs)
        Ks_rs = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*Xr,diag(1./ell_s)*Xs));    % Ks(Xr,Xs)
        Ks_sr = Ks_rs';                                                          % Ks(Xs,Xr)
        KF_rr = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*Xr)) + ...                 % Ks(Xr,Xr)
                sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*Xr));
    end

    K = [Ks_ss Ks_sr;
         Ks_rs KF_rr];

end

% A delta query is needed in this function, because either x or z are
% potential single query points where the E[dH] is computed:
function K = get_kzz(hyp,x,z)

    [ell_s,sf2_s,ell_e,sf2_e] = get_hyp(hyp,x);

    if nargin <= 2
        
        delta_q = get_delta_query(x);
        
        K = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*x')) + ...
            delta_q * sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*x')); 
        
    else
        
        delta_q = get_delta_query(x,z);
        
        K = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*x',diag(1./ell_s)*z')) + ...
            delta_q * sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*x',diag(1./ell_e)*z'));
    end

end

% A delta query is needed in this function, because z is a potential single
% query points where the E[dH] is computed:
function K = get_kzX(hyp,x,z)

    delta_q = get_delta_query(z);

    [ell_s,sf2_s,ell_e,sf2_e] = get_hyp(hyp,x);

    [Xs,Xr] = get_sets(x);

    % Transpose because sq_dist() needs row vectors:
    Xs = Xs'; Xr = Xr';
    z = z';

    if isempty(Xs)
        Ks_zs = [];
        KF_zr = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*z,diag(1./ell_s)*Xr)) + ...
                delta_q * sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*z,diag(1./ell_e)*Xr)); % KF(zbel,Xr) = Ke(zbel,Xr)
    elseif  isempty(Xr)
        Ks_zs = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*z,diag(1./ell_s)*Xs)); % Ks(zbel,Xs)
        KF_zr = [];
    else     
        Ks_zs = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*z,diag(1./ell_s)*Xs)); % Ks(zbel,Xs)
        KF_zr = sf2_s*exp(-0.5*sq_dist(diag(1./ell_s)*z,diag(1./ell_s)*Xr)) + ...
                delta_q * sf2_e*exp(-0.5*sq_dist(diag(1./ell_e)*z,diag(1./ell_e)*Xr)); % KF(zbel,Xr)
    end

    % Composite Gramm Matrix:
    K = [Ks_zs KF_zr];    

end

function delta_q = get_delta_query(x,z)

    global delta_query;
    
    if isempty(delta_query)
        delta_q = 1;
        warning('Variable delta for new query points has not been globally defined');
        return;
    end
    
    if nargin <= 1
        if size(x,1) == 1
            delta_q = delta_query;
        else
            delta_q = 1;
        end
    else
        if size(x,1) == 1 || size(z,1) == 1
            delta_q = delta_query;
        else
            delta_q = 1;
        end
    end
    
end

function [ell_s,sf2_s,ell_e,sf2_e] = get_hyp(hyp,x)

    [~,D] = size(x);
%     warning(['n = ' num2str(n) ', D = ' num2str(D)]);
    ell_s = exp(hyp(1:D));                               % characteristic length scale
    sf2_s = exp(2*hyp(D+1));                                         % signal variance
    ell_e = exp(hyp(D+2:2*D+1));                               % characteristic length scale
    sf2_e = exp(2*hyp(2*D+2));                                         % signal variance   

end