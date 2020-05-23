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
function X_next = get_most_informative_point(representers,GP,in,S0)

    % Compute most informative point, as next evaluation:
    [~,X_next] = get_expected_change_in_entropy(representers,GP,in,S0);

end

% Wrapper to compute_max_EdH(), to make it more readable:
function [xdhbest,xp] = get_expected_change_in_entropy(representers_specific,GP,in,S0)

    % Extract variables from representers structure:
    zb          = representers_specific.zb;
    logP        = representers_specific.logP;
    dlogPdM   	= representers_specific.dlogPdM;
    dlogPdV   	= representers_specific.dlogPdV;
    ddlogPdMdM  = representers_specific.ddlogPdMdM;
    lmb         = representers_specific.lmb;

    [xdhbest,xp] = compute_max_EdH(zb,GP,logP,dlogPdM,dlogPdV,ddlogPdMdM,in,lmb,S0);

end

% Get the maximum of the expected change in entropy:
function [xdhbest,xp] = compute_max_EdH(zb,GP,logP,dlogPdM,dlogPdV,ddlogPdMdM,in,lmb,S0)
    
    display(' ');
    display(['*** Computing E[dH]...']);

    % We want to minimize dH_fun, which is the same as maximizing dH_fun_p
    dH_fun     = dH_MC_local(zb,GP,logP,dlogPdM,dlogPdV,ddlogPdMdM,in.W,lmb,in.xmin,in.xmax,false,in.LossFunc);
    dH_fun_p   = dH_MC_local(zb,GP,logP,dlogPdM,dlogPdV,ddlogPdMdM,in.W,lmb,in.xmin,in.xmax,true,in.LossFunc);
    % sample some evaluation points. Start with the most likely min in zb.
    [~,mi]     = max(logP);
    xx         = zb(mi,:);
    Xstart     = zeros(in.Ne,in.D);
    Xend       = zeros(in.Ne,in.D);
    Xdhi       = zeros(in.Ne,1);
    Xdh        = zeros(in.Ne,1);
    fprintf('    Sampling start points for search for optimal evaluation points...\n');
    xxs = zeros(10*in.Ne,in.D);
    for i = 1:10 * in.Ne
        if mod(i,10) == 1 && i > 1; xx = in.xmin + (in.xmax - in.xmin) .* rand(1,in.D); end;
        xx     = Slice_ShrinkRank_nolog(xx,dH_fun_p,S0,true);
        xxs(i,:) = xx;
        if mod(i,10) == 0; Xstart(i/10,:) = xx; Xdhi(i/10) = dH_fun(xx); end
    end

    % optimize for each evaluation point:
    fprintf('    Local optimizations of evaluation points...\n');
    for i = 1:in.Ne
        [Xend(i,:),Xdh(i)] = fmincon(dH_fun,Xstart(i,:),[],[],[],[],in.xmin,in.xmax,[], ...
            optimset('MaxFunEvals',20,'TolX',eps,'Display','off','GradObj','on','Algorithm','interior-point'));

    end
    % which one is the best?
    [xdhbest,xdhbv]   = min(Xdh);

    xp                = Xend(xdhbv,:);
    
    % Plotting only in 1D:
    if in.D == 1
        include_EdH_in_GP_plot(dH_fun);
    end

end

function include_EdH_in_GP_plot(dH_fun)

    global GP_plot;

    EdH = zeros(GP_plot.Nbel,1);
    for k = 1:GP_plot.Nbel
        EdH(k) = -dH_fun(GP_plot.z(k));
    end
    
    GP_plot.EdH = EdH;

end