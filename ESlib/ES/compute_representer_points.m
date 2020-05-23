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
function representers = compute_representer_points(GP,in,BestGuesses)

    [zb,lmb]   = SampleBeliefLocations(GP,in.xmin,in.xmax,in.Nb,BestGuesses,in.PropFunc);

    GP.z = zb;
    GP = get_posterior(GP);

    % Belief over the minimum on the sampled set:
    Mb = GP.mpost;
    Vb = GP.vpost;
    [logP,dlogPdM,dlogPdV,ddlogPdMdM] = joint_min(Mb,Vb);        % p(x=xmin)

    Hs = - sum(exp(logP) .* (logP + lmb));       % current Entropy

    representers = update_representers(zb,lmb,Mb,Vb,logP,dlogPdM,dlogPdV,ddlogPdMdM,Hs);
    
    fprintf '\n';

end

function representers = update_representers(zb,lmb,Mb,Vb,logP,dlogPdM,dlogPdV,ddlogPdMdM,Hs)

    representers.zb            = zb;
    representers.lmb           = lmb;
    representers.Mb            = Mb;
    representers.Vb            = Vb;
    representers.logP          = logP;
    representers.dlogPdM       = dlogPdM;
    representers.dlogPdV       = dlogPdV;
    representers.ddlogPdMdM    = ddlogPdMdM;
    representers.Hs            = Hs;

end