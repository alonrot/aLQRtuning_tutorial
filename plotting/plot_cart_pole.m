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
function plot_cart_pole(fig_hdl,hdl_splot,x_simu,F_simu,sims)

    [phi,phid,s,sd] = get_individual_states(x_simu);
    
    figure(fig_hdl);
    
    % Plot pole angle:
    axes(hdl_splot.phi); hold on; 
    grid on; grid minor;
    plot(sims.t,phi);
    
    % Plot pole angular velocity:
    axes(hdl_splot.phid); hold on; 
    grid on; grid minor;
    plot(sims.t,phid);
    
    % Plot cart positon:
    axes(hdl_splot.s); hold on; 
    grid on; grid minor;
    plot(sims.t,s);
    
    % Plot cart velocity:
    axes(hdl_splot.sd); hold on; 
    grid on; grid minor;
    plot(sims.t,sd);
    
    % Plot force:
    axes(hdl_splot.F); hold on; 
    grid on; grid minor;
    plot(sims.t,F_simu);

end