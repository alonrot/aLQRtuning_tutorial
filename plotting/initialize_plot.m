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
function [fig_hdl,hdl_splot] = initialize_plots()

    % Get subplot information:
    splot = get_subplot_info();
    
    % Initialize plots:
	fig_hdl = figure;
    
    set(fig_hdl,'Position',[50 50 800 800]);
    
    % Plot angle:
    hdl_splot.phi = subplot(splot.structure(1),splot.structure(2),splot.phi);
%     xlabel('time [s]','interpreter','tex'); 
    ylabel('\phi [deg]','interpreter','tex');
    title('Pole angle \phi(t)','interpreter','tex');
    
    % Plot angular velocity:
    hdl_splot.phid = subplot(splot.structure(1),splot.structure(2),splot.phid);
%     xlabel('time [s]','interpreter','tex'); 
    ylabel('d\phi/dt [deg/s]','interpreter','tex');
    title('Pole angular velocity d\phi/dt(t)','interpreter','tex');
    
    % Cart position:
    hdl_splot.s = subplot(splot.structure(1),splot.structure(2),splot.s);
%     xlabel('time [s]','interpreter','tex'); 
    ylabel('s [m]','interpreter','tex');
    title('Cart position s(t)','interpreter','tex');
    
    % Plot angular velocity:
    hdl_splot.sd = subplot(splot.structure(1),splot.structure(2),splot.sd);
%     xlabel('time [s]','interpreter','tex'); 
    ylabel('ds/dt [m/s]','interpreter','tex');
    title('Cart velocity ds/dt(t)','interpreter','tex');
    
    % Plot input force:
    hdl_splot.F = subplot(splot.structure(1),splot.structure(2),splot.F);
    xlabel('time [s]','interpreter','tex'); 
    ylabel('F [m/s^2]','interpreter','tex');
    title('Force F(t)','interpreter','tex');
    

end

function splot = get_subplot_info()

    % Subplots information:
    splot.structure = [5 1];
    splot.phi       = 1;
    splot.phid      = 2;
    splot.s         = 3;
    splot.sd        = 4;
    splot.F         = 5;

end