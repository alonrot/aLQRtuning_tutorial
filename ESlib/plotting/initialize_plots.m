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
function [fig_hdl,axes_hdl] = initialize_plots(GP,in)

    if in.D > 1
        warning('No plots will be produced for specs.D > 1');
        fig_hdl = []; axes_hdl = [];
        return;
    end

    global GP_plot;
    GP_plot = get_plotting_atributes(GP_plot);
    GP_plot = extraxt_needed_variables(GP_plot,GP,in);

    fig_hdl         = figure;
    fig_hdl.Position = [222 73 2276 1386];
    
    axes_hdl.gp     = subplot(in.subplot_struct(1),in.subplot_struct(2),in.subplot_num.gp);
    title('F_{real}(x_*) = F_{sim}(x_*) + F_{err}(x_*) | F_{real}(X_r), F_{sim}(X_s)','interpreter','tex');
    xlabel('x_*','interpreter','tex');
    ylabel('GP','interpreter','tex');
    ylim([ 20 30])
    
    axes_hdl.pmin   = subplot(in.subplot_struct(1),in.subplot_struct(2),in.subplot_num.pmin);
    title('p_{min} at representer points','interpreter','tex');%,'fontsize',GP_plot.font_size);
    xlabel('x_*','interpreter','tex');%,'fontsize',GP_plot.font_size); 
    ylabel('p_{min}','interpreter','tex');%,'fontsize',GP_plot.font_size);
	xlim([GP_plot.xmin,GP_plot.xmax]);

    axes_hdl.EdH    = subplot(in.subplot_struct(1),in.subplot_struct(2),in.subplot_num.EdH);
    title('Expected change in Entropy at query points','interpreter','tex');%,'fontsize',GP_plot.font_size);
    xlabel('x_*','interpreter','tex');%,'fontsize',GP_plot.font_size); 
    ylabel('E[\Delta H]','interpreter','tex');%,'fontsize',GP_plot.font_size);

end

function GP = get_plotting_atributes(GP)

    GP.uncertainty_color = [240,248,255]/255;
    GP.simu_color        = [0.466 0.674 0.188];
    GP.real_color        = [205,92,92]/255;
    GP.minimum_color     = [106,90,205]/255;
    GP.fontsize          = 12;

end

function GPp = extraxt_needed_variables(GPp,GP,in)

    GPp.x                   = GP.x;
    GPp.y                   = GP.y;
    GPp.z                   = GP.z_plot;
    GPp.Nbel                = in.Nbel;
    GPp.hyp                 = GP.hyp;
    GPp.mean                = GP.mean;
    GPp.covfunc             = GP.covfunc;
    GPp.xmin                = in.xmin;
    GPp.xmax                = in.xmax;
    GPp.f_true              = in.f_true;
    GPp.Ws                  = in.Ws;

end