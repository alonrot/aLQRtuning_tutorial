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
function update_plot(fig_hdl,axes_hdl,varargin)

    if isempty(fig_hdl) && isempty(axes_hdl)
        return;
    end

	display('*** Update plots!');

    if nargin == 3
        
        for k = 1:length(varargin)
            plot_only(fig_hdl,axes_hdl,varargin{k});
        end
        drawnow;
        
        return;
    end

    % Update GP:
    plot_GP(fig_hdl,axes_hdl.gp);
    
    % Update pmin:
    plot_pmin(fig_hdl,axes_hdl.pmin);
    
    % Update EdH:
    plot_EdH(fig_hdl,axes_hdl.EdH);
    
    drawnow;

end

function plot_only(fig_hdl,axes_hdl,str_plot)

    switch str_plot
        case 'only_global_minimum'
            plot_gm(fig_hdl,axes_hdl.gp);
    end

end

function plot_GP(fig_hdl,axes_hdl)

    global GP_plot;

    GP_plot_for_cond = GP_plot;
    GP_plot_for_cond = get_posterior(GP_plot_for_cond);
    GP_plot.mpost   = GP_plot_for_cond.mpost;
    GP_plot.stdpost = GP_plot_for_cond.stdpost;
    
    figure(fig_hdl); 
    subplot(axes_hdl);
    cla(axes_hdl);
    hold on; grid on; box on;
    % Special case for no data:
    if isempty(GP_plot.x)
        
        f_ = [GP_plot.mpost + 2*GP_plot.stdpost; flip(GP_plot.mpost - 2*GP_plot.stdpost,1)];
        fill([GP_plot.z; flip(GP_plot.z,1)], f_, GP_plot.uncertainty_color)
        plot(GP_plot.z,GP_plot.mpost,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)
        plot(GP_plot.z,GP_plot.f_true,'LineStyle','-','Color',0.7 * [1 1 1],'LineWidth',1.5)
        
    else
            
        [Xs,Xr,Ys,Yr] = get_sets(GP_plot.x,GP_plot.y);

        f_ = [GP_plot.mpost + 2*GP_plot.stdpost; flip(GP_plot.mpost - 2*GP_plot.stdpost,1)];
        fill([GP_plot.z; flip(GP_plot.z,1)], f_, GP_plot.uncertainty_color)
        plot(GP_plot.z,GP_plot.mpost,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)
        plot(Xs,Ys,'o','color',GP_plot.simu_color,'MarkerFaceColor',GP_plot.simu_color);
        plot(Xr,Yr,'o','color',GP_plot.real_color,'MarkerFaceColor',GP_plot.real_color);
        plot(GP_plot.z,GP_plot.f_true,'LineStyle','-','Color',0.7 * [1 1 1],'LineWidth',1.5)
        
    end

end

function plot_gm(fig_hdl,axes_hdl)

    global GP_plot;
    figure(fig_hdl); 
    subplot(axes_hdl);
    plot(GP_plot.x_globalmin,interp1(GP_plot.z,GP_plot.mpost,GP_plot.x_globalmin),'o','color',GP_plot.minimum_color,'MarkerFaceColor',GP_plot.minimum_color);

end

function plot_pmin(fig_hdl,axes_hdl)

    global GP_plot;

    figure(fig_hdl); 
    subplot(axes_hdl);
    cla(axes_hdl);
    hold on; grid on; box on;

    if isfield(GP_plot,'pmin') && isfield(GP_plot,'pmin')
        
        [pmin,zb] = sort_pmin(GP_plot.pmin,GP_plot.zb);
    
        plot(zb,pmin,'color',GP_plot.real_color,'MarkerFaceColor',GP_plot.real_color,'linestyle','--','Marker','o');
        legend('Real');

    end
    
end

    
function [pmin_sorted,zb_sorted] = sort_pmin(pmin,zb)

    p_min_aux = exp(pmin);
    [zb_sorted,ind_]   = sort(zb);
    % Sort out:
    pmin_sorted = p_min_aux(ind_);

end

function plot_EdH(fig_hdl,axes_hdl)

    global GP_plot;

    figure(fig_hdl); 
    subplot(axes_hdl);
    cla(axes_hdl);
    hold on; grid on; box on;

    if isfield(GP_plot,'EdH')
        
        plot(GP_plot.z,GP_plot.EdH,'color',GP_plot.simu_color);
        
    end
    
end