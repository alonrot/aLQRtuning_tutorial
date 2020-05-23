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
function visual_simulation(x,cp,sims)
%VISUAL_SIMULATION
%   VISUAL_SIMULATION() simulates the dynamics of a cart-pole system.
%
%   Input arguments:
%       x       States trajectories. Each time step must be stored as a
%               column vector, with:
%               [phi(k);phid(k);s(k);sd(k)], where
%               phi     pole angle
%               phid    pole angular velocity
%               s       cart position
%               sd      cart velocity
%
%       cp      Structure that contains information about the physical
%               parameters of the pole. See construct_cart_pole().
%       sims    Structure that contains information about the simulation.
%               See construct_simulator_settings().
%
% (C) 2016 Max Planck Society. All rights reserved.
%
% Alonso Marco Valle
%
% See also CONSTRUCT_CART_POLE, CONSTRUCT_SIMULATOR_SETTINGS

    % Get cart size:
    [~,h] = get_cart_size();

    % Initialize visualization:
    [~,hdl_plot] = initialize_visualization(cp,x,h);
    
    % Stop condition:
%     which_values = find(abs(x(3,:))<0.01);
%     aux_ind = find(diff(which_values)>1,1,'last');
%     K_stable = which_values(aux_ind+1);
    
    % Update cart-pole:
    for k = 1:sims.Kf
        
        % Update cart:
        [hdl_plot.cart.XData,hdl_plot.cart.YData] = get_cart_polygon(x(3,k));
        
        % Update stick:
        pole = get_pole_bottom_and_tip(x(1,k),x(3,k),cp.l,h);
        hdl_plot.pole.XData = [pole.bottom(1) pole.tip(1)];
        hdl_plot.pole.YData = [pole.bottom(2) pole.tip(2)];
        
        % Update mass at the tip:
        hdl_plot.ball.XData = pole.tip(1);
        hdl_plot.ball.YData = pole.tip(2);
        
        % Little pause, to be able to see it:
        pause(0.0001);
        
        if k == 1
            pause(1);
        end
        
    end
    
%     display(['Final position reached at t = ' num2str((K_stable-1)*sims.Ts) 's < ' num2str(sims.tf) 's']);

end

function pole = get_pole_bottom_and_tip(phi,s,len,h)

    pole = struct();
    pole.bottom = [s,h];
    pole.tip    = [s,h] + [len*sin(phi),len*cos(phi)];

end

function [x_lim,y_lim] = get_plotting_limits(cp,x,h)

    x_lim = [min(x(3,:))-cp.l , max(x(3,:))+cp.l];
    y_lim = [-(cp.l-h)-0.1 , cp.l+h+0.1];

end

function [X,Y] = get_cart_polygon(sx)

    % Get cart size:
    [w,h] = get_cart_size();
    
    % Center of mass height:
    sy = h;
    
    % Get polygon:
    X = [sx-w sx-w sx+w sx+w];
    Y = [sy-h sy+h sy+h sy-h];

end

function [w,h] = get_cart_size()

    % Cart width/2 and height/2:
    w = 0.2;
    h = 0.1;

end

function [fig_hdl,hdl_plot] = initialize_visualization(cp,x,h)
    
    % Initialize plots:
	fig_hdl = figure; hold on;
    set(fig_hdl,'Position',[50 50 800 400]);    
    set(fig_hdl,'MenuBar','none');
    set(fig_hdl,'Name','Cart-pole visualization');
    
    % Plot angle:
    hdl_plot.main = plot(1,1);
    
    % Get plotting limits:
    [x_lim,y_lim] = get_plotting_limits(cp,x,h);
    
    % Floor:
    hdl_plot.floor = line(x_lim,[0.1 0.1]);
    set(hdl_plot.floor,'linewidth',2);
    set(hdl_plot.floor,'Color','k');
    
    % Cart:
    hdl_plot.cart = fill([0,0,0,0],[0,0,0,0],[135,206,250]/255);
    
    % Pole as a blue line:
    hdl_plot.pole = line([0 0],[0 0]);
    set(hdl_plot.pole,'linewidth',3);
    set(hdl_plot.pole,'Color',[0 0 0.8]);

    % Pole mass:
    hdl_plot.ball = plot(0,0);
    set(hdl_plot.ball,'Marker','o','MarkerSize',20,'MarkerFaceColor','r');
    
    title('Cart pole simulation','interpreter','tex');
    xlim(x_lim); ylim(y_lim);
%     grid on; grid minor;
    axis equal

end
