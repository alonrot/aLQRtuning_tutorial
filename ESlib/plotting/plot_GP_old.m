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
function plot_GP(GP_struct,N,which_dim,N_div,plot_log,plot_length,varargin)
%PLOT_GP   Plots a 3D Gaussian Process
%   PLOT_GP(GP_struct,N,which_dim,N_div,plot_log) plots a Gaussian Process
%   as a 3D surface using the function GP_moments. N specifies the number
%   of input dimensions. When this is higher than 2, the vector which_dim
%   is used as a selector of dimensions, i.e. which_dim = [1 3] will plot a
%   GP using the dimensions 1 and 3, and excluding the rest. N_div is used
%   to specify the resultion of the grid. plot_log is set to 1 if we want
%   to plot the GP in logarithmic scale, and 0 for decimal scale.

% Preparing the variables:
    GP_plot              = struct;
    GP_plot.covfunc      = GP_struct.covfunc;
    GP_plot.covfunc_dx   = GP_struct.covfunc_dx;
    GP_plot.likfunc      = @likGauss; % noise type
    GP_plot.hyp.lik      = GP_struct.hyp.lik;
    
    GP_plot.hyp.cov(1:N) = GP_struct.hyp.cov([which_dim]);
    GP_plot.hyp.cov(N+1:length(GP_struct.hyp.cov)) = GP_struct.hyp.cov(N+1:length(GP_struct.hyp.cov));
    
    GP_plot.res          = 1;
    GP_plot.deriv        = 0; % derivative observations?
    GP_plot.poly         = GP_struct.poly;
    GP_plot.log          = 0;
    GP_plot.xmin         = GP_struct.xmin;
    GP_plot.xmax         = GP_struct.xmax;
    
    if ~isfield(GP_struct,'x') || ~isfield(GP_struct,'y')
        GP_plot.x        = []; 
        GP_plot.y        = [];
    else
        GP_plot.x        = GP_struct.x(:,which_dim);
        GP_plot.y        = GP_struct.y;
        GP_plot.K    = k_matrix(GP_plot,GP_plot.x) + diag(GP_noise_var(GP_plot,GP_plot.y));
        GP_plot.cK   = chol(chol_fix(GP_plot.K));
    end
    
% Plotting the current best guess for the minimum location and also where
% to evaluate next:
    figure; hold on;
    if nargin == 7 % Plotting the next evaluation:
        xp_next  = varargin{1};
        [mp_next,~]  = GP_moments(GP_plot,xp_next);
        plot3(xp_next(1),xp_next(2),mp_next,'o','Color',[0 0.447 0.741],'MarkerSize',25,'MarkerFaceColor',[0 0 1]);
        legend('Next evaluation');
    elseif nargin == 8 % Plotting the current minimum Best Guess and the next evaluation:
        if ~isempty(varargin{1})
            xp_next  = varargin{1}; xp_minBG = varargin{2};
            [mp_next,~]  = GP_moments(GP_plot,xp_next);
            [mp_minBG,~] = GP_moments(GP_plot,xp_minBG);
            plot3(xp_next(1),xp_next(2),mp_next,'o','Color',[0 0.447 0.741],'MarkerSize',25,'MarkerFaceColor',[0 0 1]);
            hold on
            plot3(xp_minBG(1),xp_minBG(2),mp_minBG,'o','Color',[0.169 0.506 0.337],'MarkerSize',25,'MarkerFaceColor',[0.498 1.0 0.0]);
            legend('Next evaluation','Current best guess for the location of the minimum')
        else
            xp_minBG = varargin{2};
            [mp_minBG,~] = GP_moments(GP_plot,xp_minBG);
            plot3(xp_minBG(1),xp_minBG(2),mp_minBG,'o','Color',[0.169 0.506 0.337],'MarkerSize',25,'MarkerFaceColor',flip([0.498 1.0 0.0]));
            legend('Current best guess for the location of the minimum')
        end
    end
    
% Plot size:
    r = 1400/1800;
    width_ = 1000;
    pos_win = get(gcf,'Position');
    set(gcf,'Position',[pos_win(1) pos_win(2) width_ width_*r])
    view(60,30)

    
% Getting the Gaussian Process for plotting purposes:
    if plot_log
        [xx1,xx2] = meshgrid(logspace(log10(GP_plot.xmin(which_dim(1))),log10(GP_plot.xmax(which_dim(1))),N_div),...
                             logspace(log10(GP_plot.xmin(which_dim(2))),log10(GP_plot.xmax(which_dim(2))),N_div));
    else
        [xx1,xx2] = meshgrid(linspace(GP_plot.xmin(which_dim(1)),GP_plot.xmax(which_dim(1)),N_div),...
                             linspace(GP_plot.xmin(which_dim(2)),GP_plot.xmax(which_dim(2)),N_div));
    end
    zz = [xx1(:),xx2(:)]; 
    [Mm,Vv]    = GP_moments(GP_plot,zz);
    Mm = reshape(Mm,size(xx1)); Ss = reshape(sqrt(diag(Vv)),size(xx1));
    
% Plotting the mean and +- 2*standard deviation:
    if plot_log 
        surf(xx1,xx2,Mm,'FaceColor','r');
        set(gca,'Xscale','log','Yscale','log');
    else
        surf(xx1,xx2,Mm,'FaceColor',flip([0.894 0.063 0.478]),'EdgeColor',flip([1.0 0.702 0.855]),'FaceAlpha',0.6);
        surf(xx1,xx2,Mm + 2*Ss,'FaceColor',0.314*[1 1 1],'EdgeColor',0.502*[1 1 1],'FaceAlpha',0.3,'EdgeAlpha',0.3);
        surf(xx1,xx2,Mm - 2*Ss,'FaceColor',0.314*[1 1 1],'EdgeColor',0.502*[1 1 1],'FaceAlpha',0.3,'EdgeAlpha',0.3);
    end
    
% Plotting the initial set of data:

    if isfield(GP_struct,'x') && isfield(GP_struct,'y')
        plot3(GP_plot.x(1:plot_length,1),GP_plot.x(1:plot_length,2),GP_plot.y(1:plot_length),'o','MarkerSize',20,'MarkerFaceColor',flip([0.392 0.694 1.0]),'Color',[0.871 0.490 0.0]); grid on;
        xlabel(strcat(['Parameter ' num2str(which_dim(1))]));
        ylabel(strcat(['Parameter ' num2str(which_dim(2))]));
        zlabel('Cost value');
        r = 1400/1800;
        width_ = 2000;
        pos_win = get(gcf,'Position');
        set(gcf,'Position',[pos_win(1) pos_win(2) width_ width_*r])
        view(60,30)
    end

    view(3);
    set(gcf, 'Position', [3000 400 1200 1000])
    drawnow;
end