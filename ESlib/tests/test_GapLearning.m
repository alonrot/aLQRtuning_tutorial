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
%% Initialization:
    clc; close all; clear all;

%% Modfying the path:
    run('./gpml-matlab-v3.1-2010-09-27/startup.m');
    addpath ./utils/;
    
%% Initialization:

    % Number of input dimensions:
    D = 1;

    % Domain:
    X_min = -10*ones(D,1);
    X_max =  10*ones(D,1);


%% Prepare GP:
    specs.GP       = initialize_GP();
    
%% Get new evaluation:
    specs.GP    = get_new_evaluation(specs.GP);
    
%% Compute posterior:
     specs.GP    = get_posterior(specs.GP);
     
     
    
%% 

%% Some functions:
    center = @(I) (I(1) + I(2))/2;
    compress_interval = @(I,fac_) [center(I) - abs(center(I)-I(1))*(1-fac_),center(I) + abs(center(I)-I(1))*(1-fac_)];
    extend_interval   = @(I,fac_) [center(I) - abs(center(I)-I(1))*(1+fac_),center(I) + abs(center(I)-I(1))*(1+fac_)];

%% Define an appropiate kernel:

% Number of input dimensions:
D = 1;

% Domain:
X_min = -10*ones(D,1);
X_max =  10*ones(D,1);

% Kernel for simulation:
K_sim = {@covSEard};
L_sim = 5;
sf_sim = 8;
hyp_sim.cov = log([L_sim;sf_sim]);

% Kernel for the error:
K_err = {@covSEard};
L_err = 1;
sf_err = 4;
hyp_err.cov = log([L_err;sf_err]);

% Mean:
m_sim = {@meanZero}; hyp_sim.mean = [];
% m_err = {@meanZero}; hyp_err.mean = [];
m_err = {@meanConst_err}; hyp_err.mean = 0;

% Inference method:
infer_ = @infExact;

% Likelihood function:
lik = @likGauss;

% Test points for obtaining the posterior belief:
N_grid = 100;
zbel = linspace(X_min,X_max,N_grid)';

% Modified covariance function:
% k_F = {'covSum',{K_sim,K_err}};
k_F = {@covSEard_F};
hyp_F.cov = [hyp_sim.cov; hyp_err.cov];

% Mean of F (only for sampling):
% m_F = {'meanSum',{m_sim,m_err}};
% hyp_F.mean = [hyp_sim.mean;hyp_err.mean];
m_F = {@meanConst_err};
hyp_F.mean = hyp_err.mean;

% Likelihood noise (std) of F:
sn_F = 1;
hyp_F.lik = log(sn_F);

% Likelihood noise (std) of F_sim:
sn_sim = 0.01;
hyp_sim.lik = log(sn_sim);

% Likelihood noise (std) of F_err:
if sn_F <= sn_sim, error('sn_F <= sn_sim !!!'); else
sn_err = sqrt(sn_F^2-sn_sim^2); end;
hyp_err.lik = log(sn_err);

%% Printing out assumptions:
display(' '); display(' ');
display('*****************************************************');
display('***************** Prior assumptions *****************');
display('*****************************************************');
display(' ');
display(['Domain: ' num2str(X_min(:,1)) ' < Xr,Xs < ' num2str(X_max(:,1))]);
display(' ');
display('F_real = F_sim + F_err');
display(' ');
display('    Y_real = F_real(Xr)');
display('    F_real = GP(m_real,cov_real) |  F_real(Xr),  F_sim(Xs)');
display(['              m_real     = @' func2str(m_sim{1}) ' + @' func2str(m_err{1})]);
display(['              cov_real   = @' func2str(K_sim{1}) ' + delta*@' func2str(K_err{1})]);
display(['                           l   = {l_sim,l_err} = {' num2str(L_sim) ',' num2str(L_err) '}' ]);
display(['                           std = {std_sim,std_err} = {' num2str(sf_sim) ',' num2str(sf_err) '}' ]);
display(['              lik_real   = @' func2str(lik) ]);
display(['                           sn_real = ' num2str(sn_F) ]);
display(' ');
display('    Y_sim  = F_sim(Xs)');
display('    F_sim  = GP(m_sim,cov_sim) |  F_sim(Xs)');
display(['              m_sim   = @' func2str(m_sim{1})]);
display(['              cov_sim = @' func2str(K_sim{1})]);
display(['                         l_sim = ' num2str(L_sim) ]);
display(['                         std_sim = ' num2str(sf_sim)]);
display(['              lik_sim  = @' func2str(lik) ]);
display(['                           sn_sim  = ' num2str(sn_sim) ]);
display(' ');
display('*****************************************************');
display(' '); display(' ');

%% Atributes:

uncertainty_color = [240,248,255]/255;

%% Sample F_real function from the corresponding prior GP:

% Compute covariance Gramm Matrix and prior mean in the domain:
    global delta;
    delta = logical(ones(size(zbel,1),1));
    k_F_GM = chol_fix(feval(k_F{:},hyp_F.cov,zbel,[]));
    mu_F_r = feval(m_F{:},delta,hyp_F.mean,zbel);

% Factorize Gramm Matrix:
    % [T,err] = cholcov(sigma); % Cannot be used if k_F_GM is non-symmetric
    [T,err] = chol(k_F_GM);     % Allows non-symmetric matrices

% Samples from a multivariate Gaussian distribution centered in mu_F_, with
% covariance k_F_GM.
    N_F_samples = 4;
    [F_real_samples,T] = mvnrnd(mu_F_r,k_F_GM,N_F_samples,T);
    
%% Sample F_sim function from the corresponding prior GP:

% Compute covariance Gramm Matrix and prior mean in the domain:
    k_s_GM = chol_fix(feval(K_sim{:},hyp_sim.cov,zbel,[]));
    mu_F_s = feval(m_sim{:},hyp_sim.mean,zbel);

% Factorize Gramm Matrix:
    % [T,err] = cholcov(sigma); % Cannot be used if k_F_GM is non-symmetric
    [T,err] = chol(k_s_GM);     % Allows non-symmetric matrices

% Samples from a multivariate Gaussian distribution centered in mu_F_, with
% covariance k_F_GM.
    N_F_samples = 4;
    [F_sim_samples,T] = mvnrnd(mu_F_s,k_s_GM,N_F_samples,T);
    
%% Sample F_err function from the corresponding prior GP:

% Compute covariance Gramm Matrix and prior mean in the domain:
    k_e_GM = chol_fix(feval(K_err{:},hyp_err.cov,zbel,[]));
    delta = logical(ones(size(zbel,1),1));
    mu_F_e = feval(m_err{:},delta,hyp_err.mean,zbel);

% Factorize Gramm Matrix:
    % [T,err] = cholcov(sigma); % Cannot be used if k_F_GM is non-symmetric
    [T,err] = chol(k_e_GM);     % Allows non-symmetric matrices

% Samples from a multivariate Gaussian distribution centered in mu_F_, with
% covariance k_F_GM.
    N_F_samples = 4;
    [F_err_samples,T] = mvnrnd(mu_F_e,k_e_GM,N_F_samples,T);
    
%% Plotting prior samples:

    fig0 = figure;
    hdl_plot_F_real_pri = subplot(3,1,1);
    hold on
    std_F_r = sqrt(diag(k_F_GM));
    f_ = [mu_F_r+2*std_F_r; flipdim(mu_F_r-2*std_F_r,1)];
    fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
    % Plotting function evaluations:
    color_palete = cool(N_F_samples);
    plot(zbel,mu_F_r,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)

    for k = 1:N_F_samples
        plot(zbel,F_real_samples(k,:),'color',color_palete(k,:))
    end
    
    % We plot the simulation 2*std:
    line([zbel(1) zbel(end)],mu_F_r(1) + [2*sf_sim 2*sf_sim],'linestyle','--','color',[186,85,211]/255,'LineWidth',1.5)
    line([zbel(1) zbel(end)],mu_F_r(1) - [2*sf_sim 2*sf_sim],'linestyle','--','color',[186,85,211]/255,'LineWidth',1.5)
    
    title('Prior: F_{real}(x_*) = F_{sim}(x_*) + F_{err}(x_*)','interpreter','tex')
    xlabel('x_*');
    
    hdl_plot_F_sim_pri = subplot(3,1,2);
    hold on
    std_F_s = sqrt(diag(k_s_GM));
    f_ = [mu_F_s+2*std_F_s; flipdim(mu_F_s-2*std_F_s,1)];
    fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
    % Plotting function evaluations:
    color_palete = cool(N_F_samples);
    plot(zbel,mu_F_s,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)

    for k = 1:N_F_samples
        plot(zbel,F_sim_samples(k,:),'color',color_palete(k,:))
    end
    
    title('Prior: F_{sim}(x_*)','interpreter','tex')
    xlabel('x_*');
    
    hdl_plot_F_err_pri = subplot(3,1,3);
    hold on
    std_F_e = sqrt(diag(k_e_GM));
    f_ = [mu_F_e+2*std_F_e; flipdim(mu_F_e-2*std_F_e,1)];
    fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
    % Plotting function evaluations:
    color_palete = cool(N_F_samples);
    plot(zbel,mu_F_e,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)

    for k = 1:N_F_samples
        plot(zbel,F_err_samples(k,:),'color',color_palete(k,:))
    end
    
    title('Prior: F_{err}(x_*)','interpreter','tex')
    xlabel('x_*');
    
    
    fig0.Position = [500 300 1500 1000];
    
    % Re-sizing the axes:
    hdl_plot_F_sim_.YLim = hdl_plot_F_real_pri.YLim;
    hdl_plot_F_err_.YLim = hdl_plot_F_real_pri.YLim;
    
    
%% Pick one of the samples as the function of the real system (Fr):

%     randi(N_F_samples,[2,1])
    Fe = F_err_samples(randi(N_F_samples),:)';
    Fs = F_sim_samples(randi(N_F_samples),:)';
    Fr = Fs + Fe;
    
    % Sampling directly from Fr is incorrect, because the triangle
    % inequaliy (takin matrix norms) holds for the cholesky factor.
%     Fr = F_real_samples(randi(N_F_samples),:)';
    
%% Pick some randomly sampled evaluations from Fr and Fs:

    N = 20;
    samples_ind = randi(length(Fr),N,1);
    
    % We create the sets Xs and Xr:
    Ns = 3;
    
    delta = logical([zeros(Ns,1);ones(N-Ns,1)]);
    ind_Xr = samples_ind(delta);
    ind_Xs = samples_ind(~delta);
    
    Xr = zbel(ind_Xr);
    Xs = zbel(ind_Xs);
    X = [Xs;Xr];
    
    % We evaluate at locations Xs and Xr:
    Yr = Fr(ind_Xr) + exp(hyp_F.lik)*randn(1,length(ind_Xr))';
    Ys = Fs(ind_Xs) + exp(hyp_sim.lik)*randn(1,length(ind_Xs))';
    Y = [Ys;Yr];
    
%% Get the posterior:

% Prepare the GP:
GP.deriv      = 0;
GP.covfunc    = {@covSEard_F};
GP.hyp        = hyp_F;
GP.x          = X;
GP.y          = Y;
GP.K          = k_matrix(GP,GP.x) + diag(GP_noise_var(GP,GP.y));
GP.cK         = chol(chol_fix(GP.K));

%% Condition:
zbel            = linspace(X_min,X_max,100)';
[Mb,Vb]         = GP_moments(GP,zbel);
    
%% Posterior of F_real:

% In order to avoid computing the kernel derivative, we have to call gp()
% with more than 7 inputs.
[mu_F,s2_F,fmu,fs2,lp] = gp(hyp_F, infer_, m_F, k_F, lik, X, Y, zbel,[],delta);

%% Plotting F_real:

fig1 = figure;
hdl_plot_F_real_post = subplot(3,1,1);
std_F = sqrt(s2_F);
f_ = [mu_F+2*std_F; flipdim(mu_F-2*std_F,1)];
fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
hold on
% Plotting the mean:
plot(zbel,mu_F,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)
plot(zbel,Fr,'Color',[240,128,128]/255,'linestyle','-.','LineWidth',1.5)
plot(zbel,Fs,'Color',[186,85,211]/255,'linestyle','-.','LineWidth',1.5)
% Plotting function evaluations:
plot(Xr,Yr,'o','MarkerFaceColor',[255,99,71]/255)
plot(Xs,Ys,'o','MarkerFaceColor',[138,43,226]/255)
title('F_{real}(x_*) = F_{sim}(x_*) + F_{err}(x_*) | F_{real}(X_r), F_{sim}(X_s)','interpreter','tex')
% plot_lim = extend_interval([min(Y) max(Y)],0.15)
% ylim(plot_lim);
fig1.Position = [520 280 1500 1000];
% fig1.Position = [507 310 750 500];
xlabel('x_*');

% Just to avoid forgetting it:
warning('TODO: The likelihood should not be the same everywhere in F_{real}');

%% Posterior of F_sim:

[mu_F_s,s2_F_s,fmu,fs2,lp] = gp(hyp_sim, infer_, m_sim, K_sim, lik, Xs, Ys, zbel,[],[]);

%% Plotting F_sim:

% fig2 = figure;
hdl_plot_F_sim_post = subplot(3,1,2);
std_F_s = sqrt(s2_F_s);
f_ = [mu_F_s+2*std_F_s; flipdim(mu_F_s-2*std_F_s,1)];
fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
hold on
% Plotting the mean:
plot(zbel,mu_F_s,'LineStyle','-','Color',[65,105,225]/255,'LineWidth',1.5)
plot(zbel,Fs,'Color',[186,85,211]/255,'linestyle','-.','LineWidth',1.5)
% Plotting function evaluations:
% plot(Xr,Yr,'o','MarkerFaceColor',[255,99,71]/255)
plot(Xs,Ys,'o','MarkerFaceColor',[138,43,226]/255)
title('F_{sim}(x_*) | F_{sim}(X_s)','interpreter','tex')
% plot_lim = extend_interval([min(Y) max(Y)],0.15)
% ylim(plot_lim);
% fig2.Position = [507 310 1500 1000];
xlabel('x_*');

%% Posterior of F_error (I):

K_err_post      = {@covSEard_err};
hyp_err_post.cov = hyp_F.cov;
hyp_err_post.lik = hyp_err.lik;
hyp_err_post.mean = hyp_err.mean;
[mu_F_err,s2_F_err,fmu,fs2,lp] = gp(hyp_err_post, infer_, m_err, K_err_post, lik, X, Y, zbel,[],delta);

% True error:
% We subtract the true functions.
true_err = Fr - Fs;

%% Computing the error mean, and an uncertainty measure (I):

hdl_plot_F_err_post = subplot(3,1,3);
hold on
std_F_err = sqrt(s2_F_err);
f_ = [mu_F_err+2*std_F_err; flipdim(mu_F_err-2*std_F_err,1)];
fill([zbel; flipdim(zbel,1)], f_, uncertainty_color)
plot(zbel,mu_F_err,'LineStyle','-','Color','k','LineWidth',1)
plot(zbel,true_err,'LineStyle','--','Color','k','LineWidth',0.5)
title('F_{err}(x_*) | F_{real}(X_r), F_{sim}(X_s)','interpreter','tex')
xlabel('x_*');
line([zbel(1) zbel(end)],[0 0],'linestyle','-','color',[100,100,100]/255,'LineWidth',0.5)

%% Re-sizing the axes:

hdl_plot_F_sim_post.YLim = hdl_plot_F_real_post.YLim;
hdl_plot_F_err_post.YLim = hdl_plot_F_real_post.YLim;

%% Posterior of F_error (II):
% Incorrect:
% We don't have measurements of F_err, but just of F_real and F_sim.
% Therefore, we can only compute the F_err posterior conditioning on the
% observations F_err(Xr) and F_sim(Xs).

% [mu_F_err,s2_F_err,fmu,fs2,lp] = gp(hyp_err, infer_, m_err, K_err, lik, X, Y, zbel,[],[]);

%% Bring to front:

figure(fig1)
