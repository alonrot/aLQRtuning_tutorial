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
function out = EntropySearch(in)
% probabilistic line search algorithm that adapts it search space
% stochastically, by sampling search points from their marginal probability of
% being smaller than the current best function guess.
%
% (C) Philipp Hennig & Christian Schuler, August 2011

display(' ');
display('*************************************************************************************');
display('****************************** Entropy Search Started! ******************************');
display('*************************************************************************************');
display(' '); display(' ');

%% fill in default values where possible
if ~isfield(in,'likfunc'); in.likfunc = @likGauss; end; % noise type
if ~isfield(in,'poly'); in.poly = -1; end; % polynomial mean? 
if ~isfield(in,'log'); in.log = 0; end;  % logarithmic transformed observations?
if ~isfield(in,'with_deriv'); in.with_deriv = 0; end; % derivative observations?
if ~isfield(in,'x'); in.x = []; end;  % prior observation locations
if ~isfield(in,'y'); in.y = []; end;  % prior observation values
if ~isfield(in,'T'); in.T = 200; end; % number of samples in entropy prediction
if ~isfield(in,'Ne'); in.Ne = 10; end; % number of restart points for search
if ~isfield(in,'Nb'); in.Nb = 50; end; % number of representers
if ~isfield(in,'LossFunc'); in.LossFunc = {@LogLoss}; end;
if ~isfield(in,'PropFunc'); in.PropFunc = {@EI_fun}; end;
% in.D = size(in.xmax,2); % dimensionality of inputs (search domain)

% the following should be provided by the user
%in.covfunc      = {@covSEard};       % GP kernel
%in.covfunc_dx   = {@covSEard_dx_MD}; % derivative of GP kernel. You can use
%covSEard_dx_MD and covRQard_dx_MD if you use Carl's & Hannes' covSEard,
%covRQard, respectively.
%in.hyp          = hyp;  % hyperparameters, with fields .lik and .cov
%in.xmin         = xmin; % lower bounds of rectangular search domain
%in.xmax         = xmax; % upper bounds of rectangular search domain
%in.MaxEval      = H;    % Horizon (number of evaluations allowed)
%in.f            = @(x) f(x) % handle to objective function

%% Set up some variables:

    % Set up GP:
    GP              = in.GP;

    % Not needed GP variables:
    %GP.covfunc_dxdz = in.covfunc_dxdz;
    %GP.SampleHypers = in.SampleHypers;
    %GP.HyperSamples = in.HyperSamples;
    %GP.HyperPrior   = in.HyperPrior;
    %GP.dy           = in.dy;

    % Plotting variables:
    global GP_plot;
    GP_plot = struct;
    [fig_hdl,axes_hdl] = initialize_plots(GP,in);

    % Update kXX (GP.K) and chol(kXX) (GP.cK), needed afterwards in EI_fun:
    GP = get_posterior(GP);

    % Others:
    D = in.D;
    S0= 0.5 * norm(in.xmax - in.xmin);
    

%% Entropy search loop:
converged   = false;
numiter     = 0;
MeanEsts    = zeros(0,D);
MAPEsts     = zeros(0,D);
BestGuesses = zeros(0,D);
while ~converged && (numiter < in.MaxEval)
    
    % Update the general ES loop counter:
    numiter = numiter + 1;

    display(' ');
    display('**************************************************************************************');
    display('*********************************** ENTROPY SEARCH ***********************************');
    display('--------------------------------------------------------------------------------------');
    display(['********************************* Iteration number ' num2str(numiter) ' *********************************']);
    display('**************************************************************************************');
    display(' ');

    % Get representer points:
    representers = compute_representer_points(GP,in,BestGuesses);

    % Update Best Guesses:
        logP    = representers.logP;
        lmb     = representers.lmb;
        zb      = representers.zb;
        Mb      = representers.Mb;
        Vb      = representers.Vb;

        % store the best current guess as start point for later optimization.
        [~,bli] = max(logP + lmb);

        % is this far from all the best guesses? If so, then add it in.
        % Lengthscale from siumlation:
        ell = exp(GP.hyp.cov(1:D))';
        if isempty(BestGuesses)
            BestGuesses(1,:) = zb(bli,:);
        else
            dist = min(sqrt(sum(bsxfun(@minus,zb(bli,:)./ell,bsxfun(@rdivide,BestGuesses,ell)).^2,2)./D));
            if dist > 0.1
                BestGuesses(size(BestGuesses,1)+1,:) = zb(bli,:);
            end
        end
        
    % Get current entropy (for informative purposes, not used at this level
    % of ES):
    Loss = - sum(exp(logP) .* (logP + lmb));
    H = - Loss;

    % Update GP_plot:
        GP_plot.Mb = Mb;
        GP_plot.Vb = Vb;

        GP_plot.pmin = logP + lmb;
        GP_plot.zb   = zb;
    

    % Compute most informative point, as next evaluation:
    xp = get_most_informative_point(representers,GP,in,S0);

    display(' ');
    display(['*** Selecting next point']);
    display_vec(xp,'xp');
    save('./new_point','xp');
    display('    Saved the new point!');

    % Plot some stuff:
    update_plot(fig_hdl,axes_hdl);

    % Evaluate at xp:
    yp                = in.f(xp);

    % Update GP including new measurements, and sorting them
    % according to delta.
    GP = add_measurement2GP_sorted(GP,xp,yp);
    
    % Store in GP plot AFTER including the new measurements:
    GP_plot.x = GP.x;
    GP_plot.y = GP.y;

    % Update the posterior:
    GP = get_posterior(GP);
    
    % Update plot here as well????

    MeanEsts(numiter,:) = sum(bsxfun(@times,zb,exp(logP)),1);
    [~,MAPi]            = max(logP + lmb);
    MAPEsts(numiter,:)  = zb(MAPi,:);

    fprintf('*** Finding current best guess\n')
    [out.FunEst(numiter,:),FunVEst] = FindGlobalGPMinimum(BestGuesses,GP,in.xmin,in.xmax);
    % is the new point very close to one of the best guesses?
    [cv,ci] = min(sum(bsxfun(@minus,out.FunEst(numiter,:)./ell,bsxfun(@rdivide,BestGuesses,ell)).^2,2)./D);
    if cv < 2.5e-1 % yes. Replace it with this improved guess
        BestGuesses(ci,:)  = out.FunEst(numiter,:);
    else % no. Add it to the best guesses
        BestGuesses(size(BestGuesses,1)+1,:) = out.FunEst(numiter,:);
    end
    
    GP_plot.x_globalmin = out.FunEst(numiter,:);
    
    % Plot some stuff:
    update_plot(fig_hdl,axes_hdl,'only_global_minimum');

    % Optimize hyperparameters:
    if in.LearnHypers
        minimizeopts.length    = 10;
        minimizeopts.verbosity = 1;
        GP.hyp = minimize(GP.hyp,@(hyp_learn)in.HyperPrior(hyp_learn,GP.x,GP.y,GP.covfunc,[],GP.likfunc),minimizeopts);
        
        % Update the posterior:
        GP = get_posterior(GP);
        
    end

    % Update output structure:
    out.GPs{numiter}            = GP;
    out.representers{numiter}   = representers;
    out.GP_plots{numiter}       = GP_plot;
    out.H{numiter}              = H;
    out.specs                   = in;

    % Saving a copy of the up-to-date iteration just in case something goes wrong:
    save('currentESstatus','out');
    display('*** Security saving: current out structure saved in currentESstatus.mat!');

    if in.ask_user
        converged = ask_user(numiter);
    end
    
end

display(' ');
display('*******************************************************************************************************');
display('****************************** Entropy Search has finished sucessfully!! ******************************');
display('*******************************************************************************************************');
display(' '); display(' ');

end