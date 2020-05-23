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
close all; clc; clear all;

%% Preparing:

% Stochastic part (related to an integral):

% Varibales for a given query x:
dlogPdM = [

   -0.0294    0.0093    0.0095    0.0053    0.0053
    0.0092   -0.0291    0.0092    0.0054    0.0054
    0.0095    0.0093   -0.0294    0.0053    0.0053
    0.0093    0.0095    0.0093   -0.2307    0.2026
    0.0093    0.0095    0.0093    0.2026   -0.2307];

dMdx = [

    0.0000
    0.0000
    0.0000
   13.1851
   14.9983];

W = [

    0.1142    0.2299    0.3487    0.4727    0.6045    0.7478    0.9083    1.0966    1.3349    1.6901];


lmb = [

   -2.9957
   -2.9957
   -2.9957
   -2.9957
   -2.9957 ];


logP = [

   -1.4240
   -1.4151
   -1.4240
   -1.9820
   -1.9820];

%% Calculations:


stochange = (dlogPdM * dMdx) * W                   % stochastic part of change
lPred     = bsxfun(@plus,logP,stochange)      % predicted new logP
lselP     = logsumexp(lPred,1)
lPred     = bsxfun(@minus,lPred,lselP)                             % normalise
logl = @LogLoss;
dHp       = feval(logl,logP,lmb,lPred)
dH        = mean(dHp)