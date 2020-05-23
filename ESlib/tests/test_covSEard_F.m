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
    close all; clear all; clc;
    
%% Test:

% Input parameters:
    D = 1;
    hyp.cov = [log([2;4]);log([2;4])];
    kerf = {@covSEard_delta};
    
% Evaluations:
%     X = [-9 -8;5 4;-3 -2;4 3];
    X = [-9;5;-3;4];
    global delta
    delta = logical([1 1 0 1]);
    Nbel = 50;
    zbel = -10 + 20*rand(Nbel,D);
    
% Before entering the kernel function, check assertions:
    display('Checking that the dimension of X is comprised in the number of columns...');
    assert(D == size(X,2)); display('OK!');
    
    display('Checking that the dimension of zbel is comprised in the number of columns...');
    assert(D == size(zbel,2)); display('OK!');
    
    display('Checking that the number of delta elements agrees with the number of rows in X...');
    assert(length(delta) == size(X,1)); display('OK!');
    

%% Check all posibilities:
    
% Acess only with hyp:
    feval(kerf{:},hyp.cov)
    
%% One input argument: get k(X,X)
    feval(kerf{:},hyp.cov,X,'k(X,X)')
    
%% One input argument: get k(zbel,zbel)
    feval(kerf{:},hyp.cov,zbel,'k(z,z)')
    
%% Two input arguments: get k(X,zbel)
    kXz = feval(kerf{:},hyp.cov,X,zbel,'k(X,z)')
    
%% Two input arguments: get k(zbel,zbel)
    feval(kerf{:},hyp.cov,zbel,zbel,'k(z,z)')

%% Two input arguments: get k(X,X)
    feval(kerf{:},hyp.cov,X,X,'k(X,X)')
    
%% Two input arguments: get k(zbel,X)
    kzX = feval(kerf{:},hyp.cov,zbel,X,'k(z,X)')
    
%% One input argument: get k(z_*,z*)
    z = -9;
    global delta_query;
    delta_query = logical(1);
    kzz = feval(kerf{:},hyp.cov,z,'k(z,z)')
    
%% One input argument: get k(z_*,X)
    z = -9;
    global delta_query;
    delta_query = logical(0);
    kzX = feval(kerf{:},hyp.cov,z,X,'k(z,X)')
    
%% One input argument: get k(X,z_*)
    z = -9;
    global delta_query;
    delta_query = logical(0);
    kzX = feval(kerf{:},hyp.cov,X,z,'k(X,z)')