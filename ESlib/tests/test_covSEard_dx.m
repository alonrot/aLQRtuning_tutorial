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
    run('./gpml-matlab-v3.1-2010-09-27/startup.m');
    addpath './utils/';
    addpath './tests/';
    
%% Test:

% Input parameters:
    kerf = {@covSEard_delta};
    
% Evaluations:
    X = [-9 -8;5 4;-3 -2;4 3];
    D = size(X,2);
    hyp.cov = [log([3;1;3]);log([3;1;3])];
%     X = [-9;5;-3;4];
%     hyp.cov = [log([1;2]);log([1;2])];
    global delta
    delta = logical([0 0 0 0]);
    Nbel = 6;
    zbel = -10 + 20*rand(Nbel,D);
    
% Before entering the kernel function, check assertions:
    display('Checking that the dimension of X is comprised in the number of columns...');
    assert(D == size(X,2)); display('OK!');
    
    display('Checking that the dimension of zbel is comprised in the number of columns...');
    assert(D == size(zbel,2)); display('OK!');
    
    display('Checking that the number of delta elements agrees with the number of rows in X...');
    assert(length(delta) == size(X,1)); display('OK!');


%% Test k(z,X):

% Call to covSEard_dx_MD():
    kerd = {@covSEard_dx_MD};
    hypd.cov = log([3;1;3]);
    Kd_zX = feval(kerd{:},hypd.cov,zbel,X);
    
% Call to get_der_block():
    ker = {@covSEard_delta};
    ell = [3;1];
    K0_zX = feval(ker{:},hyp.cov,zbel,X,'k(z,X)');
    K_zX = get_der_block(zbel,X,K0_zX,ell);
    
% Call to covSEard_dx_MD_delta():
    kerd_delta = {@covSEard_dx_MD_delta};
    Kd_delta_zX = feval(kerd_delta{:},hyp.cov,zbel,X,'k(z,X)');
    
    Kd_zX(:,:,1)
    K_zX(:,:,1)
    Kd_delta_zX(:,:,1)
    
%% Test k(X,z):

% Call to covSEard_dx_MD():
    kerd = {@covSEard_dx_MD};
    hypd.cov = log([3;1;3]);
    Kd_Xz = feval(kerd{:},hypd.cov,X,zbel);
    
% Call to get_der_block():
    ker = {@covSEard_delta};
    ell = [3;1];
    K0_Xz = feval(ker{:},hyp.cov,X,zbel,'k(X,z)');
    K_Xz = get_der_block(X,zbel,K0_Xz,ell);
    
% Call to covSEard_dx_MD_delta():
    kerd_delta = {@covSEard_dx_MD_delta};
    Kd_delta_Xz = feval(kerd_delta{:},hyp.cov,X,zbel,'k(X,z)');
    
    Kd_Xz(:,:,1)
    K_Xz(:,:,1)
    Kd_delta_Xz(:,:,1)
    
    
%% Test k(X,X):

% Call to covSEard_dx_MD():
    kerd = {@covSEard_dx_MD};
    hypd.cov = log([3;1;3]);
    Kd_XX = feval(kerd{:},hypd.cov,X,X);
    
% Call to get_der_block():
    ker = {@covSEard_delta};
    ell = [3;1];
    K0_XX = feval(ker{:},hyp.cov,X,X,'k(X,X)');
    K_XX = get_der_block(X,X,K0_XX,ell);
    
% Call to covSEard_dx_MD_delta():
    kerd_delta = {@covSEard_dx_MD_delta};
    Kd_delta_XX = feval(kerd_delta{:},hyp.cov,X,X,'k(X,X)');
    
    Kd_XX(:,:,1)
    K_XX(:,:,1)
    Kd_delta_XX(:,:,1)
    
%% Test k(z,z):

% Call to covSEard_dx_MD():
    kerd = {@covSEard_dx_MD};
    hypd.cov = log([3;1;3]);
    Kd_zz = feval(kerd{:},hypd.cov,zbel,zbel);
    
% Call to get_der_block():
    ker = {@covSEard_delta};
    ell = [3;1];
    K0_zz = feval(ker{:},hyp.cov,zbel,zbel,'k(z,z)');
    K_zz = get_der_block(zbel,zbel,K0_zz,ell);
    
% Call to covSEard_dx_MD_delta():
    kerd_delta = {@covSEard_dx_MD_delta};
    Kd_delta_zz = feval(kerd_delta{:},hyp.cov,zbel,zbel,'k(z,z)');
    
    Kd_zz(:,:,1)
    K_zz(:,:,1)
    Kd_delta_zz(:,:,1)
    
%% Test k(x*,x*):

x_ = [-9.5 -4.5];

% Call to covSEard_dx_MD():
    kerd = {@covSEard_dx_MD};
    hypd.cov = log([3;1;3]);
    Kd_zz = feval(kerd{:},hypd.cov,x_,x_);
    
% Call to get_der_block():
    ker = {@covSEard_delta};
    ell = [3;1];
    K0_zz = feval(ker{:},hyp.cov,x_,x_,'k(z,z)');
    K_zz = get_der_block(x_,x_,K0_zz,ell);
    
% Call to covSEard_dx_MD_delta():
    kerd_delta = {@covSEard_dx_MD_delta};
    Kd_delta_zz = feval(kerd_delta{:},hyp.cov,x_,x_,'k(z,z)');
    
    Kd_zz(:,:,1)
    K_zz(:,:,1)
    Kd_delta_zz(:,:,1)