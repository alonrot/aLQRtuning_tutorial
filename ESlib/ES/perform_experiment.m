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
function [Y,U] = perform_experiment(F_n,noise,models)
  
    % Initialization:
    tF_ = models.tF_;
    Ts = models.Ts;
    n = size(models.Ar_d,2);
    
    % Noise computation:
        % Measurements noise  (i.i.d with variance V_n):
            if noise.noisy_states_measurement
                v = sqrt(diag([0 2.5e-6]))*randn(2,tF_/Ts+1); 
            else
                v = zeros(size(models.Ar_d,2),tF_/Ts+1);
            end
        
        % Noise at the input (process noise):
            if noise.process_noise
                w = models.Br_d*sqrt(noise.sigma_n2)*randn(1,tF_/Ts+1);
            else
                w = zeros(size(models.Ar_d,2),tF_/Ts+1);
            end
            
    % Output and input signals computation:
        % Closed-loop system, with the real model:
        A_c = models.Ar_d - models.Br_d*F_n;
        B_c = [eye(n) -models.Br_d*F_n];
        C_c = [eye(n);-F_n];
        D_c = [zeros(n) eye(n);zeros(1,n) -F_n];
        sys_c = ss(A_c, B_c, C_c, D_c, Ts);
        
        % Simulation of the real experiment:
        y_sim = lsim(sys_c,[w', v'],0:Ts:tF_,models.X_i);
        
        % Output and input:
        Y = y_sim(:,1:n)';
        U = y_sim(:,n+1)';
    
end