%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%

%% test of native matlab interface
clear all
clc

model_path = fullfile(pwd,'...','pendulum_on_cart_model');
addpath(model_path)

check_acados_requirements()

% set_aNE=[-0.5,-0.5;...
%                 0.0,0.0];
% set_aNE=[-0.4,-0.5;...
%                 0.4,-0.5];
% set_aNE=[-0.4,-0.3;...
%                 0.3,0.4];
set_aNE=[-0.4,-0.5;...
                0.4,-0.5];
% initial state
x0 =vertcat(0, 0, 0, 0,...
                set_aNE(1,1), set_aNE(1,2), 9.788); % start at stable position
u0 = [0; 0; 0];  % start at stable position

%% discretization
h = 0.001; % sampling time = length of first shooting interval

ctrl_aN=[zeros(1,0),ones(1,600),ones(1,2000)]*set_aNE(2,1);%
ctrl_aE=[zeros(1,0),ones(1,600),ones(1,2000)]*set_aNE(2,2);
ctrl_qwx=-ctrl_aE./2;
ctrl_qwy=ctrl_aN./2;
ctrl_dyaw=[zeros(1,1000),zeros(1,600),zeros(1,2350)]*pi*60/180;%[zeros(1,200)];

N_sim = 2000;
ts=(0:N_sim)*h;
omega_max=pi/2;%change in Fo
angle_max=pi/9*2;

Fo();
simmodel_make();


% ctrl_yaw=pi;
% ANGCTRL="euler";%%own,euler,quart
% mainrun_real();


ctrl_yaw=pi/12*12;
ANGCTRL="euler";%%own,euler,quart
mainrun_real();
ANGCTRL="quart";%%own,euler,quart
mainrun_real();
ANGCTRL="own";%%own,euler,quart
mainrun_real();



% ANGCTRL="euler";%%own,euler,quart
% for ctrl_yaw=pi/36:pi/36:pi%pi/2;15,22/36
%     mainrun_real();
% end

% figure(7); 
%     subplot(2, 1, 1);
%     hold on;plot(ts, Fo_aN,'k--');
%     subplot(2, 1, 2);
%     hold on;plot(ts, Fo_aE,'k--');
% 
% figure(8); 
%     hold on;plot(Fo_aN,Fo_aE,'k--');grid on;
%     hold off;

%First-Order
%Proposed
%Quart
%Euler_{worst}

%100,100,600,600