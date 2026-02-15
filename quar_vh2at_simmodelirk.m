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

function model = quar_vh2at_simmodelirk()

import casadi.*

%% system dimensions
nx = 9;
nu = 4;

%% system parameters
g = 9.788; % gravity constant [m/s^2]

%% named symbolic variables
ph = SX.sym('ph');
vu = SX.sym('vu');
vv = SX.sym('vv');
vw = SX.sym('vw');

qw = SX.sym('qw');
qx = SX.sym('qx');
qy = SX.sym('qy');
qz = SX.sym('qz');
T = SX.sym('T');

wx = SX.sym('wx');
wy = SX.sym('wy');
wz = SX.sym('wz');
Tc = SX.sym('Tc');

%% (unnamed) symbolic variables
Trac = vertcat(0, 0, 0, 0,...
                0, 0, 0, 0,...
                9.788);
sym_x = vertcat(ph, vu, vv, vw, qw, qx, qy, qz, T);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(wx, wy, wz, Tc);
sym_p = [];
sym_e = sym_x - Trac;
%% dynamics
expr_f_expl = vertcat(1.0*vw,...%ph
                      1.0*(-T)*(2*qw*qy+2*qx*qz), ...%vu
                      1.0*(-T)*(2*qy*qz-2*qw*qx),...%vv
                      1*(g-T*(1 - 2*qx^2 - 2*qy^2)),...%vw
                      - (wx*qx)/2 - (wy*qy)/2 - (wz*qz)/2,...%qw
                        (qw*wx)/2 + (wz*qy)/2 - (wy*qz)/2,...%qx
                        (qw*wy)/2 - (wz*qx)/2 + (wx*qz)/2,...%qy
                        (qw*wz)/2 + (wy*qx)/2 - (wx*qy)/2,...%qz
                      1*(Tc-T));%T
expr_f_impl = expr_f_expl - sym_xdot;

%% populate structure
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_p = sym_p;
model.expr_f_impl = expr_f_impl;

end
