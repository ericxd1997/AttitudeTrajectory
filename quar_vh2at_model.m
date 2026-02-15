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

function model = quar_vh2at_model()

import casadi.*

%% system dimensions
nx = 10;
nu = 5;

%% system parameters
g = 9.788; % gravity constant [m/s^2]

%% named symbolic variables
ph = SX.sym('ph');
vu = SX.sym('vu');
vv = SX.sym('vv');
vw = SX.sym('vw');

qwx = SX.sym('qwx');
qwy = SX.sym('qwy');
T = SX.sym('T');

qwxd = SX.sym('qwxd');
qwyd = SX.sym('qwyd');
Tc = SX.sym('Tc');

dqwx = SX.sym('dwx');
dqwy = SX.sym('dwy');
dTc = SX.sym('Tc');

vw_max = SX.sym('vw_max');
vw_min = SX.sym('vw_min');

angle = (qwx)^2+(qwy)^2;
angle_c = (qwxd)^2+(qwyd)^2;
angle_rate =  (dqwx)^2+(dqwy)^2;
%% (unnamed) symbolic variables
% Trac = vertcat(0, 0, 0, 0,...
%                 0, 0, 9.788,...
%                 0, 0, 9.788);
% Uc = vertcat(0, 0, 0);
sym_x = vertcat(ph, vu, vv, vw, qwx, qwy, T, qwxd, qwyd, Tc);
sym_xdot = SX.sym('xdot', nx, 1);
sym_p = [];%vertcat(vw_max);
sym_u = vertcat(dqwx,dqwy,dTc,vw_max,vw_min);
sym_e = sym_x;% - Trac;
sym_ue = sym_u;% - Uc;
%% dynamics
expr_f_expl = vertcat(1.0*vw,...%ph
                      1.0*(-T)*(2*qwy), ...%vu
                      1.0*(-T)*(-2*qwx),...%vv
                      1*(g-T*sqrt(1 - 4*qwx^2 - 4*qwy^2)),...%vw
                      5*(qwxd-qwx),...%qwx
                      5*(qwyd-qwy),...%qwy
                      1*(Tc-T),...%T
                      dqwx,...
                      dqwy,...
                      dTc);
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints
expr_h = vertcat(vu, vv, vw-vw_max+vw_min, qwx, qwy, T, qwxd, qwyd, Tc, dqwx, dqwy, dTc,vw_max,vw_min);
% expr_h = vertcat(vu, vv, vw, qwx, qwy, angle, T, qwxd, qwyd, angle_c, Tc, angle_rate, dTc);
expr_h_e = vertcat(vu, vv, vw, qwx, qwy, angle, T, qwxd, qwyd, angle_c, Tc);
% expr_h = vertcat(vu, vv, vw, angle, T, angle_c, Tc, angle_rate, dTc);
% expr_h_e = vertcat(vu, vv, vw, angle, T, angle_c, Tc);
% expr_h = vertcat(vu, vv, vw, (qwx^2+qwy^2), T, (qwxd^2+qwyd^2), Tc, (dqwx^2+dqwy^2), dTc);
% expr_h_e = vertcat(vu, vv, vw, (qwx^2+qwy^2), T, (qwxd^2+qwyd^2), Tc);

%% cost
W_x = diag([4e2, 4e2, 4e2, 4e2,...
    1e-6, 1e-6, 1e-6,...
    1e-6, 1e-6, 1e-6]);
W_u = diag([1e1, 1e1, 1e1, 1e5, 1e5]);

expr_ext_cost_e = sym_e'* W_x * sym_e;
expr_ext_cost = expr_ext_cost_e + sym_ue' * W_u * sym_ue;
% nonlinear least sqares
cost_expr_y = vertcat(sym_e, sym_ue);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_e;
model.W_e = W_x;



%% populate structure
model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_p = sym_p;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_h_e = expr_h_e;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;

model.cost_expr_y = cost_expr_y;
model.W = W;

end
