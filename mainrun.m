% integrator model
model_sim_method = 'erk';
model_sim_method_num_stages = 4;
model_sim_method_num_steps = 3;

% integrator plant
plant_sim_method = 'irk';
plant_sim_method_num_stages = 4;
plant_sim_method_num_steps = 3;

%% model dynamics
model = quar_vh2at_model;
nx = model.nx;
nu = model.nu;

%% model to create the solver
ocp_model = acados_ocp_model();
model_name = 'nmpc_model';

%% acados ocp model
ocp_model.set('name', model_name);
ocp_model.set('T', T);
% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
% ocp_model.set('sym_p', model.sym_p);
ocp_model.set('sym_xdot', model.sym_xdot);

% nonlinear-least squares cost
ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e', model.expr_ext_cost_e);

% dynamics
if (strcmp(model_sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else % irk irk_gnsf
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end

% constraints
ocp_model.set('constr_type', 'auto');
ocp_model.set('constr_expr_h', model.expr_h);
% ocp_model.set('constr_expr_h_e', model.expr_h_e);

angle_max=cos(10/360*pi)*sin(10/360*pi);
angle_rate_max=10/180*pi;
U_min = [-20, -20,   -1, -angle_max, -angle_max,  7.3, -angle_max, -angle_max,  7.8,-angle_rate_max,-angle_rate_max, -10, 0, 0];
U_max = [ 20,  20,  0.5,  angle_max,  angle_max, 12.3,  angle_max,  angle_max, 11.8, angle_rate_max, angle_rate_max,  10, 2, 2];
ocp_model.set('constr_lh', U_min); % lower bound on h
ocp_model.set('constr_uh', U_max);  % upper bound on h
% Ue_min = U_min(1:11);
% Ue_max = U_max(1:11);
% ocp_model.set('constr_lh_e', Ue_min); % lower bound on h
% ocp_model.set('constr_uh_e', Ue_max);  % upper bound on h

ocp_model.set('constr_x0', x0);

%% acados ocp set opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('shooting_nodes', shooting_nodes);

ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('nlp_solver_max_iter', 10);%nu
stop_cr=1e-5;
ocp_opts.set('nlp_solver_tol_stat', stop_cr);
ocp_opts.set('nlp_solver_tol_eq', stop_cr);
ocp_opts.set('nlp_solver_tol_ineq', stop_cr);
ocp_opts.set('nlp_solver_tol_comp', stop_cr);
ocp_opts.set('sim_method', model_sim_method);
ocp_opts.set('sim_method_num_stages', model_sim_method_num_stages);
ocp_opts.set('sim_method_num_steps', model_sim_method_num_steps);

ocp_opts.set('sim_method_newton_iter', 3);

ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_iter_max', 20);%nx
% ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('nlp_solver_tol_stat', 1e-5);
ocp_opts.set('nlp_solver_tol_eq', 1e-5);
ocp_opts.set('nlp_solver_tol_ineq', 1e-5);
ocp_opts.set('nlp_solver_tol_comp', 1e-5);
% ... see ocp_opts.opts_struct to see what other fields can be set

%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);

x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu, N);


%% plant: create acados integrator
% acados sim model

simmodel = quar_vh2at_simmodelirk();
sim_model = acados_sim_model();
sim_model.set('name', [model_name '_plant']);
sim_model.set('T', h/Kh_c);

sim_model.set('sym_x', simmodel.sym_x);
sim_model.set('sym_u', simmodel.sym_u);
% sim_model.set('sym_p', simmodel.sym_p);
sim_model.set('sym_xdot', simmodel.sym_xdot);
sim_model.set('dyn_type', 'implicit');
sim_model.set('dyn_expr_f', simmodel.expr_f_impl);

% acados sim opts
sim_opts = acados_sim_opts();
sim_opts.set('method', plant_sim_method);
sim_opts.set('num_stages', plant_sim_method_num_stages);
sim_opts.set('num_steps', plant_sim_method_num_steps);

sim = acados_sim(sim_model, sim_opts);

%% Simulation

x_sim = zeros(max(size(simmodel.sym_x)), N_sim+1);
u_sim = zeros(max(size(simmodel.sym_u)), N_sim);

% time-variant reference: move the cart with constant velocity while
% keeping the pendulum in upwards position
yref = vertcat(0, 0, 0, 0,...
                0, 0, 9.788,...
                0, 0, 9.788,...
                0, 0, 0, 0, 0)*ones(1,N_sim);
yref_e = vertcat(0, 0, 0, 0,...
                0, 0, 9.788,...
                0, 0, 9.788)*ones(1,N_sim);

ctrl_y=[0;0;0];
ctrl_duv=[0;0];
for i=2:N_sim
    ctrl_yaw(i)=ctrl_yaw(i-1)+ctrl_dyaw(i-1)*0.1;
    ctrl_duv(:,i)=[cos(ctrl_yaw(i)),-sin(ctrl_yaw(i));sin(ctrl_yaw(i)),cos(ctrl_yaw(i))]*[ctrl_bx(i-1);ctrl_by(i-1)];
    ctrl_y(:,i)=[ctrl_y(1,i-1);0;0]+[ctrl_dh(i-1);0;0]*0.1+[0;ctrl_duv(1,i);ctrl_duv(2,i)];
end

yref = yref+[ctrl_y;ctrl_dh;zeros(11,N_sim)];
yref_e = yref_e+[ctrl_y;ctrl_dh;zeros(6,N_sim)];

ts=[0];
u0r=x0;
x_opc=x0;

for i=1:1
    % update initial state
     x0m(:,i)=x0;
    ocp.set('constr_x0', x0);

    % compute reference position on the nonuniform grid
    t  = (i-1)*h;
    ts = [ts,t];

    for k=0:N-1
        ocp.set('cost_y_ref', yref(:,i), k);
    end
    ocp.set('cost_y_ref_e', yref_e(:,i), N);

    % solve
        ocp.solve();
        % get solution
        status = ocp.get('status'); % 0 - success
        i
        ocp.print('stat')

    % set initial state
    [w,x,y,z]=qwxyz2wxyz(x0(5,i),x0(6,i),yaw2bz(ctrl_yaw(i)));
    sim_x=[x0(1:4,i);w;x;y;z;x0(7,i)];
    sim.set('x', sim_x);
    sim_u=angleT_controller(x0(8:10,i),[w,x,y,z]);%angle_controller(x_sim(8:10,i),[w,x,y,z]);
    sim.set('u', sim_u);
    
    x_sim(:,i)=sim_x;
    % solve
    sim_status = sim.solve();
    if sim_status ~= 0
        disp(['acados integrator returned error status ', num2str(sim_status)])
    end

    qp_iter(i)=ocp.get('qp_iter');
    cost_iter(i)=ocp.get_cost;
    uocp(:,i)=ocp.get('u',1);
    sqp_iter(i)=ocp.get('sqp_iter') ;
    xl = ocp.get('x', 1);
    x_opc(:,i+1)=xl;
    % get simulated state
    x_sim(:,i) = sim.get('xn');
    u_sim(:,i) =  sim_u;
end

for i=2:N_sim
    
    x0m(:,i) = [sim_x(1:4);rotorq(sim_x(5:8));sim_x(9);xl(end-2:end)];
%     x0s = sim_x;
    ocp.set('constr_x0',  x0m(:,i));

    % compute reference position on the nonuniform grid
    t = (i-1)*h;
    ts=[ts,t];


    for k=0:N-1

        ocp.set('cost_y_ref', yref(:,i), k);
    end
    ocp.set('cost_y_ref_e', yref_e(:,i), N);
    
        % solve
        ocp.solve();
        % get solution
        status = ocp.get('status'); % 0 - success
        i
        ocp.print('stat')
        
        
%     set initial state
    x_sim(:,i)=sim_x;
    for j=1:Kh_c
        sim.set('x', sim_x);
        sim_u=angleT_controller(x0m(8:10,i),sim_x(5:8))+[0,0,ctrl_dyaw(i),0];%angle_controller(x_sim(8:10,i),[w,x,y,z]);
        sim.set('u', sim_u);
        
        sim_status = sim.solve();
        if sim_status ~= 0
            disp(['acados integrator returned error status ', num2str(sim_status)])
        end
        
        sim_x = sim.get('xn');
    end
    u_sim(:,i) = sim_u;

    % solve


    qp_iter(i)=ocp.get('qp_iter');
    sqp_iter(i)=ocp.get('sqp_iter');
    uocp(:,i)=ocp.get('u',1);
    cost_iter(i)=ocp.get_cost;
    xl = ocp.get('x', 1);
    x_opc(:,i+1)=xl;
    % get simulated state
    x_sim(:,i+1) = sim.get('xn');
    u_sim(:,i) = sim_u;
end
%% Plots
figure(1); 
States = {'h','u', 'v', 'w'};
y_ref = [yref,yref(:,end)];
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, x_sim(i,:)); grid on;hold on;
    plot(ts, x_opc(i,:));
    plot(ts, y_ref(i, :)); hold off;
    ylabel(States{i});
    xlabel('t [s]')
%     legend('closed-loop', 'reference')
    hold off;
end

figure(2); 
States = {'qwx', 'qwy'};
x_sim_qxy=rotorq(x_sim(5:8,:));
for i=1:length(States)
    subplot(length(States), 1, i);
        plot(ts, x_sim_qxy(i,:)); grid on;hold on;
        plot(ts, x_opc(i+7,:));
        plot(ts, x_opc(i+4,:)); hold off;
    ylabel(States{i});
    xlabel('t [s]')
%     legend('closed-loop')
    hold off;
end

figure(3); 
States = {'T'};
for i=1:length(States)
    subplot(length(States), 1, i);
        plot(ts, x_sim(i+8,:)); grid on;hold on;
        plot(ts, x_opc(i+9,:)); hold off;
    ylabel(States{i});
    xlabel('t [s]')
%     legend('closed-loop')
    hold off;
end

figure(4); 
States = {'dqwx', 'dqwy','dTc'};
for i=1:length(States)
    subplot(length(States), 1, i);
    plot(ts, [uocp(i,:) uocp(i,end)]); grid on;
    ylabel(States{i});
    xlabel('t [s]')
%     legend('closed-loop')
    hold off;
end

% figure(3),
% subplot(2, 1, 1);
% plot(ts,[uocp(4,1),uocp(4,:)]);hold on;
% plot(ts, x_sim(4,:)-0.5); hold off;
% subplot(2, 1, 2);
% plot(ts,-[uocp(5,1),uocp(5,:)]);hold on;
% plot(ts, x_sim(4,:)+1); hold off;

        
figure(5),plot(cost_iter)
% plot(ts,[qp_iter(1),qp_iter]);hold on;