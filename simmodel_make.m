%% plant: create acados integrator

% integrator plant
plant_sim_method = 'irk';
plant_sim_method_num_stages = 4;
plant_sim_method_num_steps = 3;

% acados sim model

simmodel = quar_vh2at_simmodelirk();
sim_model = acados_sim_model();
sim_model.set('name', ['plant']);
sim_model.set('T', h);

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