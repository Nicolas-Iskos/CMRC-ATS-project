phys_consts = [32. 0.00237];
init_conditions = [600. 1100. 0.];
noise = [0.001 0.001];
vehicle_params = [1.4966 0.125 0.125 0.2570 3.];
drag_model = [0.4298 0 0 0 0; 
              1.1299 -6.948 76.458 0 0; 
              0 0 0 0 0];
control_terms = [1e-5 2e-4];
pred_params = [5100. 0.0208];




[pred_states, actual_states, obs_states, controls, apo_achieved] = ...
simulation_routine(phys_consts, init_conditions, ...
                   noise, ...
                   vehicle_params, drag_model, ...
                   control_terms, pred_params);
           