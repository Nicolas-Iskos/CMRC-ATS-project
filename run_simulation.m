%{ 
This is an example test script that uses the mex file 
simulation_routine.cpp to test the ATS Library (written in C++).

Using MATLAB test scripts like this one, it is possible for to easily set 
simulation parameters and perform detailed analysis on results.

Units:
feet, slugs, seconds
%}

% simulation parameters
%%
% [gravitational constant, air density]
phys_consts = [32. 0.00237]; 

% [initial velocity, initial altitude, initial polar angle]
init_conditions = [600. 1100. 0.];

% [inverse snr on altimeters, inverse snr on inertial measurement unit]
noise = [0.000 0.000];

% [mass, flap width, flap length, body radius, number of flaps]
vehicle_params = [1.4966 0.125 0.125 0.2570 3.];

% [base case drag coeff;
% [drag coeff contribution of jth power of flap extension;
% [drag coeff contribution of jth power of vertical velocity]
drag_model = [0.4298 0 0 0 0; 
              1.1299 -6.948 76.458 0 0; 
              0 0 0 0 0];
          
% [proportional gain, integral gain]
control_terms = [1e-5 2e-4];

% target apogee
pred_params = 5100;
%%


% calling the C++ simulation routine in simulation_routine.cpp
[pred_states, actual_states, obs_states, controls, apo_achieved] = ...
simulation_routine(phys_consts, init_conditions, ...
                   noise, ...
                   vehicle_params, drag_model, ...
                   control_terms, pred_params);
           