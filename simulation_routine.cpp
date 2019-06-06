#include "mex.h"
#include <cstdlib>
#include <sstream>
#include "stdlib.h"

#ifndef ATS_INT_H
#define ATS_INT_H
#include "ATS_int.h"
#endif

#include "LVD.h"
#include "pi_controller.h"
#include "sensor.h"

#define N_STATES_IN_FLIGHT    1000


void run_simulation_routine(double *phys_consts, double *init_conditions,
                double *noise,
                double *vehicle_params, double *drag_model,
                double *control_terms, double *pred_params,
        
                double *pred_states, double *actual_states,
                double *obs_states,double *controls,
                double *apo_achieved)
{   
    double g = phys_consts[0];
    double ro = phys_consts[1];
    
	double v0 = init_conditions[0];
    double h0 = init_conditions[1];
    double th0 = init_conditions[2];
    
    double inv_snr_alt = noise[0];
    double inv_snr_imu = noise[1];
    
    double p = control_terms[0];
    double i = control_terms[1];
    
    double apo_goal = pred_params[0];
    double dt = pred_params[1];
    
    double m = vehicle_params[0];
    double flap_width = vehicle_params[1];
    double flap_length = vehicle_params[2];
    double body_rad = vehicle_params[3];
    double n_flaps = vehicle_params[4];
    
    double drag_model_in[N_D_IMPACTORS][N_D_COEFF_TERMS];
    for(int i = 0; i < N_D_IMPACTORS; i++)
    {
        for(int j = 0; j < N_D_COEFF_TERMS; j++)
        {
            drag_model_in[i][j] = drag_model[N_D_COEFF_TERMS * i + j];
        }
    }
    
    
    
    /* ATS Library object instantiation */
    LVD r = LVD(m, flap_width, body_rad, n_flaps,
                drag_model_in,
                g, ro,
                apo_goal, dt);
    
    pi_controller c = pi_controller(p, i, flap_length);
    
    sensor s = sensor();
    
    
    
    /* simulation data initialization */
    
    /* the actual current state */
    state_t X_t = new state;
    X_t->velocity = v0;
    X_t->altitude = h0;
    X_t->theta = th0;
    
    /* the observed state one time step ago */
    state_t X_tm1o = new state;
    X_tm1o->velocity = v0;
    X_tm1o->altitude = h0;
    X_tm1o->theta = th0;
    
    /* the observed current state */
    state_t X_to = new state;
    X_to->velocity = 0;
    X_to->altitude = 0;
    X_to->theta = 0;
    
    /* the predicted state at apogee */
    state_t X_tf = new state;
    X_tf->velocity = 0;
    X_tf->altitude = 0;
    X_tf->theta = 0;
    
    /* the control */
    control_t U_t = 0;
    
    /* the controller errors */
    error_t e = new error;
    e->inst = 0;
    e->acc = 0;
    
    /* count the number of states so that we can record
     * the predictions and controls and states over the course
     * of the simulation */
    int state_counter = 0;
    
    /* At this point, the ATS simulation loop starts. It will
     continuously make predictions of apogee and enact control
     on this prediction at time */

    while(X_t->velocity > 0){
        
        /* we take advantage of the single step predict function
         * to simulate the next state of the launch vehicle
         * a time SAMPLE_T into the future
         */
        for(int i = 0; i < lround(SAMPLE_T/dt); i++){
            r.ss_predict(X_t, X_t, U_t);
        }
        
        /* build the observed state taking noise into account and
         * recognizing that velocity is calculated using the technique
         * of finite differences
         */
        X_to->altitude = 
        X_t->altitude + 
        X_t->altitude * inv_snr_alt * (2 * (double)rand()/RAND_MAX - 1);
        
        X_to->velocity = s.get_vertical_speed(X_tm1o,
        X_to->altitude, X_to->altitude, X_to->altitude, X_to->altitude);

        X_to->theta = 
        X_t->theta + 
        X_t->theta * inv_snr_imu * (2 * (double)rand()/RAND_MAX - 1);
        
        /* make a prediction of apogee based on control and observed 
         * state */
        r.ms_predict(X_to, X_tf, U_t);
        
        /* update error structure based on prediction of apogee */
        e->inst = X_tf->altitude - apo_goal;
        e->acc += e->inst;
        
        /* enact control based on predicted error on apogee */
        U_t = c.get_control(e);

        /* set past observed state for next loop iteration */
        X_tm1o->altitude = X_to->altitude;
        X_tm1o->velocity = X_to->velocity;
        X_tm1o->theta = X_to->theta;
        
        /* record control, prediction, observed state and actual state */
        
        actual_states[N_STATE_TYPES * state_counter] = X_t->velocity;
        actual_states[N_STATE_TYPES * state_counter + 1] = X_t->altitude;
        actual_states[N_STATE_TYPES * state_counter + 2] = X_t->theta;

        obs_states[N_STATE_TYPES * state_counter] = X_to->velocity;
        obs_states[N_STATE_TYPES * state_counter + 1] = X_to->altitude;
        obs_states[N_STATE_TYPES * state_counter + 2] = X_to->theta;

        pred_states[N_STATE_TYPES * state_counter] = X_tf->velocity;
        pred_states[N_STATE_TYPES * state_counter + 1] = X_tf->altitude;
        pred_states[N_STATE_TYPES * state_counter + 2] = X_tf->theta;

        controls[state_counter] = U_t;

        *apo_achieved = X_tf->altitude;
        
        state_counter++;
    }
    
    delete X_t;
    delete X_tm1o;
    delete X_to;
    
    delete e;
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    /* parse input arguments */
    double *phys_consts = mxGetDoubles(prhs[0]);
    double *init_conditions = mxGetDoubles(prhs[1]);
    double *noise = mxGetDoubles(prhs[2]);
    double *vehicle_params = mxGetDoubles(prhs[3]);
    double *drag_model = mxGetDoubles(prhs[4]);
    double *control_terms = mxGetDoubles(prhs[5]);
    double *pred_params = mxGetDoubles(prhs[6]);
    
    
    /* prepare output data */
    plhs[0] = 
          mxCreateDoubleMatrix(N_STATE_TYPES, N_STATES_IN_FLIGHT, mxREAL);
    plhs[1] = 
          mxCreateDoubleMatrix(N_STATE_TYPES, N_STATES_IN_FLIGHT, mxREAL);
    plhs[2] = 
          mxCreateDoubleMatrix(N_STATE_TYPES, N_STATES_IN_FLIGHT, mxREAL);
    plhs[3] = 
          mxCreateDoubleMatrix(1, N_STATES_IN_FLIGHT, mxREAL);
    plhs[4] = mxCreateDoubleScalar(0);
    
    double *pred_states = mxGetDoubles(plhs[0]);
    double *actual_states = mxGetDoubles(plhs[1]);
    double *obs_states = mxGetDoubles(plhs[2]);
    double *controls = mxGetDoubles(plhs[3]);
    double *apogee_achieved = mxGetDoubles(plhs[4]);
    
    /* call computational routine */
    run_simulation_routine(phys_consts, init_conditions, 
                       noise,
                       vehicle_params, drag_model,
                       control_terms, pred_params,
                       pred_states, actual_states, 
                       obs_states, controls,
                       apogee_achieved);
}




