/**
 * This file contains the implementation of the launch vehicle dynamics model (LVD).
 * It contains functions that are used to predict the apogee of the launch vehicle
 * given a current state including altitude, vertical speed and attitude. 
 *
 * X_t : current state including altitude, vertical velocity and attitude
 * X_tp : predicted state at some time into the future
 * U_t : current flap extension
 * 
 */

#include "LVD.h"


LVD::LVD(double m_in, double flap_width_in, double body_rad_in, double n_flaps_in,
         double drag_model_in[][N_D_COEFF_TERMS],
         double g_in, double ro_in,
         double apo_goal_in)
{
    m = m_in;
    flap_width = flap_width_in;
    body_rad = body_rad_in;
    n_flaps = n_flaps_in;

    for(int i = 0; i < N_D_COEFF_TERMS; i++)
    {
        for(int j = 0; j < N_D_IMPACTORS; j++)
        {
            drag_model[i][j] = drag_model_in[i][j];
        }
    }

    g = g_in;
    ro = ro_in;

    apo_goal = apo_goal_in;
}

double LVD::compute_cd(state_t X_t, control_t U_t){
    double v1 = X_t->velocity;
    control_t u1 = U_t;

    double u_powers[N_D_COEFF_TERMS];
    double v_powers[N_D_COEFF_TERMS];

    for(int i = 0; i < N_D_COEFF_TERMS; i++)
    {
        u_powers[i] = pow(u1,i+1);
        v_powers[i] = pow(v1,i+1);
    }

    return
    drag_model[STATIC_COEFF][0] + 
    d_coeff_dot_product(drag_model[FLAP_COEFF], u_powers) +
    d_coeff_dot_product(drag_model[V_COEFF], v_powers);
}

double LVD::compute_fd(state_t X_t, control_t U_t){
    double area = compute_area(U_t);
    
    double vy = X_t->velocity;
    double vy2 = pow(vy,2);

    double theta = X_t->theta;
    double vx2 = pow(vy*tan(theta),2); 
    

    double v2 = vx2 + vy2;

    return compute_cd(X_t, U_t) * 0.5 * ro * area * v2;
}

double LVD::compute_area(control_t U_t){
    double area_flaps = n_flaps * flap_width * U_t;
    double area_rocket = M_PI * pow(body_rad,2);
    return area_flaps + area_rocket;
}

void LVD::ss_predict(state_t X_t, state_t X_tp, control_t U_t){
    double theta = X_t->theta;
    double vy = X_t->velocity;
    double vx = vy * tan(theta);
    double h = X_t->altitude;
    
    double dh = vy * DT;

    double fd = compute_fd(X_t,U_t);

    // include force of drag and gravity
    double dvy = (-g - cos(theta) * fd/m) * DT;

    // include force of drag
    double dvx = (-sin(theta) * fd/m) * DT;

    double vy_p = vy + dvy;
    double vx_p = vx + dvx;

    X_tp->velocity = vy_p;
    X_tp->altitude = h + dh;
    X_tp->theta = atan(vx_p/vy_p);
}

void LVD::ms_predict(state_t X_t, state_t X_tp, control_t U_t){
    X_tp->velocity = X_t->velocity;
    X_tp->altitude = X_t->altitude;
    X_tp->theta = X_t->theta;

    /** 
     * use Euler's method to predict the state of the launch
     * vehicle at apogee
     */
    while(X_tp->velocity > 0){
        ss_predict(X_tp, X_tp, U_t);
    }
}

double LVD::get_apo_goal(){
    return apo_goal;
}

double LVD::d_coeff_dot_product(double coeffs[], double powers[])
{
    double result = 0;

    for(int i = 0; i < N_D_COEFF_TERMS; i++)
    {
        result += coeffs[i] * powers[i];
    }

    return result;
}



