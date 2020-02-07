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

    /* initialize drag model */
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

double LVD::compute_cd(state_t X_t, control_t U_t)
{
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

double LVD::compute_fd(state_t X_t, control_t U_t)
{
    double area = compute_area(U_t);
    
    double vy = X_t->velocity;
    double vy2 = pow(vy,2);

    double theta = X_t->theta;
    double vx2 = pow(vy*tan(theta),2); 
    

    double v2 = vx2 + vy2;

    return compute_cd(X_t, U_t) * 0.5 * ro * area * v2;
}

double LVD::compute_area(control_t U_t)
{
    double area_flaps = n_flaps * flap_width * U_t;
    double area_rocket = M_PI * pow(body_rad,2);
    return area_flaps + area_rocket;
}

void LVD::ss_predict(state_t X_t, state_t X_tp, control_t U_t)
{
    double theta = X_t->theta;
    double vy = X_t->velocity;
    double vx = vy * tan(theta);
    double h = X_t->altitude;
    
    double fd0 = compute_fd(X_t,U_t);
    double ay0 = (g - cos(theta) * fd0/m);
    double ax0 = (-sin(theta) * fd0/m);

    double dvy0 = ay0 * DT;
    double dvx0 = ax0 * DT;
    
    state_t X_t_temp1 = new state;
    X_t_temp1->velocity = vy + 0.5 * dvy0;
    X_t_temp1->altitude = 0;
    X_t_temp1->theta = atan((vx + 0.5 * dvx0)/(vy + 0.5 * dvy0));
    
    double theta1 = X_t_temp1->theta;
    double fd1 = compute_fd(X_t_temp1, U_t);
    double ay1 = g - cos(theta1) * (fd1/m);
    double ax1 = -sin(theta1) * (fd1/m);

    double dvy1 = ay1 * DT;
    double dvx1 = ax1 * DT;
    
    state_t X_t_temp2 = new state;
    X_t_temp2->velocity = vy + 0.5 * dvy1;
    X_t_temp2->altitude = 0;
    X_t_temp2->theta = atan((vx + 0.5 * dvx1)/(vy + 0.5 * dvy1));;

    double theta2 = X_t_temp2->theta;
    double fd2 = compute_fd(X_t_temp2, U_t);
    double ay2 = g - cos(theta2) * (fd2/m);
    double ax2 = -sin(theta2) * (fd2/m);

    double dvy2 = ay2 * DT;
    double dvx2 = ax2 * DT;
    
    state_t X_t_temp3 = new state;
    X_t_temp3->velocity = vy + dvy2;
    X_t_temp3->altitude = 0;
    X_t_temp3->theta = atan((vx + dvx2)/(vy + dvy2));;
    
    double theta3 = X_t_temp3->theta;
    double fd3 = compute_fd(X_t_temp3, U_t);
    double ay3 = g - cos(theta3) * (fd3/m);
    double ax3 = -sin(theta3) * (fd3/m);
    
    double dvy3 = ay3 * DT;
    double dvx3 = ax3 * DT;
    
    double dvyp = (1./6) * (dvy0 + 2*dvy1 + 2*dvy2 + dvy3);
    double dvxp = (1./6) * (dvx0 + 2*dvx1 + 2*dvx2 + dvx3);
        
    double vyp = vy + dvyp;
    double vxp = vx + dvxp;
    double hp = h + 0.5 * (vy + vyp) * DT;

    X_tp->velocity = vyp;
    X_tp->altitude = hp;
    X_tp->theta = atan((vx + dvxp)/(vy + dvyp));;
    
    delete X_t_temp1;
    delete X_t_temp2;
    delete X_t_temp3;
}

void LVD::ms_predict(state_t X_t, state_t X_tpf, control_t U_t)
{
    X_tpf->velocity = X_t->velocity;
    X_tpf->altitude = X_t->altitude;
    X_tpf->theta = X_t->theta;

    /** 
     * use Euler's method to predict the state of the launch
     * vehicle at apogee
     */
    while(X_tpf->velocity > 0)
    {
        ss_predict(X_tpf, X_tpf, U_t);
    }
}

double LVD::get_apo_goal()
{
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