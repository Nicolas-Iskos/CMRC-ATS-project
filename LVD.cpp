/**
 * This file contains the implementation of the launch vehicle dynamics model (LVD).
 * It contains functions that are used to predict the apogee of the launch vehicle
 * given a current state including altitude, vertical speed and attitude. 
 *
 * X_t : current state including altitude, vertical velocity and attitude
 * X_tp : predicted state at some time into the future
 * U_t : current flap extension
 *
 * Units:
 * feet, slugs, seconds
 * 
 */

#include "LVD.h"
#include <math.h>

LVD::LVD(){
    // constant system parameters
    m = 1.4966;
    fin_wid = 0.125;
    rocket_rad = 0.2570;

    // constant physical parameters
    g = 32.;
    ro = 0.00237; // slugs/ft3

    // constant general prediction parameters
    apo_goal = 5100.;
    dt = 0.104;

    w0 = 0.4298;
    wd1 = 1.1299;
    wd2 = -6.948;
    wd3 = 76.458;
}

double LVD::compute_cd(state_t X_t, double U_t){
    double v1 = X_t->velocity;
    return w0 + wd1 * pow(U_t,2) + wd2 * pow(U_t,3) + wd3 * pow(U_t,4);
}

double LVD::compute_fd(state_t X_t, double U_t){
    double area = compute_area(U_t);
    
    double vy = X_t->velocity;
    double vy2 = pow(vy,2);

    double theta = X_t->theta;
    double vx2 = pow(vy*tan(theta),2); 
    

    double v2 = vx2 + vy2;

    return compute_cd(X_t, U_t) * 0.5 * ro * area * v2;
}

double LVD::compute_area(double U_t){
    double area_flaps = 3 * fin_wid * U_t;
    double area_rocket = M_PI * rocket_rad * rocket_rad;
    return area_flaps + area_rocket;
}

void LVD::ss_predict(state_t X_t, state_t X_tp, double U_t){
    double theta = X_t->theta;
    double vy = X_t->velocity;
    double vx = vy * tan(theta);
    double h = X_t->altitude;
    
    double dh = vy * dt;

    double fd = compute_fd(X_t,U_t);

    // include force of drag and gravity
    double dvy = (-g - cos(theta) * fd/m) * dt;

    // include force of drag
    double dvx = (-sin(theta) * fd/m) * dt;

    double vy_p = vy + dvy;
    double vx_p = vx + dvx;

    X_tp->velocity = vy_p;
    X_tp->altitude = h + dh;
    X_tp->theta = atan(vx_p/vy_p);
}

void LVD::ms_predict(state_t X_t, state_t X_tp, double U_t){
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
