/**
 * This file contains the declarations for the LVD (launch vehicle dynamics) class. 
 * An LVD object contains a number of constant properties intrinsic to the
 * launch vehicle as well as functions that can be used to predict
 * a future state given a current state
 */

#ifndef LVD_H
#define LVD_H

#ifndef ATS_INT_H
#define ATS_INT_H
#include "ATS_int.h"
#endif


class LVD
{

private:
    double m, flap_width, body_rad, n_flaps;
    /* gravitational constant, air density */
    double g, ro;
    double apo_goal;
          
    /* 
     * row contents:
     * 
     * [base case drag coeff;
     * [drag coeff contribution of jth power of flap extension;
     * [drag coeff contribution of jth power of vertical velocity]
     */ 
    double drag_model[N_D_IMPACTORS][N_D_COEFF_TERMS];

public:

    LVD(double m, double fin_width, double body_rad, double n_flaps,
          double drag_model[][N_D_COEFF_TERMS],
          double g, double ro,
          double apo_goal);

    /**
    * computes the coefficient of the launch vehicle as a function
    * of the current state and current control
    */
    double compute_cd(state_t X_to, control_t U_t);

    /**
    * computes the force of drag acting on the launch vehicle
    * as a function of the current state and control
    */
    double compute_fd(state_t X_t, control_t U_t);

    /** 
    * returns the cross sectional area of the launch vehicle
    * based on the current level of flap extension
    */
    double compute_area(control_t U_t);

    /**
    * This function takes a current state and then 
    * sets a predicted state a small time DT into the future 
    * based on the current state and the current control.
    *  
    * Inputs:
    * X_t[in] : the current state
    * X_tp[out] : the predicted state a small time DT into the future
    * U_t[in] : the current control
    *
    * Outputs:
    * None 
    */
    void ss_predict(state_t X_t, state_t X_tp, control_t U_t);

    /**
    * This function takes a current state and then 
    * sets a predicted state at apogee
    * based on the current state and the current control.
    *  
    * Inputs:
    * X_t[in] : the current state
    * X_tpf[out] : the predicted state at apogee
    * U_t[in] : the current control
    *
    * Outputs:
    * None 
    */
    void ms_predict(state_t X_t, state_t X_tpf, control_t U_t);

    /* returns the target apogee */
    double get_apo_goal();

    double d_coeff_dot_product(double coeffs[], double powers[]);
};

#endif