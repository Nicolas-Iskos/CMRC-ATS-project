/**
 * This file contains the declarations for the LVD class. An LVD
 * object contains a number of constant properties intrinsic to the
 * launch vehicle as well as functions that can be used to predict
 * a future state given a current state
 */

#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H
#include "state_struct.h"
#endif

#ifndef LVD_H
#define LVD_H


#include "Arduino.h"


class LVD{
  private:
  double m, fin_wid, rocket_rad;
  double g, ro, MATH_PI;
  double apo_goal, dt;

  /** 
   * Coefficient of Drag (CD):
   * CD_total = w0 + (wd1 * d) + (wd2 * d^2) + (wd3 * d^3) + (wd4 * d^4)
   */
  double w0;
  double wd1;
  double wd2;
  double wd3;
  double wd4;

  public:

  /**
   * default constructor which assumes values for the launch vehicle
   * attributes of the CMRC SCOTTIE launch vehicle, used in the 2019 
   * NASA SL competition
   */
  LVD();

  /**
   * computes the coefficient of the launch vehicle as a function
   * of the current state and current control
   */
  double compute_cd(state_t X_t, double U_t);

  /**
   * computes the force of drag acting on the launch vehicle
   * as a function of the current state and control
   */
  double compute_fd(state_t X_t, double U_t);

  /** 
   * returns the cross sectional area of the launch vehicle
   * based on the current level of flap extension
   */
  double compute_area(double U_t);
  
  /**
   * This function takes a current state and then 
   * sets a predicted state a small time dt into the future 
   * based on the current state and the current control.
   *  
   * Inputs:
   *  X_t[in] : the current state
   *  X_tp[out] : the predicted state a small time dt into the future
   *  U_t[in] : the current control
   *
   * Outputs:
   *  None 
   */
  void ss_predict(state_t X_t, state_t X_tp, double U_t);

   /**
   * This function takes a current state and then 
   * sets a predicted state at apogee
   * based on the current state and the current control.
   *  
   * Inputs:
   *  X_t[in] : the current state
   *  X_tp[out] : the predicted state at apogee
   *  U_t[in] : the current control
   *
   * Outputs:
   *  None 
   */
  void ms_predict(state_t X_t, state_t X_tp, double U_t);

  /* returns the target apogee */
  double get_apo_goal();
};

#endif
