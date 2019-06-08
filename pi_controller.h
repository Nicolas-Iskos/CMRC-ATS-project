/**
 * This file contains the declaration of the pi_controller class.
 * Each object contains values for the controller gains of a 
 * proportional-integral controller as well
 * as functions that can be used to determine the appropriate 
 * control for a given error.
 *
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#ifndef ATS_INT_H
#define ATS_INT_H
#include "ATS_int.h"
#endif


class pi_controller{
private:
  // gain of proportional term
    double p;

  // gain of integral term
    double i;

    /**
     * Where d is flap extension:
     * 
     * angle = rt9 * d^9 + rt8 * d^8 + ... + rt1 * d + rt0
     */
    double servo_ext_model[N_SERVO_COEFFS];

    double flap_length;

public:
    /**
     * default constructor which builds a pi_controller object
     * optimized for the CMRC SCOTTIE launch vehicle
     */ 
    pi_controller(double p_in, double i_in, double flap_length_in);

    pi_controller(double p_in, double i_in, double servo_ext_model_in[], 
                  double flap_length_in);
    /**
     * This function returns the appropriate control
     * in flap extension for a given error:
     *
     * Inputs:
     * 	e : an array containing the instantaneous error
     * 		as well as the accrued error over time
     * 
     * Outputs:
     * 	The appropriate distance of flap extension
     */
    control_t get_control(error_t e);
    
    /**
     * The CMRC SCOTTIE launch vehicle uses a servo 
     * motor connected to a rotating turnpiece to deploy
     * the airbrakes. This function returns the appropriate control
     * in servo angle for a given error:
     *
     * Inputs:
     * 	e : an array containing the instantaneous error
     * 		as well as the accrued error over time
     * 
     * Outputs:
     * 	The appropriate angle in degrees from which the servo must
     * 	rotate, where 90 degees corresponds to flaps fully retracted
     */
    double get_theta(error_t e);

    double servo_ext_dot_product(double coeffs[], double powers[]);
};

#endif
