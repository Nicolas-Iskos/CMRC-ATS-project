/**
 * This file contains the declaration of the pi_controller class.
 * Each object contains values for the controller gains of a 
 * proportional-integral controller as well
 * as functions that can be used to determine the appropriate 
 * control for a given error. The control used throughout the ATS
 * code is flap extension.
 *
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#ifndef ATS_INT_H
#define ATS_INT_H
#include "ATS_int.h"
#endif


class pi_controller
{

private:
    /* gain of proportional term */
    double p;

    /* gain of integral term */
    double i;

    
    /**
     * In the case that a servo is used to rotate a mechanism
     * which deploys the flaps, 
     * when a dot product is taken between this vector
     * and the first N_SERVO_COEFFS powers of the flap extension,
     * the result is the servo angle in degrees 
     * corresponding to that extension
     */
    double servo_ext_model[N_SERVO_COEFFS];

    double flap_length;

public:
    /**
     * builds a pi_controller object for a vehicle that
     * does not need to calculate servo extension
     */ 
    pi_controller(double p_in, double i_in, double flap_length_in);

    /** 
     * builds an object for a vehicle that does need to calculate
     * servo extension
     */
    pi_controller(double p_in, double i_in, double servo_ext_model_in[], 
                  double flap_length_in);
    /**
     * This function returns the appropriate control
     * in flap extension for a given error:
     *
     * Inputs:
     * e : an array containing the instantaneous error
     * 	   as well as the accrued error over time
     * 
     * Outputs:
     * The appropriate distance of flap extension
     */
    control_t get_control(error_t e);
    
    /**
     * This function returns the appropriate control
     * in servo angle for a given error:
     *
     * Inputs:
     * 	e : an array containing the instantaneous error
     * 		as well as the accrued error over time
     * 
     * Outputs:
     * The appropriate angle in degrees from which the servo must
     * rotate
     */
    double get_theta(error_t e);

    double servo_ext_dot_product(double coeffs[], double powers[]);
};

#endif