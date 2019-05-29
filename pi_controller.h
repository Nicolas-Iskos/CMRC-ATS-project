/**
 * This file contains the declaration of the pi_controller class.
 * Each object contains values for the controller gains of a 
 * proportional-integral controller as well
 * as functions that can be used to determine the appropriate 
 * control for a given error.
 *
 * Units:
 * feet, slugs, seconds
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H
#include "state_struct.h"
#endif

#ifndef ERROR_STRUCT_H
#define ERROR_STRUCT_H
#include "error_struct.h"
#endif

#include "Arduino.h"

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
  double rt0;
  double rt1;
  double rt2;
  double rt3;
  double rt4;
  double rt5;
  double rt6;
  double rt7;
  double rt8;
  double rt9;

public:
  /**
   * default constructor which builds a pi_controller object
   * optimized for the CMRC SCOTTIE launch vehicle
   */ 
  pi_controller(double p, double i);

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
  double get_extension(error_t e);
  
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
};

#endif
