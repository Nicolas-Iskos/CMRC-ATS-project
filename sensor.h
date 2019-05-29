/**
 * This file contains the declaration for a class that holds
 * the necessary functions and data to process raw data taken
 * by the ATS's onboard sensors.
 *
 * A sensor object contains a single variable, offset, which
 * corresponds to the launch vehicle's initial elevation above
 * sea-level. It also includes several functions which can be used
 * to process raw sensor data into useable data for the prediction
 * and control algorithm.
 */

#ifndef SENSOR_H
#define SENSOR_H

#ifndef STATE_STRUCT_H
#define STATE_STRUCT_H
#include "state_struct.h"
#endif
#include "Arduino.h"

class sensor{
  private:
  	// altitude above ground level (AGL) = altitude above sea level (ASL) - offset
  	double offset;

  public:
  	/* trivial default constructor */
  	sensor();

  	/**
  	 * This function is to be called once the launch vehicle is on the 
  	 * pad, but before launch. The result is to set the offset variable
  	 * to the launch pad's elevation above sea level. Subsequent
  	 * measurements of the altitude will subtract this offset to return
  	 * an altitude AGL.
  	 *
  	 * Inputs: a1, a2, a3, a4 : raw value of each of the 4 ATS altimeters
  	 *
  	 * Outputs: None
  	 */
  	void zero_altimeters(double a1, double a2, double a3, double a4);
	
  	/**
  	 * Given a current state along with the raw value from each of the 
  	 * 4 altimeters, this function returns the calculated vertical velocity.
  	 * It does this by performing a calculation of finite differences on the
  	 * altitude calculation, with appropriate corrections for accuracy.
  	 *
  	 * Inputs: X_tm : current state
  	 * 		   a1, a2, a3, a4 : raw value of each of the 4 altimeters
  	 *
  	 * Outputs: calculated vertical speed
  	 */
	double get_vertical_speed(state_t X_tm1, double a1, double a2, 
	double a3, double a4);

	/**
	 * Given a previous state along with the current raw value from each of the
	 * 4 altimeters, this function returns the calculated altitude AGL.
	 *
	 * Inputs: X_tm : current state
	 * 		   a1, a2, a3, a4 : raw value of each of the 4 altimeters
	 *
	 * Outputs: calculated altitude AGL
	 */
  	double get_altitude(state_t X_tm1, double a1, double a2, double a3, double a4);

	/**
	 * Given the previous state along with the current raw value from each of the
	 * 4 altimeters, this function returns the calculated polar angle.
	 * The polar angle is the angle between the longitudinal axis of the
	 * launch vehicle and the vertical direction.
	 *
	 * Inputs: t3_y : y Euler angle
	 *				  z Euler angle
	 *
	 * Outputs: calculated polar angle in radians
	 */
  	double get_polar_angle(double t3_y, double t3_z);

};

#endif
