#include "sensor.h"

sensor::sensor(){
  offset = 0;
}

void sensor::zero_altimeters(double a1, double a2, double a3, double a4){
  offset = (a1 + a2 + a3 + a4)/N_ALTIMETERS;
}

double sensor::get_vertical_speed(state_t X_tm1, double a1, double a2, 
								  double a3, double a4){
	/**
	 * We compute our current speed and last speed to measure acceleration. This
	 * acceleration value is used to correct for the error in the velocity
	 * measurement that will be caused due to the coarseness of the sampling
	 * interval SAMPLE_T.
	 */
	return (get_altitude(X_tm1, a1,a2,a3,a4)-X_tm1->altitude)/SAMPLE_T;
}

double sensor::get_altitude(state_t X_tm1, double a1, double a2, double a3, double a4){
	/**
	 * We take the average of the altimeter readings offset by a correction
	 * term that accounts for the fact that the altimeters are averaging over
	 * the period of SAMPLE_T.
	 */
	return ((a1 + a2 + a3 + a4)/N_ALTIMETERS - offset);
}

double sensor::get_polar_angle(double th_y, double th_z){
	th_y = th_y * M_PI/180;
	th_z = th_z * M_PI/180;

	/** 
	 * this function combines the Euler angles in y and z
	 * to form the polar angle in radians
	 */
    return atan( sqrt(tan(th_y)*tan(th_y) + tan(th_z)*tan(th_z)) );
}

double sensor::get_polar_angle(double th){
	return th;
}