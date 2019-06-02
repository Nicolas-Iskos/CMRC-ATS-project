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
	double v = (get_altitude(X_tm1, a1,a2,a3,a4)-X_tm1->altitude)/SAMPLE_T;
	double vm1 = X_tm1->velocity;
	double a_dt = v-vm1;

	return v+a_dt;
}

double sensor::get_altitude(state_t X_tm1, double a1, double a2, double a3, double a4){
	/**
	 * We take the average of the altimeter readings offset by a correction
	 * term that accounts for the fact that the altimeters are averaging over
	 * the period of SAMPLE_T.
	 */
	return ((a1 + a2 + a3 + a4)/N_ALTIMETERS - offset) + X_tm1->velocity*SAMPLE_T/2;
}

double sensor::get_polar_angle(double t3_y, double t3_z){
	t3_y = t3_y * M_PI/180;
	t3_z = t3_z * M_PI/180;

	/** 
	 * this function combines the Euler angles in y and z
	 * to form the polar angle in radians
	 */
    return atan( sqrt(tan(t3_y)*tan(t3_y) + tan(t3_z)*tan(t3_z)) );
}
