/**
 * This file contains the implementation for the sensor class,
 * which determines the launch vehicle state based on raw
 * sensor data.
 */

#include "sensor.h"

sensor::sensor()
{
    offset = 0;
}

void sensor::zero_altimeters(double a1, double a2, double a3, double a4)
{
    offset = (a1 + a2 + a3 + a4)/N_ALTIMETERS;
}

double sensor::get_vertical_speed(state_t X_tm1, double a1, double a2, 
                                  double a3, double a4){
    /**
     * We use a simple calculation of finite differences to calculate the 
     * average velocity over the last sampling period
     */
    return (get_altitude(X_tm1, a1,a2,a3,a4)-X_tm1->altitude)/SAMPLE_T;
}

double sensor::get_altitude(state_t X_tm1, double a1, double a2, double a3, double a4)
{
    /**
     * We take the average of the altimeter readings and
     * convert to an altitude AGL
     */
    return ((a1 + a2 + a3 + a4)/N_ALTIMETERS - offset);
}

double sensor::get_polar_angle(double th_y, double th_z)
{
    th_y = th_y * M_PI/180;
    th_z = th_z * M_PI/180;

    /** 
     * this function combines the Euler angles in y and z
     * to form the polar angle in radians
     */
    return atan( sqrt(tan(th_y)*tan(th_y) + tan(th_z)*tan(th_z)) );
}

double sensor::get_polar_angle(double th)
{
    return th;
}