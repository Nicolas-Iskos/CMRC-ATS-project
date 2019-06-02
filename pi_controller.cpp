#include "pi_controller.h"
#include <math.h>


pi_controller::pi_controller(double p_arg, double i_arg, double flap_length_in){
    p = p_arg;
    i = i_arg;
    flap_length = flap_length_in;
}

pi_controller::pi_controller(double p_in, double i_in, double servo_ext_model_in[],
                             double flap_length_in)
{
    p = p_in;
    i = i_in;

    for(int i = 0; i < N_SERVO_COEFFS; i++)
    {
        servo_ext_model[i] = servo_ext_model_in[i];
    }

    flap_length = flap_length_in;
}



double pi_controller::get_extension(error_t e){
    double p_o = p * e->inst + i * e->acc;
    if(p_o < 0.) p_o = 0.;
    if(p_o > flap_length) p_o = flap_length;

    return p_o;
}

double pi_controller::get_theta(error_t e){
    // current extension
    double p_o = get_extension(e);
    double powers[N_SERVO_COEFFS];
    for(int i = 0; i < N_SERVO_COEFFS; i++)
    {
        powers[i] = pow(p_o,i);
    }
    
    /** 
     * we use the current flap extension and a 10-degree polynomial fit 
     * of the equation of angle as a function of distance to calculate 
     * the appropriate servo angle
     */
    return (180/M_PI) * servo_ext_dot_product(servo_ext_model, powers);
}

double pi_controller::servo_ext_dot_product(double coeffs[], double powers[])
{
    double result = 0;

    for(int i = 0; i < N_SERVO_COEFFS; i++)
    {
        result += servo_ext_model[i] * powers[i];
    }

    return result;
}



