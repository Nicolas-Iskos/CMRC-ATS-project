#include "pi_controller.h"
#include <math.h>


pi_controller::pi_controller(double p_arg, double i_arg){
    p = p_arg;
    i = i_arg;

    rt0 = 1.5708;
    rt1 = 5.299;
    rt2 = -1.757;
    rt3 = 900.8;
    rt4 = -3481;
    rt5 = -4.481e+05;
    rt6 = 4.261e+06;
    rt7 = 6.36e+07;
    rt8 = -1.077e+09;
    rt9 = 3.967e+09;
}

double pi_controller::get_extension(error_t e){
  double p_o = p * e->inst + i * e->acc;
  if(p_o < 0.) p_o = 0.;
  if(p_o > 0.125) p_o = .125;

  return p_o;
}

double pi_controller::get_theta(error_t e){
    double output;

    // current extension
    double p_o = get_extension(e);
    
    /** 
     * we use the current flap extension and a polynomial fit of the
     * equation of angle as a function of distance to calculate the
     * appropriate servo angle
     */
    output = 
    (180/M_PI)*(rt9 * pow(p_o,9) + rt8 * pow(p_o,8) rt7 * pow(p_o,7)
    +rt6 * pow(p_o,6) + rt5 * pow(p_o,5) + rt4 * pow(p_o,4)
    +rt3 * pow(p_o,3) + rt2 * pow(p_o,2) + rt1 * p_o + rt0);

    return output;
}
