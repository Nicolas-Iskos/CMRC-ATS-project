/**
 * This file contains a number of macros and data structure defitions that are
 * common to the entire ATS library.
 */


/* uncomment this line to compile for Arduino */
/* #include "Arduino.h" */ 
#include <math.h>

/* the number of different variables that make up a state */
#define N_STATE_TYPES      3

/* the number of variables that influence drag coefficient, including base drag */
#define N_D_IMPACTORS      3

/** 
 * the number of coefficients that contribute to the total drag coefficient
 * contribution from each of the N_D_IMPACTORS variables
 */
#define N_D_COEFF_TERMS    5

/** 
 * each of these three macros is the drag model row that corresponds to the drag coefficients
 * for that variable
 */
#define STATIC_COEFF       0
#define FLAP_COEFF         1
#define V_COEFF            2

/**
 * the number of terms that makes up the model describing the mapping from
 * flap extension to servo angle, if the vehicle uses a servo motor to 
 * deploy the flaps
 */
#define N_SERVO_COEFFS     10

#define N_ALTIMETERS       4

/* state updating period */
#define SAMPLE_T           0.104

/* timestep used for numerical approximation of predicted state */
#define DT                 0.0208


struct state
{
    double velocity;
    double altitude;
    double theta;
};

typedef struct state* state_t;

struct error
{
    /* instantenous error */
    double inst;
    /* accumulated error */
    double acc;
};

typedef struct error* error_t;

typedef double control_t;