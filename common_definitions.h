/* #include "Arduino.h" */
#include <math.h>

#define N_STATE_TYPES      3

#define N_D_IMPACTORS      3
#define N_D_COEFF_TERMS    5

#define STATIC_COEFF       0
#define FLAP_COEFF         1
#define V_COEFF            2

#define N_SERVO_COEFFS     10

#define N_ALTIMETERS       4

#define SAMPLE_T           0.520



struct state {
	double velocity;
	double altitude;
	double theta;
};

typedef struct state* state_t;

struct error {
	double inst;
	double acc;
};

typedef struct error* error_t;

typedef double control_t;