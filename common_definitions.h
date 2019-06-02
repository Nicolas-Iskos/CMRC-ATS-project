#define N_D_IMPACTORS      3
#define N_D_COEFF_TERMS    5

#define STATIC_COEFF       0
#define FLAP_COEFF         1
#define V_COEFF            2

#define N_SERVO_COEFFS     10

#define N_ALTIMETERS       4

#define SAMPLE_T           0.520

/**
 * This file contains the struct representing
 * the state of the launch vehicle. It is used
 * globally in the ATS project.
 */

struct state {
	double velocity;
	double altitude;
	double theta;
};

typedef struct state* state_t;

/**
 * This file contains the struct representing
 * the error on apogee detected by the ATS.
 * It is used globally in the ATS project.
 */

struct error {
	double inst;
	double acc;
};

typedef struct error* error_t;