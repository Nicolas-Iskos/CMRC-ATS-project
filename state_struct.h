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