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