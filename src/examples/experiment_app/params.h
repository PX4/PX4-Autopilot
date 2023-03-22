/*
 * @file params.h
 *
 * Definition of parameters for fixedwing system id
 */

#include <parameters/param.h>

struct params {
	float magnitude;
	float duration;
	float fstart;
	float fend;
	int input_type;
	int output_type;
	int exp_flag;
};

struct param_handles {
	param_t magnitude;
	param_t duration;
	param_t fstart;
	param_t fend;
	param_t input_type;
	param_t output_type;
	param_t exp_flag;
};
