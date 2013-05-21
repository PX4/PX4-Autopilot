/*
 * @file attitude_estimator_so3_comp_params.h
 *
 * Parameters for EKF filter
 */

#include <systemlib/param/param.h>

struct attitude_estimator_so3_comp_params {
	float Kp;
	float Ki;
	float roll_off;
	float pitch_off;
	float yaw_off;
};

struct attitude_estimator_so3_comp_param_handles {
	param_t Kp, Ki;
	param_t roll_off, pitch_off, yaw_off;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct attitude_estimator_so3_comp_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct attitude_estimator_so3_comp_param_handles *h, struct attitude_estimator_so3_comp_params *p);
