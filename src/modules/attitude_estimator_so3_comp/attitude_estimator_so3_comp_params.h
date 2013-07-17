/*
 * Author: Hyon Lim <limhyon@gmail.com, hyonlim@snu.ac.kr>
 *
 * @file attitude_estimator_so3_comp_params.h
 *
 * Implementation of nonlinear complementary filters on the SO(3).
 * This code performs attitude estimation by using accelerometer, gyroscopes and magnetometer.
 * Result is provided as quaternion, 1-2-3 Euler angle and rotation matrix.
 * 
 * Theory of nonlinear complementary filters on the SO(3) is based on [1].
 * Quaternion realization of [1] is based on [2].
 * Optmized quaternion update code is based on Sebastian Madgwick's implementation.
 * 
 * References
 *  [1] Mahony, R.; Hamel, T.; Pflimlin, Jean-Michel, "Nonlinear Complementary Filters on the Special Orthogonal Group," Automatic Control, IEEE Transactions on , vol.53, no.5, pp.1203,1218, June 2008
 *  [2] Euston, M.; Coote, P.; Mahony, R.; Jonghyuk Kim; Hamel, T., "A complementary filter for attitude estimation of a fixed-wing UAV," Intelligent Robots and Systems, 2008. IROS 2008. IEEE/RSJ International Conference on , vol., no., pp.340,345, 22-26 Sept. 2008
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
