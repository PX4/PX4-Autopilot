/*
 * Author: Hyon Lim <limhyon@gmail.com, hyonlim@snu.ac.kr>
 *
 * @file attitude_estimator_so3_comp_params.c
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

#include "attitude_estimator_so3_comp_params.h"

/* This is filter gain for nonlinear SO3 complementary filter */
/* NOTE : How to tune the gain? First of all, stick with this default gain. And let the quad in stable place.
   Log the steady state reponse of filter. If it is too slow, increase SO3_COMP_KP.
   If you are flying from ground to high altitude in short amount of time, please increase SO3_COMP_KI which
   will compensate gyro bias which depends on temperature and vibration of your vehicle */
PARAM_DEFINE_FLOAT(SO3_COMP_KP, 1.0f); //! This parameter will give you about 15 seconds convergence time.
                                       //! You can set this gain higher if you want more fast response.
                                       //! But note that higher gain will give you also higher overshoot.
PARAM_DEFINE_FLOAT(SO3_COMP_KI, 0.05f); //! This gain will incorporate slow time-varying bias (e.g., temperature change)
					//! This gain is depend on your vehicle status.

/* offsets in roll, pitch and yaw of sensor plane and body */
PARAM_DEFINE_FLOAT(ATT_ROLL_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(ATT_PITCH_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(ATT_YAW_OFFS, 0.0f);

int parameters_init(struct attitude_estimator_so3_comp_param_handles *h)
{
	/* Filter gain parameters */
	h->Kp = 	param_find("SO3_COMP_KP");
	h->Ki = 	param_find("SO3_COMP_KI");

	/* Attitude offset (WARNING: Do not change if you do not know what exactly this variable wil lchange) */
	h->roll_off  =	param_find("ATT_ROLL_OFFS");
	h->pitch_off =	param_find("ATT_PITCH_OFFS");
	h->yaw_off   =	param_find("ATT_YAW_OFFS");

	return OK;
}

int parameters_update(const struct attitude_estimator_so3_comp_param_handles *h, struct attitude_estimator_so3_comp_params *p)
{
	/* Update filter gain */
	param_get(h->Kp, &(p->Kp));
	param_get(h->Ki, &(p->Ki));

	/* Update attitude offset */
	param_get(h->roll_off, &(p->roll_off));
	param_get(h->pitch_off, &(p->pitch_off));
	param_get(h->yaw_off, &(p->yaw_off));

	return OK;
}
