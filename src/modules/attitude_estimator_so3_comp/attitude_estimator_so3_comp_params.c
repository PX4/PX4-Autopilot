/*
 * @file attitude_estimator_so3_comp_params.c
 *
 * Parameters for SO3 complementary filter
 */

#include "attitude_estimator_so3_comp_params.h"

/* This is filter gain for nonlinear SO3 complementary filter */
PARAM_DEFINE_FLOAT(SO3_COMP_KP, 1.0f);
PARAM_DEFINE_FLOAT(SO3_COMP_KI, 0.0f);

/* offsets in roll, pitch and yaw of sensor plane and body */
PARAM_DEFINE_FLOAT(ATT_ROLL_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(ATT_PITCH_OFFS, 0.0f);
PARAM_DEFINE_FLOAT(ATT_YAW_OFFS, 0.0f);

int parameters_init(struct attitude_estimator_ekf_param_handles *h)
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

int parameters_update(const struct attitude_estimator_ekf_param_handles *h, struct attitude_estimator_ekf_params *p)
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
