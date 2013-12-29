/*
 * mc_att_control_vector_params.c
 *
 *  Created on: 26.12.2013
 *      Author: ton
 */

#include <systemlib/param/param.h>

/* multicopter attitude controller parameters */
PARAM_DEFINE_FLOAT(MC_YAWPOS_P, 2.0f);
PARAM_DEFINE_FLOAT(MC_YAWPOS_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATT_P, 6.0f);
PARAM_DEFINE_FLOAT(MC_ATT_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.3f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.0f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_P, 0.1f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_D, 0.002f);
PARAM_DEFINE_FLOAT(MC_ATTRATE_I, 0.0f);
