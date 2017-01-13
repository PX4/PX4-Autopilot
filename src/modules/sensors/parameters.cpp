/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file parameters.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "parameters.h"

namespace sensors
{

ParameterHandles::ParameterHandles() :
	/* Differential pressure offset */
	diff_pres_offset_pa(PARAM_FIND(SENS_DPRES_OFF)),
	diff_pres_analog_scale(PARAM_FIND(SENS_DPRES_ANSC)),

	/* mandatory input switched, mapped to channels 1-4 per default */
	rc_map_roll(PARAM_FIND(RC_MAP_ROLL)),
	rc_map_pitch(PARAM_FIND(RC_MAP_PITCH)),
	rc_map_yaw(PARAM_FIND(RC_MAP_YAW)),
	rc_map_throttle(PARAM_FIND(RC_MAP_THROTTLE)),
	rc_map_failsafe(PARAM_FIND(RC_MAP_FAILSAFE)),

	/* mandatory mode switches, mapped to channel 5 and 6 per default */
	rc_map_mode_sw(PARAM_FIND(RC_MAP_MODE_SW)),
	rc_map_return_sw(PARAM_FIND(RC_MAP_RETURN_SW)),
	/* optional mode switches, not mapped per default */
	rc_map_rattitude_sw(PARAM_FIND(RC_MAP_RATT_SW)),
	rc_map_posctl_sw(PARAM_FIND(RC_MAP_POSCTL_SW)),
	rc_map_loiter_sw(PARAM_FIND(RC_MAP_LOITER_SW)),
	rc_map_acro_sw(PARAM_FIND(RC_MAP_ACRO_SW)),
	rc_map_offboard_sw(PARAM_FIND(RC_MAP_OFFB_SW)),
	rc_map_kill_sw(PARAM_FIND(RC_MAP_KILL_SW)),
	rc_map_arm_sw(PARAM_FIND(RC_MAP_ARM_SW)),
	rc_map_trans_sw(PARAM_FIND(RC_MAP_TRANS_SW)),
	rc_map_gear_sw(PARAM_FIND(RC_MAP_GEAR_SW)),

	rc_map_flaps(PARAM_FIND(RC_MAP_FLAPS)),

	rc_map_aux1(PARAM_FIND(RC_MAP_AUX1)),
	rc_map_aux2(PARAM_FIND(RC_MAP_AUX2)),
	rc_map_aux3(PARAM_FIND(RC_MAP_AUX3)),
	rc_map_aux4(PARAM_FIND(RC_MAP_AUX4)),
	rc_map_aux5(PARAM_FIND(RC_MAP_AUX5)),

	rc_map_flightmode(PARAM_FIND(RC_MAP_FLTMODE)),

	/* RC thresholds */
	rc_fails_thr(PARAM_FIND(RC_FAILS_THR)),
	rc_assist_th(PARAM_FIND(RC_ASSIST_TH)),
	rc_auto_th(PARAM_FIND(RC_AUTO_TH)),
	rc_rattitude_th(PARAM_FIND(RC_RATT_TH)),
	rc_posctl_th(PARAM_FIND(RC_POSCTL_TH)),
	rc_return_th(PARAM_FIND(RC_RETURN_TH)),
	rc_loiter_th(PARAM_FIND(RC_LOITER_TH)),
	rc_acro_th(PARAM_FIND(RC_ACRO_TH)),
	rc_offboard_th(PARAM_FIND(RC_OFFB_TH)),
	rc_killswitch_th(PARAM_FIND(RC_KILLSWITCH_TH)),
	rc_armswitch_th(PARAM_FIND(RC_ARMSWITCH_TH)),
	rc_trans_th(PARAM_FIND(RC_TRANS_TH)),
	rc_gear_th(PARAM_FIND(RC_GEAR_TH)),

	battery_voltage_scaling(PARAM_FIND(BAT_CNT_V_VOLT)),
	battery_current_scaling(PARAM_FIND(BAT_CNT_V_CURR)),
	battery_current_offset(PARAM_FIND(BAT_V_OFFS_CURR)),
	battery_v_div(PARAM_FIND(BAT_V_DIV)),
	battery_a_per_v(PARAM_FIND(BAT_A_PER_V)),
	battery_source(PARAM_FIND(BAT_SOURCE)),

	/* rotations */
	board_rotation(PARAM_FIND(SENS_BOARD_ROT)),

	/* Barometer QNH */
	baro_qnh(PARAM_FIND(SENS_BARO_QNH)),

	vibe_thresh(PARAM_FIND(ATT_VIBE_THRESH)) /**< vibration threshold */
{
	/* basic r/c parameters */
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		char nbuf[16];

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		min[i] = param_find(nbuf);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		trim[i] = param_find(nbuf);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		max[i] = param_find(nbuf);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		rev[i] = param_find(nbuf);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		dz[i] = param_find(nbuf);

	}

	/* RC to parameter mapping for changing parameters with RC */
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		char name[rc_parameter_map_s::PARAM_ID_LEN];
		snprintf(name, rc_parameter_map_s::PARAM_ID_LEN, "RC_MAP_PARAM%d",
			 i + 1); // shifted by 1 because param name starts at 1
		rc_map_param[i] = param_find(name);
	}

	/* rotation offsets */
	board_offset[0] = PARAM_FIND(SENS_BOARD_X_OFF);
	board_offset[1] = PARAM_FIND(SENS_BOARD_Y_OFF);
	board_offset[2] = PARAM_FIND(SENS_BOARD_Z_OFF);

#if 0
	// These are parameters for which QGroundControl always expects to be returned in a list request.
	// We do a param_find here to force them into the list.
	(void)PARAM_FIND(RC_CHAN_CNT);
	(void)PARAM_FIND(RC_TH_USER);
	(void)PARAM_FIND(CAL_MAG0_ID);
	(void)PARAM_FIND(CAL_MAG1_ID);
	(void)PARAM_FIND(CAL_MAG2_ID);
	(void)PARAM_FIND(CAL_MAG0_ROT);
	(void)PARAM_FIND(CAL_MAG1_ROT);
	(void)PARAM_FIND(CAL_MAG2_ROT);
	(void)PARAM_FIND(CAL_MAG_SIDES);

	(void)PARAM_FIND(CAL_MAG1_XOFF);
	(void)PARAM_FIND(CAL_MAG1_XSCALE);
	(void)PARAM_FIND(CAL_MAG1_YOFF);
	(void)PARAM_FIND(CAL_MAG1_YSCALE);
	(void)PARAM_FIND(CAL_MAG1_ZOFF);
	(void)PARAM_FIND(CAL_MAG1_ZSCALE);

	(void)PARAM_FIND(CAL_MAG2_XOFF);
	(void)PARAM_FIND(CAL_MAG2_XSCALE);
	(void)PARAM_FIND(CAL_MAG2_YOFF);
	(void)PARAM_FIND(CAL_MAG2_YSCALE);
	(void)PARAM_FIND(CAL_MAG2_ZOFF);
	(void)PARAM_FIND(CAL_MAG2_ZSCALE);

	(void)PARAM_FIND(CAL_GYRO1_XOFF);
	(void)PARAM_FIND(CAL_GYRO1_XSCALE);
	(void)PARAM_FIND(CAL_GYRO1_YOFF);
	(void)PARAM_FIND(CAL_GYRO1_YSCALE);
	(void)PARAM_FIND(CAL_GYRO1_ZOFF);
	(void)PARAM_FIND(CAL_GYRO1_ZSCALE);

	(void)PARAM_FIND(CAL_GYRO2_XOFF);
	(void)PARAM_FIND(CAL_GYRO2_XSCALE);
	(void)PARAM_FIND(CAL_GYRO2_YOFF);
	(void)PARAM_FIND(CAL_GYRO2_YSCALE);
	(void)PARAM_FIND(CAL_GYRO2_ZOFF);
	(void)PARAM_FIND(CAL_GYRO2_ZSCALE);

	(void)PARAM_FIND(CAL_ACC1_XOFF);
	(void)PARAM_FIND(CAL_ACC1_XSCALE);
	(void)PARAM_FIND(CAL_ACC1_YOFF);
	(void)PARAM_FIND(CAL_ACC1_YSCALE);
	(void)PARAM_FIND(CAL_ACC1_ZOFF);
	(void)PARAM_FIND(CAL_ACC1_ZSCALE);

	(void)PARAM_FIND(CAL_ACC2_XOFF);
	(void)PARAM_FIND(CAL_ACC2_XSCALE);
	(void)PARAM_FIND(CAL_ACC2_YOFF);
	(void)PARAM_FIND(CAL_ACC2_YSCALE);
	(void)PARAM_FIND(CAL_ACC2_ZOFF);
	(void)PARAM_FIND(CAL_ACC2_ZSCALE);

	(void)PARAM_FIND(SYS_PARAM_VER);
	(void)PARAM_FIND(SYS_AUTOSTART);
	(void)PARAM_FIND(SYS_AUTOCONFIG);
	(void)PARAM_FIND(PWM_RATE);
	(void)PARAM_FIND(PWM_MIN);
	(void)PARAM_FIND(PWM_MAX);
	(void)PARAM_FIND(PWM_DISARMED);
	(void)PARAM_FIND(PWM_AUX_MIN);
	(void)PARAM_FIND(PWM_AUX_MAX);
	(void)PARAM_FIND(PWM_AUX_DISARMED);
	(void)PARAM_FIND(TRIG_MODE);
	(void)PARAM_FIND(UAVCAN_ENABLE);
	(void)PARAM_FIND(SYS_MC_EST_GROUP);
#endif

}

int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters)
{

	bool rc_valid = true;
	float tmpScaleFactor = 0.0f;
	float tmpRevFactor = 0.0f;
	int ret = PX4_OK;

	/* rc values */
	for (unsigned int i = 0; i < RC_MAX_CHAN_COUNT; i++) {

		param_get(parameter_handles.min[i], &(parameters.min[i]));
		param_get(parameter_handles.trim[i], &(parameters.trim[i]));
		param_get(parameter_handles.max[i], &(parameters.max[i]));
		param_get(parameter_handles.rev[i], &(parameters.rev[i]));
		param_get(parameter_handles.dz[i], &(parameters.dz[i]));

		tmpScaleFactor = (1.0f / ((parameters.max[i] - parameters.min[i]) / 2.0f) * parameters.rev[i]);
		tmpRevFactor = tmpScaleFactor * parameters.rev[i];

		/* handle blowup in the scaling factor calculation */
		if (!PX4_ISFINITE(tmpScaleFactor) ||
		    (tmpRevFactor < 0.000001f) ||
		    (tmpRevFactor > 0.2f)) {
			PX4_WARN("RC chan %u not sane, scaling: %8.6f, rev: %d", i, (double)tmpScaleFactor, (int)(parameters.rev[i]));
			/* scaling factors do not make sense, lock them down */
			parameters.scaling_factor[i] = 0.0f;
			rc_valid = false;

		} else {
			parameters.scaling_factor[i] = tmpScaleFactor;
		}
	}

	/* handle wrong values */
	if (!rc_valid) {
		PX4_ERR("WARNING     WARNING     WARNING\n\nRC CALIBRATION NOT SANE!\n\n");
	}

	const char *paramerr = "FAIL PARM LOAD";

	/* channel mapping */
	if (param_get(parameter_handles.rc_map_roll, &(parameters.rc_map_roll)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_pitch, &(parameters.rc_map_pitch)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_yaw, &(parameters.rc_map_yaw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_throttle, &(parameters.rc_map_throttle)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_failsafe, &(parameters.rc_map_failsafe)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_mode_sw, &(parameters.rc_map_mode_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_return_sw, &(parameters.rc_map_return_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_rattitude_sw, &(parameters.rc_map_rattitude_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_posctl_sw, &(parameters.rc_map_posctl_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_loiter_sw, &(parameters.rc_map_loiter_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_acro_sw, &(parameters.rc_map_acro_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_offboard_sw, &(parameters.rc_map_offboard_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_kill_sw, &(parameters.rc_map_kill_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_arm_sw, &(parameters.rc_map_arm_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_trans_sw, &(parameters.rc_map_trans_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_flaps, &(parameters.rc_map_flaps)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(parameter_handles.rc_map_gear_sw, &(parameters.rc_map_gear_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	param_get(parameter_handles.rc_map_aux1, &(parameters.rc_map_aux1));
	param_get(parameter_handles.rc_map_aux2, &(parameters.rc_map_aux2));
	param_get(parameter_handles.rc_map_aux3, &(parameters.rc_map_aux3));
	param_get(parameter_handles.rc_map_aux4, &(parameters.rc_map_aux4));
	param_get(parameter_handles.rc_map_aux5, &(parameters.rc_map_aux5));

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		param_get(parameter_handles.rc_map_param[i], &(parameters.rc_map_param[i]));
	}

	param_get(parameter_handles.rc_map_flightmode, &(parameters.rc_map_flightmode));

	param_get(parameter_handles.rc_fails_thr, &(parameters.rc_fails_thr));
	param_get(parameter_handles.rc_assist_th, &(parameters.rc_assist_th));
	parameters.rc_assist_inv = (parameters.rc_assist_th < 0);
	parameters.rc_assist_th = fabs(parameters.rc_assist_th);
	param_get(parameter_handles.rc_auto_th, &(parameters.rc_auto_th));
	parameters.rc_auto_inv = (parameters.rc_auto_th < 0);
	parameters.rc_auto_th = fabs(parameters.rc_auto_th);
	param_get(parameter_handles.rc_rattitude_th, &(parameters.rc_rattitude_th));
	parameters.rc_rattitude_inv = (parameters.rc_rattitude_th < 0);
	parameters.rc_rattitude_th = fabs(parameters.rc_rattitude_th);
	param_get(parameter_handles.rc_posctl_th, &(parameters.rc_posctl_th));
	parameters.rc_posctl_inv = (parameters.rc_posctl_th < 0);
	parameters.rc_posctl_th = fabs(parameters.rc_posctl_th);
	param_get(parameter_handles.rc_return_th, &(parameters.rc_return_th));
	parameters.rc_return_inv = (parameters.rc_return_th < 0);
	parameters.rc_return_th = fabs(parameters.rc_return_th);
	param_get(parameter_handles.rc_loiter_th, &(parameters.rc_loiter_th));
	parameters.rc_loiter_inv = (parameters.rc_loiter_th < 0);
	parameters.rc_loiter_th = fabs(parameters.rc_loiter_th);
	param_get(parameter_handles.rc_acro_th, &(parameters.rc_acro_th));
	parameters.rc_acro_inv = (parameters.rc_acro_th < 0);
	parameters.rc_acro_th = fabs(parameters.rc_acro_th);
	param_get(parameter_handles.rc_offboard_th, &(parameters.rc_offboard_th));
	parameters.rc_offboard_inv = (parameters.rc_offboard_th < 0);
	parameters.rc_offboard_th = fabs(parameters.rc_offboard_th);
	param_get(parameter_handles.rc_killswitch_th, &(parameters.rc_killswitch_th));
	parameters.rc_killswitch_inv = (parameters.rc_killswitch_th < 0);
	parameters.rc_killswitch_th = fabs(parameters.rc_killswitch_th);
	param_get(parameter_handles.rc_armswitch_th, &(parameters.rc_armswitch_th));
	parameters.rc_armswitch_inv = (parameters.rc_armswitch_th < 0);
	parameters.rc_armswitch_th = fabs(parameters.rc_armswitch_th);
	param_get(parameter_handles.rc_trans_th, &(parameters.rc_trans_th));
	parameters.rc_trans_inv = (parameters.rc_trans_th < 0);
	parameters.rc_trans_th = fabs(parameters.rc_trans_th);
	param_get(parameter_handles.rc_gear_th, &(parameters.rc_gear_th));
	parameters.rc_gear_inv = (parameters.rc_gear_th < 0);
	parameters.rc_gear_th = fabs(parameters.rc_gear_th);

	/* Airspeed offset */
	param_get(parameter_handles.diff_pres_offset_pa, &(parameters.diff_pres_offset_pa));
	param_get(parameter_handles.diff_pres_analog_scale, &(parameters.diff_pres_analog_scale));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(parameter_handles.battery_voltage_scaling, &(parameters.battery_voltage_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (parameters.battery_voltage_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		parameters.battery_voltage_scaling = (3.3f / 4096);
		param_set(parameter_handles.battery_voltage_scaling, &parameters.battery_voltage_scaling);
	}

	/* scaling of ADC ticks to battery current */
	if (param_get(parameter_handles.battery_current_scaling, &(parameters.battery_current_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (parameters.battery_current_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		parameters.battery_current_scaling = (3.3f / 4096);
		param_set(parameter_handles.battery_current_scaling, &parameters.battery_current_scaling);
	}

	if (param_get(parameter_handles.battery_current_offset, &(parameters.battery_current_offset)) != OK) {
		PX4_WARN("%s", paramerr);

	}

	if (param_get(parameter_handles.battery_v_div, &(parameters.battery_v_div)) != OK) {
		PX4_WARN("%s", paramerr);
		parameters.battery_v_div = 0.0f;

	} else if (parameters.battery_v_div <= 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		parameters.battery_v_div = 13.653333333f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined ( CONFIG_ARCH_BOARD_MINDPX_V2 )
		parameters.battery_v_div = 10.177939394f;
#elif defined (CONFIG_ARCH_BOARD_AEROCORE)
		parameters.battery_v_div = 7.8196363636f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		parameters.battery_v_div = 5.7013919372f;
#elif defined (CONFIG_ARCH_BOARD_SITL)
		parameters.battery_v_div = 10.177939394f;
#elif defined (CONFIG_ARCH_BOARD_TAP_V1)
		parameters.battery_v_div = 9.0f;
#elif defined (CONFIG_ARCH_BOARD_AEROFC_V1)
		parameters.battery_v_div = 9.0f;
#else
		/* ensure a missing default trips a low voltage lockdown */
		parameters.battery_v_div = 0.0f;
#endif
		param_set(parameter_handles.battery_v_div, &parameters.battery_v_div);
	}

	if (param_get(parameter_handles.battery_a_per_v, &(parameters.battery_a_per_v)) != OK) {
		PX4_WARN("%s", paramerr);
		parameters.battery_a_per_v = 0.0f;

	} else if (parameters.battery_a_per_v <= 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		/* current scaling for ACSP4 */
		parameters.battery_a_per_v = 36.367515152f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined (CONFIG_ARCH_BOARD_MINDPX_V2) || defined (CONFIG_ARCH_BOARD_AEROCORE) || defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		/* current scaling for 3DR power brick */
		parameters.battery_a_per_v = 15.391030303f;
#elif defined (CONFIG_ARCH_BOARD_SITL)
		parameters.battery_a_per_v = 15.391030303f;
#else
		/* ensure a missing default leads to an unrealistic current value */
		parameters.battery_a_per_v = 0.0f;
#endif
		param_set(parameter_handles.battery_a_per_v, &parameters.battery_a_per_v);
	}

	param_get(parameter_handles.battery_source, &(parameters.battery_source));

	param_get(parameter_handles.board_rotation, &(parameters.board_rotation));

	param_get(parameter_handles.board_offset[0], &(parameters.board_offset[0]));
	param_get(parameter_handles.board_offset[1], &(parameters.board_offset[1]));
	param_get(parameter_handles.board_offset[2], &(parameters.board_offset[2]));

	param_get(parameter_handles.baro_qnh, &(parameters.baro_qnh));

	param_get(parameter_handles.vibe_thresh, &parameters.vibration_warning_threshold);

	return ret;
}

} /* namespace sensors */
