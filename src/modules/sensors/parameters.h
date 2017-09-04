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

#pragma once

/**
 * @file parameters.h
 *
 * defines the list of parameters that are used within the sensors module
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */
#include <px4_config.h>
#include <drivers/drv_rc_input.h>

#include <systemlib/param/param.h>
#include <mathlib/mathlib.h>

#include <uORB/topics/rc_parameter_map.h>


namespace sensors
{

static const unsigned RC_MAX_CHAN_COUNT =
	input_rc_s::RC_INPUT_MAX_CHANNELS;	/**< maximum number of r/c channels we handle */

struct Parameters {
	float min[RC_MAX_CHAN_COUNT];
	float trim[RC_MAX_CHAN_COUNT];
	float max[RC_MAX_CHAN_COUNT];
	float rev[RC_MAX_CHAN_COUNT];
	float dz[RC_MAX_CHAN_COUNT];
	float scaling_factor[RC_MAX_CHAN_COUNT];

	float diff_pres_offset_pa;
	float diff_pres_analog_scale;

	int board_rotation;

	float board_offset[3];

	int rc_map_roll;
	int rc_map_pitch;
	int rc_map_yaw;
	int rc_map_throttle;
	int rc_map_failsafe;

	int rc_map_mode_sw;
	int rc_map_return_sw;
	int rc_map_rattitude_sw;
	int rc_map_posctl_sw;
	int rc_map_loiter_sw;
	int rc_map_acro_sw;
	int rc_map_offboard_sw;
	int rc_map_kill_sw;
	int rc_map_arm_sw;
	int rc_map_trans_sw;
	int rc_map_gear_sw;
	int rc_map_stab_sw;
	int rc_map_man_sw;
	int rc_map_flaps;

	int rc_map_aux1;
	int rc_map_aux2;
	int rc_map_aux3;
	int rc_map_aux4;
	int rc_map_aux5;

	int rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];

	int rc_map_flightmode;

	int32_t rc_fails_thr;
	float rc_assist_th;
	float rc_auto_th;
	float rc_rattitude_th;
	float rc_posctl_th;
	float rc_return_th;
	float rc_loiter_th;
	float rc_acro_th;
	float rc_offboard_th;
	float rc_killswitch_th;
	float rc_armswitch_th;
	float rc_trans_th;
	float rc_gear_th;
	float rc_stab_th;
	float rc_man_th;

	bool rc_assist_inv;
	bool rc_auto_inv;
	bool rc_rattitude_inv;
	bool rc_posctl_inv;
	bool rc_return_inv;
	bool rc_loiter_inv;
	bool rc_acro_inv;
	bool rc_offboard_inv;
	bool rc_killswitch_inv;
	bool rc_armswitch_inv;
	bool rc_trans_inv;
	bool rc_gear_inv;
	bool rc_stab_inv;
	bool rc_man_inv;

	float rc_flt_smp_rate;
	float rc_flt_cutoff;

	float battery_voltage_scaling;
	float battery_current_scaling;
	float battery_current_offset;
	float battery_v_div;
	float battery_a_per_v;
	int32_t battery_source;

	float baro_qnh;

	float vibration_warning_threshold;

};

struct ParameterHandles {
	param_t min[RC_MAX_CHAN_COUNT];
	param_t trim[RC_MAX_CHAN_COUNT];
	param_t max[RC_MAX_CHAN_COUNT];
	param_t rev[RC_MAX_CHAN_COUNT];
	param_t dz[RC_MAX_CHAN_COUNT];

	param_t diff_pres_offset_pa;
	param_t diff_pres_analog_scale;

	param_t rc_map_roll;
	param_t rc_map_pitch;
	param_t rc_map_yaw;
	param_t rc_map_throttle;
	param_t rc_map_failsafe;

	param_t rc_map_mode_sw;
	param_t rc_map_return_sw;
	param_t rc_map_rattitude_sw;
	param_t rc_map_posctl_sw;
	param_t rc_map_loiter_sw;
	param_t rc_map_acro_sw;
	param_t rc_map_offboard_sw;
	param_t rc_map_kill_sw;
	param_t rc_map_arm_sw;
	param_t rc_map_trans_sw;
	param_t rc_map_gear_sw;
	param_t rc_map_flaps;
	param_t rc_map_stab_sw;
	param_t rc_map_man_sw;

	param_t rc_map_aux1;
	param_t rc_map_aux2;
	param_t rc_map_aux3;
	param_t rc_map_aux4;
	param_t rc_map_aux5;

	param_t rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];
	param_t rc_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];	/**< param handles for the parameters which are bound
							  to a RC channel, equivalent float values in the
							  _parameters struct are not existing
							  because these parameters are never read. */

	param_t rc_map_flightmode;

	param_t rc_fails_thr;
	param_t rc_assist_th;
	param_t rc_auto_th;
	param_t rc_rattitude_th;
	param_t rc_posctl_th;
	param_t rc_return_th;
	param_t rc_loiter_th;
	param_t rc_acro_th;
	param_t rc_offboard_th;
	param_t rc_killswitch_th;
	param_t rc_armswitch_th;
	param_t rc_trans_th;
	param_t rc_gear_th;
	param_t rc_stab_th;
	param_t rc_man_th;

	param_t rc_flt_smp_rate;
	param_t rc_flt_cutoff;

	param_t battery_voltage_scaling;
	param_t battery_current_scaling;
	param_t battery_current_offset;
	param_t battery_v_div;
	param_t battery_a_per_v;
	param_t battery_source;

	param_t board_rotation;

	param_t board_offset[3];

	param_t baro_qnh;

	param_t vibe_thresh; /**< vibration threshold */

};

/**
 * initialize ParameterHandles struct
 * @return 0 on succes, <0 on error
 */
int initialize_parameter_handles(ParameterHandles &parameter_handles);


/**
 * Read out the parameters using the handles into the parameters struct.
 * @return 0 on succes, <0 on error
 */
int update_parameters(const ParameterHandles &parameter_handles, Parameters &parameters);

} /* namespace sensors */
