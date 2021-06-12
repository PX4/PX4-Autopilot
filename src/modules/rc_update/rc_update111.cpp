/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file rc_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "rc_update.h"
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/uORB.h>

using namespace time_literals;

namespace RCUpdate
{

RCUpdate::RCUpdate() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, "rc_update"))
{
	// initialize parameter handles
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		char nbuf[16];

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		_parameter_handles.min[i] = param_find(nbuf);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		_parameter_handles.trim[i] = param_find(nbuf);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		_parameter_handles.max[i] = param_find(nbuf);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		_parameter_handles.rev[i] = param_find(nbuf);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		_parameter_handles.dz[i] = param_find(nbuf);
	}

	// RC to parameter mapping for changing parameters with RC
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		// shifted by 1 because param name starts at 1
		char name[rc_parameter_map_s::PARAM_ID_LEN];
		snprintf(name, rc_parameter_map_s::PARAM_ID_LEN, "RC_MAP_PARAM%d", i + 1);
		_parameter_handles.rc_map_param[i] = param_find(name);
	}
	//mavlink_log_critical(&_mavlink_log_pub, " rc_update creat");
	rc_parameter_map_poll(true /* forced */);

	parameters_updated();

}

RCUpdate::~RCUpdate()
{
//mx3g-jh
#ifdef YUNEEC_INPUT_RC_MAP_MANUAL_CONTROL
	//mavlink_log_critical(&_mavlink_log_pub, " rc_update quiet");

		orb_unsubscribe(_inputs_rc.sub);
		_inputs_rc.sub = -1;
	orb_unsubscribe(_slave_rc.slave_rc_sub);
#endif
	perf_free(_loop_perf);
}

bool
RCUpdate::init()
{

//mx3g-jh
#ifdef YUNEEC_INPUT_RC_MAP_MANUAL_CONTROL
	ScheduleDelayed(10);
		mavlink_log_critical(&_mavlink_log_pub, " init start");
		mavlink_log_critical(&_mavlink_log_pub, " subscribe input_rc");
		//_inputs_rc.sub = orb_subscribe(ORB_ID(input_rc));
	_slave_rc.slave_rc_sub = orb_subscribe(ORB_ID(slave_rc));

#endif



	// if (!_input_rc_sub.registerCallback()) {
	// 	PX4_ERR("input_rc callback registration failed!");
	// 	return false;
	// }
	mavlink_log_critical(&_mavlink_log_pub, "init over");
	return true;
}

void
RCUpdate::parameters_updated()
{	//mavlink_log_critical(&_mavlink_log_pub, "parameters_updated()");

	// rc values
	for (unsigned int i = 0; i < RC_MAX_CHAN_COUNT; i++) {

		float min = 0.0f;
		param_get(_parameter_handles.min[i], &min);
		_parameters.min[i] = min;

		float trim = 0.0f;
		param_get(_parameter_handles.trim[i], &trim);
		_parameters.trim[i] = trim;

		float max = 0.0f;
		param_get(_parameter_handles.max[i], &max);
		_parameters.max[i] = max;

		float rev = 0.0f;
		param_get(_parameter_handles.rev[i], &rev);
		_parameters.rev[i] = rev < 0.0f;

		float dz = 0.0f;
		param_get(_parameter_handles.dz[i], &dz);
		_parameters.dz[i] = dz;
	}

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		param_get(_parameter_handles.rc_map_param[i], &(_parameters.rc_map_param[i]));
	}

	update_rc_functions();
}

void
RCUpdate::update_rc_functions()
{	//mavlink_log_critical(&_mavlink_log_pub, "update_rc_functions()");
	//PX4_INFO("update_rc_functions()");
	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _param_rc_map_throttle.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _param_rc_map_roll.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _param_rc_map_pitch.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _param_rc_map_yaw.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _param_rc_map_mode_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _param_rc_map_return_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE] = _param_rc_map_ratt_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _param_rc_map_posctl_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _param_rc_map_loiter_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _param_rc_map_acro_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _param_rc_map_offb_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH] = _param_rc_map_kill_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH] = _param_rc_map_arm_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION] = _param_rc_map_trans_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_GEAR] = _param_rc_map_gear_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_STAB] = _param_rc_map_stab_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MAN] = _param_rc_map_man_sw.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _param_rc_map_flaps.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _param_rc_map_aux1.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _param_rc_map_aux2.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _param_rc_map_aux3.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _param_rc_map_aux4.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _param_rc_map_aux5.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_6] = _param_rc_map_aux6.get() - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}
}


//mx3g-jh
#ifdef YUNEEC_INPUT_RC_MAP_MANUAL_CONTROL

bool
RCUpdate::map_to_control_setpoint(const int32_t desired_priority)
{
	//mavlink_log_critical(&_mavlink_log_pub, "map_to_control_setpoint()");
	//PX4_INFO("map_to_control_setpoint()");

	bool success = false;

	// RC only is required. Search for a valid RC signal


		// Get priority
		ORB_PRIO priority = ORB_PRIO_UNINITIALIZED;
		orb_priority(_inputs_rc.sub, &priority);
		//mavlink_log_critical(&_mavlink_log_pub, "map_to_control_setpoint() 11");
		// Search for default priorities
		if (_inputs_rc.signal_valid && priority == desired_priority) {
			//mavlink_log_critical(&_mavlink_log_pub, "map_to_control_setpoint() 22");
			if (_param_rc_type.get() == 0) {
				// map based on rc_channel functions

				scale_raw_channels(_inputs_rc); //scale raw channels

				if (map_from_rc_channel_functions(_inputs_rc)) {
					// Update parameters from RC Channels (tuning with RC) if activated
					if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1e6) {
						set_params_from_rc();
						_last_rc_to_param_map_time = hrt_absolute_time();
					}

					success = true;

				}

			} else if (_param_rc_type.get() == 1) {
				int32_t rc_mode;
				rc_mode = _param_rc_mode.get();
				// map predefined remotes
				int failure = _rcmapping.map(_manual_sp, _inputs_rc.input, rc_mode);

				if (failure == (int)RCMap::Error::Version) {
					// inform user that version does not match
					print_rc_error_message("Old remote control version, please update!");

				} else {
					success = true;
				}
			}

			// return if mapping succeeded, otherwise continue loop
			if (success) {
				return true;
			}
		}


	return success;
}

bool
RCUpdate::map_from_team_mode()
{
	if (_param_rc_type.get() != 1) {
		print_rc_error_message("Team-mode not supported");
		return false;
	}

	if (_slave_rc.signal_valid) {
		int32_t rc_mode;
		rc_mode = _param_rc_mode.get();

		// overwrite stick inputs with st16 gimbal mapping
		if (_rcmapping.mapSlave(_manual_sp, _slave_rc.slave_rc, rc_mode)) {
			// inform user that version does not match
			print_rc_error_message("Old slave remote control version, please update!");
			return false;
		}

	} else {
		// no slave link available
		return false;
	}

	return true;
}

void
RCUpdate::print_rc_error_message(const char *str)
{
	static hrt_abstime _last_msg = 0;
	hrt_abstime now = hrt_absolute_time();

	// print every 15 seconds
	if (now - _last_msg > 15_s) {
		_last_msg = now;
		mavlink_log_critical(&_mavlink_log_pub, "%s", str);
	}
}

void
RCUpdate::set_signal_validity(InputRCset &input_set)
{	//mavlink_log_critical(&_mavlink_log_pub, "set_signal_validity");
	//PX4_INFO("set_signal_validity(InputRCset &input_set)");
	// detect RC signal loss
	input_set.signal_valid = true;

	const bool rc_timeout = (hrt_absolute_time() - input_set.input.timestamp_last_signal) > hrt_abstime(
					_param_rc_lost_t.get() * 1e6f);

	// check flags and require at least four channels to consider the signal valid
	if (input_set.input.rc_lost || input_set.input.rc_failsafe || input_set.input.channel_count < 4 || rc_timeout) {
		// signal is lost or not enough channels
		int aaa,bbb,ccc,ddd,eee;
		aaa = input_set.input.rc_lost;
		bbb = input_set.input.rc_failsafe;
		ccc= input_set.input.channel_count;
		ddd = rc_timeout;
		eee = input_set.input.values[0];
		input_set.signal_valid = false;
		mavlink_log_critical(&_mavlink_log_pub, "input_set.signal_valid = false;");
		mavlink_log_emergency(&_mavlink_log_pub, "%d %d %d %d %d", aaa, bbb, ccc,ddd,eee);

	}
}

void
RCUpdate::set_slave_signal_validity(InputSlaveRCset &input_slave_set)
{	//mavlink_log_critical(&_mavlink_log_pub, "set_slave_signal_validity(InputSlaveRCset &input_slave_set)");
 	//PX4_INFO("set_slave_signal_validity(InputSlaveRCset &input_slave_set)");
	// detect RC signal loss
	input_slave_set.signal_valid = true;

	const bool rc_timeout = (hrt_absolute_time() - input_slave_set.slave_rc.timestamp_last_signal) > hrt_abstime(
					_param_rc_lost_t.get() * 1e6f);

	// check flags and require at least four channels to consider the signal valid
	if (input_slave_set.slave_rc.rc_lost || input_slave_set.slave_rc.rc_failsafe
	    || input_slave_set.slave_rc.channel_count < 4 || rc_timeout) {
		// signal is lost or not enough channels
		input_slave_set.signal_valid = false;

	}
}

void
RCUpdate::process_failsafe_channel(InputRCset &input_set, const rc_channels_s &channels)
{
	// check failsafe
	int8_t fs_ch = channels.function[_param_rc_map_failsafe.get()]; // get channel mapped to throttle

	if (_param_rc_map_failsafe.get() > 0) { // if not 0, use channel number instead of rc.function mapping
		fs_ch = _param_rc_map_failsafe.get() - 1;
	}

	if (_param_rc_fails_thr.get() > 0 && fs_ch >= 0) {
		// failsafe configured
		if ((_param_rc_fails_thr.get() < _parameters.min[fs_ch] && input_set.input.values[fs_ch] < _param_rc_fails_thr.get()) ||
		    (_param_rc_fails_thr.get() > _parameters.max[fs_ch] && input_set.input.values[fs_ch] > _param_rc_fails_thr.get())) {
			// failsafe triggered, signal is lost by receiver
			input_set.signal_valid = false;
		}
	}
}

void
RCUpdate::scale_raw_channels(InputRCset &input_set)
{
	unsigned channel_limit = input_set.input.channel_count;

	if (channel_limit > RC_MAX_CHAN_COUNT) {
		channel_limit = RC_MAX_CHAN_COUNT;
	}

	/* read out and scale values from raw message even if signal is invalid */
	for (unsigned int i = 0; i < channel_limit; i++) {

		/*
		 * 1) Constrain to min/max values, as later processing depends on bounds.
		 */
		if (input_set.input.values[i] < _parameters.min[i]) {
			input_set.input.values[i] = _parameters.min[i];
		}

		if (input_set.input.values[i] > _parameters.max[i]) {
			input_set.input.values[i] = _parameters.max[i];
		}

		/*
		 * 2) Scale around the mid point differently for lower and upper range.
		 *
		 * This is necessary as they don't share the same endpoints and slope.
		 *
		 * First normalize to 0..1 range with correct sign (below or above center),
		 * the total range is 2 (-1..1).
		 * If center (trim) == min, scale to 0..1, if center (trim) == max,
		 * scale to -1..0.
		 *
		 * As the min and max bounds were enforced in step 1), division by zero
		 * cannot occur, as for the case of center == min or center == max the if
		 * statement is mutually exclusive with the arithmetic NaN case.
		 *
		 * DO NOT REMOVE OR ALTER STEP 1!
		 */
		if (input_set.input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
			_rc.channels[i] = (input_set.input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(
						  _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

		} else if (input_set.input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
			_rc.channels[i] = (input_set.input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(
						  _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

		} else {
			/* in the configured dead zone, output zero */
			_rc.channels[i] = 0.0f;
		}

		_rc.channels[i] *= _parameters.rev[i];

		/* handle any parameter-induced blowups */
		if (!PX4_ISFINITE(_rc.channels[i])) {
			_rc.channels[i] = 0.0f;
		}
	}

	_rc.channel_count = input_set.input.channel_count;
	_rc.rssi = input_set.input.rssi;
	_rc.signal_lost = !input_set.signal_valid;
	_rc.timestamp = input_set.input.timestamp_last_signal;
	_rc.frame_drop_count = input_set.input.rc_lost_frame_count;
}


bool
RCUpdate::map_from_rc_channel_functions(InputRCset &input_set)
{
	/* only publish manual control if the signal is still present and was present once */
	if (!(input_set.signal_valid && input_set.input.timestamp_last_signal > 0)) {
		return false;
	}

	_manual_sp.timestamp = input_set.input.timestamp_last_signal;

	/* set mode slot to unassigned */
	_manual_sp.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;

	/* limit controls */
	_manual_sp.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
	_manual_sp.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
	_manual_sp.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
	_manual_sp.z = (get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, -1.0, 1.0) + 1.f) / 2.f;
	_manual_sp.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
	_manual_sp.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
	_manual_sp.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
	_manual_sp.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
	_manual_sp.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
	_manual_sp.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);

	/* filter controls */
	_manual_sp.y = math::constrain(_filter_roll.apply(_manual_sp.y), -1.f, 1.f);
	_manual_sp.x = math::constrain(_filter_pitch.apply(_manual_sp.x), -1.f, 1.f);
	_manual_sp.r = math::constrain(_filter_yaw.apply(_manual_sp.r), -1.f, 1.f);
	_manual_sp.z = math::constrain(_filter_throttle.apply(_manual_sp.z), 0.f, 1.f);

	if (_param_rc_map_fltmode.get() > 0) {

		/* the number of valid slots equals the index of the max marker minus one */
		const int num_slots = manual_control_setpoint_s::MODE_SLOT_NUM;

		/* the half width of the range of a slot is the total range
		 * divided by the number of slots, again divided by two
		 */
		const float slot_width_half = 2.0f / num_slots / 2.0f;

		/* min is -1, max is +1, range is 2. We offset below min and max */
		const float slot_min = -1.0f - 0.05f;
		const float slot_max = 1.0f + 0.05f;

		/* the slot gets mapped by first normalizing into a 0..1 interval using min
		 * and max. Then the right slot is obtained by multiplying with the number of
		 * slots. And finally we add half a slot width to ensure that integer rounding
		 * will take us to the correct final index.
		 */
		_manual_sp.mode_slot = (((((_rc.channels[_param_rc_map_fltmode.get() - 1] - slot_min) * num_slots) +
					  slot_width_half) /
					 (slot_max - slot_min)) + (1.0f / num_slots));

		if (_manual_sp.mode_slot >= num_slots) {
			_manual_sp.mode_slot = num_slots - 1;
		}
	}

	/* mode switches */
	_manual_sp.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE,
				 _param_rc_auto_th.get(), _param_rc_auto_th.get() < 0.f,
				 _param_rc_assist_th.get(), _param_rc_assist_th.get() < 0.f);
	_manual_sp.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
				      _param_rc_ratt_th.get(), _param_rc_ratt_th.get() < 0.f);
	_manual_sp.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL,
				   _param_rc_posctl_th.get(), _param_rc_posctl_th.get() < 0.f);
	_manual_sp.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN,
				   _param_rc_return_th.get(), _param_rc_return_th.get() < 0.f);
	_manual_sp.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER,
				   _param_rc_loiter_th.get(), _param_rc_loiter_th.get() < 0.f);
	_manual_sp.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO,
				 _param_rc_acro_th.get(), _param_rc_acro_th.get() < 0.f);
	_manual_sp.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
				     _param_rc_offb_th.get(), _param_rc_offb_th.get() < 0.f);
	_manual_sp.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
				 _param_rc_killswitch_th.get(), _param_rc_killswitch_th.get() < 0.f);
	_manual_sp.arm_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH,
				_param_rc_armswitch_th.get(), _param_rc_armswitch_th.get() < 0.f);
	_manual_sp.transition_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION,
				       _param_rc_trans_th.get(), _param_rc_trans_th.get() < 0.f);
	_manual_sp.gear_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_GEAR,
				 _param_rc_gear_th.get(), _param_rc_gear_th.get() < 0.f);
	_manual_sp.stab_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_STAB,
				 _param_rc_stab_th.get(), _param_rc_stab_th.get() < 0.f);
	_manual_sp.man_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MAN,
				_param_rc_man_th.get(), _param_rc_man_th.get() < 0.f);
	_manual_sp.obsavoid_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OBSAVOIDSWITCH,
				     _param_rc_obsavoid_th.get(), _param_rc_obsavoid_th.get() < 0,
				     _param_rc_obsavoid_mid_th.get(), _param_rc_obsavoid_mid_th.get() < 0);

	return true;
}

void
RCUpdate::publish_manual_inputs()
{	mavlink_log_critical(&_mavlink_log_pub, "publish_manual_inputs");
	//PX4_INFO("publish_manual_inputs");
	/* publish manual_control_setpoint topic */
	_manual_control_setpoint_pub.publish(_manual_sp);

	/* copy from mapped manual control to control group 3 */
	struct actuator_controls_s actuator_group_3 = {};

	actuator_group_3.timestamp = _manual_sp.timestamp;
	actuator_group_3.control[0] = _manual_sp.y;
	actuator_group_3.control[1] = _manual_sp.x;
	actuator_group_3.control[2] = _manual_sp.r;
	actuator_group_3.control[3] = _manual_sp.z;
	actuator_group_3.control[4] = _manual_sp.flaps;
	actuator_group_3.control[5] = _manual_sp.aux1;
	actuator_group_3.control[6] = _manual_sp.aux2;
	actuator_group_3.control[7] = _manual_sp.aux3;

	/* publish actuator_controls_3 topic */
	_actuator_group_3_pub.publish(actuator_group_3);
}

void
RCUpdate::publish_rc_channels()
{	mavlink_log_critical(&_mavlink_log_pub, "rc pub");
	//PX4_INFO("rc pub");
	/* publish rc_channels topic even if signal is invalid, for debug */
	_rc_pub.publish(_rc);
}

void
RCUpdate::yuneec_rc_map()
{	mavlink_log_critical(&_mavlink_log_pub, "in yuneec rc map");
   	//PX4_INFO("in yuneec rc map");
	bool rc_updated = false;


		// check if subscriber has updated
		orb_check(_inputs_rc.sub, &rc_updated);
		//mavlink_log_critical(&_mavlink_log_pub, "input_rc updated");
		// copy message into struct
		if (rc_updated) {
			mavlink_log_critical(&_mavlink_log_pub, "orb_check(_inputs_rc[i].sub, &rc_updated);");
			//PX4_INFO("input_rc updated");
			orb_copy(ORB_ID(input_rc), _inputs_rc.sub, &_inputs_rc.input);

			// Get priority
			ORB_PRIO priority = ORB_PRIO_UNINITIALIZED;
			orb_priority(_inputs_rc.sub, &priority);

			if (priority == ORB_PRIO_DEFAULT) {
				//mavlink_log_critical(&_mavlink_log_pub, "ORB_PRIO_DEFAULT");
				//PX4_INFO("ORB_PRIO_DEFAULT");
				// handle special failsafe channel for receivers
				process_failsafe_channel(_inputs_rc, _rc);
			}
		}

		// check signal validity
		set_signal_validity(_inputs_rc);


	// check if subscriber has updated
	orb_check(_slave_rc.slave_rc_sub, &rc_updated);

	if (rc_updated) {
		mavlink_log_critical(&_mavlink_log_pub, "slave_rc updated");
		//PX4_INFO("slave_rc updated");
		orb_copy(ORB_ID(slave_rc), _slave_rc.slave_rc_sub, &_slave_rc.slave_rc);

	}

	// check slave rc validity
	set_slave_signal_validity(_slave_rc);

	// generate manual setpoints based on link mode
	if (_param_rc_link_mode.get() == 0) {
			mavlink_log_critical(&_mavlink_log_pub, "link mode = 0");
			//PX4_INFO("link mode = 0");
		// rc only mapping is required
		if (map_to_control_setpoint(ORB_PRIO_HIGH)) {
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_RC;
			publish_manual_inputs();
			//always publish rc_channels
			publish_rc_channels();

		}

	} else if (_param_rc_link_mode.get() == 1) {
			//mavlink_log_critical(&_mavlink_log_pub, "link mode = 1");
		// mavlink override is required
		// check if there is input data with high priority
		if (map_to_control_setpoint(ORB_PRIO_HIGH)) {
		//	mavlink_log_critical(&_mavlink_log_pub, "link mode = 1  && priority HIGH");
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
			publish_manual_inputs();
			//publish_rc_channels();

		} else if (map_to_control_setpoint(ORB_PRIO_DEFAULT)) {
			// No mavlink message. Re-use RC if available
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_RC;
			publish_manual_inputs();
			//always publish rc_channels
			publish_rc_channels();
		}

	} else if (_param_rc_link_mode.get() == 2) {
			//mavlink_log_critical(&_mavlink_log_pub, "link mode = 2");
		// use mavlink as backup in case rc is lost
		if (map_to_control_setpoint(ORB_PRIO_DEFAULT)) {
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_RC;
			publish_manual_inputs();
			//always publish rc_channels
			publish_rc_channels();

		} else if (map_to_control_setpoint(ORB_PRIO_HIGH)) {
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;
			publish_manual_inputs();
		}

	} else if (_param_rc_link_mode.get() == 3) {
			//mavlink_log_critical(&_mavlink_log_pub, "link mode = 3");
		// use zigbee as backup in case rc is lost
		if (map_to_control_setpoint(ORB_PRIO_HIGH)) {
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0;

			// team mode slave control
			if (map_from_team_mode()) {
				// Team mode did not succeed, but nothing to do.
			}

			publish_manual_inputs();
			//always publish rc_channels
			publish_rc_channels();

		} else if (map_to_control_setpoint(ORB_PRIO_DEFAULT)) {
			_manual_sp.data_source = manual_control_setpoint_s::SOURCE_RC;

			// team mode slave control
			if (map_from_team_mode()) {
				// Team mode did not succeed, but nothing to do.
			}

			publish_manual_inputs();
		}
	}
}
#endif


void
RCUpdate::rc_parameter_map_poll(bool forced)
{
	if (_rc_parameter_map_sub.updated() || forced) {
		_rc_parameter_map_sub.copy(&_rc_parameter_map);

		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				_parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				_parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}

		}

		PX4_DEBUG("rc to parameter map updated");
		//mavlink_log_critical(&_mavlink_log_pub, "rc to parameter map updated!!");
			//PX4_INFO("rc to parameter map updated!!");
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
				  i,
				  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
				  (double)_rc_parameter_map.scale[i],
				  (double)_rc_parameter_map.value0[i],
				  (double)_rc_parameter_map.value_min[i],
				  (double)_rc_parameter_map.value_max[i]
				 );
		}
	}
}

float
RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value)
{
	if (_rc.function[func] >= 0) {
		float value = _rc.channels[_rc.function[func]];
		return math::constrain(value, min_value, max_value);

	} else {
		return 0.0f;
	}
}

switch_pos_t
RCUpdate::get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else if (mid_inv ? value < mid_th : value > mid_th) {
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

switch_pos_t
RCUpdate::get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

void
RCUpdate::set_params_from_rc()
{
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0);

		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);

			param_set(_parameter_handles.rc_param[i], &param_val);
		}
	}
}

void
RCUpdate::Run()
{

	// if (should_exit()) {
	// 	//_input_rc_sub.unregisterCallback();
	// 	exit_and_cleanup();
	// 	return;
	// }

	perf_begin(_loop_perf);
	ScheduleDelayed(10);
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
			mavlink_log_critical(&_mavlink_log_pub, "RUN  _parameter_update_sub.updated()");
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	rc_parameter_map_poll();

//mx3g-jh
#ifdef YUNEEC_INPUT_RC_MAP_MANUAL_CONTROL
	//mavlink_log_critical(&_mavlink_log_pub, "rc_update RUN");
	//PX4_INFO("rc_update RUN");
	yuneec_rc_map();

#else

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	input_rc_s rc_input;

	if (_input_rc_sub.copy(&rc_input)) {

		// warn if the channel count is changing (possibly indication of error)
		if (!rc_input.rc_lost && (_channel_count_previous != rc_input.channel_count) && (_channel_count_previous > 0)) {
			PX4_DEBUG("RC channel count changed %d -> %d", _channel_count_previous, rc_input.channel_count);
		}

		_channel_count_previous = rc_input.channel_count;

		/* detect RC signal loss */
		bool signal_lost = true;

		/* check flags and require at least four channels to consider the signal valid */
		if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;


		} else if ((rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM ||
			    rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM)
			   && rc_input.channel_count == 16) {

			// This is a specific RC lost check for RFD 868+/900 Modems on PPM.
			// The observation was that when RC is lost, 16 channels are active and the first 12 are 1000
			// and the remaining ones are 0.

			for (unsigned int i = 0; i < 16; i++) {
				if (i < 12 && rc_input.values[i] > 999 && rc_input.values[i] < 1005) {
					signal_lost = true;

				} else if (rc_input.values[i] == 0) {
					signal_lost = true;

				} else {
					signal_lost = false;
					break;
				}
			}

		} else {
			/* signal looks good */
			signal_lost = false;

			/* check failsafe */
			int8_t fs_ch = _rc.function[_param_rc_map_failsafe.get()]; // get channel mapped to throttle

			if (_param_rc_map_failsafe.get() > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _param_rc_map_failsafe.get() - 1;
			}

			if (_param_rc_fails_thr.get() > 0 && fs_ch >= 0) {
				/* failsafe configured */
				if ((_param_rc_fails_thr.get() < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _param_rc_fails_thr.get()) ||
				    (_param_rc_fails_thr.get() > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _param_rc_fails_thr.get())) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > RC_MAX_CHAN_COUNT) {
			channel_limit = RC_MAX_CHAN_COUNT;
		}

		/* read out and scale values from raw message even if signal is invalid */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			rc_input.values[i] = math::constrain(rc_input.values[i], _parameters.min[i], _parameters.max[i]);

			/*
			 * 2) Scale around the mid point differently for lower and upper range.
			 *
			 * This is necessary as they don't share the same endpoints and slope.
			 *
			 * First normalize to 0..1 range with correct sign (below or above center),
			 * the total range is 2 (-1..1).
			 * If center (trim) == min, scale to 0..1, if center (trim) == max,
			 * scale to -1..0.
			 *
			 * As the min and max bounds were enforced in step 1), division by zero
			 * cannot occur, as for the case of center == min or center == max the if
			 * statement is mutually exclusive with the arithmetic NaN case.
			 *
			 * DO NOT REMOVE OR ALTER STEP 1!
			 */
			if (rc_input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(
							  _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

			} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(
							  _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

			} else {
				/* in the configured dead zone, output zero */
				_rc.channels[i] = 0.0f;
			}

			if (_parameters.rev[i]) {
				_rc.channels[i] = -_rc.channels[i];
			}


			/* handle any parameter-induced blowups */
			if (!PX4_ISFINITE(_rc.channels[i])) {
				_rc.channels[i] = 0.0f;
			}
		}

		_rc.channel_count = rc_input.channel_count;
		_rc.rssi = rc_input.rssi;
		_rc.signal_lost = signal_lost;
		_rc.timestamp = rc_input.timestamp_last_signal;
		_rc.frame_drop_count = rc_input.rc_lost_frame_count;

		/* publish rc_channels topic even if signal is invalid, for debug */
		_rc_pub.publish(_rc);

		/* only publish manual control if the signal is still present and was present once */
		if (!signal_lost && rc_input.timestamp_last_signal > 0) {

			/* initialize manual setpoint */
			manual_control_setpoint_s manual_control_setpoint{};
			/* set mode slot to unassigned */
			manual_control_setpoint.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
			/* set the timestamp to the last signal time */
			manual_control_setpoint.timestamp = rc_input.timestamp_last_signal;
			manual_control_setpoint.data_source = manual_control_setpoint_s::SOURCE_RC;

			/* limit controls */
			manual_control_setpoint.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
			manual_control_setpoint.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
			manual_control_setpoint.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
			manual_control_setpoint.z = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, 0.0, 1.0);
			manual_control_setpoint.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
			manual_control_setpoint.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
			manual_control_setpoint.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
			manual_control_setpoint.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
			manual_control_setpoint.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
			manual_control_setpoint.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);
			manual_control_setpoint.aux6 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_6, -1.0, 1.0);

			if (_param_rc_map_fltmode.get() > 0) {
				/* number of valid slots */
				const int num_slots = manual_control_setpoint_s::MODE_SLOT_NUM;

				/* the half width of the range of a slot is the total range
				 * divided by the number of slots, again divided by two
				 */
				const float slot_width_half = 2.0f / num_slots / 2.0f;

				/* min is -1, max is +1, range is 2. We offset below min and max */
				const float slot_min = -1.0f - 0.05f;
				const float slot_max = 1.0f + 0.05f;

				/* the slot gets mapped by first normalizing into a 0..1 interval using min
				 * and max. Then the right slot is obtained by multiplying with the number of
				 * slots. And finally we add half a slot width to ensure that integer rounding
				 * will take us to the correct final index.
				 */
				manual_control_setpoint.mode_slot = (((((_rc.channels[_param_rc_map_fltmode.get() - 1] - slot_min) * num_slots) +
								       slot_width_half)
								      / (slot_max - slot_min)) + (1.0f / num_slots)) + 1;

				if (manual_control_setpoint.mode_slot > num_slots) {
					manual_control_setpoint.mode_slot = num_slots;
				}
			}

			/* mode switches */
			manual_control_setpoint.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE,
							      _param_rc_auto_th.get(), _param_rc_auto_th.get() < 0.f,
							      _param_rc_assist_th.get(), _param_rc_assist_th.get() < 0.f);

			manual_control_setpoint.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
					_param_rc_ratt_th.get(), _param_rc_ratt_th.get() < 0.f);
			manual_control_setpoint.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL,
								_param_rc_posctl_th.get(), _param_rc_posctl_th.get() < 0.f);
			manual_control_setpoint.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN,
								_param_rc_return_th.get(), _param_rc_return_th.get() < 0.f);
			manual_control_setpoint.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER,
								_param_rc_loiter_th.get(), _param_rc_loiter_th.get() < 0.f);
			manual_control_setpoint.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO,
							      _param_rc_acro_th.get(), _param_rc_acro_th.get() < 0.f);
			manual_control_setpoint.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
					_param_rc_offb_th.get(), _param_rc_offb_th.get() < 0.f);
			manual_control_setpoint.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
							      _param_rc_killswitch_th.get(), _param_rc_killswitch_th.get() < 0.f);
			manual_control_setpoint.arm_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH,
							     _param_rc_armswitch_th.get(), _param_rc_armswitch_th.get() < 0.f);
			manual_control_setpoint.transition_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION,
					_param_rc_trans_th.get(), _param_rc_trans_th.get() < 0.f);
			manual_control_setpoint.gear_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_GEAR,
							      _param_rc_gear_th.get(), _param_rc_gear_th.get() < 0.f);
			manual_control_setpoint.stab_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_STAB,
							      _param_rc_stab_th.get(), _param_rc_stab_th.get() < 0.f);
			manual_control_setpoint.man_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MAN,
							     _param_rc_man_th.get(), _param_rc_man_th.get() < 0.f);

			/* publish manual_control_setpoint topic */
			_manual_control_setpoint_pub.publish(manual_control_setpoint);

			/* copy from mapped manual_control_setpoint control to control group 3 */
			actuator_controls_s actuator_group_3{};

			actuator_group_3.timestamp = rc_input.timestamp_last_signal;

			actuator_group_3.control[0] = manual_control_setpoint.y;
			actuator_group_3.control[1] = manual_control_setpoint.x;
			actuator_group_3.control[2] = manual_control_setpoint.r;
			actuator_group_3.control[3] = manual_control_setpoint.z;
			actuator_group_3.control[4] = manual_control_setpoint.flaps;
			actuator_group_3.control[5] = manual_control_setpoint.aux1;
			actuator_group_3.control[6] = manual_control_setpoint.aux2;
			actuator_group_3.control[7] = manual_control_setpoint.aux3;

			/* publish actuator_controls_3 topic */
			_actuator_group_3_pub.publish(actuator_group_3);

			/* Update parameters from RC Channels (tuning with RC) if activated */
			if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1_s) {
				set_params_from_rc();
				_last_rc_to_param_map_time = hrt_absolute_time();
			}
		}
	}

//mx3g-jh
#endif
	perf_end(_loop_perf);
}

int
RCUpdate::task_spawn(int argc, char *argv[])
{
	RCUpdate *instance = new RCUpdate();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			//mavlink_log_critical(&_mavlink_log_pub, "task_spawn return PX4_OK");
			PX4_INFO("task_spawn rc_update return PX4_OK");

			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	//mavlink_log_critical(&_mavlink_log_pub, "task_spawn return PX4_ERROR");
	PX4_INFO("task_spawn return PX4_ERROR");
	return PX4_ERROR;

}

int
RCUpdate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
RCUpdate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches,
low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.

### Implementation
To reduce control latency, the module is scheduled on input_rc publications.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_update", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace RCUpdate

extern "C" __EXPORT int rc_update_main(int argc, char *argv[])
{
	return RCUpdate::RCUpdate::main(argc, argv);
}
