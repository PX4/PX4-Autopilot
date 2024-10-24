/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControl.hpp"

#include <px4_platform_common/events.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_command.h>

ManualControl::ManualControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ManualControl::~ManualControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ManualControl::init()
{
	ScheduleNow();
	return true;
}

void ManualControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_system_id = vehicle_status.system_id;
			_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
			_vtol = vehicle_status.is_vtol;
		}
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_stick_arm_hysteresis.set_hysteresis_time_from(false, _param_com_rc_arm_hyst.get() * 1_ms);
		_stick_disarm_hysteresis.set_hysteresis_time_from(false, _param_com_rc_arm_hyst.get() * 1_ms);
		_button_hysteresis.set_hysteresis_time_from(false, _param_com_rc_arm_hyst.get() * 1_ms);

		_selector.setRcInMode(_param_com_rc_in_mode.get());
		_selector.setTimeout(_param_com_rc_loss_t.get() * 1_s);

		// MAN_ARM_GESTURE
		if (_param_man_arm_gesture.get() == 1) {
			// RC_MAP_ARM_SW & MAN_ARM_GESTURE: disable arm gesture if an arm switch is configured
			param_t param_rc_map_arm_sw = param_find("RC_MAP_ARM_SW");

			if (param_rc_map_arm_sw != PARAM_INVALID) {
				int32_t rc_map_arm_sw = 0;
				param_get(param_rc_map_arm_sw, &rc_map_arm_sw);

				if (rc_map_arm_sw > 0) {
					_param_man_arm_gesture.set(0); // disable arm gesture
					_param_man_arm_gesture.commit();

					orb_advert_t mavlink_log_pub = ORB_ADVERT_INVALID;
					mavlink_log_critical(&mavlink_log_pub, "Arm stick gesture disabled if arm switch in use\t")
					/* EVENT
					* @description <param>MAN_ARM_GESTURE</param> is now set to disable arm/disarm stick gesture.
					*/
					events::send(events::ID("rc_update_arm_stick_gesture_disabled_with_switch"), {events::Log::Info, events::LogInternal::Disabled},
						     "Arm stick gesture disabled if arm switch in use");
				}
			}

			// MC_AIRMODE & MAN_ARM_GESTURE: check for unsafe Airmode settings: yaw airmode requires disabling the stick arm gesture
			if ((_param_man_arm_gesture.get() == 1) && (_rotary_wing || _vtol)) {
				param_t param_mc_airmode = param_find("MC_AIRMODE");

				if (param_mc_airmode != PARAM_INVALID) {
					int32_t airmode = 0;
					param_get(param_mc_airmode, &airmode);

					if (airmode == 2) {
						airmode = 1; // change to roll/pitch airmode
						param_set(param_mc_airmode, &airmode);

						orb_advert_t mavlink_log_pub = ORB_ADVERT_INVALID;
						mavlink_log_critical(&mavlink_log_pub, "Yaw Airmode requires disabling the stick arm gesture\t")
						/* EVENT
						* @description <param>MC_AIRMODE</param> is now set to roll/pitch airmode.
						*/
						events::send(events::ID("commander_airmode_requires_no_arm_gesture"), {events::Log::Error, events::LogInternal::Disabled},
							     "Yaw Airmode requires disabling the stick arm gesture");
					}
				}
			}
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	_selector.updateValidityOfChosenInput(now);

	for (int i = 0; i < MAX_MANUAL_INPUT_COUNT; i++) {
		manual_control_setpoint_s manual_control_input;

		if (_manual_control_setpoint_subs[i].update(&manual_control_input)) {
			_selector.updateWithNewInputSample(now, manual_control_input, i);
		}
	}

	manual_control_switches_s switches;
	bool switches_updated = _manual_control_switches_sub.update(&switches);

	if (_selector.setpoint().valid) {
		_published_invalid_once = false;

		processStickArming(_selector.setpoint());

		// User override by stick
		const float dt_s = (now - _last_time) / 1e6f;
		const float minimum_stick_change = 0.01f * _param_com_rc_stick_ov.get();

		_selector.setpoint().sticks_moving = (fabsf(_roll_diff.update(_selector.setpoint().roll, dt_s)) > minimum_stick_change)
						     || (fabsf(_pitch_diff.update(_selector.setpoint().pitch, dt_s)) > minimum_stick_change)
						     || (fabsf(_yaw_diff.update(_selector.setpoint().yaw, dt_s)) > minimum_stick_change)
						     || (fabsf(_throttle_diff.update(_selector.setpoint().throttle, dt_s)) > minimum_stick_change);

		if (switches_updated) {
			// Only use switches if current source is RC as well.
			if (_selector.setpoint().data_source == manual_control_setpoint_s::SOURCE_RC) {
				if (_previous_switches_initialized) {
					if (switches.mode_slot != _previous_switches.mode_slot) {
						evaluateModeSlot(switches.mode_slot);
					}

					if (_param_com_arm_swisbtn.get()) {
						// Arming button
						const bool previous_button_hysteresis = _button_hysteresis.get_state();
						_button_hysteresis.set_state_and_update(switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON, now);

						if (!previous_button_hysteresis && _button_hysteresis.get_state()) {
							sendActionRequest(action_request_s::ACTION_TOGGLE_ARMING, action_request_s::SOURCE_RC_BUTTON);
						}

					} else {
						// Arming switch
						if (switches.arm_switch != _previous_switches.arm_switch) {
							if (switches.arm_switch == manual_control_switches_s::SWITCH_POS_ON) {
								sendActionRequest(action_request_s::ACTION_ARM, action_request_s::SOURCE_RC_SWITCH);

							} else if (switches.arm_switch == manual_control_switches_s::SWITCH_POS_OFF) {
								sendActionRequest(action_request_s::ACTION_DISARM, action_request_s::SOURCE_RC_SWITCH);
							}
						}
					}

					if (switches.return_switch != _previous_switches.return_switch) {
						if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
							sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_SWITCH,
									  vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

						} else if (switches.return_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							evaluateModeSlot(switches.mode_slot);
						}
					}

					if (switches.loiter_switch != _previous_switches.loiter_switch) {
						if (switches.loiter_switch == manual_control_switches_s::SWITCH_POS_ON) {
							sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_SWITCH,
									  vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);

						} else if (switches.loiter_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							evaluateModeSlot(switches.mode_slot);
						}
					}

					if (switches.offboard_switch != _previous_switches.offboard_switch) {
						if (switches.offboard_switch == manual_control_switches_s::SWITCH_POS_ON) {
							sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_SWITCH,
									  vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

						} else if (switches.offboard_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							evaluateModeSlot(switches.mode_slot);
						}
					}

					if (switches.kill_switch != _previous_switches.kill_switch) {
						if (switches.kill_switch == manual_control_switches_s::SWITCH_POS_ON) {
							sendActionRequest(action_request_s::ACTION_KILL, action_request_s::SOURCE_RC_SWITCH);

						} else if (switches.kill_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							sendActionRequest(action_request_s::ACTION_UNKILL, action_request_s::SOURCE_RC_SWITCH);
						}
					}

					if (switches.gear_switch != _previous_switches.gear_switch
					    && _previous_switches.gear_switch != manual_control_switches_s::SWITCH_POS_NONE) {

						if (switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
							publishLandingGear(landing_gear_s::GEAR_UP);

						} else if (switches.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							publishLandingGear(landing_gear_s::GEAR_DOWN);
						}
					}

					if (switches.transition_switch != _previous_switches.transition_switch) {
						if (switches.transition_switch == manual_control_switches_s::SWITCH_POS_ON) {
							sendActionRequest(action_request_s::ACTION_VTOL_TRANSITION_TO_FIXEDWING, action_request_s::SOURCE_RC_SWITCH);

						} else if (switches.transition_switch == manual_control_switches_s::SWITCH_POS_OFF) {
							sendActionRequest(action_request_s::ACTION_VTOL_TRANSITION_TO_MULTICOPTER, action_request_s::SOURCE_RC_SWITCH);
						}
					}

					if (switches.photo_switch != _previous_switches.photo_switch) {
						if (switches.photo_switch == manual_control_switches_s::SWITCH_POS_ON) {
							send_camera_mode_command(CameraMode::Image);
							send_photo_command();
						}
					}

					if (switches.video_switch != _previous_switches.video_switch) {
						if (switches.video_switch == manual_control_switches_s::SWITCH_POS_ON) {
							send_camera_mode_command(CameraMode::Video);
							send_video_command();
						}
					}

				} else {
					// Send an initial request to switch to the mode requested by RC
					evaluateModeSlot(switches.mode_slot);
				}

				_previous_switches_initialized = true;
				_previous_switches = switches;

			} else {
				_previous_switches_initialized = false;
			}
		}

		_selector.setpoint().timestamp = now;
		_manual_control_setpoint_pub.publish(_selector.setpoint());

		// If it's valid, this should really be valid but better safe than sorry.
		const int instance = _selector.instance();

		// Attach scheduling to new samples of the chosen input
		if (instance != _previous_manual_control_input_instance) {
			if ((0 <= _previous_manual_control_input_instance)
			    && (_previous_manual_control_input_instance < MAX_MANUAL_INPUT_COUNT)) {
				_manual_control_setpoint_subs[_previous_manual_control_input_instance].unregisterCallback();
			}

			if ((0 <= instance) && (instance < MAX_MANUAL_INPUT_COUNT)) {
				_manual_control_setpoint_subs[instance].registerCallback();
			}

			_previous_manual_control_input_instance = instance;
		}

		_manual_control_switches_sub.registerCallback();

	} else {
		if (!_published_invalid_once) {
			_published_invalid_once = true;
			_manual_control_setpoint_pub.publish(_selector.setpoint());
		}

		_roll_diff.reset();
		_pitch_diff.reset();
		_yaw_diff.reset();
		_throttle_diff.reset();
		_stick_arm_hysteresis.set_state_and_update(false, now);
		_stick_disarm_hysteresis.set_state_and_update(false, now);
		_button_hysteresis.set_state_and_update(false, now);
	}

	_last_time = now;

	// reschedule timeout
	ScheduleDelayed(200_ms);

	perf_end(_loop_perf);
}

void ManualControl::processStickArming(const manual_control_setpoint_s &input)
{
	// Arm gesture
	const bool right_stick_centered = (fabsf(input.pitch) < 0.1f) && (fabsf(input.roll) < 0.1f);
	const bool left_stick_lower_right = (input.throttle < -0.8f) && (input.yaw > 0.9f);

	const bool previous_stick_arm_hysteresis = _stick_arm_hysteresis.get_state();
	_stick_arm_hysteresis.set_state_and_update(left_stick_lower_right && right_stick_centered, input.timestamp);

	if (_param_man_arm_gesture.get() && !previous_stick_arm_hysteresis && _stick_arm_hysteresis.get_state()) {
		sendActionRequest(action_request_s::ACTION_ARM, action_request_s::SOURCE_RC_STICK_GESTURE);
	}

	// Disarm gesture
	const bool left_stick_lower_left = (input.throttle < -0.8f) && (input.yaw < -0.9f);

	const bool previous_stick_disarm_hysteresis = _stick_disarm_hysteresis.get_state();
	_stick_disarm_hysteresis.set_state_and_update(left_stick_lower_left && right_stick_centered, input.timestamp);

	if (_param_man_arm_gesture.get() && !previous_stick_disarm_hysteresis && _stick_disarm_hysteresis.get_state()) {
		sendActionRequest(action_request_s::ACTION_DISARM, action_request_s::SOURCE_RC_STICK_GESTURE);
	}
}

void ManualControl::evaluateModeSlot(uint8_t mode_slot)
{
	switch (mode_slot) {
	case manual_control_switches_s::MODE_SLOT_NONE:
		break;

	case manual_control_switches_s::MODE_SLOT_1:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_1.get()));
		break;

	case manual_control_switches_s::MODE_SLOT_2:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_2.get()));
		break;

	case manual_control_switches_s::MODE_SLOT_3:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_3.get()));
		break;

	case manual_control_switches_s::MODE_SLOT_4:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_4.get()));
		break;

	case manual_control_switches_s::MODE_SLOT_5:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_5.get()));
		break;

	case manual_control_switches_s::MODE_SLOT_6:
		sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_MODE_SLOT,
				  navStateFromParam(_param_fltmode_6.get()));
		break;

	default:
		PX4_WARN("mode slot overflow");
		break;
	}
}

void ManualControl::sendActionRequest(int8_t action, int8_t source, int8_t mode)
{
	// We catch default unassigned mode slots which have value -1
	if (action == action_request_s::ACTION_SWITCH_MODE && mode < 0) {
		return;
	}

	action_request_s action_request{};
	action_request.action = action;
	action_request.source = source;
	action_request.mode = mode;
	action_request.timestamp = hrt_absolute_time();
	_action_request_pub.publish(action_request);
}

void ManualControl::publishLandingGear(int8_t action)
{
	landing_gear_s landing_gear{};
	landing_gear.landing_gear = action;
	landing_gear.timestamp = hrt_absolute_time();
	_landing_gear_pub.publish(landing_gear);
}

void ManualControl::send_camera_mode_command(CameraMode camera_mode)
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_SET_CAMERA_MODE;
	command.param2 = static_cast<float>(camera_mode);
	command.target_system = _system_id;
	command.target_component = 100; // any camera

	uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};
	command.timestamp = hrt_absolute_time();
	command_pub.publish(command);
}

void ManualControl::send_photo_command()
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_IMAGE_START_CAPTURE;
	command.param3 = 1; // one picture
	command.param4 = _image_sequence++;
	command.target_system = _system_id;
	command.target_component = 100; // any camera

	uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};
	command.timestamp = hrt_absolute_time();
	command_pub.publish(command);
}

void ManualControl::send_video_command()
{
	vehicle_command_s command{};

	if (_video_recording) {
		command.command = vehicle_command_s::VEHICLE_CMD_VIDEO_STOP_CAPTURE;
		command.param2 = 1; // status at 1 Hz

	} else {
		command.command = vehicle_command_s::VEHICLE_CMD_VIDEO_START_CAPTURE;
	}

	command.target_system = _system_id;
	command.target_component = 100; // any camera

	uORB::Publication<vehicle_command_s> command_pub{ORB_ID(vehicle_command)};
	command.timestamp = hrt_absolute_time();
	command_pub.publish(command);

	_video_recording = !_video_recording;
}

int ManualControl::task_spawn(int argc, char *argv[])
{
	ManualControl *instance = new ManualControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ManualControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ManualControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ManualControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module consuming manual_control_inputs publishing one manual_control_setpoint.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("manual_control", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int8_t ManualControl::navStateFromParam(int32_t param_value)
{
	switch(param_value) {
		case 0: return vehicle_status_s::NAVIGATION_STATE_MANUAL;
		case 1: return vehicle_status_s::NAVIGATION_STATE_ALTCTL;
		case 2: return vehicle_status_s::NAVIGATION_STATE_POSCTL;
		case 3: return vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
		case 4: return vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		case 5: return vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		case 6: return vehicle_status_s::NAVIGATION_STATE_ACRO;
		case 7: return vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		case 8: return vehicle_status_s::NAVIGATION_STATE_STAB;
		case 10: return vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF;
		case 11: return vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		case 12: return vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET;
		case 13: return vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND;
		case 14: return vehicle_status_s::NAVIGATION_STATE_ORBIT;
		case 15: return vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF;
	}
	return -1;
}

extern "C" __EXPORT int manual_control_main(int argc, char *argv[])
{
	return ManualControl::main(argc, argv);
}
