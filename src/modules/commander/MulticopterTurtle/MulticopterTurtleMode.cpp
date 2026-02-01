
#include "MulticopterTurtleMode.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/shutdown.h>
#include <lib/mathlib/math/Limits.hpp>
#include <uORB/topics/actuator_motors.h>
#include <parameters/param.h>
#include <inttypes.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

using namespace MulticopterTurtleUtil;

// Forward declaration of dshot_main (only available on hardware builds)
#if !defined(__PX4_POSIX)
extern "C" int dshot_main(int argc, char *argv[]);
#endif

// Wrapper function to handle dshot_main calls in both SITL and hardware builds
static int call_dshot_main(int argc, char *argv[])
{
#if defined(__PX4_POSIX)
	// In SITL, dshot_main is not available, so we simulate it
	// Just log and return success since there's no actual hardware
	const char *cmd = (argc > 1) ? argv[1] : "unknown";
	PX4_INFO("dshot command simulated (SITL): %s", cmd);
	return 0;
#else
	// On hardware, call the real dshot_main
	return dshot_main(argc, argv);
#endif
}

MulticopterTurtleMode::MulticopterTurtleMode(ModuleParams *parent) :
	ModuleParams(parent)
{
	getMotorData(_motor_data);
	_thr_factor = thr_factor();
	updateDshot3dParameter(false, false);


}

void MulticopterTurtleMode::update(const bool armed)
{
	// Check if we need to send 3d_off command after disarming
	// Only send if ESC state is ON but we're disarmed and not in ACTIVE_TURTLE or ENTERING_TURTLE
	// This handles the case when exiting ACTIVE_TURTLE - 3d_off will be sent after disarm completes
	if (_turtle_mode_esc_internal_state == TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON && 
	    !armed && 
	    _turtle_mode_state != TurtleModeState::ACTIVE_TURTLE &&
	    _turtle_mode_state != TurtleModeState::ENTERING_TURTLE) {
		// If we're in EXITING_TURTLE or OPTIONAL_TURTLE and disarmed, ensure 3D is off
		if (_turtle_mode_state == TurtleModeState::EXITING_TURTLE || 
		    _turtle_mode_state == TurtleModeState::OPTIONAL_TURTLE) {
			updateDshot3dParameter(false, armed);
		}
	}
	if (_param_com_turtle_en.get()) {

		// check if the TURTLE MODE need to be enabled
		// Allow regular arming even when in turtle mode - don't force exit from turtle mode

		const float aux_channel_value = getAuxChannelValue();
		const bool aux_triggered = (aux_channel_value > AUX_CHANNEL_THRESHOLD);

		switch (_turtle_mode_state) {
		case TurtleModeState::FLYING_DISABLED:
			if (!armed || _turtle_mode_armed) {
				// Don't allow entering ACTIVE_TURTLE while waiting for disarm
				if (aux_triggered && !_should_disarm_on_exit) {
					// Transition to ENTERING_TURTLE to enable 3D mode
					setState(TurtleModeState::ENTERING_TURTLE);
					updateDshot3dParameter(true, armed);
				} else {
					setState(TurtleModeState::OPTIONAL_TURTLE);
				}
			}
			break;

		case TurtleModeState::OPTIONAL_TURTLE:
			// Don't allow entering ACTIVE_TURTLE while waiting for disarm
			if (aux_triggered && !_should_disarm_on_exit) {
				// Transition to ENTERING_TURTLE to enable 3D mode
				setState(TurtleModeState::ENTERING_TURTLE);
				updateDshot3dParameter(true, armed);
			}
			break;

		case TurtleModeState::ENTERING_TURTLE:
			// Wait for 3D mode to be enabled, then transition to ACTIVE_TURTLE
			// Only transition to ACTIVE if 3D mode is successfully enabled
			// Note: updateDshot3dParameter only works when disarmed, so 3D mode can only be enabled when !armed
			if (!armed) {
				updateDshot3dParameter(true, armed);
			}
			if (_turtle_mode_esc_internal_state == TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON) {
				setState(TurtleModeState::ACTIVE_TURTLE);
			} else if (!aux_triggered) {
				// If aux channel released before 3D enabled, go back to OPTIONAL
				setState(TurtleModeState::OPTIONAL_TURTLE);
				if (!armed) {
					updateDshot3dParameter(false, armed);
				}
			}
			break;

		case TurtleModeState::ACTIVE_TURTLE:
			if (!aux_triggered) {
				// Transition to EXITING_TURTLE to disable 3D mode
				const bool was_turtle_armed = _turtle_mode_armed;
				setState(TurtleModeState::EXITING_TURTLE);
				_turtle_mode_armed = false;
				_should_disarm_on_exit = was_turtle_armed;
				_nav_state_change_requested = false;
				_waiting_for_dshot_command = false;
				updateDshot3dParameter(false, armed);
			} else {
				// While in ACTIVE_TURTLE: publish motor commands continuously
				// ESC internal state should already be ON (set when enable command succeeded)
				const MotorCommands motor_cmds = getMotorCommands();
				Quadrants quadrant = MulticopterTurtleUtil::getQuadrant(-MulticopterTurtleUtil::Threshold(motor_cmds.roll, 0.1f), -MulticopterTurtleUtil::Threshold(motor_cmds.pitch, 0.1f));
				PX4_INFO("current joystick Quadrant: %d", (int)quadrant);
				float motor_commands[12] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
				for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
					if (_motor_data[i].motor_quadrant == quadrant) {
						motor_commands[i] = _thr_factor*1.5f;
					} else {
						motor_commands[i] = NAN;
					}
				}
				publishMotorCommands(motor_commands);
			}
			break;

		case TurtleModeState::EXITING_TURTLE:
			
			_should_disarm_on_exit = true;
			px4_usleep(100000); // 100ms delay to allow 3D mode to disable
			updateDshot3dParameter(false, armed);
			setState(TurtleModeState::OPTIONAL_TURTLE);
			
			// Reboot the drone after exiting turtle mode (only on hardware, not in SITL)
#if defined(CONFIG_BOARDCTL_RESET)
			PX4_INFO("Exiting turtle mode - rebooting system");
			px4_reboot_request(REBOOT_REQUEST, 100000); // 100ms delay to allow 3D mode to disable
#else
			PX4_INFO("Exiting turtle mode - reboot skipped (SITL/non-hardware build)");
#endif
			
			break;
		}

		// Continue publishing disabled motors while waiting to disarm after leaving turtle mode
		// This prevents the control allocator from controlling motors until disarm completes

		if (_should_disarm_on_exit && armed) {
			const float disabled_motors[12] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
			publishMotorCommands(disabled_motors);
		} else if (!armed && _should_disarm_on_exit) {
			// Disarm completed, clear the flag
			_should_disarm_on_exit = false;
		}

	} else if (_turtle_mode_state != TurtleModeState::FLYING_DISABLED) {
		// Feature disabled: disable 3D mode first if needed, then reset to FLYING_DISABLED
		if (_turtle_mode_esc_internal_state == TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON) {
			// If in ACTIVE or ENTERING, transition through EXITING to properly disable 3D
			if (_turtle_mode_state == TurtleModeState::ACTIVE_TURTLE || 
			    _turtle_mode_state == TurtleModeState::ENTERING_TURTLE) {
				setState(TurtleModeState::EXITING_TURTLE);
				updateDshot3dParameter(false, armed);
			} else {
				// Already exiting or in other state, just disable 3D and reset
				updateDshot3dParameter(false, armed);
				setState(TurtleModeState::FLYING_DISABLED);
			}
		} else {
			// 3D already off, just reset state
			setState(TurtleModeState::FLYING_DISABLED);
		}
	}
}

void MulticopterTurtleMode::setState(TurtleModeState new_state)
{
	_previous_state = _turtle_mode_state;
	_turtle_mode_state = new_state;
}

void MulticopterTurtleMode::updateDshot3dParameter(bool enable, bool armed)
{
	// DShot commands only execute when motors are disarmed (output == 0)
	// For enable: we'll delay arming until command completes (300ms)
	// For disable: command will be sent after disarming

	if (armed) {
		// Do nothing if armed - > return
		return;
	}
	

	param_t param_handle = param_find("DSHOT_3D_ENABLE"); // find param for scaling to 3d 
	int result_param = PX4_ERROR;
	
	if (param_handle != PARAM_INVALID) {
		int32_t value = enable ? 1 : 0; // set param to 1 if enable, 0 if disable
		result_param = param_set(param_handle, &value);
	}

	const char *cmd_str = enable ? "3d_on" : "3d_off"; // change the dshot hardware  capability to be 3d or not (mavlink shell command)
	char *cmd_argv[] = { 
		(char*)"dshot", 
		(char*)cmd_str,
		nullptr 
	};


	int result_dshot = call_dshot_main(2, cmd_argv); // send the dshot command to the dshot hardware

	if (result_dshot == 0 && result_param == PX4_OK) {
		// Command succeeded - update internal state
		_turtle_mode_esc_internal_state = enable ? 
			TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON : 
			TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_OFF; // if have telemetry data from the esc need to add here function for each motor 
		if (enable) {
			// For enable: delay arming until command completes
			_waiting_for_dshot_command = true;
			_dshot_command_start_time = hrt_absolute_time();
		} else {
			return;
		}
	} else {
		if (result_param != PX4_OK || result_dshot != 0) {
			PX4_WARN("failed send dshot command to the dshot hardware"); // maby set 3d on to try run this function again
		}
	}
}

uint8_t MulticopterTurtleMode::getDesiredNavState() const
{
	// Only change nav_state when ACTIVE_TURTLE
	if (_turtle_mode_state == TurtleModeState::ACTIVE_TURTLE) {
		return vehicle_status_s::NAVIGATION_STATE_TURTLE;
	}
	
	// Revert nav_state once if we were in ACTIVE_TURTLE and are now leaving it
	if (_previous_state == TurtleModeState::ACTIVE_TURTLE && 
	    _turtle_mode_state != TurtleModeState::ACTIVE_TURTLE && 
	    !_nav_state_change_requested) {
		return vehicle_status_s::NAVIGATION_STATE_STAB;
	}
	
	return vehicle_status_s::NAVIGATION_STATE_MAX;
}

bool MulticopterTurtleMode::shouldArm() const
{
	// Only arm on transition to ACTIVE_TURTLE state (from ENTERING_TURTLE)
	if (_turtle_mode_state != TurtleModeState::ACTIVE_TURTLE || 
	    _previous_state != TurtleModeState::ENTERING_TURTLE) {
		return false;
	}
	
	// If we're waiting for DShot command to complete, delay arming
	// Command needs ~200ms to complete (10 repetitions at ~20ms each, with some margin)
	if (_waiting_for_dshot_command) {
		const hrt_abstime elapsed = hrt_elapsed_time(&_dshot_command_start_time);
		if (elapsed < 300000) { // 300ms delay to ensure command completes
			return false; // Not ready to arm yet
		}
		// Command should be complete now, allow arming
		const_cast<MulticopterTurtleMode*>(this)->_waiting_for_dshot_command = false;
		const_cast<MulticopterTurtleMode*>(this)->_turtle_mode_armed = true;
	}
	
	return true;
}


bool MulticopterTurtleMode::shouldDisarm() const
{
	// Disarm when exiting turtle mode (flag is set when leaving ACTIVE state)
	return _should_disarm_on_exit;
}

void MulticopterTurtleMode::clearDisarmFlag()
{
	_should_disarm_on_exit = false;
}

void MulticopterTurtleMode::markNavStateChangeRequested()
{
	_nav_state_change_requested = true;
}

float MulticopterTurtleMode::getAuxChannel() const
{
	return static_cast<float>(_param_turtle_aux_chn.get());
}


float MulticopterTurtleMode::getAuxChannelValue()
{
	manual_control_setpoint_s manual_control_setpoint;
	if (!_manual_control_setpoint_sub.copy(&manual_control_setpoint) || !manual_control_setpoint.valid) {
		return 0.0f;
	}
	const int32_t aux_channel = _param_turtle_aux_chn.get();
	const float aux_values[] = {
		0.0f,  // 0 (invalid)
		manual_control_setpoint.aux1,
		manual_control_setpoint.aux2,
		manual_control_setpoint.aux3,
		manual_control_setpoint.aux4,
		manual_control_setpoint.aux5,
		manual_control_setpoint.aux6
	};
	return (aux_channel >= 1 && aux_channel <= 6) ? aux_values[aux_channel] : 0.0f;
}




MulticopterTurtleUtil::MotorCommands MulticopterTurtleMode::getMotorCommands()
{
	manual_control_setpoint_s manual_control_setpoint;
	// Update manual control setpoint from subscription
	if (!_manual_control_setpoint_sub.copy(&manual_control_setpoint) || !manual_control_setpoint.valid) {
		MotorCommands cmds{};
		cmds.roll = NAN;
		cmds.pitch = NAN;
		return cmds;
	}

	float threshold = 0.1f;
	MotorCommands cmds{};
	cmds.roll = Threshold(manual_control_setpoint.roll, threshold);
	cmds.pitch = Threshold(manual_control_setpoint.pitch, threshold);
	return cmds;
}


void MulticopterTurtleMode::publishMotorCommands(const float throttle[12])
{
	if (!_actuator_motors_pub.advertised()) {
		_actuator_motors_pub.advertise();
	}
	
	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = actuator_motors.timestamp;
	actuator_motors.reversible_flags = ALL_MOTORS_REVERSIBLE;
	
	// Copy throttle values to motor controls
	// Clamp values to valid range [-1, 1] and handle NAN
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		if (PX4_ISFINITE(throttle[i])) {
			// Valid finite value - clamp to valid range [-1, 1]
			actuator_motors.control[i] = math::constrain(throttle[i], -1.0f, 1.0f);
		} else {
			// NAN or invalid value = motor disabled
			actuator_motors.control[i] = NAN;
		}
	}
	
	_actuator_motors_pub.publish(actuator_motors);
}

void MulticopterTurtleMode::getMotorData(MulticopterTurtleUtil::Motor_data motor_data[]){
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		char buffer[20];
		param_t param_handle;
		float value;

		motor_data[i].motor_number = i + 1;

		// Get rotor X position
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		param_handle = param_find(buffer);
		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &motor_data[i].x_position);
		} else {
			motor_data[i].x_position = 0.0f;
		}

		// Get rotor Y position
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		param_handle = param_find(buffer);
		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &motor_data[i].y_position);
		} else {
			motor_data[i].y_position = 0.0f;
		}

		// Get rotor direction from KM (moment coefficient)
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		param_handle = param_find(buffer);
		if (param_handle != PARAM_INVALID) {
			param_get(param_handle, &value);
			motor_data[i].motor_direction = (value > 0.0f);
		} else {
			motor_data[i].motor_direction = true;
		}
		motor_data[i].motor_quadrant = getQuadrant(motor_data[i].x_position, motor_data[i].y_position);
		// PX4_INFO("Motor %u: X=%f, Y=%f, Direction=%d, Quadrant=%d", i, (double)motor_data[i].x_position, (double)motor_data[i].y_position, motor_data[i].motor_direction, (int)motor_data[i].motor_quadrant);
	}
}





float MulticopterTurtleMode::thr_factor()
{
	float hover_throttle = _param_mpc_thr_hover.get();
	if (hover_throttle < 10.0f || hover_throttle > 90.0f) {
		return 50.0f;
	} else {
		return hover_throttle * 120.0f;
	}
}
