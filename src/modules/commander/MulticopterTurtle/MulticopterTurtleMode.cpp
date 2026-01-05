
#include "MulticopterTurtleMode.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/math/Limits.hpp>
#include <uORB/topics/actuator_motors.h>
#include <parameters/param.h>
#include <inttypes.h>
#include <math.h>
#include <float.h>

// Forward declaration for dshot_main
extern "C" int dshot_main(int argc, char *argv[]);

MulticopterTurtleMode::MulticopterTurtleMode(ModuleParams *parent) :
	ModuleParams(parent)
{
	PX4_INFO("MulticopterTurtleMode initialized, aux channel: %.0f", (double)getAuxChannel());
}

void MulticopterTurtleMode::update(const bool armed)
{
	// Check if we need to send 3d_off command after disarming
	// Only send if ESC state is ON but we're disarmed and not in ACTIVE_TURTLE
	// This handles the case when exiting ACTIVE_TURTLE - 3d_off will be sent after disarm completes
	// PX4_INFO("turtle mode state: %d", (int)_turtle_mode_state);
	// PX4_INFO("turtle mode esc internal state: %d", (int)_turtle_mode_esc_internal_state);

	if (_turtle_mode_esc_internal_state == TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON && 
	    !armed && 
	    _turtle_mode_state != TurtleModeState::ACTIVE_TURTLE) {

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
					setState(TurtleModeState::ACTIVE_TURTLE);
					updateDshot3dParameter(true, armed);
				} else {
					setState(TurtleModeState::OPTIONAL_TURTLE);
				}
			}
			break;

		case TurtleModeState::OPTIONAL_TURTLE:
			// Don't allow entering ACTIVE_TURTLE while waiting for disarm
			if (aux_triggered && !_should_disarm_on_exit) {
				setState(TurtleModeState::ACTIVE_TURTLE);
				updateDshot3dParameter(true, armed);
			}
			else {
				updateDshot3dParameter(false, armed);
			}
			break;


		case TurtleModeState::ACTIVE_TURTLE:
			if (!aux_triggered) {
				// Transition to OPTIONAL: disable 3D mode and disarm
				const bool was_turtle_armed = _turtle_mode_armed;
				setState(TurtleModeState::OPTIONAL_TURTLE);
				_turtle_mode_armed = false;
				_should_disarm_on_exit = was_turtle_armed;
				_nav_state_change_requested = false;
				_waiting_for_dshot_command = false;
				// Don't send 3d_off here if armed - it will be sent after disarming by the check at the top
				// The check at lines 28-32 will handle sending 3d_off after disarming
			} else {
				// While in ACTIVE_TURTLE: publish motor commands continuously
				// ESC internal state should already be ON (set when enable command succeeded)
				const MotorCommands motor_cmds = getMotorCommands();
				const float motor_commands[12] = {motor_cmds.roll, motor_cmds.pitch, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
				publishMotorCommands(motor_commands);
			}
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
		// Feature disabled: reset to FLYING_DISABLED
		setState(TurtleModeState::FLYING_DISABLED);
		// Disable 3D mode if it was enabled
		if (_turtle_mode_esc_internal_state == TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON) {
			updateDshot3dParameter(false, armed);
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
		PX4_WARN("Turtle mode: Cannot update DSHOT_3D_ENABLE while armed");
		return;
	}
	
	param_t param_handle = param_find("DSHOT_3D_ENABLE");
	int result_param = PX4_ERROR;
	
	if (param_handle != PARAM_INVALID) {
		int32_t value = enable ? 1 : 0;
		result_param = param_set(param_handle, &value);
	}
	
	usleep(1000000);
	// Send DShot command to change ESC direction
	const char *cmd_str = enable ? "3d_on" : "3d_off";
	char *cmd_argv[] = { 
		(char*)"dshot", 
		(char*)cmd_str,
		nullptr 
	};

	int result_dshot = dshot_main(2, cmd_argv);

	if (result_dshot == 0 && result_param == PX4_OK) {
		// Command succeeded - update internal state
		_turtle_mode_esc_internal_state = enable ? 
			TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_ON : 
			TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_OFF;
		
		if (enable) {
			// For enable: delay arming until command completes
			_waiting_for_dshot_command = true;
			_dshot_command_start_time = hrt_absolute_time();
		} else {
			return;
		}
	} else {
		if (result_param != PX4_OK) {
			PX4_WARN("Turtle mode: Failed to set DSHOT_3D_ENABLE parameter: %d", result_param);
		}
		if (result_dshot != 0) {
			PX4_WARN("Turtle mode: Failed to send DShot %s command: %d", cmd_str, result_dshot);
		}
		// On failure, don't update internal state - keep it as is
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
	// Only arm on transition to ACTIVE_TURTLE state
	if (_turtle_mode_state != TurtleModeState::ACTIVE_TURTLE || 
	    _previous_state == TurtleModeState::ACTIVE_TURTLE) {
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


float MulticopterTurtleMode::Threshold(float value, float threshold){

	if (value < threshold && value > -threshold) {
		return NAN;
	}
	return value;
}

MulticopterTurtleMode::MotorCommands MulticopterTurtleMode::getMotorCommands()
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
	PX4_INFO("ROLL PITCH PRINT: %f, %f", (double)cmds.roll, (double)cmds.pitch);
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
