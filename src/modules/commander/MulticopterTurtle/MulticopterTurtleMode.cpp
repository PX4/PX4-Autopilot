
#include "MulticopterTurtleMode.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>
#include <lib/mathlib/math/Limits.hpp>
#include <uORB/topics/vehicle_status.h>
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
	if (_pending_dshot_disable && !armed) {
		// Now that we're disarmed, send the 3d_off command
		PX4_INFO("Turtle mode: Sending 3d_off command now that motors are disarmed");
		const char *cmd_str = "3d_off";
		char *cmd_argv[] = { 
			(char*)"dshot", 
			(char*)cmd_str,
			nullptr 
		};
		
		int result = dshot_main(2, cmd_argv);
		if (result == 0) {
			PX4_INFO("Turtle mode: DShot 3d_off command sent successfully");
		} else {
			PX4_WARN("Turtle mode: DShot 3d_off command failed with code %d", result);
		}
		
		_pending_dshot_disable = false;
	}

	if (_param_com_turtle_en.get()) {
		// If armed but not by turtle mode, disable turtle mode to allow normal flight
		if (armed && !_turtle_mode_armed && _turtle_mode_state != TurtleModeState::FLYING_DISABLED) {
			setState(TurtleModeState::FLYING_DISABLED);
			updateDshot3dParameter(false);
			return;
		}

		const float aux_channel_value = getAuxChannelValue();
		const bool aux_triggered = (aux_channel_value > AUX_CHANNEL_THRESHOLD);

		switch (_turtle_mode_state) {
		case TurtleModeState::FLYING_DISABLED:
			if (!armed || _turtle_mode_armed) {
				if (aux_triggered) {
					setState(TurtleModeState::ACTIVE_TURTLE);
					updateDshot3dParameter(true);
				} else {
					setState(TurtleModeState::OPTIONAL_TURTLE);
				}
			}
			break;

		case TurtleModeState::OPTIONAL_TURTLE:
			if (aux_triggered) {
				setState(TurtleModeState::ACTIVE_TURTLE);
				updateDshot3dParameter(true);
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
				// Set flag to send 3d_off command after disarming
				_pending_dshot_disable = true;
				// Update parameter immediately
				updateDshot3dParameter(false);
			} else {
				// While in ACTIVE_TURTLE: publish disabled motors continuously
				// This prevents control allocator from taking control
				// User can override by calling publishMotorCommands() at higher rate
				const float disabled_motors[12] = {0.1, 0.1, 0.1, 0.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
				publishMotorCommands(disabled_motors);
				
				// Rate-limited debug print to show continuous operation
				const hrt_abstime now = hrt_absolute_time();
				if (now - _last_debug_print_time >= DEBUG_PRINT_INTERVAL) {
					PX4_INFO("Turtle mode: ACTIVE - publishing disabled motors (call publishMotorCommands() to control)");
					_last_debug_print_time = now;
				}
			}
			break;
		}

		// Continue publishing disabled motors while waiting to disarm after leaving turtle mode
		// This prevents the control allocator from controlling motors until disarm completes
		if (_should_disarm_on_exit && armed) {
			const float disabled_motors[12] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};
			publishMotorCommands(disabled_motors);
			PX4_INFO("Turtle mode: Publishing disabled motors");
		}

	} else if (_turtle_mode_state != TurtleModeState::FLYING_DISABLED) {
		// Feature disabled: reset to FLYING_DISABLED
		setState(TurtleModeState::FLYING_DISABLED);
		// Set flag to send 3d_off command after disarming
		_pending_dshot_disable = true;
		updateDshot3dParameter(false);
	}
}

void MulticopterTurtleMode::setState(TurtleModeState new_state)
{
	_previous_state = _turtle_mode_state;
	_turtle_mode_state = new_state;
}



void MulticopterTurtleMode::updateDshot3dParameter(bool enable)
{
	// DShot commands only execute when motors are disarmed (output == 0)
	// For enable: we'll delay arming until command completes (300ms)
	// For disable: command will be sent after disarming
	
	param_t param_handle = param_find("DSHOT_3D_ENABLE");
	int result_param = PX4_ERROR;
	
	if (param_handle != PARAM_INVALID) {
		int32_t value = enable ? 1 : 0;
		result_param = param_set(param_handle, &value);
	}
	
	if (enable) {
		// Send 3d_on command (will execute when motors are at 0, before arming)
		const char *cmd_str = "3d_on";
		char *cmd_argv[] = { 
			(char*)"dshot", 
			(char*)cmd_str,
			nullptr 
		};

		PX4_INFO("Turtle mode: Sending DShot %s command (motors must be disarmed)", cmd_str);
		
		int result_dshot = dshot_main(2, cmd_argv);

		if (result_dshot == 0 && result_param == PX4_OK) {
			// Mark that we're waiting for the command to complete
			// The command needs ~200ms to complete (10 repetitions at ~20ms each)
			// We'll delay arming in shouldArm() until this time has passed
			_waiting_for_dshot_command = true;
			_dshot_command_start_time = hrt_absolute_time();
			PX4_INFO("Turtle mode: DShot %s command queued, waiting 300ms for completion before arming", cmd_str);
		} else {
			PX4_WARN("Turtle mode: DShot %s command failed (param: %d, dshot: %d)", cmd_str, result_param, result_dshot);
			// If the command fails, transition to OPTIONAL_TURTLE state
			setState(TurtleModeState::OPTIONAL_TURTLE);
		}
	} else {
		// For disable: parameter is updated, command will be sent after disarming
		if (result_param == PX4_OK) {
			PX4_INFO("Turtle mode: DSHOT_3D_ENABLE parameter set to 0");
		} else {
			PX4_WARN("Turtle mode: Failed to set DSHOT_3D_ENABLE parameter: %d", result_param);
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


void MulticopterTurtleMode::publishMotorCommands(const float throttle[12])
{
	// Publish motor commands with custom throttle values for each motor
	// throttle values should be in range [-1, 1] where:
	//   1 = maximum positive thrust
	//  -1 = maximum negative thrust (requires DShot 3D enabled)
	//  NAN = motor disabled
	// 
	// IMPORTANT: This function must be called at a high rate (250Hz recommended)
	// from your own code while in ACTIVE_TURTLE state to control motors.
	
	// Ensure publication is advertised
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
