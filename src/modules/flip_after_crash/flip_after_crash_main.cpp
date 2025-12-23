#include "flip_after_crash.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

FlipAfterCrash::FlipAfterCrash() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	PX4_INFO("Flip after crash module constructed");
}

FlipAfterCrash::~FlipAfterCrash()
{
	PX4_INFO("Flip after crash module destructed");
	ScheduleClear();
}

int FlipAfterCrash::task_spawn(int argc, char *argv[])
{
	FlipAfterCrash *instance = new FlipAfterCrash();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->init();
	PX4_INFO("Flip after crash module initialized");

	return PX4_OK;
}

bool FlipAfterCrash::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

int FlipAfterCrash::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlipAfterCrash::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flip after crash module.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flip_after_crash", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool FlipAfterCrash::isAuxSwitchEnabled()
{
	const int32_t selected_channel = _param_fac_aux.get();

	// Check if channel is disabled (0) or invalid
	if (selected_channel <= 0 || selected_channel > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		return false;
	}

	// Use rc_channels topic which has normalized values [-1, 1]
	rc_channels_s rc_channels{};

	if (!_rc_channels_sub.update(&rc_channels)) {
		return false; // No new RC data
	}

	// Convert from 1-indexed parameter (1-18) to 0-indexed array index
	uint8_t channel_index = selected_channel - 1;

	if (channel_index >= rc_channels.channel_count) {
		return false; // Channel not available
	}

	// Get normalized value from rc_channels (already scaled to -1..1)
	float rc_normalized = rc_channels.channels[channel_index];

	// Check if switch is pressed (value > 0.0 in normalized [-1, 1] range)
	// This corresponds to PWM > 1500us (middle of 1000-2000us range)
	return rc_normalized > 0.0f;
}

void FlipAfterCrash::Run()
{
	if (should_exit()) {
		if (_flip_active) {
			PX4_INFO("Flip after crash module destructed (flip action stopped)");
			_flip_active = false;
		}
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Update parameters
	parameters_update();

	// Check if parameter is disabled - stop the module
	if (_param_fac_aux.get() == 0) {
		if (_flip_active) {
			PX4_INFO("Flip after crash module destructed (parameter disabled)");
			_flip_active = false;
		}
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check current switch state
	bool switch_pressed = isAuxSwitchEnabled();

	// Detect switch press: was not pressed, now pressed (construct/activate turtle mode)
	if (!_was_switch_pressed && switch_pressed) {
		PX4_INFO("Flip after crash module constructed - entering turtle mode");
		
		// Save current navigation state to restore later
		vehicle_status_s vehicle_status{};
		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_previous_nav_state = vehicle_status.nav_state;
		}
		
		// Send command to enter turtle mode
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
		cmd.param1 = vehicle_status_s::NAVIGATION_STATE_TURTLE;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.confirmation = false;
		_vehicle_command_pub.publish(cmd);
		
		_flip_active = true;
	}

	// Detect switch release: was pressed, now not pressed (destruct/exit turtle mode)
	if (_was_switch_pressed && !switch_pressed) {
		PX4_INFO("Flip after crash module destructed - exiting turtle mode");
		
		// Send command to restore previous navigation state
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
		cmd.param1 = _previous_nav_state;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.confirmation = false;
		_vehicle_command_pub.publish(cmd);
		
		_flip_active = false;
	}

	// Update state
	_was_switch_pressed = switch_pressed;

	// Only perform flip action when switch is pressed and flip is active
	if (_flip_active && switch_pressed) {
		PX4_INFO("Flip after crash loop");
		// TODO: Implement flip logic here
	}
}

void FlipAfterCrash::parameters_update()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

extern "C" __EXPORT int flip_after_crash_main(int argc, char *argv[])
{
	return FlipAfterCrash::main(argc, argv);
}

