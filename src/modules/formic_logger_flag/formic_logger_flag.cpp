#include "formic_logger_flag.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

using namespace time_literals;

FormicLoggerFlag::FormicLoggerFlag() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_update(true);
}

bool FormicLoggerFlag::init()
{
	ScheduleOnInterval(100_ms);
	parameters_update();
	return true;
}

void FormicLoggerFlag::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void FormicLoggerFlag::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();
	run_once();
	float click_value = on_press_logger_flag();
	_click_state = find_click_state(click_value);
	publish_debug_flag(_click_state);

}

int FormicLoggerFlag::task_spawn(int argc, char *argv[])
{
	FormicLoggerFlag *instance = new FormicLoggerFlag();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int FormicLoggerFlag::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicLoggerFlag::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal skeleton module ready for custom logic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_logger_flag", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

FormicLoggerFlag *FormicLoggerFlag::instantiate(int argc, char *argv[])
{
	return new FormicLoggerFlag();
}

void FormicLoggerFlag::run_once()
{
	const int32_t selected_aux = _param_formic_logger_flag_aux.get();

	if (selected_aux != _active_aux_channel) {
		_active_aux_channel = selected_aux;
	}
}

float FormicLoggerFlag::on_press_logger_flag(){

	// Check if a valid aux channel is selected (parameter is 1-indexed: 1-6, 0 = disabled)
	if (_active_aux_channel <= 0 || _active_aux_channel > 6) {
		return 0.0f; // Invalid or disabled aux channel
	}

	// Use manual_control_setpoint topic which has normalized values [-1, 1]
	manual_control_setpoint_s manual_control_setpoint{};
	
	if (!_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		return 0.0f; // No new manual control data
	}

	// Get the aux channel value based on the selected aux (1-6)
	float aux_value = NAN;
	switch (_active_aux_channel) {
		case 1:
			aux_value = manual_control_setpoint.aux1;
			break;
		case 2:
			aux_value = manual_control_setpoint.aux2;
			break;
		case 3:
			aux_value = manual_control_setpoint.aux3;
			break;
		case 4:
			aux_value = manual_control_setpoint.aux4;
			break;
		case 5:
			aux_value = manual_control_setpoint.aux5;
			break;
		case 6:
			aux_value = manual_control_setpoint.aux6;
			break;
		default:
			return -1.0f; // Invalid aux channel
	}

	// Check if aux value is valid (not NaN)
	if (!PX4_ISFINITE(aux_value)) {
		return 0.0f; // Aux channel not available
	}
	
	// Convert from [-1, 1] to [0, 1] range
	return (aux_value + 1.0f) * 0.5f;
}


float FormicLoggerFlag::get_max_value() {
	if (_param_formic_logger_flag_invert.get() > 0.5f) {
		return 0.0f;  // When inverted, detect low values (close to 0)
	} else {
		return 1.0f;  // When not inverted, detect high values (close to 1)
	}
}

ClickState FormicLoggerFlag::find_click_state(float current_click_value) {
	if (_param_formic_logger_flag_invert.get() > 0.5f) {
		// Inverted: detect when value is low (below threshold)
		if (current_click_value < get_max_value() + _threshold_value) {
			return ClickState::onClick;
		}
	} else {
		// Not inverted: detect when value is high (above threshold)
		if (current_click_value > get_max_value() - _threshold_value) {
			return ClickState::onClick;
		}
	}
	return ClickState::rest;
}



extern "C" __EXPORT int formic_logger_flag_main(int argc, char *argv[])
{
	return FormicLoggerFlag::main(argc, argv);
}



void FormicLoggerFlag::publish_debug_flag(ClickState click_state) {
	debug_flag_s debug_flag{};
	debug_flag.timestamp = hrt_absolute_time();
      
	if (!_flag_activate_state && click_state == ClickState::onClick) {
	  // publish debug flag
	  _flag_activate_state = true;
	  debug_flag.flag_activate = true;
	  _debug_flag_pub.publish(debug_flag);
	} else if (_flag_activate_state && click_state == ClickState::rest) {
	  // ready to get another click
	  _flag_activate_state = false;
	}
      }
