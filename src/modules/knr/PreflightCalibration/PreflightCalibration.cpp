#include "PreflightCalibration.hpp"

using namespace time_literals;
using matrix::Vector3f;

PreflightCalibration::PreflightCalibration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

PreflightCalibration::~PreflightCalibration()
{
	perf_free(_loop_interval_perf);
	perf_free(_calibration_updated_perf);
}

bool PreflightCalibration::init()
{
	if (!_actuator_outputs.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	for(int i=0;i<NUM_ACTUATORS;i++) {
		_sv_states[i].value = _sv_cal_min.get();
		_sv_states[i].forward = true;
	}

	return true;
}

void PreflightCalibration::Run()
{
	perf_count(_loop_interval_perf);

	preflight_calibration_control_s control_msg{};

	if(_preflight_calibration_control_sub.update(&control_msg)) {
		switch (control_msg.action) {
		case preflight_calibration_control_s::ACTION_START:
			if (!_calibration_active) {
				PX4_INFO("Preflight calibration triggered via uORB message.");
				_calibration_active = true;
				_last_calibration_update = 0;

				for (int i = 0; i < NUM_ACTUATORS; i++) {
					_sv_states[i].reached_min = false;
					_sv_states[i].reached_max = false;
					_sv_states[i].value = _sv_cal_min.get();
					_sv_states[i].forward = true;
					_sv_states[i].status = static_cast<uint8_t>(ServoStatus::NOT_INITIALIZED);
				}
			}
			break;

		case preflight_calibration_control_s::ACTION_STOP:
			if (_calibration_active) {
				PX4_INFO("Preflight calibration stopped via uORB message.");
				_calibration_active = false;
			}
			return;

		case preflight_calibration_control_s::ACTION_SAVE: {
			actuator_outputs_s outputs{};
			if (_actuator_outputs.update(&outputs)) {
				_save_servo_positions(outputs);
			} else {
				PX4_ERR("Failed to get current servo positions");
			}
			return;
		}

		case preflight_calibration_control_s::ACTION_LOAD:
			_load_servo_positions();
			return;

		default:
			PX4_WARN("Unknown preflight_calibration_control action: %u", control_msg.action);
			return;
		}
	}

	if (!_calibration_active) {
		return;
	}

	if(_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		_parameters_updated();
	}

	actuator_test_s actuator_test{};
	actuator_outputs_s outputs{};
	preflight_calibration_status_s status_msg{};

	const hrt_abstime now = hrt_absolute_time();
	if (now - _last_calibration_update < CAL_INTERVAL) {
		return;
	}

	const float max = _sv_cal_max.get();
	const float min = _sv_cal_min.get();
	const float step = _sv_cal_step.get();

	status_msg.timestamp = now;
	status_msg.in_progress = !_do_calibration_ended();
	status_msg.calibration_successful = false;

	for(int i=0; i<NUM_ACTUATORS; i++) {
		auto &s = _sv_states[i];

		s.status = static_cast<uint8_t>(ServoStatus::CALIBRATING);
		s.value += s.forward ? step : -step;

		if(!s.forward && s.value <= min) {
			s.reached_min = true;
		} else if(s.forward && s.value >= max) {
			s.reached_max = true;
			s.forward = false;
		}

		if (!(s.reached_min && s.reached_max)) {
			status_msg.in_progress = true;
			status_msg.calibration_successful = false;
		}

		if(s.reached_min && s.reached_max) {
			s.status = static_cast<uint8_t>(ServoStatus::CALIBRATED);
			status_msg.servo_status[i] = static_cast<uint8_t>(s.status);
			continue; // Skip this servo if done calibrating
		}

		actuator_test.timestamp = now;
		actuator_test.timeout_ms = CAL_TIMEOUT;
		actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
		actuator_test.function = actuator_test_s::FUNCTION_SERVO1 + i;
		actuator_test.value = s.value;
		_actuator_test_pub.publish(actuator_test);

		status_msg.servo_status[i] = static_cast<uint8_t>(s.status);

		PX4_INFO("Calibrating actuator %d to value %.3f", i+1, static_cast<double>(actuator_test.value));

		if (_actuator_outputs.update(&outputs)) {

		PX4_INFO("Current servos position: [%.3f, %.3f, %.3f, %.3f]",
			(double)outputs.output[0],
			(double)outputs.output[1],
			(double)outputs.output[2],
			(double)outputs.output[3]
		);
		}
	}

	status_msg.calibration_successful = _is_calibration_successful();
	_preflight_calibration_status_pub.publish(status_msg);

	if(_do_calibration_ended()) {
		PX4_INFO("Preflight calibration completed for all actuators.");
		_calibration_active = false;

		 // Stop the scheduled work item
		ScheduleClear();

		_request_stop = true;
		exit_and_cleanup();
		return;
	}

	perf_count(_calibration_updated_perf);
}

int PreflightCalibration::task_spawn(int argc, char *argv[])
{
	PreflightCalibration *instance = new PreflightCalibration();

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

int PreflightCalibration::print_status()
{
	// TODO - print status

	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_calibration_updated_perf);
	return 0;
}

int PreflightCalibration::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PreflightCalibration::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
	[KNR_MODULE] Preflight calibration as additional step for autonomous mission start.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("preflight_calibration", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void PreflightCalibration::_parameters_updated()
{
	ModuleParams::updateParams();
}

void PreflightCalibration::_save_servo_positions(const actuator_outputs_s &outputs)
{
	_sv_pos_saved_1.set(outputs.output[0]);
	_sv_pos_saved_2.set(outputs.output[1]);
	_sv_pos_saved_3.set(outputs.output[2]);
	_sv_pos_saved_4.set(outputs.output[3]);
	_sv_pos_saved_5.set(outputs.output[4]);
	_sv_pos_saved_6.set(outputs.output[5]);
	_sv_pos_saved_7.set(outputs.output[6]);
	_sv_pos_saved_8.set(outputs.output[7]);
	_sv_pos_saved_9.set(outputs.output[8]);
	_sv_pos_saved_10.set(outputs.output[9]);
	_sv_pos_saved_11.set(outputs.output[10]);
	_sv_pos_saved_12.set(outputs.output[11]);
	_sv_pos_saved_13.set(outputs.output[12]);
	_sv_pos_saved_14.set(outputs.output[13]);
	_sv_pos_saved_15.set(outputs.output[14]);
	_sv_pos_saved_16.set(outputs.output[15]);

	PX4_INFO("Servo positions saved successfully");
}

void PreflightCalibration::_load_servo_positions()
{
	actuator_test_s actuator_test{};
	const hrt_abstime now = hrt_absolute_time();

	const float saved_positions[] = {
		_sv_pos_saved_1.get(),
		_sv_pos_saved_2.get(),
		_sv_pos_saved_3.get(),
		_sv_pos_saved_4.get(),
		_sv_pos_saved_5.get(),
		_sv_pos_saved_6.get(),
		_sv_pos_saved_7.get(),
		_sv_pos_saved_8.get(),
		_sv_pos_saved_9.get(),
		_sv_pos_saved_10.get(),
		_sv_pos_saved_11.get(),
		_sv_pos_saved_12.get(),
		_sv_pos_saved_13.get(),
		_sv_pos_saved_14.get(),
		_sv_pos_saved_15.get(),
		_sv_pos_saved_16.get()
	};

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		actuator_test.timestamp = now;
		actuator_test.timeout_ms = 100;
		actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
		actuator_test.function = actuator_test_s::FUNCTION_SERVO1 + i;
		actuator_test.value = saved_positions[i];

		_actuator_test_pub.publish(actuator_test);

		PX4_INFO("Loading actuator %d to saved position %.3f", i + 1, 
			(double)saved_positions[i]);
	}

	PX4_INFO("All saved servo positions loaded");
}

bool PreflightCalibration::_is_calibration_successful()
{
	for (const auto &s : _sv_states) {
		if (!(s.status == static_cast<uint8_t>(ServoStatus::CALIBRATED))) {
			return false;
		}
	}

	return true;
}

bool PreflightCalibration::_do_calibration_ended()
{
	bool all_done = true;
	for(int i=0; i<NUM_ACTUATORS; i++) {

		if(_sv_states[i].status != static_cast<uint8_t>(ServoStatus::CALIBRATED)) {
			all_done = false;
		}
	}
	return all_done;
}

extern "C" __EXPORT int preflight_calibration_main(int argc, char *argv[])
{
	return PreflightCalibration::main(argc, argv);
}
