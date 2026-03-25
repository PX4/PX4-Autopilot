#include "PreflightCalibration.hpp"
#include <cstring>

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
		_last_action = control_msg.action;

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
			_publish_status_snapshot();
			break;

		case preflight_calibration_control_s::ACTION_STOP:
			if (_calibration_active) {
				PX4_INFO("Preflight calibration stopped via uORB message.");
				_calibration_active = false;
			}
			_publish_status_snapshot();
			return;

		case preflight_calibration_control_s::ACTION_CAPTURE_FRONT: {
			actuator_outputs_s outputs{};
			if (_actuator_outputs.update(&outputs)) {
				_capture_front_pose(outputs);
			} else {
				PX4_ERR("Failed to get current servo positions");
			}
			_publish_status_snapshot();
			return;
		}

		case preflight_calibration_control_s::ACTION_CAPTURE_UP: {
			actuator_outputs_s outputs{};
			if (_actuator_outputs.update(&outputs)) {
				_capture_up_pose(outputs);
			} else {
				PX4_ERR("Failed to get current servo positions");
			}
			_publish_status_snapshot();
			return;
		}

		case preflight_calibration_control_s::ACTION_READ_FRONT:
			Read_Front();
			_publish_status_snapshot();
			return;

		case preflight_calibration_control_s::ACTION_READ_UP:
			Read_Up();
			_publish_status_snapshot();
			return;

		default:
			PX4_WARN("Unknown preflight_calibration_control action: %u", control_msg.action);
			_publish_status_snapshot();
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
	status_msg.front_pose_saved = _front_pose_saved;
	status_msg.up_pose_saved = _up_pose_saved;
	status_msg.last_action = _last_action;

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
	PX4_INFO("Saved FRONT: [%.3f, %.3f, %.3f, %.3f]",
		(double)_sv_pos_front_1.get(),
		(double)_sv_pos_front_2.get(),
		(double)_sv_pos_front_3.get(),
		(double)_sv_pos_front_4.get());

	PX4_INFO("Saved UP:    [%.3f, %.3f, %.3f, %.3f]",
		(double)_sv_pos_up_1.get(),
		(double)_sv_pos_up_2.get(),
		(double)_sv_pos_up_3.get(),
		(double)_sv_pos_up_4.get());

	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_calibration_updated_perf);
	return 0;
}

int PreflightCalibration::custom_command(int argc, char *argv[])
{
	PreflightCalibration *instance = get_instance();

	if (instance == nullptr) {
		return print_usage("module not running");
	}

	if (argc > 0) {
		if (!strcmp(argv[0], "read_front")) {
			instance->Read_Front();
			return PX4_OK;
		}

		if (!strcmp(argv[0], "read_up")) {
			instance->Read_Up();
			return PX4_OK;
		}
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("read_front", "Move servos 1-4 to saved FRONT position");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read_up", "Move servos 1-4 to saved UP position");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void PreflightCalibration::_parameters_updated()
{
	ModuleParams::updateParams();
}

void PreflightCalibration::_capture_front_pose(const actuator_outputs_s &outputs)
{
	_sv_pos_front_1.set(outputs.output[0]);
	_sv_pos_front_2.set(outputs.output[1]);
	_sv_pos_front_3.set(outputs.output[2]);
	_sv_pos_front_4.set(outputs.output[3]);
	_front_pose_saved = true;

	PX4_INFO("Captured FRONT pose for servos 1-4");
}

void PreflightCalibration::_capture_up_pose(const actuator_outputs_s &outputs)
{
	_sv_pos_up_1.set(outputs.output[0]);
	_sv_pos_up_2.set(outputs.output[1]);
	_sv_pos_up_3.set(outputs.output[2]);
	_sv_pos_up_4.set(outputs.output[3]);
	_up_pose_saved = true;

	PX4_INFO("Captured UP pose for servos 1-4");
}

void PreflightCalibration::_publish_status_snapshot()
{
	preflight_calibration_status_s status_msg{};
	status_msg.timestamp = hrt_absolute_time();
	status_msg.in_progress = _calibration_active;
	status_msg.calibration_successful = _is_calibration_successful();
	status_msg.front_pose_saved = _front_pose_saved;
	status_msg.up_pose_saved = _up_pose_saved;
	status_msg.last_action = _last_action;

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		status_msg.servo_status[i] = _sv_states[i].status;
	}

	_preflight_calibration_status_pub.publish(status_msg);
}

void PreflightCalibration::Read_Front()
{
	const float pose[4] = {
		_sv_pos_front_1.get(),
		_sv_pos_front_2.get(),
		_sv_pos_front_3.get(),
		_sv_pos_front_4.get()
	};

	_apply_pose(pose, "FRONT");
}

void PreflightCalibration::Read_Up()
{
	const float pose[4] = {
		_sv_pos_up_1.get(),
		_sv_pos_up_2.get(),
		_sv_pos_up_3.get(),
		_sv_pos_up_4.get()
	};

	_apply_pose(pose, "UP");
}

void PreflightCalibration::_apply_pose(const float pose[4], const char *pose_name)
{
	actuator_test_s actuator_test{};
	const hrt_abstime now = hrt_absolute_time();

	for (int i = 0; i < 4; i++) {
		actuator_test.timestamp = now;
		actuator_test.timeout_ms = 500;
		actuator_test.action = actuator_test_s::ACTION_DO_CONTROL;
		actuator_test.function = actuator_test_s::FUNCTION_SERVO1 + i;
		actuator_test.value = pose[i];
		_actuator_test_pub.publish(actuator_test);

		PX4_INFO("Applying %s pose: servo %d -> %.3f", pose_name, i + 1, (double)pose[i]);
	}
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
