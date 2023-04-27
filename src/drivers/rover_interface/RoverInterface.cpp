#include "RoverInterface.hpp"
#include <px4_platform_common/log.h>

RoverInterface *RoverInterface::_instance;

// CAN interface | default is can0
const char *const RoverInterface::CAN_IFACE = "can0";

RoverInterface::RoverInterface(uint8_t rover_type, uint32_t bitrate, float manual_throttle_max,
			       float mission_throttle_max)
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rover_interface),
	  _rover_type(rover_type),
	  _bitrate(bitrate),
	  _manual_throttle_max(manual_throttle_max),
	  _mission_throttle_max(mission_throttle_max)
{
	pthread_mutex_init(&_node_mutex, nullptr);
}


RoverInterface::~RoverInterface()
{
	if (_instance) {
		// Tell the task we want it to go away
		_task_should_exit.store(true);
		ScheduleNow();

		unsigned i = 1000;

		do {
			// Wait for it to exit or timeout
			usleep(5000);

			if (--i == 0) {
				PX4_ERR("Failed to Stop Task - reboot needed");
				break;
			}

		} while (_instance);
	}

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}


int RoverInterface::start(uint8_t rover_type, uint32_t bitrate, float manual_throttle_max, float mission_throttle_max)
{
	if (_instance != nullptr) {
		PX4_ERR("Already started");
		return -1;
	}

	_instance = new RoverInterface(rover_type, bitrate, manual_throttle_max, mission_throttle_max);

	if (_instance == nullptr) {
		PX4_ERR("Failed to allocate RoverInterface object");
		return -1;
	}

	_instance->ScheduleOnInterval(ScheduleIntervalMs);

	return PX4_OK;
}


void RoverInterface::Init()
{
	_initialized = false;

	// Check rover type
	switch (_rover_type) {
	case 0:
		// Scout Mini
		PX4_INFO("Scout Mini (rover type 0) is supported. Initializing...");
		break;

	case 1:
		// Scout
		PX4_INFO("Scout (rover type 1) is not supported. Aborted");
		return;

	case 2:
		// Scout Pro
		PX4_INFO("Scout Pro (rover type 2) is not supported. Aborted");
		return;

	case 3:
		// Scout 2
		PX4_INFO("Scout 2 (rover type 3) is not supported. Aborted");
		return;

	case 4:
		// Scout 2 Pro
		PX4_INFO("Scout 2 Pro (rover type 4) is not supported. Aborted");
		return;

	case 5:
		// Bunker
		PX4_INFO("Bunker (rover type 5) is supported. Initializing...");
		break;

	case 6:
		// Bunker Mini
		PX4_INFO("Bunker Mini (rover type 6) is supported. Initializing...");
		break;

	default:
		// Unknown Rover type
		PX4_INFO("Unknown rover type. Aborted");
		return;
	}

	// Check protocol version and create ScoutRobot object
	if (_protocol_version == scoutsdk::ProtocolVersion::AGX_V1) {
		PX4_INFO("AGX V1 protocol version is not supported. Aborted");

	} else if (_protocol_version == scoutsdk::ProtocolVersion::AGX_V2) {
		PX4_INFO("Detected AGX V2 protocol");
		_scout = new scoutsdk::ScoutRobot(scoutsdk::ProtocolVersion::AGX_V2, true);

	} else {
		PX4_INFO("Unknown protocol version");
	}

	if (_scout == nullptr) {
		PX4_ERR("Failed to create the ScoutRobot object. Aborted");
		return;
	}

	// Finally establish connection to rover
	_scout->Connect(CAN_IFACE, _bitrate);

	if (!_scout->GetCANConnected()) {
		PX4_ERR("Failed to connect to the rover CAN bus");
		return;
	}

	// Use CAN command mode
	_scout->EnableCommandMode();

	// Retrieve system version (blocking)
	_scout->QuerySystemVersion(SystemVersionQueryLimitMs);

	// Setup rover state publisher
	if (!orb_advert_valid(_rover_status_pub)) {
		_rover_status_pub = orb_advertise(ORB_ID(rover_status), &_rover_status_msg);
	}

	// Breathing mode light by default when not armed
	_scout->SetLightCommand(LightMode::BREATH, 0);

	_initialized = true;
}


void RoverInterface::Run()
{
	pthread_mutex_lock(&_node_mutex);

	if (_instance != nullptr && _task_should_exit.load()) {
		ScheduleClear();

		if (_initialized) { _initialized = false; }

		// Clean up
		if (_scout != nullptr) {
			delete _scout;
			_scout = nullptr;
		}

		_instance = nullptr;
		pthread_mutex_unlock(&_node_mutex);
		return;
	}

	if (_instance != nullptr && !_initialized) {
		// Try initializing for the first time
		if (!_init_try_count) {
			Init();
			_init_try_count++;
		}

		// Return early if still not initialized
		if (!_initialized) {
			pthread_mutex_unlock(&_node_mutex);
			return;
		}
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);


	// Check for actuator armed command to rover
	ActuatorArmedUpdate();

	// Check for action request (manual lock down)
	ActionRequestUpdate();

	// Check for actuator controls command to rover
	if (!_kill_switch && _armed) { VehicleTorqueAndThrustUpdate(); }

	// Check for vehicle control mode
	VehicleControlModeUpdate();

	// Check for receive msgs from the rover
	_scout->CheckUpdateFromRover();

	// Update from rover and publish the rover state
	if (hrt_elapsed_time(&_last_rover_status_publish_time) > RoverStatusPublishIntervalMs) {
		PublishRoverState();
	}

	perf_end(_cycle_perf);

	pthread_mutex_unlock(&_node_mutex);
}


void RoverInterface::VehicleTorqueAndThrustUpdate()
{
	bool do_update = false;

	// Check torque setppoint update
	if (_vehicle_torque_setpoint_sub.updated()) {
		vehicle_torque_setpoint_s vehicle_torque_setpoint_msg;

		if (_vehicle_torque_setpoint_sub.copy(&vehicle_torque_setpoint_msg)) {
			_yaw_control = vehicle_torque_setpoint_msg.xyz[2];
			do_update = true;
		}
	}

	// Check thrust setpoint update
	if (_vehicle_thrust_setpoint_sub.updated()) {
		vehicle_thrust_setpoint_s vehicle_thrust_setpoint_msg;

		if (_vehicle_thrust_setpoint_sub.copy(&vehicle_thrust_setpoint_msg)) {
			_throttle_control = vehicle_thrust_setpoint_msg.xyz[0];
			do_update = true;
		}
	}

	if (do_update) {
		auto throttle = (_is_manual_mode ? _manual_throttle_max : _mission_throttle_max) * _throttle_control;
		auto steering = _yaw_control;
		_scout->SetMotionCommand(throttle, steering);
	}
}


void RoverInterface::ActuatorArmedUpdate()
{
	if (_actuator_armed_sub.updated()) {
		actuator_armed_s actuator_armed_msg;

		if (_actuator_armed_sub.copy(&actuator_armed_msg)) {
			// Arm or disarm the rover
			if (!_armed && actuator_armed_msg.armed) {
				_scout->SetLightCommand(LightMode::CONST_ON, 0);
				_armed = true;

			} else if (_armed && !actuator_armed_msg.armed) {
				_scout->SetLightCommand(LightMode::BREATH, 0);
				_armed = false;
			}
		}
	}
}


void RoverInterface::ActionRequestUpdate()
{
	if (_action_request_sub.updated()) {
		action_request_s action_request_msg;

		if (_action_request_sub.copy(&action_request_msg)) {
			// Check for kill switch (expected pub rate is 5Hz)
			switch (action_request_msg.action) {
			case action_request_s::ACTION_KILL:
				_kill_switch = true;
				_scout->SetMotionCommand(0.0, 0.0);
				break;

			case action_request_s::ACTION_UNKILL:
				_kill_switch = false;
				break;
			}
		}
	}
}


void RoverInterface::VehicleControlModeUpdate()
{
	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode_msg;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode_msg)) {
			_is_manual_mode = vehicle_control_mode_msg.flag_control_manual_enabled;
		}
	}
}


void RoverInterface::PublishRoverState()
{
	// Get rover state
	auto &robot_state = _scout->GetRobotState();

	// Assign the values to the PX4 side ORB msg
	_rover_status_msg.timestamp = hrt_absolute_time();
	_rover_status_msg.linear_velocity = robot_state.motion_state.linear_velocity;
	_rover_status_msg.angular_velocity = robot_state.motion_state.angular_velocity;
	_rover_status_msg.vehicle_state = robot_state.system_state.vehicle_state;
	_rover_status_msg.control_mode = robot_state.system_state.control_mode;
	_rover_status_msg.error_code = robot_state.system_state.error_code;
	_rover_status_msg.battery_voltage = robot_state.system_state.battery_voltage;
	_rover_status_msg.light_control_enable = robot_state.light_state.enable_cmd_ctrl;
	_rover_status_msg.front_light_mode = robot_state.light_state.front_light.mode;
	_rover_status_msg.front_light_custom_value = robot_state.light_state.front_light.custom_value;
	_rover_status_msg.rear_light_mode = robot_state.light_state.rear_light.mode;
	_rover_status_msg.rear_light_custom_value = robot_state.light_state.rear_light.custom_value;

	if (orb_advert_valid(_rover_status_pub)) {
		orb_publish(ORB_ID(rover_status), &_rover_status_pub, &_rover_status_msg);
		_last_rover_status_publish_time = _rover_status_msg.timestamp;
	}
}


void RoverInterface::print_status()
{
	pthread_mutex_lock(&_node_mutex);

	if (_scout == nullptr) {
		PX4_ERR("Scout Robot object not initialized");
		pthread_mutex_unlock(&_node_mutex);
		return;
	}

	// CAN connection info
	PX4_INFO("CAN interface: %s. Status: %s",
		 RoverInterface::CAN_IFACE, _scout->GetCANConnected() ? "connected" : "disconnected");

	// Rover info
	if (_scout->GetCANConnected()) {
		PX4_INFO("Rover Type: %d. Protocol Version: %s",
			 _rover_type,
			 _protocol_version == scoutsdk::ProtocolVersion::AGX_V2 ? "AGX_V2" : "Unknown");
		PX4_INFO("Rover system version: %s", _scout->GetSystemVersion());
	}

	// Arm / disarm / kill switch status
	PX4_INFO("Rover is armed: %s. Kill switch: %s", _armed ? "true" : "false", _kill_switch ? "true" : "false");

	// Subscription info
	PX4_INFO("Subscribed to topics: %s, %s, %s",
		 _vehicle_thrust_setpoint_sub.get_topic()->o_name,
		 _vehicle_torque_setpoint_sub.get_topic()->o_name,
		 _actuator_armed_sub.get_topic()->o_name);

	// Publication info
	if (orb_advert_valid(_rover_status_pub)) { PX4_INFO("Publishing rover_status topic"); }

	// Performance counters
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	pthread_mutex_unlock(&_node_mutex);
}


static void print_usage()
{
	PX4_INFO("Usage: \n\trover_interface {start|status|stop}");
}


extern "C" __EXPORT int rover_interface_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (RoverInterface::instance()) {
			PX4_ERR("Already started");
			return 1;
		}

		// Rover type | default is Scout Mini
		int32_t rover_type = 0;
		param_get(param_find("RI_ROVER_TYPE"), &rover_type);

		// Rover interface CAN bitrate | default is 500Kbit/s
		int32_t can_bitrate = 0;
		param_get(param_find("RI_CAN_BITRATE"), &can_bitrate);

		// Manual control mode max throttle (1m/s to 3m/s)
		float manual_throttle_max = 1.0;
		param_get(param_find("RI_MAN_THR_MAX"), &manual_throttle_max);

		// Mission control mode max throttle (1m/s to 3m/s)
		float mission_throttle_max = 1.0;
		param_get(param_find("RI_MIS_THR_MAX"), &mission_throttle_max);

		// Start
		PX4_INFO("Start Rover Interface to rover type %d at CAN iface %s with bitrate %d bit/s",
			 rover_type, RoverInterface::CAN_IFACE, can_bitrate);
		return RoverInterface::start(static_cast<uint8_t>(rover_type),
					     can_bitrate,
					     manual_throttle_max,
					     mission_throttle_max
					    );
	}

	/* commands below assume that the app has been already started */
	RoverInterface *const inst = RoverInterface::instance();

	if (!inst) {
		PX4_ERR("Application not running");
		return 1;
	}

	if (!strcmp(argv[1], "status") || !strcmp(argv[1], "info")) {
		inst->print_status();
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
