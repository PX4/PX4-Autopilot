// #define DEBUG_BUILD
#include "GZGimbal.hpp"

bool GZGimbal::init(const std::string &world_name, const std::string &model_name)
{
	// Gazebo communication
	const std::string gimbal_roll_topic = "/model/" + model_name + "/command/gimbal_roll";
	_gimbal_roll_cmd_publisher = _node.Advertise<gz::msgs::Double>(gimbal_roll_topic);

	if (!_gimbal_roll_cmd_publisher.Valid()) {
		PX4_ERR("failed to advertise %s", gimbal_roll_topic.c_str());
		return false;
	}

	const std::string gimbal_pitch_topic = "/model/" + model_name + "/command/gimbal_pitch";
	_gimbal_pitch_cmd_publisher = _node.Advertise<gz::msgs::Double>(gimbal_pitch_topic);

	if (!_gimbal_pitch_cmd_publisher.Valid()) {
		PX4_ERR("failed to advertise %s", gimbal_pitch_topic.c_str());
		return false;
	}

	const std::string gimbal_yaw_topic = "/model/" + model_name + "/command/gimbal_yaw";
	_gimbal_yaw_cmd_publisher = _node.Advertise<gz::msgs::Double>(gimbal_yaw_topic);

	if (!_gimbal_yaw_cmd_publisher.Valid()) {
		PX4_ERR("failed to advertise %s", gimbal_yaw_topic.c_str());
		return false;
	}

	const std::string gimbal_imu_topic = "/world/" + world_name + "/model/" + model_name +
					     "/link/camera_link/sensor/camera_imu/imu";

	if (!_node.Subscribe(gimbal_imu_topic, &GZGimbal::gimbalIMUCallback, this)) {
		PX4_ERR("failed to subscribe to %s", gimbal_imu_topic.c_str());
		return false;
	}

	// Mount parameters
	_mnt_range_roll_handle = param_find("MNT_RANGE_ROLL");
	_mnt_range_pitch_handle = param_find("MNT_RANGE_PITCH");
	_mnt_range_yaw_handle = param_find("MNT_RANGE_YAW");
	_mnt_mode_out_handle = param_find("MNT_MODE_OUT");

	if (_mnt_range_roll_handle == PARAM_INVALID ||
	    _mnt_range_pitch_handle == PARAM_INVALID ||
	    _mnt_range_yaw_handle == PARAM_INVALID ||
	    _mnt_mode_out_handle == PARAM_INVALID) {
		return false;
	}

	pthread_mutex_init(&_node_mutex, nullptr);

	updateParameters();

	ScheduleOnInterval(200_ms); // @5Hz

	// Schedule on vehicle command messages
	if (!_vehicle_command_sub.registerCallback()) {
		return false;
	}

	return true;
}

void GZGimbal::Run()
{
	pthread_mutex_lock(&_node_mutex);

	const hrt_abstime now = hrt_absolute_time();
	const float dt = (now - _last_time_update) / 1e6f;
	_last_time_update = now;

	updateParameters();

	if (pollSetpoint()) {
		//TODO handle device flags
		publishJointCommand(_gimbal_roll_cmd_publisher, _roll_stp, _roll_rate_stp, _last_roll_stp, _roll_min, _roll_max, dt);
		publishJointCommand(_gimbal_pitch_cmd_publisher, _pitch_stp, _pitch_rate_stp, _last_pitch_stp, _pitch_min, _pitch_max,
				    dt);
		publishJointCommand(_gimbal_yaw_cmd_publisher, _yaw_stp, _yaw_rate_stp, _last_yaw_stp, _yaw_min, _yaw_max, dt);
	}

	if (_mnt_mode_out == 2) {
		// We have a Mavlink gimbal capable of sending messages
		publishDeviceInfo();
		publishDeviceAttitude();
	}

	pthread_mutex_unlock(&_node_mutex);
}

void GZGimbal::stop()
{
	ScheduleClear();
}

void GZGimbal::gimbalIMUCallback(const gz::msgs::IMU &IMU_data)
{
	pthread_mutex_lock(&_node_mutex);

	static const matrix::Quatf q_FLU_to_FRD = matrix::Quatf(0.0f, 1.0f, 0.0f, 0.0f);
	const matrix::Quatf q_gimbal_FLU = matrix::Quatf(IMU_data.orientation().w(),
					   IMU_data.orientation().x(),
					   IMU_data.orientation().y(),
					   IMU_data.orientation().z());
	_q_gimbal = q_FLU_to_FRD * q_gimbal_FLU * q_FLU_to_FRD.inversed();

	matrix::Vector3f rate = q_FLU_to_FRD.rotateVector(matrix::Vector3f(IMU_data.angular_velocity().x(),
				IMU_data.angular_velocity().y(),
				IMU_data.angular_velocity().z()));

	_gimbal_rate[0] = rate(0);
	_gimbal_rate[1] = rate(1);
	_gimbal_rate[2] = rate(2);

	pthread_mutex_unlock(&_node_mutex);
}

void GZGimbal::updateParameters()
{
	param_get(_mnt_range_roll_handle, &_mnt_range_roll);
	param_get(_mnt_range_pitch_handle, &_mnt_range_pitch);
	param_get(_mnt_range_yaw_handle, &_mnt_range_yaw);
	param_get(_mnt_mode_out_handle, &_mnt_mode_out);
}

bool GZGimbal::pollSetpoint()
{
	if (_gimbal_device_set_attitude_sub.updated()) {
		gimbal_device_set_attitude_s msg;

		if (_gimbal_device_set_attitude_sub.copy(&msg)) {
			const matrix::Eulerf gimbal_att_stp(matrix::Quatf(msg.q));
			_roll_stp = gimbal_att_stp.phi();
			_pitch_stp = gimbal_att_stp.theta();
			_yaw_stp = gimbal_att_stp.psi();
			_roll_rate_stp = msg.angular_velocity_x;
			_pitch_rate_stp = msg.angular_velocity_y;
			_yaw_rate_stp = msg.angular_velocity_z;
			_gimbal_device_flags = msg.flags;

			return true;
		}

	} else if (_gimbal_controls_sub.updated()) {
		gimbal_controls_s msg;

		if (_gimbal_controls_sub.copy(&msg)) {
			// map control inputs from [-1;1] to [min_angle; max_angle] using the range parameters
			_roll_stp = math::constrain(math::radians(msg.control[msg.INDEX_ROLL] * _mnt_range_roll / 2), _roll_min, _roll_max);
			_pitch_stp = math::constrain(math::radians(msg.control[msg.INDEX_PITCH] * _mnt_range_pitch / 2), _pitch_min,
						     _pitch_max);
			_yaw_stp = math::constrain(math::radians(msg.control[msg.INDEX_YAW] * _mnt_range_yaw / 2), _yaw_min, _yaw_max);

			return true;
		}
	}

	return false;
}

void GZGimbal::publishDeviceInfo()
{
	if (_vehicle_command_sub.updated()) {
		vehicle_command_s cmd;
		_vehicle_command_sub.copy(&cmd);

		if (cmd.command == vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE &&
		    (uint16_t)cmd.param1 == vehicle_command_s::VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION) {
			// Acknowledge the command
			vehicle_command_ack_s command_ack{};

			command_ack.command = cmd.command;
			command_ack.result = (uint8_t)vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			command_ack.target_system = cmd.source_system;
			command_ack.target_component = cmd.source_component;
			command_ack.timestamp = hrt_absolute_time();

			_vehicle_command_ack_pub.publish(command_ack);

			// Send the requested message
			gimbal_device_information_s device_info{};

			memcpy(device_info.vendor_name, _vendor_name, sizeof(_vendor_name));
			memcpy(device_info.model_name, _model_name, sizeof(_model_name));
			memcpy(device_info.custom_name, _custom_name, sizeof(_custom_name));
			device_info.firmware_version = _firmware_version;
			device_info.hardware_version = _hardware_version;
			device_info.uid = _uid;
			device_info.cap_flags = _cap_flags;
			device_info.custom_cap_flags = _custom_cap_flags;
			device_info.roll_min = _roll_min;
			device_info.roll_max = _roll_max;
			device_info.pitch_min = _pitch_min;
			device_info.pitch_max = _pitch_max;
			device_info.yaw_min = _yaw_min;
			device_info.yaw_max = _yaw_max;
			device_info.gimbal_device_id = _gimbal_device_id;
			device_info.timestamp = hrt_absolute_time();

			_gimbal_device_information_pub.publish(device_info);
		}
	}
}

void GZGimbal::publishDeviceAttitude()
{
	// TODO handle flags

	gimbal_device_attitude_status_s gimbal_att{};

	gimbal_att.target_system = 0; // Broadcast
	gimbal_att.target_component = 0; // Broadcast
	gimbal_att.device_flags = 0;
	_q_gimbal.copyTo(gimbal_att.q);
	gimbal_att.angular_velocity_x = _gimbal_rate[0];
	gimbal_att.angular_velocity_y = _gimbal_rate[1];
	gimbal_att.angular_velocity_z = _gimbal_rate[2];
	gimbal_att.failure_flags = 0;
	gimbal_att.timestamp = hrt_absolute_time();

	_gimbal_device_attitude_status_pub.publish(gimbal_att);
}

void GZGimbal::publishJointCommand(gz::transport::Node::Publisher &publisher, const float att_stp, const float rate_stp,
				   float &last_stp, const float min_stp, const float max_stp, const float dt)
{
	gz::msgs::Double msg;

	float new_stp = computeJointSetpoint(att_stp, rate_stp, last_stp, dt);
	new_stp = math::constrain(new_stp, min_stp, max_stp);
	last_stp = new_stp;
	msg.set_data(new_stp);

	publisher.Publish(msg);
}

float GZGimbal::computeJointSetpoint(const float att_stp, const float rate_stp, const float last_stp, const float dt)
{

	if (PX4_ISFINITE(rate_stp)) {
		const float rate_diff = dt * rate_stp;
		const float stp_from_rate = last_stp + rate_diff;

		if (PX4_ISFINITE(att_stp)) {
			// We use the attitude rate setpoint but we constrain it by the desired angle
			return rate_diff > 0 ? math::min(att_stp, stp_from_rate) : math::max(att_stp, stp_from_rate);

		} else {
			// The rate setpoint is valid while the angle one is not
			return stp_from_rate;
		}

	} else if (PX4_ISFINITE(att_stp)) {
		// Only the angle setpoint is valid
		return att_stp;

	} else {
		// Neither setpoint is valid so we steer the gimbal to the default position (roll = pitch = yaw = 0)
		return 0.0f;
	}
}
