#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/gimbal_device_set_attitude.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/gimbal_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <parameters/param.h>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <lib/matrix/matrix/Quaternion.hpp>
#include <drivers/drv_hrt.h>
#include <math.h>

using namespace time_literals;

class GZGimbal : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	GZGimbal(gz::transport::Node &node, pthread_mutex_t &node_mutex) :
		px4::ScheduledWorkItem(MODULE_NAME "-gimbal", px4::wq_configurations::rate_ctrl),
		ModuleParams(nullptr),
		_node(node),
		_node_mutex(node_mutex)
	{}

private:
	friend class GZBridge;

	gz::transport::Node &_node;
	pthread_mutex_t &_node_mutex;

	uORB::Subscription _gimbal_device_set_attitude_sub{ORB_ID(gimbal_device_set_attitude)};
	uORB::Subscription _gimbal_controls_sub{ORB_ID(gimbal_controls)};
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};

	uORB::Publication<gimbal_device_attitude_status_s> _gimbal_device_attitude_status_pub{ORB_ID(gimbal_device_attitude_status)};
	uORB::Publication<gimbal_device_information_s> _gimbal_device_information_pub{ORB_ID(gimbal_device_information)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	gz::transport::Node::Publisher _gimbal_roll_cmd_publisher;
	gz::transport::Node::Publisher _gimbal_pitch_cmd_publisher;
	gz::transport::Node::Publisher _gimbal_yaw_cmd_publisher;

	float _roll_stp = NAN;
	float _pitch_stp = NAN;
	float _yaw_stp = NAN;

	float _last_roll_stp = 0.0f;
	float _last_pitch_stp = 0.0f;
	float _last_yaw_stp = 0.0f;

	float _roll_rate_stp = NAN;
	float _pitch_rate_stp = NAN;
	float _yaw_rate_stp = NAN;

	hrt_abstime _last_time_update;

	// Mount parameters
	param_t _mnt_range_pitch_handle = PARAM_INVALID;
	param_t _mnt_range_roll_handle = PARAM_INVALID;
	param_t _mnt_range_yaw_handle = PARAM_INVALID;
	param_t _mnt_mode_out_handle = PARAM_INVALID;
	float _mnt_range_pitch = 0.0f;
	float _mnt_range_roll = 0.0f;
	float _mnt_range_yaw = 0.0f;
	int32_t _mnt_mode_out = 0;

	matrix::Quatf _q_gimbal = matrix::Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	float _gimbal_rate[3] = {NAN};

	// Device information attributes
	const char _vendor_name[32] = "PX4";
	const char _model_name[32] = "Gazebo Gimbal";
	const char _custom_name[32] = "";
	const uint8_t _firmware_dev_version = 0;
	const uint8_t _firmware_patch_version = 0;
	const uint8_t _firmware_minor_version = 0;
	const uint8_t _firmware_major_version = 1;
	const uint32_t _firmware_version = (_firmware_dev_version & 0xff) << 24 | (_firmware_patch_version & 0xff) << 16 |
					   (_firmware_minor_version & 0xff) << 8 | (_firmware_major_version & 0xff);
	const uint8_t _hardware_dev_version = 0;
	const uint8_t _hardware_patch_version = 0;
	const uint8_t _hardware_minor_version = 0;
	const uint8_t _hardware_major_version = 1;
	const uint32_t _hardware_version = (_hardware_dev_version & 0xff) << 24 | (_hardware_patch_version & 0xff) << 16 |
					   (_hardware_minor_version & 0xff) << 8 | (_hardware_major_version & 0xff);
	const uint64_t _uid = 0x9a77a55b3c10971f ;
	const uint16_t _cap_flags = gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW |
				    gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW;
	const uint16_t _custom_cap_flags = 0;

	// This module act as the gimbal driver. In case of a Mavlink compatible gimbal, the driver is aware of
	// its mechanical limits. So the values below have to match the characteristics of the simulated gimbal
	const float _roll_min = -0.785398f;
	const float _roll_max = 0.785398f;
	const float _pitch_min = -2.35619f;
	const float _pitch_max = 0.785398f;
	const float _yaw_min = NAN; 		// infinite yaw
	const float _yaw_max = NAN;		// infinite yaw

	const uint8_t _gimbal_device_id = 154;	// TODO the implementation differs from the protocol
	uint16_t _gimbal_device_flags = 0;  // GIMBAL_DEVICE_FLAGS

	bool init(const std::string &world_name, const std::string &model_name);
	void Run() override;
	void stop();
	void gimbalIMUCallback(const gz::msgs::IMU &IMU_data);
	void updateParameters();
	/// @brief Poll for new gimbal setpoints either from mavlink gimbal v2 protocol (gimbal_device_set_attitude topic) or from RC inputs (gimbal_controls topic).
	/// @return true if a new setpoint has been requested; false otherwise.
	bool pollSetpoint();
	/// @brief Respond to the gimbal manager when it requests GIMBAL_DEVICE_INFORMATION messages.
	void publishDeviceInfo();
	/// @brief Broadcast gimbal device attitude status message.
	void publishDeviceAttitude();
	/// @brief Compute joint position setpoint taking into account both desired position and velocity. Then publish the command using the specified gazebo node.
	/// @param publisher Gazebo node that will publish the setpoint
	/// @param att_stp desired joint attitude [rad]
	/// @param rate_stp desired joint attitude rate [rad/s]
	/// @param last_stp last joint attitude setpoint [rad]
	/// @param min_stp minimum joint attitude [rad]
	/// @param max_stp maximum joint attitude [rad]
	/// @param dt time interval since the last computation [s]
	static void publishJointCommand(gz::transport::Node::Publisher &publisher, const float att_stp, const float rate_stp,
					float &last_stp,
					const float min_stp, const float max_stp, const float dt);
	/// @brief Compute joint position setpoint taking into account both desired position and velocity.
	/// @param att_stp desired joint attitude [rad]
	/// @param rate_stp desired joint attitude rate [rad/s]
	/// @param last_stp last joint attitude setpoint [rad]
	/// @param dt time interval since the last computation [s]
	/// @return new joint setpoint
	static float computeJointSetpoint(const float att_stp, const float rate_stp, const float last_stp, const float dt);
};
