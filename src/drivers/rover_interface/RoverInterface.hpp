#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/rover_status.h>
#include <uORB/topics/vehicle_control_mode.h>

#include "scout_sdk/ScoutRobot.hpp"

using namespace time_literals;

class RoverInterface : public ModuleParams, public px4::ScheduledWorkItem
{
	/*
	 * Base interval, has to be compliant with the rate of the actuator_controls
	 * topic subscription and CAN bus update rate
	 */
	static constexpr uint64_t ScheduleIntervalMs{10_ms};

	static constexpr uint64_t RoverStatusPublishIntervalMs{1000_ms};

	static constexpr uint64_t SystemVersionQueryLimitMs{10_ms};

	static constexpr uint64_t ControlSubIntervalMs{50_ms};

public:
	static const char *const CAN_IFACE;

	RoverInterface(uint8_t rover_type, uint32_t bitrate, float vehicle_speed_max);
	~RoverInterface() override;

	static int start(uint8_t rover_type, uint32_t bitrate, float vehicle_speed_max);

	void print_status();

	static RoverInterface *instance() { return _instance; }

private:
	void Init();
	void Run() override;

	void VehicleTorqueAndThrustUpdate();
	void ActuatorArmedUpdate();
	void ActionRequestUpdate();
	void VehicleControlModeUpdate();
	void PublishRoverState();

	// Flag to indicate to tear down the rover interface
	px4::atomic_bool _task_should_exit{false};

	bool _armed{false};

	bool _kill_switch{false};

	bool _initialized{false};

	uint8_t _init_try_count{0};

	bool _is_manual_mode{false};

	static RoverInterface *_instance;

	pthread_mutex_t _node_mutex;

	uint8_t _rover_type;

	uint32_t _bitrate;

	float _vehicle_speed_max;

	float _throttle_control;

	float _yaw_control;

	scoutsdk::ProtocolVersion _protocol_version{scoutsdk::ProtocolVersion::AGX_V2};

	const char *_can_iface{nullptr};

	scoutsdk::ScoutRobot *_scout{nullptr};

	// Subscription
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionInterval _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint), ControlSubIntervalMs};
	uORB::SubscriptionInterval _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint), ControlSubIntervalMs};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _action_request_sub{ORB_ID(action_request)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	// Publication
	orb_advert_t _rover_status_pub{ORB_ADVERT_INVALID};
	rover_status_s _rover_status_msg{};
	hrt_abstime _last_rover_status_publish_time{0};

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RI_MAN_SPD_SC>) _param_man_speed_scale
	)

	/**
	 * Update our local parameter cache.
	 */
	void parameters_update(bool force = false);
};
