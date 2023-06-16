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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rover_status.h>

#include "scout_sdk/ScoutRobot.hpp"

using namespace time_literals;

class RoverInterface : public ModuleParams, public px4::ScheduledWorkItem
{
	/*
	 * Base interval, has to be compliant with the rate of the actuator_controls
	 * topic subscription and CAN bus update rate
	 */
	static constexpr uint64_t ScheduleIntervalMs{500_us};

	static constexpr uint64_t RoverStatusPublishIntervalMs{1000_ms};

	static constexpr uint64_t SystemVersionQueryLimitMs{10_ms};

	static constexpr uint64_t ActuatorControlSubIntervalMs{20_ms};

	static constexpr uint64_t ActuatorArmedSubIntervalMs{1000_ms};

public:
	static const char *const CAN_IFACE;

	RoverInterface(uint8_t rover_type, uint32_t bitrate);
	~RoverInterface() override;

	static int start(uint8_t rover_type, uint32_t bitrate);

	void print_status();

	static RoverInterface *instance() { return _instance; }

private:
	void Init();
	void Run() override;

	void ActuatorControlsUpdate();
	void ActuatorArmedUpdate();
	void PublishRoverState();

	// Flag to indicate to tear down the rover interface
	px4::atomic_bool _task_should_exit{false};

	bool _armed{false};

	bool _manual_lockdown{false};

	bool _initialized{false};

	uint8_t _init_try_count{0};

	static RoverInterface *_instance;

	pthread_mutex_t _node_mutex;

	uint8_t _rover_type;

	uint32_t _bitrate;

	scoutsdk::ProtocolVersion _protocol_version{scoutsdk::ProtocolVersion::AGX_V2};

	const char *_can_iface{nullptr};

	scoutsdk::ScoutRobot *_scout{nullptr};

	// Subscription
	uORB::SubscriptionInterval _actuator_controls_sub{ORB_ID(actuator_controls_0), ActuatorControlSubIntervalMs};
	uORB::SubscriptionInterval _actuator_armed_sub{ORB_ID(actuator_armed), ActuatorArmedSubIntervalMs};

	// Publication
	orb_advert_t _rover_status_pub{ORB_ADVERT_INVALID};
	rover_status_s _rover_status_msg{};
	hrt_abstime _last_rover_status_publish_time{0};

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": cycle interval")};
};
