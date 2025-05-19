/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file RpmControl.hpp
 *
 * Minimal version of RPM controller for helicopter rotor (no uORB).
 */

#pragma once

#include <lib/pid/PID.hpp>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#include <lib/pid/PID.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/rpm.h>
#include <uORB/topics/water_detection.h>
#include <uORB/topics/water_contact_state.h>


using namespace time_literals;

using uORB::SubscriptionData;

class RpmControl : public ModuleParams, public px4::WorkItem
{
public:
	RpmControl(ModuleParams *parent);
	~RpmControl();

	void Run() override;

	void setSpoolupProgress(float spoolup_progress);
	float getActuatorCorrection();
	void thrusterSafety();

private:

	#define OFF 	0
	#define ON 	1

	uORB::Subscription _rpm_sub{ORB_ID(rpm)};

	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _water_detection_sub{this, ORB_ID(water_detection)};
	uORB::Publication<water_contact_state_s> _water_contact_state_pub{ORB_ID(water_contact_state)};

	// uORB::Publication<water_contact_state_s>    	_water_contact_state_pub;
	water_detection_s 	_water_detection{};
	water_contact_state_s 	_water_contact_state{};

	PID _pid;

	static constexpr float SPOOLUP_PROGRESS_WITH_CONTROLLER_ENGAGED = .8f; // [0,1]
	static constexpr float PID_OUTPUT_LIMIT = .5f; // [0,1]

	bool _rpm_invalid{true};
	float _actuator_correction{0.f};
	float _spoolup_progress{1.f};
	uint64_t _timestamp_last_measurement{0};

	void droneStateMsg(uint8_t state);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_HELI_RPM_SP>) _param_ca_heli_rpm_sp,
		(ParamFloat<px4::params::CA_HELI_RPM_P>)  _param_ca_heli_rpm_p,
		(ParamFloat<px4::params::CA_HELI_RPM_I>)  _param_ca_heli_rpm_i,
		(ParamFloat<px4::params::PWM_OUT_W_MAX>)  _param_pwm_out_w_max,
		(ParamFloat<px4::params::PWM_ON_W_MAX>)   _param_pwm_on_w_max,
		(ParamFloat<px4::params::PWM_IN_W_MAX>)   _param_pwm_in_w_max
	)
};
