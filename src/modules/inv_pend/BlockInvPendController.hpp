#pragma once

// system
#include <poll.h>

// subscription topics
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

// publication topics
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/actuator_controls.h>

// control blocks
#include <controllib/blocks.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

using namespace control;

class BlockInvPendController : public control::SuperBlock
{
public:
	BlockInvPendController();
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};

	// subscriptions
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<manual_control_setpoint_s> _manual;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;

	// publications
	uORB::Publication<vehicle_attitude_setpoint_s> _attCmd;
	uORB::Publication<actuator_controls_s> _actuators;

	// control blocks
	BlockP _th2v; // pitch error to voltage cmd (PD P term with q2v)
	BlockP _q2v; // pitch rate error to voltage cmd (PD D term with th2v)
	BlockLimitSym _thLimit; // pitch limit
	BlockParamFloat _thStop; // angle at which motors are stopped (safety)

	// sysid
	BlockParamFloat _sysIdAmp; // amplitude of sysid wave
	BlockParamFloat _sysIdFreq; // frequency of sysid wave

	// timing
	struct pollfd _attPoll; // attitude polling
	uint64_t _timeStamp; // timestamp for loop timing
};
