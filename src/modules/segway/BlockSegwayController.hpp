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
#include <uORB/topics/encoders.h>
#include <uORB/topics/battery_status.h>

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

class BlockSegwayController : public control::SuperBlock
{
public:
	BlockSegwayController();
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};

	// subscriptions
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<vehicle_global_position_s> _pos;
	uORB::Subscription<position_setpoint_triplet_s> _posCmd;
	uORB::Subscription<vehicle_local_position_s> _localPos;
	uORB::Subscription<vehicle_local_position_setpoint_s> _localPosCmd;
	uORB::Subscription<manual_control_setpoint_s> _manual;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Subscription<encoders_s> _encoders;
	uORB::Subscription<battery_status_s> _battery;

	// publications
	uORB::Publication<vehicle_attitude_setpoint_s> _attCmd;
	uORB::Publication<vehicle_rates_setpoint_s> _ratesCmd;
	uORB::Publication<vehicle_global_velocity_setpoint_s> _globalVelCmd;
	uORB::Publication<actuator_controls_s> _actuators;

	// control blocks
	BlockP _yaw2r; // yaw error to yaw rate cmd
	BlockP _r2v; // yaw rate error to voltage cmd
	BlockP _th2v; // pitch error to voltage cmd (PD P term with q2v)
	BlockP _q2v; // pitch rate error to voltage cmd (PD D term with th2v)
	BlockPI _x2vel; // position error to velocity cmd
	BlockPI _vel2th; // velocity error to pitch cmd
	BlockLimitSym _thLimit; // pitch limit
	BlockLimitSym _velLimit; // velocity limit
	BlockParamFloat _thStop; // angle at which motors are stopped (safety)

	// dynamic inversion
	BlockParamFloat _pulsesPerRev; // encoder pulses per revolution
	BlockParamFloat _mgl; // torque due to gravity
	BlockParamFloat _J; // back emf constant
	BlockParamFloat _k_emf; // emf constant
	BlockParamFloat _k_damp; // back emf constant
	BlockParamFloat _wn_theta; // natural frequency of theta loop
	BlockParamFloat _zeta_theta; // damping of theta loop

	// sysid
	BlockParamFloat _trimPitch; // trim pitch angle
	BlockParamFloat _sysIdAmp; // amplitude of sysid wave
	BlockParamFloat _sysIdFreq; // frequency of sysid wave

	// timing
	struct pollfd _attPoll; // attitude polling
	uint64_t _timeStamp; // timestamp for loop timing

	// functions
	float computeVelocityCmd(float posCmd);
	float computeYawRateCmd(float yawCmd);
	float computePitchCmd(float velCmd);

};
