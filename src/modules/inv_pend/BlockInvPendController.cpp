#include "BlockInvPendController.hpp"

// px4
#include <geo/geo.h>
#include <drivers/drv_hrt.h>

BlockInvPendController::BlockInvPendController() :
	SuperBlock(NULL, "INVP"),

	// subscriptions
	_att(&getSubscriptions(), ORB_ID(vehicle_attitude), 3),
	_manual(&getSubscriptions(), ORB_ID(manual_control_setpoint), 3),
	_status(&getSubscriptions(), ORB_ID(vehicle_status), 3),
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz

	// publications
	_attCmd(&getPublications(), ORB_ID(vehicle_attitude_setpoint)),
	_actuators(&getPublications(), ORB_ID(actuator_controls_1)),

	_th2v(this, "TH2V"),
	_q2v(this, "Q2V"),
	_thLimit(this, "TH_LIM"),
	_thStop(this, "TH_STOP"),
	_sysIdAmp(this, "SYSID_AMP"),
	_sysIdFreq(this, "SYSID_FREQ"),
	_attPoll(),
	_timeStamp(0)
{
	orb_set_interval(_att.getHandle(), 10); // set attitude update rate to 100 Hz (period 10 ms)
	_attPoll.fd = _att.getHandle();
	_attPoll.events = POLLIN;
}

void BlockInvPendController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++) {
		_actuators.control[i] = 0.0f;
	}

	// commands for inner stabilization loop
	float thCmd = 0; // pitch command

	// signals for system id
	float sineWave = _sysIdAmp.get() * sinf(2.0f * M_PI_F * _sysIdFreq.get() * _timeStamp / 1.0e6f);
	float squareWave = 0;

	if (sineWave > 0) {
		squareWave = _sysIdAmp.get();

	} else {
		squareWave = -_sysIdAmp.get();
	}

	// modes
	if (_status.main_state == MAIN_STATE_MANUAL) {
		// user controls vel cmd and yaw rate cmd
		if (_manual.y > 0.5f) {
			thCmd = 0.1f;
		} else if (_manual.y < -0.5f) {
			thCmd = -0.1f;
		} else {
			thCmd = 0.0f;
		}
	} else if (_status.main_state == MAIN_STATE_ALTCTL) {
		// used for sysid
		thCmd = squareWave;
	} else if (_status.main_state == MAIN_STATE_POSCTL) {
		// used for sysid
		thCmd = sineWave;
	} else {
		thCmd = 0;
	}

	// compute control for pitch
	float controlPitch = _th2v.update(thCmd - _att.pitch)
			     - _q2v.update(_att.pitchspeed);

	// output scaling by manual throttle
	controlPitch *= _manual.z;

	// attitude set point
	_attCmd.timestamp = _timeStamp;
	_attCmd.pitch_body = thCmd;
	_attCmd.roll_body = 0;
	_attCmd.yaw_body = 0;
	_attCmd.R_valid = false;
	_attCmd.q_d_valid = false;
	_attCmd.q_e_valid = false;
	_attCmd.thrust = 0;
	_attCmd.roll_reset_integral = false;
	_attCmd.update();

	// send outputs if armed and pitch less
	// than shut off pitch
	if (_status.arming_state == ARMING_STATE_ARMED &&
	    fabsf(_att.pitch) < _thStop.get()) {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = controlPitch; // pitch
		_actuators.control[2] = 0; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();

	} else {
		// controls
		_actuators.timestamp = _timeStamp;
		_actuators.control[0] = 0; // roll
		_actuators.control[1] = 0; // pitch
		_actuators.control[2] = 0; // yaw
		_actuators.control[3] = 0; // thrust
		_actuators.update();
	}
}

