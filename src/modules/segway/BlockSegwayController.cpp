#include "BlockSegwayController.hpp"

void BlockSegwayController::update() {
	// wait for a sensor update, check for exit condition every 100 ms
	if (poll(&_attPoll, 1, 100) < 0) return; // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) return;

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
	if (_param_update.updated()) updateParams();

	// get new information from subscriptions
	updateSubscriptions();

	// default all output to zero unless handled by mode
	for (unsigned i = 2; i < NUM_ACTUATOR_CONTROLS; i++)
		_actuators.control[i] = 0.0f;

	// only update guidance in auto mode
	if (_status.state_machine == SYSTEM_STATE_AUTO) {
		// update guidance
	}

	// XXX handle STABILIZED (loiter on spot) as well
	// once the system switches from manual or auto to stabilized
	// the setpoint should update to loitering around this position

	// handle autopilot modes
	if (_status.state_machine == SYSTEM_STATE_AUTO ||
	    _status.state_machine == SYSTEM_STATE_STABILIZED) {

		float spdCmd = phi2spd.update(_att.phi);

		// output
		_actuators.control[0] = spdCmd;
		_actuators.control[1] = spdCmd;

	} else if (_status.state_machine == SYSTEM_STATE_MANUAL) {
		_actuators.control[0] = _manual.roll;
		_actuators.control[1] = _manual.roll;
	}

	// update all publications
	updatePublications();

}

