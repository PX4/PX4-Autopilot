#include "BlockSegwayController.hpp"

void BlockSegwayController::update()
{
	// wait for a sensor update, check for exit condition every 100 ms
	if (px4_poll(&_attPoll, 1, 100) < 0) { return; } // poll error

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

	// only update guidance in auto mode
	if (_status.main_state == MAIN_STATE_AUTO) {
		// update guidance
	}

	// compute speed command
	float spdCmd = -th2v.update(_att.pitch) - q2v.update(_att.pitchspeed);

	// handle autopilot modes
	if (_status.main_state == MAIN_STATE_AUTO ||
	    _status.main_state == MAIN_STATE_ALTCTL ||
	    _status.main_state == MAIN_STATE_POSCTL) {
		_actuators.control[0] = spdCmd;
		_actuators.control[1] = spdCmd;

	} else if (_status.main_state == MAIN_STATE_MANUAL) {
		if (_status.navigation_state == NAVIGATION_STATE_DIRECT) {
			_actuators.control[CH_LEFT] = _manual.throttle;
			_actuators.control[CH_RIGHT] = _manual.pitch;

		} else if (_status.navigation_state == NAVIGATION_STATE_STABILIZE) {
			_actuators.control[0] = spdCmd;
			_actuators.control[1] = spdCmd;
		}
	}

	// update all publications
	updatePublications();

}

