#include "FlightTask.hpp"
#include <mathlib/mathlib.h>

constexpr uint64_t FlightTask::_timeout;

int FlightTask::update()
{
	_time = hrt_elapsed_time(&_starting_time_stamp) / 1e6f;
	_deltatime  = math::min(hrt_elapsed_time(&_last_time_stamp), _timeout) / 1e6f;
	_last_time_stamp = hrt_absolute_time();
	updateSubscriptions();
	_evaluate_vehicle_position();
	return 0;
}

void FlightTask::_evaluate_vehicle_position()
{
	if (hrt_elapsed_time(&_sub_vehicle_local_position.get().timestamp) < _timeout) {
		_position = matrix::Vector3f(&_sub_vehicle_local_position.get().x);
		_velocity = matrix::Vector3f(&_sub_vehicle_local_position.get().vx);
		_yaw = _sub_vehicle_local_position.get().yaw;

	} else {
		_velocity.zero(); /* default velocity is all zero */
	}
}
