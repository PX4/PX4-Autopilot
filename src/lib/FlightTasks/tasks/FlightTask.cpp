#include "FlightTask.hpp"
#include <mathlib/mathlib.h>

constexpr uint64_t FlightTask::_timeout;


bool FlightTask::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!subscription_array.get(ORB_ID(vehicle_local_position), _sub_vehicle_local_position)) {
		return false;
	}

	return true;
}

int FlightTask::update()
{
	_time_stamp_current = hrt_absolute_time();
	_time = (_time_stamp_current - _time_stamp_activate) / 1e6f;
	_deltatime  = math::min((_time_stamp_current - _time_stamp_last), _timeout) / 1e6f;
	_time_stamp_last = _time_stamp_current;
	int ret = _evaluate_vehicle_position();
	return ret;
}

int FlightTask::_evaluate_vehicle_position()
{
	if ((_time_stamp_current - _sub_vehicle_local_position->get().timestamp) < _timeout) {
		_position = matrix::Vector3f(&_sub_vehicle_local_position->get().x);
		_velocity = matrix::Vector3f(&_sub_vehicle_local_position->get().vx);
		_yaw = _sub_vehicle_local_position->get().yaw;
		return 0;

	} else {
		_velocity.zero(); /* default velocity is all zero */
		return -1;
	}
}
