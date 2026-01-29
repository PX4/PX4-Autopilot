#include "FlightTaskAltitudeCruise.hpp"

FlightTaskAltitudeCruise::FlightTaskAltitudeCruise()
{
	_sticks_data_required = false; // disable stick requirement to not report flight task failure when they're lost
}

void FlightTaskAltitudeCruise::reActivate()
{
	FlightTaskManualAltitudeSmoothVel::reActivate();
	_stick_tilt_xy.reset();
}

void FlightTaskAltitudeCruise::_updateXYSetpoint()
{
	_acceleration_setpoint.xy() =
		_stick_tilt_xy.generateAccelerationSetpointsForAltitudeCruise(
			_sticks.getPitchRoll(), _deltatime, _yaw, _yaw_setpoint);
}
