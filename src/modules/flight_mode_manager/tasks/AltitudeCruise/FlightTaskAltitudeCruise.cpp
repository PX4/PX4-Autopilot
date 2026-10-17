#include "FlightTaskAltitudeCruise.hpp"

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
