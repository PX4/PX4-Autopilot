#include "FlightTaskAltitudeVoyager.hpp"

void FlightTaskAltitudeVoyager::reActivate()
{
	FlightTaskManualAltitude::reActivate();
	_stick_tilt_xy.reset();
}

void FlightTaskAltitudeVoyager::_updateXYSetpoint()
{
	_acceleration_setpoint.xy() =
		_stick_tilt_xy.generateAccelerationSetpointsForVoyager(
			_sticks.getPitchRoll(), _deltatime, _yaw, _yaw_setpoint);
}
