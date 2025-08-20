#pragma once

#include "FlightTaskManualAltitude.hpp"

class FlightTaskAltitudeVoyager : public FlightTaskManualAltitude
{
public:
	FlightTaskAltitudeVoyager();
	virtual ~FlightTaskAltitudeVoyager() = default;

	void reActivate() override;

protected:
	void _updateXYSetpoint() override;
};
