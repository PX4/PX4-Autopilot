#pragma once

#include "FlightTaskManualAltitude.hpp"

class FlightTaskAltitudeCruise : public FlightTaskManualAltitude
{
public:
	FlightTaskAltitudeCruise();
	virtual ~FlightTaskAltitudeCruise() = default;

	void reActivate() override;

protected:
	void _updateXYSetpoint() override;
};
