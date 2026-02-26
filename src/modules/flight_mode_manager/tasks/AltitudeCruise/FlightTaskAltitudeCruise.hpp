#pragma once

#include "FlightTaskManualAltitudeSmoothVel.hpp"

class FlightTaskAltitudeCruise : public FlightTaskManualAltitudeSmoothVel
{
public:
	FlightTaskAltitudeCruise();
	virtual ~FlightTaskAltitudeCruise() = default;

	void reActivate() override;

protected:
	void _updateXYSetpoint() override;
};
