#pragma once

#include "FlightTaskManualAltitudeSmoothVel.hpp"

class FlightTaskAltitudeCruise : public FlightTaskManualAltitudeSmoothVel
{
public:
	FlightTaskAltitudeCruise() = default;
	virtual ~FlightTaskAltitudeCruise() = default;

	// Keeps running with centered sticks when the manual control signal is lost
	static constexpr uint8_t kRequiredInputs = FlightTaskManualAltitudeSmoothVel::kRequiredInputs & ~ManualControl;
	uint8_t requiredInputs() const override { return kRequiredInputs; }

	void reActivate() override;

protected:
	void _updateXYSetpoint() override;
};
