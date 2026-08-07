#pragma once

// NOTE: I've inferred that the Position-mode flight task is named
// FlightTaskManualPosition (and its header FlightTaskManualPosition.hpp)
// from it appearing as "ManualPosition" in flight_mode_manager/CMakeLists.txt
// in the diff you pasted -- I have not directly seen that file's contents,
// so please confirm the exact header path/class name in your checkout
// (it should live at
//  src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.hpp)
// and fix the #include below if it differs.
#include "FlightTaskManualPosition.hpp"

// A pure copy of Position mode: zero behavior changes, just reachable
// under its own name/mode slot. This mirrors how FlightTaskAltitudeCruise
// subclasses FlightTaskManualAltitudeSmoothVel -- except here nothing is
// overridden yet. Add overrides here later once this confirmed builds and
// switches correctly.
class FlightTaskMyMode : public FlightTaskManualPosition
{
public:
	using FlightTaskManualPosition::FlightTaskManualPosition;
	virtual ~FlightTaskMyMode() = default;
};
