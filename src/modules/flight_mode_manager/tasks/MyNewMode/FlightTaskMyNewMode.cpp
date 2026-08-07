#include "FlightTaskMyNewMode.hpp"

// Intentionally empty: FlightTaskMyMode inherits 100% of its behavior from
// FlightTaskManualPosition via the constructor-inheriting `using` in the
// header. This file exists so the CMakeLists' px4_add_library() has a
// translation unit to compile; add method overrides here once you want
// MyMode to actually differ from Position mode.
