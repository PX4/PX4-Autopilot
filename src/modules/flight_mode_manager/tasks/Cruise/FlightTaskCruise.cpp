
#include "FlightTaskCruise.hpp"
#include <matrix/math.hpp>

bool FlightTaskCruise::activate(const trajectory_setpoint_s &last_setpoint)
{
    bool ret = FlightTask::activate(last_setpoint);
    // Set 10 m/s forward velocity (body frame)
    _velocity_setpoint = matrix::Vector3f(10.0f, 0.0f, 0.0f);
    // Maintain current altitude and heading
    _position_setpoint(2) = NAN; // Z-axis (altitude)
    _yaw_setpoint = NAN; // Current yaw
    return ret;
}

bool FlightTaskCruise::update()
{
    // Keep constant velocity (set in activate)
    _velocity_setpoint = matrix::Vector3f(10.0f, 0.0f, 0.0f);
  //  _position_setpoint(2) = NAN; // Maintain altitude
 //   _yaw_setpoint = NAN; // Maintain yaw

    return true;
}


