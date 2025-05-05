#include "cruise.h"
#include "navigator.h"

CruiseMode::CruiseMode(Navigator *navigator) : NavigatorMode(navigator)
{
}

void CruiseMode::initialize()
{
    // No specific initialization needed
}

void CruiseMode::on_active()
{
    position_setpoint_triplet_s triplet = {};
    triplet.current.valid = true;
   // triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
    triplet.current.type = position_setpoint_s::SETPOINT_TYPE_VELOCITY;
    triplet.current.vx = 10.0f; // Matches FlightTaskCruise, though ignored
    triplet.current.vy = 0.0f;
    triplet.current.vz = 0.0f;
    triplet.current.alt = _navigator->get_global_position()->alt; // Maintain altitude
    triplet.current.yaw = _navigator->get_local_position()->heading; // Maintain heading
    triplet.current.timestamp = hrt_absolute_time();

    _navigator->_pos_sp_triplet = triplet; // Set directly, allowed via friend
    _navigator->publish_position_setpoint_triplet();
}
