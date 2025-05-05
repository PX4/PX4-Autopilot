#pragma once

#include <FlightTask.hpp>

class FlightTaskCruise : public FlightTask
{
public:
    FlightTaskCruise() = default;
    virtual ~FlightTaskCruise() = default;

    bool update() override;
    bool activate(const trajectory_setpoint_s &last_setpoint) override;
};



