#pragma once

#include "navigator_mode.h"
#include <uORB/topics/position_setpoint_triplet.h>

class Navigator;

class CruiseMode : public NavigatorMode
{
public:
    CruiseMode(Navigator *navigator);
    virtual ~CruiseMode() = default;
    void initialize() override;

protected:
    void on_active() override;
};
