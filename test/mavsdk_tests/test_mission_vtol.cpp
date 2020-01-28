//
// VTOL mission test.
//
// Author: Lorenz Meier <lorenz@px4.io>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"


TEST_CASE("Takeoff and transition and RTL", "[vtol]")
{
    AutopilotTester tester;
    tester.connect(connection_url);
    tester.wait_until_ready();
    tester.arm();
    tester.takeoff();
    tester.wait_until_hovering();
    tester.transition_to_fixedwing();
    tester.execute_rtl();
    tester.wait_until_disarmed();
}
