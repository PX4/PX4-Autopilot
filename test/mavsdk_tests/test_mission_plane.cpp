//
// Multicopter mission test.
//
// Author: Julian Oes <julian@oes.ch>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <string>
#include "autopilot_tester.h"


TEST_CASE("Takeoff and land (Plane)", "[plane]")
{
    AutopilotTester tester;
    tester.connect(connection_url);
    tester.wait_until_ready();
    tester.arm();
    tester.takeoff();
    tester.wait_until_hovering();
    tester.land();
    tester.wait_until_disarmed();
}
