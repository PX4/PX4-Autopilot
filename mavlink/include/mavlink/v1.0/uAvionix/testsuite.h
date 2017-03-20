/** @file
 *    @brief MAVLink comm protocol testsuite generated from uAvionix.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef UAVIONIX_TESTSUITE_H
#define UAVIONIX_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_uAvionix(system_id, component_id, last_msg);
}
#endif





static void mavlink_test_uAvionix(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UAVIONIX_TESTSUITE_H
