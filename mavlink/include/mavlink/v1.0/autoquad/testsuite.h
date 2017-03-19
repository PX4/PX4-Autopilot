/** @file
 *    @brief MAVLink comm protocol testsuite generated from autoquad.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef AUTOQUAD_TESTSUITE_H
#define AUTOQUAD_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_autoquad(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_autoquad(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_aq_telemetry_f(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AQ_TELEMETRY_F >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aq_telemetry_f_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,21395
    };
    mavlink_aq_telemetry_f_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.value1 = packet_in.value1;
        packet1.value2 = packet_in.value2;
        packet1.value3 = packet_in.value3;
        packet1.value4 = packet_in.value4;
        packet1.value5 = packet_in.value5;
        packet1.value6 = packet_in.value6;
        packet1.value7 = packet_in.value7;
        packet1.value8 = packet_in.value8;
        packet1.value9 = packet_in.value9;
        packet1.value10 = packet_in.value10;
        packet1.value11 = packet_in.value11;
        packet1.value12 = packet_in.value12;
        packet1.value13 = packet_in.value13;
        packet1.value14 = packet_in.value14;
        packet1.value15 = packet_in.value15;
        packet1.value16 = packet_in.value16;
        packet1.value17 = packet_in.value17;
        packet1.value18 = packet_in.value18;
        packet1.value19 = packet_in.value19;
        packet1.value20 = packet_in.value20;
        packet1.Index = packet_in.Index;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AQ_TELEMETRY_F_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AQ_TELEMETRY_F_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_telemetry_f_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aq_telemetry_f_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_telemetry_f_pack(system_id, component_id, &msg , packet1.Index , packet1.value1 , packet1.value2 , packet1.value3 , packet1.value4 , packet1.value5 , packet1.value6 , packet1.value7 , packet1.value8 , packet1.value9 , packet1.value10 , packet1.value11 , packet1.value12 , packet1.value13 , packet1.value14 , packet1.value15 , packet1.value16 , packet1.value17 , packet1.value18 , packet1.value19 , packet1.value20 );
    mavlink_msg_aq_telemetry_f_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_telemetry_f_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Index , packet1.value1 , packet1.value2 , packet1.value3 , packet1.value4 , packet1.value5 , packet1.value6 , packet1.value7 , packet1.value8 , packet1.value9 , packet1.value10 , packet1.value11 , packet1.value12 , packet1.value13 , packet1.value14 , packet1.value15 , packet1.value16 , packet1.value17 , packet1.value18 , packet1.value19 , packet1.value20 );
    mavlink_msg_aq_telemetry_f_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aq_telemetry_f_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_1 , packet1.Index , packet1.value1 , packet1.value2 , packet1.value3 , packet1.value4 , packet1.value5 , packet1.value6 , packet1.value7 , packet1.value8 , packet1.value9 , packet1.value10 , packet1.value11 , packet1.value12 , packet1.value13 , packet1.value14 , packet1.value15 , packet1.value16 , packet1.value17 , packet1.value18 , packet1.value19 , packet1.value20 );
    mavlink_msg_aq_telemetry_f_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aq_esc_telemetry(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_AQ_ESC_TELEMETRY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_aq_esc_telemetry_t packet_in = {
        963497464,{ 963497672, 963497673, 963497674, 963497675 },{ 963498504, 963498505, 963498506, 963498507 },{ 19107, 19108, 19109, 19110 },137,204,15,{ 82, 83, 84, 85 },{ 94, 95, 96, 97 }
    };
    mavlink_aq_esc_telemetry_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.time_boot_ms = packet_in.time_boot_ms;
        packet1.seq = packet_in.seq;
        packet1.num_motors = packet_in.num_motors;
        packet1.num_in_seq = packet_in.num_in_seq;
        
        mav_array_memcpy(packet1.data0, packet_in.data0, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.data1, packet_in.data1, sizeof(uint32_t)*4);
        mav_array_memcpy(packet1.status_age, packet_in.status_age, sizeof(uint16_t)*4);
        mav_array_memcpy(packet1.escid, packet_in.escid, sizeof(uint8_t)*4);
        mav_array_memcpy(packet1.data_version, packet_in.data_version, sizeof(uint8_t)*4);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_esc_telemetry_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_aq_esc_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_esc_telemetry_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.seq , packet1.num_motors , packet1.num_in_seq , packet1.escid , packet1.status_age , packet1.data_version , packet1.data0 , packet1.data1 );
    mavlink_msg_aq_esc_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_esc_telemetry_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.seq , packet1.num_motors , packet1.num_in_seq , packet1.escid , packet1.status_age , packet1.data_version , packet1.data0 , packet1.data1 );
    mavlink_msg_aq_esc_telemetry_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_aq_esc_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_aq_esc_telemetry_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.seq , packet1.num_motors , packet1.num_in_seq , packet1.escid , packet1.status_age , packet1.data_version , packet1.data0 , packet1.data1 );
    mavlink_msg_aq_esc_telemetry_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_autoquad(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_aq_telemetry_f(system_id, component_id, last_msg);
    mavlink_test_aq_esc_telemetry(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // AUTOQUAD_TESTSUITE_H
