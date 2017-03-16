/** @file
 *    @brief MAVLink comm protocol testsuite generated from slugs.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef SLUGS_TESTSUITE_H
#define SLUGS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_slugs(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_slugs(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_cpu_load(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CPU_LOAD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cpu_load_t packet_in = {
        17235,139,206
    };
    mavlink_cpu_load_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.batVolt = packet_in.batVolt;
        packet1.sensLoad = packet_in.sensLoad;
        packet1.ctrlLoad = packet_in.ctrlLoad;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cpu_load_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cpu_load_pack(system_id, component_id, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
    mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cpu_load_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
    mavlink_msg_cpu_load_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cpu_load_send(MAVLINK_COMM_1 , packet1.sensLoad , packet1.ctrlLoad , packet1.batVolt );
    mavlink_msg_cpu_load_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensor_bias(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_BIAS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_bias_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_sensor_bias_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.axBias = packet_in.axBias;
        packet1.ayBias = packet_in.ayBias;
        packet1.azBias = packet_in.azBias;
        packet1.gxBias = packet_in.gxBias;
        packet1.gyBias = packet_in.gyBias;
        packet1.gzBias = packet_in.gzBias;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_BIAS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_bias_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_bias_pack(system_id, component_id, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
    mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_bias_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
    mavlink_msg_sensor_bias_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_bias_send(MAVLINK_COMM_1 , packet1.axBias , packet1.ayBias , packet1.azBias , packet1.gxBias , packet1.gyBias , packet1.gzBias );
    mavlink_msg_sensor_bias_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_diagnostic(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DIAGNOSTIC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_diagnostic_t packet_in = {
        17.0,45.0,73.0,17859,17963,18067
    };
    mavlink_diagnostic_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.diagFl1 = packet_in.diagFl1;
        packet1.diagFl2 = packet_in.diagFl2;
        packet1.diagFl3 = packet_in.diagFl3;
        packet1.diagSh1 = packet_in.diagSh1;
        packet1.diagSh2 = packet_in.diagSh2;
        packet1.diagSh3 = packet_in.diagSh3;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DIAGNOSTIC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DIAGNOSTIC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_diagnostic_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_diagnostic_pack(system_id, component_id, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
    mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_diagnostic_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
    mavlink_msg_diagnostic_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_diagnostic_send(MAVLINK_COMM_1 , packet1.diagFl1 , packet1.diagFl2 , packet1.diagFl3 , packet1.diagSh1 , packet1.diagSh2 , packet1.diagSh3 );
    mavlink_msg_diagnostic_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_navigation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLUGS_NAVIGATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slugs_navigation_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,223,34
    };
    mavlink_slugs_navigation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.u_m = packet_in.u_m;
        packet1.phi_c = packet_in.phi_c;
        packet1.theta_c = packet_in.theta_c;
        packet1.psiDot_c = packet_in.psiDot_c;
        packet1.ay_body = packet_in.ay_body;
        packet1.totalDist = packet_in.totalDist;
        packet1.dist2Go = packet_in.dist2Go;
        packet1.h_c = packet_in.h_c;
        packet1.fromWP = packet_in.fromWP;
        packet1.toWP = packet_in.toWP;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLUGS_NAVIGATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLUGS_NAVIGATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_navigation_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_navigation_pack(system_id, component_id, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP , packet1.h_c );
    mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_navigation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP , packet1.h_c );
    mavlink_msg_slugs_navigation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_navigation_send(MAVLINK_COMM_1 , packet1.u_m , packet1.phi_c , packet1.theta_c , packet1.psiDot_c , packet1.ay_body , packet1.totalDist , packet1.dist2Go , packet1.fromWP , packet1.toWP , packet1.h_c );
    mavlink_msg_slugs_navigation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_data_log(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DATA_LOG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_data_log_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_data_log_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.fl_1 = packet_in.fl_1;
        packet1.fl_2 = packet_in.fl_2;
        packet1.fl_3 = packet_in.fl_3;
        packet1.fl_4 = packet_in.fl_4;
        packet1.fl_5 = packet_in.fl_5;
        packet1.fl_6 = packet_in.fl_6;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DATA_LOG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DATA_LOG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_log_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_log_pack(system_id, component_id, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
    mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_log_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
    mavlink_msg_data_log_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_data_log_send(MAVLINK_COMM_1 , packet1.fl_1 , packet1.fl_2 , packet1.fl_3 , packet1.fl_4 , packet1.fl_5 , packet1.fl_6 );
    mavlink_msg_data_log_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_date_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_DATE_TIME >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_date_time_t packet_in = {
        5,72,139,206,17,84,151,218,29,96,163,230
    };
    mavlink_gps_date_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.year = packet_in.year;
        packet1.month = packet_in.month;
        packet1.day = packet_in.day;
        packet1.hour = packet_in.hour;
        packet1.min = packet_in.min;
        packet1.sec = packet_in.sec;
        packet1.clockStat = packet_in.clockStat;
        packet1.visSat = packet_in.visSat;
        packet1.useSat = packet_in.useSat;
        packet1.GppGl = packet_in.GppGl;
        packet1.sigUsedMask = packet_in.sigUsedMask;
        packet1.percentUsed = packet_in.percentUsed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_date_time_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_date_time_pack(system_id, component_id, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.clockStat , packet1.visSat , packet1.useSat , packet1.GppGl , packet1.sigUsedMask , packet1.percentUsed );
    mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_date_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.clockStat , packet1.visSat , packet1.useSat , packet1.GppGl , packet1.sigUsedMask , packet1.percentUsed );
    mavlink_msg_gps_date_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_date_time_send(MAVLINK_COMM_1 , packet1.year , packet1.month , packet1.day , packet1.hour , packet1.min , packet1.sec , packet1.clockStat , packet1.visSat , packet1.useSat , packet1.GppGl , packet1.sigUsedMask , packet1.percentUsed );
    mavlink_msg_gps_date_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mid_lvl_cmds(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MID_LVL_CMDS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mid_lvl_cmds_t packet_in = {
        17.0,45.0,73.0,41
    };
    mavlink_mid_lvl_cmds_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.hCommand = packet_in.hCommand;
        packet1.uCommand = packet_in.uCommand;
        packet1.rCommand = packet_in.rCommand;
        packet1.target = packet_in.target;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MID_LVL_CMDS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mid_lvl_cmds_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mid_lvl_cmds_pack(system_id, component_id, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
    mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mid_lvl_cmds_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
    mavlink_msg_mid_lvl_cmds_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mid_lvl_cmds_send(MAVLINK_COMM_1 , packet1.target , packet1.hCommand , packet1.uCommand , packet1.rCommand );
    mavlink_msg_mid_lvl_cmds_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ctrl_srfc_pt(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CTRL_SRFC_PT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ctrl_srfc_pt_t packet_in = {
        17235,139
    };
    mavlink_ctrl_srfc_pt_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.bitfieldPt = packet_in.bitfieldPt;
        packet1.target = packet_in.target;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CTRL_SRFC_PT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ctrl_srfc_pt_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ctrl_srfc_pt_pack(system_id, component_id, &msg , packet1.target , packet1.bitfieldPt );
    mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ctrl_srfc_pt_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.bitfieldPt );
    mavlink_msg_ctrl_srfc_pt_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ctrl_srfc_pt_send(MAVLINK_COMM_1 , packet1.target , packet1.bitfieldPt );
    mavlink_msg_ctrl_srfc_pt_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_camera_order(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slugs_camera_order_t packet_in = {
        5,72,139,206,17
    };
    mavlink_slugs_camera_order_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target = packet_in.target;
        packet1.pan = packet_in.pan;
        packet1.tilt = packet_in.tilt;
        packet1.zoom = packet_in.zoom;
        packet1.moveHome = packet_in.moveHome;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLUGS_CAMERA_ORDER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_camera_order_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slugs_camera_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_camera_order_pack(system_id, component_id, &msg , packet1.target , packet1.pan , packet1.tilt , packet1.zoom , packet1.moveHome );
    mavlink_msg_slugs_camera_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_camera_order_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.pan , packet1.tilt , packet1.zoom , packet1.moveHome );
    mavlink_msg_slugs_camera_order_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slugs_camera_order_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_camera_order_send(MAVLINK_COMM_1 , packet1.target , packet1.pan , packet1.tilt , packet1.zoom , packet1.moveHome );
    mavlink_msg_slugs_camera_order_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_control_surface(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONTROL_SURFACE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_control_surface_t packet_in = {
        17.0,45.0,29,96
    };
    mavlink_control_surface_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mControl = packet_in.mControl;
        packet1.bControl = packet_in.bControl;
        packet1.target = packet_in.target;
        packet1.idSurface = packet_in.idSurface;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONTROL_SURFACE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_surface_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_control_surface_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_surface_pack(system_id, component_id, &msg , packet1.target , packet1.idSurface , packet1.mControl , packet1.bControl );
    mavlink_msg_control_surface_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_surface_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.idSurface , packet1.mControl , packet1.bControl );
    mavlink_msg_control_surface_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_control_surface_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_surface_send(MAVLINK_COMM_1 , packet1.target , packet1.idSurface , packet1.mControl , packet1.bControl );
    mavlink_msg_control_surface_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_mobile_location(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slugs_mobile_location_t packet_in = {
        17.0,45.0,29
    };
    mavlink_slugs_mobile_location_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.target = packet_in.target;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLUGS_MOBILE_LOCATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_mobile_location_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slugs_mobile_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_mobile_location_pack(system_id, component_id, &msg , packet1.target , packet1.latitude , packet1.longitude );
    mavlink_msg_slugs_mobile_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_mobile_location_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.latitude , packet1.longitude );
    mavlink_msg_slugs_mobile_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slugs_mobile_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_mobile_location_send(MAVLINK_COMM_1 , packet1.target , packet1.latitude , packet1.longitude );
    mavlink_msg_slugs_mobile_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs_configuration_camera(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_slugs_configuration_camera_t packet_in = {
        5,72,139
    };
    mavlink_slugs_configuration_camera_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target = packet_in.target;
        packet1.idOrder = packet_in.idOrder;
        packet1.order = packet_in.order;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SLUGS_CONFIGURATION_CAMERA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_configuration_camera_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_slugs_configuration_camera_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_configuration_camera_pack(system_id, component_id, &msg , packet1.target , packet1.idOrder , packet1.order );
    mavlink_msg_slugs_configuration_camera_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_configuration_camera_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.idOrder , packet1.order );
    mavlink_msg_slugs_configuration_camera_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_slugs_configuration_camera_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_slugs_configuration_camera_send(MAVLINK_COMM_1 , packet1.target , packet1.idOrder , packet1.order );
    mavlink_msg_slugs_configuration_camera_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_isr_location(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ISR_LOCATION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_isr_location_t packet_in = {
        17.0,45.0,73.0,41,108,175,242
    };
    mavlink_isr_location_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.height = packet_in.height;
        packet1.target = packet_in.target;
        packet1.option1 = packet_in.option1;
        packet1.option2 = packet_in.option2;
        packet1.option3 = packet_in.option3;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ISR_LOCATION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isr_location_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_isr_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isr_location_pack(system_id, component_id, &msg , packet1.target , packet1.latitude , packet1.longitude , packet1.height , packet1.option1 , packet1.option2 , packet1.option3 );
    mavlink_msg_isr_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isr_location_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.latitude , packet1.longitude , packet1.height , packet1.option1 , packet1.option2 , packet1.option3 );
    mavlink_msg_isr_location_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_isr_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_isr_location_send(MAVLINK_COMM_1 , packet1.target , packet1.latitude , packet1.longitude , packet1.height , packet1.option1 , packet1.option2 , packet1.option3 );
    mavlink_msg_isr_location_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_volt_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VOLT_SENSOR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_volt_sensor_t packet_in = {
        17235,17339,17
    };
    mavlink_volt_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage = packet_in.voltage;
        packet1.reading2 = packet_in.reading2;
        packet1.r2Type = packet_in.r2Type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VOLT_SENSOR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_volt_sensor_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_volt_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_volt_sensor_pack(system_id, component_id, &msg , packet1.r2Type , packet1.voltage , packet1.reading2 );
    mavlink_msg_volt_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_volt_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.r2Type , packet1.voltage , packet1.reading2 );
    mavlink_msg_volt_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_volt_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_volt_sensor_send(MAVLINK_COMM_1 , packet1.r2Type , packet1.voltage , packet1.reading2 );
    mavlink_msg_volt_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ptz_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PTZ_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ptz_status_t packet_in = {
        17235,17339,17
    };
    mavlink_ptz_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pan = packet_in.pan;
        packet1.tilt = packet_in.tilt;
        packet1.zoom = packet_in.zoom;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PTZ_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ptz_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ptz_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ptz_status_pack(system_id, component_id, &msg , packet1.zoom , packet1.pan , packet1.tilt );
    mavlink_msg_ptz_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ptz_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.zoom , packet1.pan , packet1.tilt );
    mavlink_msg_ptz_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ptz_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ptz_status_send(MAVLINK_COMM_1 , packet1.zoom , packet1.pan , packet1.tilt );
    mavlink_msg_ptz_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uav_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAV_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uav_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,65
    };
    mavlink_uav_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.altitude = packet_in.altitude;
        packet1.speed = packet_in.speed;
        packet1.course = packet_in.course;
        packet1.target = packet_in.target;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAV_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_status_pack(system_id, component_id, &msg , packet1.target , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.course );
    mavlink_msg_uav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.course );
    mavlink_msg_uav_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_status_send(MAVLINK_COMM_1 , packet1.target , packet1.latitude , packet1.longitude , packet1.altitude , packet1.speed , packet1.course );
    mavlink_msg_uav_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_status_gps(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_STATUS_GPS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_status_gps_t packet_in = {
        17.0,17443,151,218,29,96,163
    };
    mavlink_status_gps_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.magVar = packet_in.magVar;
        packet1.csFails = packet_in.csFails;
        packet1.gpsQuality = packet_in.gpsQuality;
        packet1.msgsType = packet_in.msgsType;
        packet1.posStatus = packet_in.posStatus;
        packet1.magDir = packet_in.magDir;
        packet1.modeInd = packet_in.modeInd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_status_gps_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_status_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_status_gps_pack(system_id, component_id, &msg , packet1.csFails , packet1.gpsQuality , packet1.msgsType , packet1.posStatus , packet1.magVar , packet1.magDir , packet1.modeInd );
    mavlink_msg_status_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_status_gps_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.csFails , packet1.gpsQuality , packet1.msgsType , packet1.posStatus , packet1.magVar , packet1.magDir , packet1.modeInd );
    mavlink_msg_status_gps_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_status_gps_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_status_gps_send(MAVLINK_COMM_1 , packet1.csFails , packet1.gpsQuality , packet1.msgsType , packet1.posStatus , packet1.magVar , packet1.magDir , packet1.modeInd );
    mavlink_msg_status_gps_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_novatel_diag(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NOVATEL_DIAG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_novatel_diag_t packet_in = {
        963497464,45.0,17651,163,230,41,108
    };
    mavlink_novatel_diag_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.receiverStatus = packet_in.receiverStatus;
        packet1.posSolAge = packet_in.posSolAge;
        packet1.csFails = packet_in.csFails;
        packet1.timeStatus = packet_in.timeStatus;
        packet1.solStatus = packet_in.solStatus;
        packet1.posType = packet_in.posType;
        packet1.velType = packet_in.velType;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NOVATEL_DIAG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_novatel_diag_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_novatel_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_novatel_diag_pack(system_id, component_id, &msg , packet1.timeStatus , packet1.receiverStatus , packet1.solStatus , packet1.posType , packet1.velType , packet1.posSolAge , packet1.csFails );
    mavlink_msg_novatel_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_novatel_diag_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timeStatus , packet1.receiverStatus , packet1.solStatus , packet1.posType , packet1.velType , packet1.posSolAge , packet1.csFails );
    mavlink_msg_novatel_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_novatel_diag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_novatel_diag_send(MAVLINK_COMM_1 , packet1.timeStatus , packet1.receiverStatus , packet1.solStatus , packet1.posType , packet1.velType , packet1.posSolAge , packet1.csFails );
    mavlink_msg_novatel_diag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sensor_diag(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_DIAG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_diag_t packet_in = {
        17.0,45.0,17651,163
    };
    mavlink_sensor_diag_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.float1 = packet_in.float1;
        packet1.float2 = packet_in.float2;
        packet1.int1 = packet_in.int1;
        packet1.char1 = packet_in.char1;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_DIAG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_diag_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_diag_pack(system_id, component_id, &msg , packet1.float1 , packet1.float2 , packet1.int1 , packet1.char1 );
    mavlink_msg_sensor_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_diag_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.float1 , packet1.float2 , packet1.int1 , packet1.char1 );
    mavlink_msg_sensor_diag_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_diag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_diag_send(MAVLINK_COMM_1 , packet1.float1 , packet1.float2 , packet1.int1 , packet1.char1 );
    mavlink_msg_sensor_diag_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_boot(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BOOT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_boot_t packet_in = {
        963497464
    };
    mavlink_boot_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.version = packet_in.version;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BOOT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BOOT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_boot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_pack(system_id, component_id, &msg , packet1.version );
    mavlink_msg_boot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.version );
    mavlink_msg_boot_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_boot_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_boot_send(MAVLINK_COMM_1 , packet1.version );
    mavlink_msg_boot_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_slugs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_cpu_load(system_id, component_id, last_msg);
    mavlink_test_sensor_bias(system_id, component_id, last_msg);
    mavlink_test_diagnostic(system_id, component_id, last_msg);
    mavlink_test_slugs_navigation(system_id, component_id, last_msg);
    mavlink_test_data_log(system_id, component_id, last_msg);
    mavlink_test_gps_date_time(system_id, component_id, last_msg);
    mavlink_test_mid_lvl_cmds(system_id, component_id, last_msg);
    mavlink_test_ctrl_srfc_pt(system_id, component_id, last_msg);
    mavlink_test_slugs_camera_order(system_id, component_id, last_msg);
    mavlink_test_control_surface(system_id, component_id, last_msg);
    mavlink_test_slugs_mobile_location(system_id, component_id, last_msg);
    mavlink_test_slugs_configuration_camera(system_id, component_id, last_msg);
    mavlink_test_isr_location(system_id, component_id, last_msg);
    mavlink_test_volt_sensor(system_id, component_id, last_msg);
    mavlink_test_ptz_status(system_id, component_id, last_msg);
    mavlink_test_uav_status(system_id, component_id, last_msg);
    mavlink_test_status_gps(system_id, component_id, last_msg);
    mavlink_test_novatel_diag(system_id, component_id, last_msg);
    mavlink_test_sensor_diag(system_id, component_id, last_msg);
    mavlink_test_boot(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SLUGS_TESTSUITE_H
