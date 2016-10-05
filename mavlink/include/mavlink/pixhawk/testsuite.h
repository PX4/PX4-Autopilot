/** @file
 *	@brief MAVLink comm protocol testsuite generated from pixhawk.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef PIXHAWK_TESTSUITE_H
#define PIXHAWK_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_pixhawk(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_pixhawk(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mavros_test_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAVROS_TEST_MSG >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mavros_test_msg_t packet_in = {
		93372036854775807ULL,73.0
    };
	mavlink_mavros_test_msg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.test = packet_in.test;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mavros_test_msg_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mavros_test_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mavros_test_msg_pack(system_id, component_id, &msg , packet1.timestamp , packet1.test );
	mavlink_msg_mavros_test_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mavros_test_msg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.test );
	mavlink_msg_mavros_test_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mavros_test_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mavros_test_msg_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.test );
	mavlink_msg_mavros_test_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_sia(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_SIA >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_sia_t packet_in = {
		93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,213.0,101,168
    };
	mavlink_gps_sia_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.gps_jingdu = packet_in.gps_jingdu;
        packet1.gps_weidu = packet_in.gps_weidu;
        packet1.gps_haiba = packet_in.gps_haiba;
        packet1.gps_bei = packet_in.gps_bei;
        packet1.gps_dong = packet_in.gps_dong;
        packet1.gps_di = packet_in.gps_di;
        packet1.gps_weixing = packet_in.gps_weixing;
        packet1.gps_dingwei = packet_in.gps_dingwei;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_SIA_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_SIA_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_sia_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_sia_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_sia_pack(system_id, component_id, &msg , packet1.timestamp , packet1.gps_jingdu , packet1.gps_weidu , packet1.gps_haiba , packet1.gps_bei , packet1.gps_dong , packet1.gps_di , packet1.gps_weixing , packet1.gps_dingwei );
	mavlink_msg_gps_sia_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_sia_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.gps_jingdu , packet1.gps_weidu , packet1.gps_haiba , packet1.gps_bei , packet1.gps_dong , packet1.gps_di , packet1.gps_weixing , packet1.gps_dingwei );
	mavlink_msg_gps_sia_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_sia_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_sia_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.gps_jingdu , packet1.gps_weidu , packet1.gps_haiba , packet1.gps_bei , packet1.gps_dong , packet1.gps_di , packet1.gps_weixing , packet1.gps_dingwei );
	mavlink_msg_gps_sia_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_buckets_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_buckets_position_t packet_in = {
		123.0,179.0,235.0,291.0,347.0,403.0,459.0,515.0,571.0,627.0,683.0,739.0,795.0,851.0,907.0,963.0,1019.0,1075.0,1131.0,1187.0,1137.0,1165.0,1193.0,1221.0,1249.0,1277.0,1305.0,1333.0,1361.0,1389.0
    };
	mavlink_mission_buckets_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.bucketA_lat_curr = packet_in.bucketA_lat_curr;
        packet1.bucketA_lon_curr = packet_in.bucketA_lon_curr;
        packet1.bucketA_lat_next = packet_in.bucketA_lat_next;
        packet1.bucketA_lon_next = packet_in.bucketA_lon_next;
        packet1.bucketB_lat_curr = packet_in.bucketB_lat_curr;
        packet1.bucketB_lon_curr = packet_in.bucketB_lon_curr;
        packet1.bucketB_lat_next = packet_in.bucketB_lat_next;
        packet1.bucketB_lon_next = packet_in.bucketB_lon_next;
        packet1.bucketC_lat_curr = packet_in.bucketC_lat_curr;
        packet1.bucketC_lon_curr = packet_in.bucketC_lon_curr;
        packet1.bucketC_lat_next = packet_in.bucketC_lat_next;
        packet1.bucketC_lon_next = packet_in.bucketC_lon_next;
        packet1.bucketD_lat_curr = packet_in.bucketD_lat_curr;
        packet1.bucketD_lon_curr = packet_in.bucketD_lon_curr;
        packet1.bucketD_lat_next = packet_in.bucketD_lat_next;
        packet1.bucketD_lon_next = packet_in.bucketD_lon_next;
        packet1.copter_lat_home = packet_in.copter_lat_home;
        packet1.copter_lon_home = packet_in.copter_lon_home;
        packet1.copter_lat_guide = packet_in.copter_lat_guide;
        packet1.copter_lon_guide = packet_in.copter_lon_guide;
        packet1.bucketA_alt_curr = packet_in.bucketA_alt_curr;
        packet1.bucketA_alt_next = packet_in.bucketA_alt_next;
        packet1.bucketB_alt_curr = packet_in.bucketB_alt_curr;
        packet1.bucketB_alt_next = packet_in.bucketB_alt_next;
        packet1.bucketC_alt_curr = packet_in.bucketC_alt_curr;
        packet1.bucketC_alt_next = packet_in.bucketC_alt_next;
        packet1.bucketD_alt_curr = packet_in.bucketD_alt_curr;
        packet1.bucketD_alt_next = packet_in.bucketD_alt_next;
        packet1.copter_alt_home = packet_in.copter_alt_home;
        packet1.copter_alt_guide = packet_in.copter_alt_guide;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_buckets_position_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_buckets_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_buckets_position_pack(system_id, component_id, &msg , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.copter_lat_home , packet1.copter_lon_home , packet1.copter_alt_home , packet1.copter_lat_guide , packet1.copter_lon_guide , packet1.copter_alt_guide );
	mavlink_msg_mission_buckets_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_buckets_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.copter_lat_home , packet1.copter_lon_home , packet1.copter_alt_home , packet1.copter_lat_guide , packet1.copter_lon_guide , packet1.copter_alt_guide );
	mavlink_msg_mission_buckets_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mission_buckets_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_buckets_position_send(MAVLINK_COMM_1 , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.copter_lat_home , packet1.copter_lon_home , packet1.copter_alt_home , packet1.copter_lat_guide , packet1.copter_lon_guide , packet1.copter_alt_guide );
	mavlink_msg_mission_buckets_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_pos_poll(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_POS_POLL >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_pos_poll_t packet_in = {
		5
    };
	mavlink_mission_pos_poll_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.poll_buckets = packet_in.poll_buckets;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_POS_POLL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_POS_POLL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_pos_poll_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_pos_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_pos_poll_pack(system_id, component_id, &msg , packet1.poll_buckets );
	mavlink_msg_mission_pos_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_pos_poll_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.poll_buckets );
	mavlink_msg_mission_pos_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mission_pos_poll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_pos_poll_send(MAVLINK_COMM_1 , packet1.poll_buckets );
	mavlink_msg_mission_pos_poll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_mission_start_stop(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MISSION_START_STOP >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_mission_start_stop_t packet_in = {
		5,72,139
    };
	mavlink_mission_start_stop_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.start_flag = packet_in.start_flag;
        packet1.main_commander = packet_in.main_commander;
        packet1.sub_commander = packet_in.sub_commander;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_start_stop_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_mission_start_stop_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_start_stop_pack(system_id, component_id, &msg , packet1.start_flag , packet1.main_commander , packet1.sub_commander );
	mavlink_msg_mission_start_stop_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_start_stop_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.start_flag , packet1.main_commander , packet1.sub_commander );
	mavlink_msg_mission_start_stop_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_mission_start_stop_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_mission_start_stop_send(MAVLINK_COMM_1 , packet1.start_flag , packet1.main_commander , packet1.sub_commander );
	mavlink_msg_mission_start_stop_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_COMMAND >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_command_t packet_in = {
		5
    };
	mavlink_vision_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vision_cmd = packet_in.vision_cmd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_command_pack(system_id, component_id, &msg , packet1.vision_cmd );
	mavlink_msg_vision_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vision_cmd );
	mavlink_msg_vision_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_command_send(MAVLINK_COMM_1 , packet1.vision_cmd );
	mavlink_msg_vision_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_circle_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_CIRCLE_ALL >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_circle_all_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,37,104,171,238,49,116,183,250,61,128,195
    };
	mavlink_vision_circle_all_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.circle_0_x = packet_in.circle_0_x;
        packet1.circle_0_y = packet_in.circle_0_y;
        packet1.circle_0_z = packet_in.circle_0_z;
        packet1.circle_1_x = packet_in.circle_1_x;
        packet1.circle_1_y = packet_in.circle_1_y;
        packet1.circle_1_z = packet_in.circle_1_z;
        packet1.circle_2_x = packet_in.circle_2_x;
        packet1.circle_2_y = packet_in.circle_2_y;
        packet1.circle_2_z = packet_in.circle_2_z;
        packet1.circle_3_x = packet_in.circle_3_x;
        packet1.circle_3_y = packet_in.circle_3_y;
        packet1.circle_3_z = packet_in.circle_3_z;
        packet1.circle_4_x = packet_in.circle_4_x;
        packet1.circle_4_y = packet_in.circle_4_y;
        packet1.circle_4_z = packet_in.circle_4_z;
        packet1.circle_5_x = packet_in.circle_5_x;
        packet1.circle_5_y = packet_in.circle_5_y;
        packet1.circle_5_z = packet_in.circle_5_z;
        packet1.circle_6_x = packet_in.circle_6_x;
        packet1.circle_6_y = packet_in.circle_6_y;
        packet1.circle_6_z = packet_in.circle_6_z;
        packet1.circle_7_x = packet_in.circle_7_x;
        packet1.circle_7_y = packet_in.circle_7_y;
        packet1.circle_7_z = packet_in.circle_7_z;
        packet1.circle_sum = packet_in.circle_sum;
        packet1.marker_sum = packet_in.marker_sum;
        packet1.number_mode = packet_in.number_mode;
        packet1.circle_0_number = packet_in.circle_0_number;
        packet1.circle_1_number = packet_in.circle_1_number;
        packet1.circle_2_number = packet_in.circle_2_number;
        packet1.circle_3_number = packet_in.circle_3_number;
        packet1.circle_4_number = packet_in.circle_4_number;
        packet1.circle_5_number = packet_in.circle_5_number;
        packet1.circle_6_number = packet_in.circle_6_number;
        packet1.circle_7_number = packet_in.circle_7_number;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_circle_all_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_circle_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_circle_all_pack(system_id, component_id, &msg , packet1.circle_sum , packet1.marker_sum , packet1.number_mode , packet1.circle_0_number , packet1.circle_0_x , packet1.circle_0_y , packet1.circle_0_z , packet1.circle_1_number , packet1.circle_1_x , packet1.circle_1_y , packet1.circle_1_z , packet1.circle_2_number , packet1.circle_2_x , packet1.circle_2_y , packet1.circle_2_z , packet1.circle_3_number , packet1.circle_3_x , packet1.circle_3_y , packet1.circle_3_z , packet1.circle_4_number , packet1.circle_4_x , packet1.circle_4_y , packet1.circle_4_z , packet1.circle_5_number , packet1.circle_5_x , packet1.circle_5_y , packet1.circle_5_z , packet1.circle_6_number , packet1.circle_6_x , packet1.circle_6_y , packet1.circle_6_z , packet1.circle_7_number , packet1.circle_7_x , packet1.circle_7_y , packet1.circle_7_z );
	mavlink_msg_vision_circle_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_circle_all_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.circle_sum , packet1.marker_sum , packet1.number_mode , packet1.circle_0_number , packet1.circle_0_x , packet1.circle_0_y , packet1.circle_0_z , packet1.circle_1_number , packet1.circle_1_x , packet1.circle_1_y , packet1.circle_1_z , packet1.circle_2_number , packet1.circle_2_x , packet1.circle_2_y , packet1.circle_2_z , packet1.circle_3_number , packet1.circle_3_x , packet1.circle_3_y , packet1.circle_3_z , packet1.circle_4_number , packet1.circle_4_x , packet1.circle_4_y , packet1.circle_4_z , packet1.circle_5_number , packet1.circle_5_x , packet1.circle_5_y , packet1.circle_5_z , packet1.circle_6_number , packet1.circle_6_x , packet1.circle_6_y , packet1.circle_6_z , packet1.circle_7_number , packet1.circle_7_x , packet1.circle_7_y , packet1.circle_7_z );
	mavlink_msg_vision_circle_all_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_circle_all_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_circle_all_send(MAVLINK_COMM_1 , packet1.circle_sum , packet1.marker_sum , packet1.number_mode , packet1.circle_0_number , packet1.circle_0_x , packet1.circle_0_y , packet1.circle_0_z , packet1.circle_1_number , packet1.circle_1_x , packet1.circle_1_y , packet1.circle_1_z , packet1.circle_2_number , packet1.circle_2_x , packet1.circle_2_y , packet1.circle_2_z , packet1.circle_3_number , packet1.circle_3_x , packet1.circle_3_y , packet1.circle_3_z , packet1.circle_4_number , packet1.circle_4_x , packet1.circle_4_y , packet1.circle_4_z , packet1.circle_5_number , packet1.circle_5_x , packet1.circle_5_y , packet1.circle_5_z , packet1.circle_6_number , packet1.circle_6_x , packet1.circle_6_y , packet1.circle_6_z , packet1.circle_7_number , packet1.circle_7_x , packet1.circle_7_y , packet1.circle_7_z );
	mavlink_msg_vision_circle_all_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hc_buckets_poll(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HC_BUCKETS_POLL >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hc_buckets_poll_t packet_in = {
		123.0,179.0,235.0,291.0,347.0,403.0,459.0,515.0,571.0,627.0,683.0,739.0,795.0,851.0,907.0,963.0,1019.0,1075.0,1131.0,1187.0,1137.0,1165.0,1193.0,1221.0,1249.0,1277.0,1305.0,1333.0,1361.0,1389.0
    };
	mavlink_hc_buckets_poll_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.bucketA_lat_curr = packet_in.bucketA_lat_curr;
        packet1.bucketA_lon_curr = packet_in.bucketA_lon_curr;
        packet1.bucketA_lat_next = packet_in.bucketA_lat_next;
        packet1.bucketA_lon_next = packet_in.bucketA_lon_next;
        packet1.bucketB_lat_curr = packet_in.bucketB_lat_curr;
        packet1.bucketB_lon_curr = packet_in.bucketB_lon_curr;
        packet1.bucketB_lat_next = packet_in.bucketB_lat_next;
        packet1.bucketB_lon_next = packet_in.bucketB_lon_next;
        packet1.bucketC_lat_curr = packet_in.bucketC_lat_curr;
        packet1.bucketC_lon_curr = packet_in.bucketC_lon_curr;
        packet1.bucketC_lat_next = packet_in.bucketC_lat_next;
        packet1.bucketC_lon_next = packet_in.bucketC_lon_next;
        packet1.bucketD_lat_curr = packet_in.bucketD_lat_curr;
        packet1.bucketD_lon_curr = packet_in.bucketD_lon_curr;
        packet1.bucketD_lat_next = packet_in.bucketD_lat_next;
        packet1.bucketD_lon_next = packet_in.bucketD_lon_next;
        packet1.lat_home = packet_in.lat_home;
        packet1.lon_home = packet_in.lon_home;
        packet1.lat_guide = packet_in.lat_guide;
        packet1.lon_guide = packet_in.lon_guide;
        packet1.bucketA_alt_curr = packet_in.bucketA_alt_curr;
        packet1.bucketA_alt_next = packet_in.bucketA_alt_next;
        packet1.bucketB_alt_curr = packet_in.bucketB_alt_curr;
        packet1.bucketB_alt_next = packet_in.bucketB_alt_next;
        packet1.bucketC_alt_curr = packet_in.bucketC_alt_curr;
        packet1.bucketC_alt_next = packet_in.bucketC_alt_next;
        packet1.bucketD_alt_curr = packet_in.bucketD_alt_curr;
        packet1.bucketD_alt_next = packet_in.bucketD_alt_next;
        packet1.alt_home = packet_in.alt_home;
        packet1.alt_guide = packet_in.alt_guide;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HC_BUCKETS_POLL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HC_BUCKETS_POLL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_buckets_poll_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_hc_buckets_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_buckets_poll_pack(system_id, component_id, &msg , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.lat_home , packet1.lon_home , packet1.alt_home , packet1.lat_guide , packet1.lon_guide , packet1.alt_guide );
	mavlink_msg_hc_buckets_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_buckets_poll_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.lat_home , packet1.lon_home , packet1.alt_home , packet1.lat_guide , packet1.lon_guide , packet1.alt_guide );
	mavlink_msg_hc_buckets_poll_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_hc_buckets_poll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_buckets_poll_send(MAVLINK_COMM_1 , packet1.bucketA_lat_curr , packet1.bucketA_lon_curr , packet1.bucketA_alt_curr , packet1.bucketA_lat_next , packet1.bucketA_lon_next , packet1.bucketA_alt_next , packet1.bucketB_lat_curr , packet1.bucketB_lon_curr , packet1.bucketB_alt_curr , packet1.bucketB_lat_next , packet1.bucketB_lon_next , packet1.bucketB_alt_next , packet1.bucketC_lat_curr , packet1.bucketC_lon_curr , packet1.bucketC_alt_curr , packet1.bucketC_lat_next , packet1.bucketC_lon_next , packet1.bucketC_alt_next , packet1.bucketD_lat_curr , packet1.bucketD_lon_curr , packet1.bucketD_alt_curr , packet1.bucketD_lat_next , packet1.bucketD_lon_next , packet1.bucketD_alt_next , packet1.lat_home , packet1.lon_home , packet1.alt_home , packet1.lat_guide , packet1.lon_guide , packet1.alt_guide );
	mavlink_msg_hc_buckets_poll_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_hc_state_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HC_STATE_STATUS >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_hc_state_status_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,101,168,235,46,113,180,247
    };
	mavlink_hc_state_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pos_sp_x = packet_in.pos_sp_x;
        packet1.pos_sp_y = packet_in.pos_sp_y;
        packet1.pos_sp_z = packet_in.pos_sp_z;
        packet1.vel_sp_x = packet_in.vel_sp_x;
        packet1.vel_sp_y = packet_in.vel_sp_y;
        packet1.vel_sp_z = packet_in.vel_sp_z;
        packet1.buckets_vaild = packet_in.buckets_vaild;
        packet1.cmd_recevied = packet_in.cmd_recevied;
        packet1.poll_recevied = packet_in.poll_recevied;
        packet1.main_state = packet_in.main_state;
        packet1.sec_state = packet_in.sec_state;
        packet1.pos_en = packet_in.pos_en;
        packet1.vel_en = packet_in.vel_en;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_state_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_hc_state_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_state_status_pack(system_id, component_id, &msg , packet1.timestamp , packet1.buckets_vaild , packet1.cmd_recevied , packet1.poll_recevied , packet1.main_state , packet1.sec_state , packet1.pos_sp_x , packet1.pos_sp_y , packet1.pos_sp_z , packet1.vel_sp_x , packet1.vel_sp_y , packet1.vel_sp_z , packet1.pos_en , packet1.vel_en );
	mavlink_msg_hc_state_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_state_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.buckets_vaild , packet1.cmd_recevied , packet1.poll_recevied , packet1.main_state , packet1.sec_state , packet1.pos_sp_x , packet1.pos_sp_y , packet1.pos_sp_z , packet1.vel_sp_x , packet1.vel_sp_y , packet1.vel_sp_z , packet1.pos_en , packet1.vel_en );
	mavlink_msg_hc_state_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_hc_state_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_hc_state_status_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.buckets_vaild , packet1.cmd_recevied , packet1.poll_recevied , packet1.main_state , packet1.sec_state , packet1.pos_sp_x , packet1.pos_sp_y , packet1.pos_sp_z , packet1.vel_sp_x , packet1.vel_sp_y , packet1.vel_sp_z , packet1.pos_en , packet1.vel_en );
	mavlink_msg_hc_state_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fixed_target_position_g2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FIXED_TARGET_POSITION_G2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_fixed_target_position_g2p_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
	mavlink_fixed_target_position_g2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.home_x = packet_in.home_x;
        packet1.home_y = packet_in.home_y;
        packet1.home_z = packet_in.home_z;
        packet1.observe_x = packet_in.observe_x;
        packet1.observe_y = packet_in.observe_y;
        packet1.observe_z = packet_in.observe_z;
        packet1.spray_left_x = packet_in.spray_left_x;
        packet1.spray_left_y = packet_in.spray_left_y;
        packet1.spray_left_z = packet_in.spray_left_z;
        packet1.spray_right_x = packet_in.spray_right_x;
        packet1.spray_right_y = packet_in.spray_right_y;
        packet1.spray_right_z = packet_in.spray_right_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FIXED_TARGET_POSITION_G2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FIXED_TARGET_POSITION_G2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_g2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_fixed_target_position_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_g2p_pack(system_id, component_id, &msg , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_g2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_fixed_target_position_g2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_g2p_send(MAVLINK_COMM_1 , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_g2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fixed_target_position_p2m(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_fixed_target_position_p2m_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0
    };
	mavlink_fixed_target_position_p2m_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.home_x = packet_in.home_x;
        packet1.home_y = packet_in.home_y;
        packet1.home_z = packet_in.home_z;
        packet1.observe_x = packet_in.observe_x;
        packet1.observe_y = packet_in.observe_y;
        packet1.observe_z = packet_in.observe_z;
        packet1.spray_left_x = packet_in.spray_left_x;
        packet1.spray_left_y = packet_in.spray_left_y;
        packet1.spray_left_z = packet_in.spray_left_z;
        packet1.spray_right_x = packet_in.spray_right_x;
        packet1.spray_right_y = packet_in.spray_right_y;
        packet1.spray_right_z = packet_in.spray_right_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_p2m_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_fixed_target_position_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_p2m_pack(system_id, component_id, &msg , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_p2m_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_fixed_target_position_p2m_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_position_p2m_send(MAVLINK_COMM_1 , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_position_p2m_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fixed_target_return_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FIXED_TARGET_RETURN_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_fixed_target_return_m2p_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0
    };
	mavlink_fixed_target_return_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.home_x = packet_in.home_x;
        packet1.home_y = packet_in.home_y;
        packet1.home_z = packet_in.home_z;
        packet1.observe_x = packet_in.observe_x;
        packet1.observe_y = packet_in.observe_y;
        packet1.observe_z = packet_in.observe_z;
        packet1.spray_left_x = packet_in.spray_left_x;
        packet1.spray_left_y = packet_in.spray_left_y;
        packet1.spray_left_z = packet_in.spray_left_z;
        packet1.spray_right_x = packet_in.spray_right_x;
        packet1.spray_right_y = packet_in.spray_right_y;
        packet1.spray_right_z = packet_in.spray_right_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FIXED_TARGET_RETURN_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FIXED_TARGET_RETURN_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_fixed_target_return_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_fixed_target_return_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_fixed_target_return_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FIXED_TARGET_RETURN_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_fixed_target_return_p2g_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0
    };
	mavlink_fixed_target_return_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.home_x = packet_in.home_x;
        packet1.home_y = packet_in.home_y;
        packet1.home_z = packet_in.home_z;
        packet1.observe_x = packet_in.observe_x;
        packet1.observe_y = packet_in.observe_y;
        packet1.observe_z = packet_in.observe_z;
        packet1.spray_left_x = packet_in.spray_left_x;
        packet1.spray_left_y = packet_in.spray_left_y;
        packet1.spray_left_z = packet_in.spray_left_z;
        packet1.spray_right_x = packet_in.spray_right_x;
        packet1.spray_right_y = packet_in.spray_right_y;
        packet1.spray_right_z = packet_in.spray_right_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FIXED_TARGET_RETURN_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FIXED_TARGET_RETURN_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_fixed_target_return_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_fixed_target_return_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_fixed_target_return_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.home_x , packet1.home_y , packet1.home_z , packet1.observe_x , packet1.observe_y , packet1.observe_z , packet1.spray_left_x , packet1.spray_left_y , packet1.spray_left_z , packet1.spray_right_x , packet1.spray_right_y , packet1.spray_right_z );
	mavlink_msg_fixed_target_return_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_yaw_sp_calculated_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_YAW_SP_CALCULATED_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_yaw_sp_calculated_m2p_t packet_in = {
		93372036854775807ULL,73.0
    };
	mavlink_yaw_sp_calculated_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.yaw_sp = packet_in.yaw_sp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_YAW_SP_CALCULATED_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_YAW_SP_CALCULATED_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_yaw_sp_calculated_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_yaw_sp_calculated_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_yaw_sp_calculated_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_YAW_SP_CALCULATED_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_yaw_sp_calculated_p2g_t packet_in = {
		93372036854775807ULL,73.0
    };
	mavlink_yaw_sp_calculated_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.yaw_sp = packet_in.yaw_sp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_YAW_SP_CALCULATED_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_YAW_SP_CALCULATED_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_yaw_sp_calculated_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_yaw_sp_calculated_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_yaw_sp_calculated_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.yaw_sp );
	mavlink_msg_yaw_sp_calculated_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_task_status_change_g2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_task_status_change_g2p_t packet_in = {
		17.0,17,84
    };
	mavlink_task_status_change_g2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spray_duration = packet_in.spray_duration;
        packet1.task_status = packet_in.task_status;
        packet1.loop_value = packet_in.loop_value;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_g2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_task_status_change_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_g2p_pack(system_id, component_id, &msg , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_g2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_g2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_task_status_change_g2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_g2p_send(MAVLINK_COMM_1 , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_g2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_task_status_change_p2m(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATUS_CHANGE_P2M >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_task_status_change_p2m_t packet_in = {
		17.0,17,84
    };
	mavlink_task_status_change_p2m_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.spray_duration = packet_in.spray_duration;
        packet1.task_status = packet_in.task_status;
        packet1.loop_value = packet_in.loop_value;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATUS_CHANGE_P2M_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATUS_CHANGE_P2M_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_p2m_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_task_status_change_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_p2m_pack(system_id, component_id, &msg , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_p2m_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_p2m_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_task_status_change_p2m_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_change_p2m_send(MAVLINK_COMM_1 , packet1.spray_duration , packet1.task_status , packet1.loop_value );
	mavlink_msg_task_status_change_p2m_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_task_status_monitor_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATUS_MONITOR_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_task_status_monitor_m2p_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,77,144
    };
	mavlink_task_status_monitor_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.spray_duration = packet_in.spray_duration;
        packet1.target_x = packet_in.target_x;
        packet1.target_y = packet_in.target_y;
        packet1.target_z = packet_in.target_z;
        packet1.task_status = packet_in.task_status;
        packet1.loop_value = packet_in.loop_value;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATUS_MONITOR_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATUS_MONITOR_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_task_status_monitor_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_task_status_monitor_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_task_status_monitor_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_task_status_monitor_p2g_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,77,144
    };
	mavlink_task_status_monitor_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.spray_duration = packet_in.spray_duration;
        packet1.target_x = packet_in.target_x;
        packet1.target_y = packet_in.target_y;
        packet1.target_z = packet_in.target_z;
        packet1.task_status = packet_in.task_status;
        packet1.loop_value = packet_in.loop_value;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_task_status_monitor_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_task_status_monitor_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_task_status_monitor_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.spray_duration , packet1.task_status , packet1.loop_value , packet1.target_x , packet1.target_y , packet1.target_z );
	mavlink_msg_task_status_monitor_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_num_scan_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_num_scan_m2p_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,65,132
    };
	mavlink_vision_num_scan_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.board_x = packet_in.board_x;
        packet1.board_y = packet_in.board_y;
        packet1.board_z = packet_in.board_z;
        packet1.board_num = packet_in.board_num;
        packet1.board_valid = packet_in.board_valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_num_scan_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_num_scan_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_num_scan_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_NUM_SCAN_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_num_scan_p2g_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,65,132
    };
	mavlink_vision_num_scan_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.board_x = packet_in.board_x;
        packet1.board_y = packet_in.board_y;
        packet1.board_z = packet_in.board_z;
        packet1.board_num = packet_in.board_num;
        packet1.board_valid = packet_in.board_valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_NUM_SCAN_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_NUM_SCAN_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_num_scan_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_num_scan_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_num_scan_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.board_num , packet1.board_x , packet1.board_y , packet1.board_z , packet1.board_valid );
	mavlink_msg_vision_num_scan_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_one_num_get_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_one_num_get_m2p_t packet_in = {
		93372036854775807ULL,29,96
    };
	mavlink_vision_one_num_get_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.loop_value = packet_in.loop_value;
        packet1.num = packet_in.num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_one_num_get_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_one_num_get_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vision_one_num_get_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vision_one_num_get_p2g_t packet_in = {
		93372036854775807ULL,29,96
    };
	mavlink_vision_one_num_get_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.loop_value = packet_in.loop_value;
        packet1.num = packet_in.num;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vision_one_num_get_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vision_one_num_get_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vision_one_num_get_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.loop_value , packet1.num );
	mavlink_msg_vision_one_num_get_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obstacle_position_m2p(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OBSTACLE_POSITION_M2P >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obstacle_position_m2p_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,65
    };
	mavlink_obstacle_position_m2p_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.obstacle_x = packet_in.obstacle_x;
        packet1.obstacle_y = packet_in.obstacle_y;
        packet1.obstacle_z = packet_in.obstacle_z;
        packet1.obstacle_valid = packet_in.obstacle_valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OBSTACLE_POSITION_M2P_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OBSTACLE_POSITION_M2P_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_m2p_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obstacle_position_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_m2p_pack(system_id, component_id, &msg , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_m2p_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_m2p_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obstacle_position_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_m2p_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_m2p_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_obstacle_position_p2g(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_obstacle_position_p2g_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,65
    };
	mavlink_obstacle_position_p2g_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.obstacle_x = packet_in.obstacle_x;
        packet1.obstacle_y = packet_in.obstacle_y;
        packet1.obstacle_z = packet_in.obstacle_z;
        packet1.obstacle_valid = packet_in.obstacle_valid;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_OBSTACLE_POSITION_P2G_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_p2g_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_obstacle_position_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_p2g_pack(system_id, component_id, &msg , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_p2g_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_p2g_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_obstacle_position_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_obstacle_position_p2g_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.obstacle_x , packet1.obstacle_y , packet1.obstacle_z , packet1.obstacle_valid );
	mavlink_msg_obstacle_position_p2g_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pixhawk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_mavros_test_msg(system_id, component_id, last_msg);
	mavlink_test_gps_sia(system_id, component_id, last_msg);
	mavlink_test_mission_buckets_position(system_id, component_id, last_msg);
	mavlink_test_mission_pos_poll(system_id, component_id, last_msg);
	mavlink_test_mission_start_stop(system_id, component_id, last_msg);
	mavlink_test_vision_command(system_id, component_id, last_msg);
	mavlink_test_vision_circle_all(system_id, component_id, last_msg);
	mavlink_test_hc_buckets_poll(system_id, component_id, last_msg);
	mavlink_test_hc_state_status(system_id, component_id, last_msg);
	mavlink_test_fixed_target_position_g2p(system_id, component_id, last_msg);
	mavlink_test_fixed_target_position_p2m(system_id, component_id, last_msg);
	mavlink_test_fixed_target_return_m2p(system_id, component_id, last_msg);
	mavlink_test_fixed_target_return_p2g(system_id, component_id, last_msg);
	mavlink_test_yaw_sp_calculated_m2p(system_id, component_id, last_msg);
	mavlink_test_yaw_sp_calculated_p2g(system_id, component_id, last_msg);
	mavlink_test_task_status_change_g2p(system_id, component_id, last_msg);
	mavlink_test_task_status_change_p2m(system_id, component_id, last_msg);
	mavlink_test_task_status_monitor_m2p(system_id, component_id, last_msg);
	mavlink_test_task_status_monitor_p2g(system_id, component_id, last_msg);
	mavlink_test_vision_num_scan_m2p(system_id, component_id, last_msg);
	mavlink_test_vision_num_scan_p2g(system_id, component_id, last_msg);
	mavlink_test_vision_one_num_get_m2p(system_id, component_id, last_msg);
	mavlink_test_vision_one_num_get_p2g(system_id, component_id, last_msg);
	mavlink_test_obstacle_position_m2p(system_id, component_id, last_msg);
	mavlink_test_obstacle_position_p2g(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PIXHAWK_TESTSUITE_H
