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


static void mavlink_test_set_cam_shutter(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_cam_shutter_t packet_in = {
		17.0,
	}17443,
	}17547,
	}29,
	}96,
	}163,
	};
	mavlink_set_cam_shutter_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gain = packet_in.gain;
        	packet1.interval = packet_in.interval;
        	packet1.exposure = packet_in.exposure;
        	packet1.cam_no = packet_in.cam_no;
        	packet1.cam_mode = packet_in.cam_mode;
        	packet1.trigger_pin = packet_in.trigger_pin;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_cam_shutter_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_cam_shutter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_cam_shutter_pack(system_id, component_id, &msg , packet1.cam_no , packet1.cam_mode , packet1.trigger_pin , packet1.interval , packet1.exposure , packet1.gain );
	mavlink_msg_set_cam_shutter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_cam_shutter_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cam_no , packet1.cam_mode , packet1.trigger_pin , packet1.interval , packet1.exposure , packet1.gain );
	mavlink_msg_set_cam_shutter_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_cam_shutter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_cam_shutter_send(MAVLINK_COMM_1 , packet1.cam_no , packet1.cam_mode , packet1.trigger_pin , packet1.interval , packet1.exposure , packet1.gain );
	mavlink_msg_set_cam_shutter_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_image_triggered(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_image_triggered_t packet_in = {
		93372036854775807ULL,
	}963497880,
	}101.0,
	}129.0,
	}157.0,
	}185.0,
	}213.0,
	}241.0,
	}269.0,
	}297.0,
	}325.0,
	}353.0,
	};
	mavlink_image_triggered_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timestamp = packet_in.timestamp;
        	packet1.seq = packet_in.seq;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.local_z = packet_in.local_z;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.ground_x = packet_in.ground_x;
        	packet1.ground_y = packet_in.ground_y;
        	packet1.ground_z = packet_in.ground_z;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_image_triggered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_pack(system_id, component_id, &msg , packet1.timestamp , packet1.seq , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_triggered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.seq , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_triggered_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_image_triggered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_triggered_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.seq , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_triggered_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_image_trigger_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_image_trigger_control_t packet_in = {
		5,
	};
	mavlink_image_trigger_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.enable = packet_in.enable;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_trigger_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_image_trigger_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_trigger_control_pack(system_id, component_id, &msg , packet1.enable );
	mavlink_msg_image_trigger_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_trigger_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.enable );
	mavlink_msg_image_trigger_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_image_trigger_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_trigger_control_send(MAVLINK_COMM_1 , packet1.enable );
	mavlink_msg_image_trigger_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_image_available(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_image_available_t packet_in = {
		93372036854775807ULL,
	}93372036854776311ULL,
	}93372036854776815ULL,
	}963498712,
	}963498920,
	}963499128,
	}963499336,
	}297.0,
	}325.0,
	}353.0,
	}381.0,
	}409.0,
	}437.0,
	}465.0,
	}493.0,
	}521.0,
	}549.0,
	}577.0,
	}21603,
	}21707,
	}21811,
	}147,
	}214,
	};
	mavlink_image_available_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.cam_id = packet_in.cam_id;
        	packet1.timestamp = packet_in.timestamp;
        	packet1.valid_until = packet_in.valid_until;
        	packet1.img_seq = packet_in.img_seq;
        	packet1.img_buf_index = packet_in.img_buf_index;
        	packet1.key = packet_in.key;
        	packet1.exposure = packet_in.exposure;
        	packet1.gain = packet_in.gain;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.local_z = packet_in.local_z;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.ground_x = packet_in.ground_x;
        	packet1.ground_y = packet_in.ground_y;
        	packet1.ground_z = packet_in.ground_z;
        	packet1.width = packet_in.width;
        	packet1.height = packet_in.height;
        	packet1.depth = packet_in.depth;
        	packet1.cam_no = packet_in.cam_no;
        	packet1.channels = packet_in.channels;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_available_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_image_available_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_available_pack(system_id, component_id, &msg , packet1.cam_id , packet1.cam_no , packet1.timestamp , packet1.valid_until , packet1.img_seq , packet1.img_buf_index , packet1.width , packet1.height , packet1.depth , packet1.channels , packet1.key , packet1.exposure , packet1.gain , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_available_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_available_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cam_id , packet1.cam_no , packet1.timestamp , packet1.valid_until , packet1.img_seq , packet1.img_buf_index , packet1.width , packet1.height , packet1.depth , packet1.channels , packet1.key , packet1.exposure , packet1.gain , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_available_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_image_available_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_image_available_send(MAVLINK_COMM_1 , packet1.cam_id , packet1.cam_no , packet1.timestamp , packet1.valid_until , packet1.img_seq , packet1.img_buf_index , packet1.width , packet1.height , packet1.depth , packet1.channels , packet1.key , packet1.exposure , packet1.gain , packet1.roll , packet1.pitch , packet1.yaw , packet1.local_z , packet1.lat , packet1.lon , packet1.alt , packet1.ground_x , packet1.ground_y , packet1.ground_z );
	mavlink_msg_image_available_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_position_control_offset(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_position_control_offset_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}53,
	}120,
	};
	mavlink_set_position_control_offset_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_position_control_offset_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_position_control_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_position_control_offset_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_position_control_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_position_control_offset_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_position_control_offset_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_position_control_offset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_position_control_offset_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_set_position_control_offset_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_position_control_setpoint(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_position_control_setpoint_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}18067,
	};
	mavlink_position_control_setpoint_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.yaw = packet_in.yaw;
        	packet1.id = packet_in.id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_position_control_setpoint_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_position_control_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_position_control_setpoint_pack(system_id, component_id, &msg , packet1.id , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_position_control_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_position_control_setpoint_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_position_control_setpoint_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_position_control_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_position_control_setpoint_send(MAVLINK_COMM_1 , packet1.id , packet1.x , packet1.y , packet1.z , packet1.yaw );
	mavlink_msg_position_control_setpoint_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_marker(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_marker_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}18483,
	};
	mavlink_marker_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.id = packet_in.id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_marker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_pack(system_id, component_id, &msg , packet1.id , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_marker_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_marker_send(MAVLINK_COMM_1 , packet1.id , packet1.x , packet1.y , packet1.z , packet1.roll , packet1.pitch , packet1.yaw );
	mavlink_msg_marker_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_raw_aux(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_raw_aux_t packet_in = {
		963497464,
	}17443,
	}17547,
	}17651,
	}17755,
	}17859,
	}17963,
	};
	mavlink_raw_aux_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.baro = packet_in.baro;
        	packet1.adc1 = packet_in.adc1;
        	packet1.adc2 = packet_in.adc2;
        	packet1.adc3 = packet_in.adc3;
        	packet1.adc4 = packet_in.adc4;
        	packet1.vbat = packet_in.vbat;
        	packet1.temp = packet_in.temp;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_aux_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_raw_aux_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_aux_pack(system_id, component_id, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.vbat , packet1.temp , packet1.baro );
	mavlink_msg_raw_aux_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_aux_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.vbat , packet1.temp , packet1.baro );
	mavlink_msg_raw_aux_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_raw_aux_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_raw_aux_send(MAVLINK_COMM_1 , packet1.adc1 , packet1.adc2 , packet1.adc3 , packet1.adc4 , packet1.vbat , packet1.temp , packet1.baro );
	mavlink_msg_raw_aux_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_watchdog_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_watchdog_heartbeat_t packet_in = {
		17235,
	}17339,
	};
	mavlink_watchdog_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.watchdog_id = packet_in.watchdog_id;
        	packet1.process_count = packet_in.process_count;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_heartbeat_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_watchdog_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_heartbeat_pack(system_id, component_id, &msg , packet1.watchdog_id , packet1.process_count );
	mavlink_msg_watchdog_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.watchdog_id , packet1.process_count );
	mavlink_msg_watchdog_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_watchdog_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_heartbeat_send(MAVLINK_COMM_1 , packet1.watchdog_id , packet1.process_count );
	mavlink_msg_watchdog_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_watchdog_process_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_watchdog_process_info_t packet_in = {
		963497464,
	}17443,
	}17547,
	}"IJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABC",
	}"EFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRST",
	};
	mavlink_watchdog_process_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.timeout = packet_in.timeout;
        	packet1.watchdog_id = packet_in.watchdog_id;
        	packet1.process_id = packet_in.process_id;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*100);
        	mav_array_memcpy(packet1.arguments, packet_in.arguments, sizeof(char)*147);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_info_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_watchdog_process_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_info_pack(system_id, component_id, &msg , packet1.watchdog_id , packet1.process_id , packet1.name , packet1.arguments , packet1.timeout );
	mavlink_msg_watchdog_process_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.watchdog_id , packet1.process_id , packet1.name , packet1.arguments , packet1.timeout );
	mavlink_msg_watchdog_process_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_watchdog_process_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_info_send(MAVLINK_COMM_1 , packet1.watchdog_id , packet1.process_id , packet1.name , packet1.arguments , packet1.timeout );
	mavlink_msg_watchdog_process_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_watchdog_process_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_watchdog_process_status_t packet_in = {
		963497464,
	}17443,
	}17547,
	}17651,
	}163,
	}230,
	};
	mavlink_watchdog_process_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pid = packet_in.pid;
        	packet1.watchdog_id = packet_in.watchdog_id;
        	packet1.process_id = packet_in.process_id;
        	packet1.crashes = packet_in.crashes;
        	packet1.state = packet_in.state;
        	packet1.muted = packet_in.muted;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_watchdog_process_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_status_pack(system_id, component_id, &msg , packet1.watchdog_id , packet1.process_id , packet1.state , packet1.muted , packet1.pid , packet1.crashes );
	mavlink_msg_watchdog_process_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.watchdog_id , packet1.process_id , packet1.state , packet1.muted , packet1.pid , packet1.crashes );
	mavlink_msg_watchdog_process_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_watchdog_process_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_process_status_send(MAVLINK_COMM_1 , packet1.watchdog_id , packet1.process_id , packet1.state , packet1.muted , packet1.pid , packet1.crashes );
	mavlink_msg_watchdog_process_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_watchdog_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_watchdog_command_t packet_in = {
		17235,
	}17339,
	}17,
	}84,
	};
	mavlink_watchdog_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.watchdog_id = packet_in.watchdog_id;
        	packet1.process_id = packet_in.process_id;
        	packet1.target_system_id = packet_in.target_system_id;
        	packet1.command_id = packet_in.command_id;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_command_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_watchdog_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_command_pack(system_id, component_id, &msg , packet1.target_system_id , packet1.watchdog_id , packet1.process_id , packet1.command_id );
	mavlink_msg_watchdog_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system_id , packet1.watchdog_id , packet1.process_id , packet1.command_id );
	mavlink_msg_watchdog_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_watchdog_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_watchdog_command_send(MAVLINK_COMM_1 , packet1.target_system_id , packet1.watchdog_id , packet1.process_id , packet1.command_id );
	mavlink_msg_watchdog_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pattern_detected(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pattern_detected_t packet_in = {
		17.0,
	}17,
	}"FGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZ",
	}128,
	};
	mavlink_pattern_detected_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.confidence = packet_in.confidence;
        	packet1.type = packet_in.type;
        	packet1.detected = packet_in.detected;
        
        	mav_array_memcpy(packet1.file, packet_in.file, sizeof(char)*100);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pattern_detected_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pattern_detected_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pattern_detected_pack(system_id, component_id, &msg , packet1.type , packet1.confidence , packet1.file , packet1.detected );
	mavlink_msg_pattern_detected_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pattern_detected_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.confidence , packet1.file , packet1.detected );
	mavlink_msg_pattern_detected_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pattern_detected_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pattern_detected_send(MAVLINK_COMM_1 , packet1.type , packet1.confidence , packet1.file , packet1.detected );
	mavlink_msg_pattern_detected_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_point_of_interest(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_point_of_interest_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}17859,
	}175,
	}242,
	}53,
	}"RSTUVWXYZABCDEFGHIJKLMNOP",
	};
	mavlink_point_of_interest_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.timeout = packet_in.timeout;
        	packet1.type = packet_in.type;
        	packet1.color = packet_in.color;
        	packet1.coordinate_system = packet_in.coordinate_system;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*26);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_point_of_interest_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_pack(system_id, component_id, &msg , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.x , packet1.y , packet1.z , packet1.name );
	mavlink_msg_point_of_interest_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.x , packet1.y , packet1.z , packet1.name );
	mavlink_msg_point_of_interest_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_point_of_interest_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_send(MAVLINK_COMM_1 , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.x , packet1.y , packet1.z , packet1.name );
	mavlink_msg_point_of_interest_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_point_of_interest_connection(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_point_of_interest_connection_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}129.0,
	}157.0,
	}18483,
	}211,
	}22,
	}89,
	}"DEFGHIJKLMNOPQRSTUVWXYZAB",
	};
	mavlink_point_of_interest_connection_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.xp1 = packet_in.xp1;
        	packet1.yp1 = packet_in.yp1;
        	packet1.zp1 = packet_in.zp1;
        	packet1.xp2 = packet_in.xp2;
        	packet1.yp2 = packet_in.yp2;
        	packet1.zp2 = packet_in.zp2;
        	packet1.timeout = packet_in.timeout;
        	packet1.type = packet_in.type;
        	packet1.color = packet_in.color;
        	packet1.coordinate_system = packet_in.coordinate_system;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*26);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_connection_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_point_of_interest_connection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_connection_pack(system_id, component_id, &msg , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.xp1 , packet1.yp1 , packet1.zp1 , packet1.xp2 , packet1.yp2 , packet1.zp2 , packet1.name );
	mavlink_msg_point_of_interest_connection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_connection_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.xp1 , packet1.yp1 , packet1.zp1 , packet1.xp2 , packet1.yp2 , packet1.zp2 , packet1.name );
	mavlink_msg_point_of_interest_connection_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_point_of_interest_connection_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_point_of_interest_connection_send(MAVLINK_COMM_1 , packet1.type , packet1.color , packet1.coordinate_system , packet1.timeout , packet1.xp1 , packet1.yp1 , packet1.zp1 , packet1.xp2 , packet1.yp2 , packet1.zp2 , packet1.name );
	mavlink_msg_point_of_interest_connection_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_brief_feature(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_brief_feature_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}18067,
	}18171,
	}65,
	}{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163 },
	};
	mavlink_brief_feature_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.response = packet_in.response;
        	packet1.size = packet_in.size;
        	packet1.orientation = packet_in.orientation;
        	packet1.orientation_assignment = packet_in.orientation_assignment;
        
        	mav_array_memcpy(packet1.descriptor, packet_in.descriptor, sizeof(uint8_t)*32);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_brief_feature_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_brief_feature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_brief_feature_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.z , packet1.orientation_assignment , packet1.size , packet1.orientation , packet1.descriptor , packet1.response );
	mavlink_msg_brief_feature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_brief_feature_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.z , packet1.orientation_assignment , packet1.size , packet1.orientation , packet1.descriptor , packet1.response );
	mavlink_msg_brief_feature_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_brief_feature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_brief_feature_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.z , packet1.orientation_assignment , packet1.size , packet1.orientation , packet1.descriptor , packet1.response );
	mavlink_msg_brief_feature_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_control_t packet_in = {
		17.0,
	}45.0,
	}73.0,
	}101.0,
	}53,
	}120,
	}187,
	}254,
	}65,
	};
	mavlink_attitude_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.thrust = packet_in.thrust;
        	packet1.target = packet_in.target;
        	packet1.roll_manual = packet_in.roll_manual;
        	packet1.pitch_manual = packet_in.pitch_manual;
        	packet1.yaw_manual = packet_in.yaw_manual;
        	packet1.thrust_manual = packet_in.thrust_manual;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_control_pack(system_id, component_id, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
	mavlink_msg_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
	mavlink_msg_attitude_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_attitude_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_control_send(MAVLINK_COMM_1 , packet1.target , packet1.roll , packet1.pitch , packet1.yaw , packet1.thrust , packet1.roll_manual , packet1.pitch_manual , packet1.yaw_manual , packet1.thrust_manual );
	mavlink_msg_attitude_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pixhawk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_set_cam_shutter(system_id, component_id, last_msg);
	mavlink_test_image_triggered(system_id, component_id, last_msg);
	mavlink_test_image_trigger_control(system_id, component_id, last_msg);
	mavlink_test_image_available(system_id, component_id, last_msg);
	mavlink_test_set_position_control_offset(system_id, component_id, last_msg);
	mavlink_test_position_control_setpoint(system_id, component_id, last_msg);
	mavlink_test_marker(system_id, component_id, last_msg);
	mavlink_test_raw_aux(system_id, component_id, last_msg);
	mavlink_test_watchdog_heartbeat(system_id, component_id, last_msg);
	mavlink_test_watchdog_process_info(system_id, component_id, last_msg);
	mavlink_test_watchdog_process_status(system_id, component_id, last_msg);
	mavlink_test_watchdog_command(system_id, component_id, last_msg);
	mavlink_test_pattern_detected(system_id, component_id, last_msg);
	mavlink_test_point_of_interest(system_id, component_id, last_msg);
	mavlink_test_point_of_interest_connection(system_id, component_id, last_msg);
	mavlink_test_brief_feature(system_id, component_id, last_msg);
	mavlink_test_attitude_control(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PIXHAWK_TESTSUITE_H
