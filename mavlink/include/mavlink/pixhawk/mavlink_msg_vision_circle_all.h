// MESSAGE VISION_CIRCLE_ALL PACKING

#define MAVLINK_MSG_ID_VISION_CIRCLE_ALL 216

MAVPACKED(
typedef struct __mavlink_vision_circle_all_t {
 float circle_0_x; /*< relative local poosition x*/
 float circle_0_y; /*< relative local poosition y*/
 float circle_0_z; /*< relative local poosition z*/
 float circle_1_x; /*< relative local poosition x*/
 float circle_1_y; /*< relative local poosition y*/
 float circle_1_z; /*< relative local poosition z*/
 float circle_2_x; /*< relative local poosition x*/
 float circle_2_y; /*< relative local poosition y*/
 float circle_2_z; /*< relative local poosition z*/
 float circle_3_x; /*< relative local poosition x*/
 float circle_3_y; /*< relative local poosition y*/
 float circle_3_z; /*< relative local poosition z*/
 float circle_4_x; /*< relative local poosition x*/
 float circle_4_y; /*< relative local poosition y*/
 float circle_4_z; /*< relative local poosition z*/
 float circle_5_x; /*< relative local poosition x*/
 float circle_5_y; /*< relative local poosition y*/
 float circle_5_z; /*< relative local poosition z*/
 float circle_6_x; /*< relative local poosition x*/
 float circle_6_y; /*< relative local poosition y*/
 float circle_6_z; /*< relative local poosition z*/
 float circle_7_x; /*< relative local poosition x*/
 float circle_7_y; /*< relative local poosition y*/
 float circle_7_z; /*< relative local poosition z*/
 uint8_t circle_sum; /*< the number of circles*/
 uint8_t marker_sum; /*< the number of makers*/
 uint8_t number_mode; /*< mode status*/
 uint8_t circle_0_number; /*< first circle's maker number*/
 uint8_t circle_1_number; /*< second circle's maker number*/
 uint8_t circle_2_number; /*< circle's maker number*/
 uint8_t circle_3_number; /*< circle's maker number*/
 uint8_t circle_4_number; /*< circle's maker number*/
 uint8_t circle_5_number; /*< circle's maker number*/
 uint8_t circle_6_number; /*< circle's maker number*/
 uint8_t circle_7_number; /*< circle's maker number*/
}) mavlink_vision_circle_all_t;

#define MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN 107
#define MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN 107
#define MAVLINK_MSG_ID_216_LEN 107
#define MAVLINK_MSG_ID_216_MIN_LEN 107

#define MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC 139
#define MAVLINK_MSG_ID_216_CRC 139



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_CIRCLE_ALL { \
	216, \
	"VISION_CIRCLE_ALL", \
	35, \
	{  { "circle_0_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_circle_all_t, circle_0_x) }, \
         { "circle_0_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_circle_all_t, circle_0_y) }, \
         { "circle_0_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_circle_all_t, circle_0_z) }, \
         { "circle_1_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_circle_all_t, circle_1_x) }, \
         { "circle_1_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_circle_all_t, circle_1_y) }, \
         { "circle_1_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_circle_all_t, circle_1_z) }, \
         { "circle_2_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_circle_all_t, circle_2_x) }, \
         { "circle_2_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_vision_circle_all_t, circle_2_y) }, \
         { "circle_2_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_vision_circle_all_t, circle_2_z) }, \
         { "circle_3_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_vision_circle_all_t, circle_3_x) }, \
         { "circle_3_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vision_circle_all_t, circle_3_y) }, \
         { "circle_3_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_vision_circle_all_t, circle_3_z) }, \
         { "circle_4_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_vision_circle_all_t, circle_4_x) }, \
         { "circle_4_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_vision_circle_all_t, circle_4_y) }, \
         { "circle_4_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_vision_circle_all_t, circle_4_z) }, \
         { "circle_5_x", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_vision_circle_all_t, circle_5_x) }, \
         { "circle_5_y", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_vision_circle_all_t, circle_5_y) }, \
         { "circle_5_z", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_vision_circle_all_t, circle_5_z) }, \
         { "circle_6_x", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_vision_circle_all_t, circle_6_x) }, \
         { "circle_6_y", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_vision_circle_all_t, circle_6_y) }, \
         { "circle_6_z", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_vision_circle_all_t, circle_6_z) }, \
         { "circle_7_x", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_vision_circle_all_t, circle_7_x) }, \
         { "circle_7_y", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_vision_circle_all_t, circle_7_y) }, \
         { "circle_7_z", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_vision_circle_all_t, circle_7_z) }, \
         { "circle_sum", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_vision_circle_all_t, circle_sum) }, \
         { "marker_sum", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_vision_circle_all_t, marker_sum) }, \
         { "number_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 98, offsetof(mavlink_vision_circle_all_t, number_mode) }, \
         { "circle_0_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 99, offsetof(mavlink_vision_circle_all_t, circle_0_number) }, \
         { "circle_1_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 100, offsetof(mavlink_vision_circle_all_t, circle_1_number) }, \
         { "circle_2_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 101, offsetof(mavlink_vision_circle_all_t, circle_2_number) }, \
         { "circle_3_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 102, offsetof(mavlink_vision_circle_all_t, circle_3_number) }, \
         { "circle_4_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 103, offsetof(mavlink_vision_circle_all_t, circle_4_number) }, \
         { "circle_5_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_vision_circle_all_t, circle_5_number) }, \
         { "circle_6_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 105, offsetof(mavlink_vision_circle_all_t, circle_6_number) }, \
         { "circle_7_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 106, offsetof(mavlink_vision_circle_all_t, circle_7_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_CIRCLE_ALL { \
	"VISION_CIRCLE_ALL", \
	35, \
	{  { "circle_0_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_vision_circle_all_t, circle_0_x) }, \
         { "circle_0_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_vision_circle_all_t, circle_0_y) }, \
         { "circle_0_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_circle_all_t, circle_0_z) }, \
         { "circle_1_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_circle_all_t, circle_1_x) }, \
         { "circle_1_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_circle_all_t, circle_1_y) }, \
         { "circle_1_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_vision_circle_all_t, circle_1_z) }, \
         { "circle_2_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_vision_circle_all_t, circle_2_x) }, \
         { "circle_2_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_vision_circle_all_t, circle_2_y) }, \
         { "circle_2_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_vision_circle_all_t, circle_2_z) }, \
         { "circle_3_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_vision_circle_all_t, circle_3_x) }, \
         { "circle_3_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_vision_circle_all_t, circle_3_y) }, \
         { "circle_3_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_vision_circle_all_t, circle_3_z) }, \
         { "circle_4_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_vision_circle_all_t, circle_4_x) }, \
         { "circle_4_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_vision_circle_all_t, circle_4_y) }, \
         { "circle_4_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_vision_circle_all_t, circle_4_z) }, \
         { "circle_5_x", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_vision_circle_all_t, circle_5_x) }, \
         { "circle_5_y", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_vision_circle_all_t, circle_5_y) }, \
         { "circle_5_z", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_vision_circle_all_t, circle_5_z) }, \
         { "circle_6_x", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_vision_circle_all_t, circle_6_x) }, \
         { "circle_6_y", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_vision_circle_all_t, circle_6_y) }, \
         { "circle_6_z", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_vision_circle_all_t, circle_6_z) }, \
         { "circle_7_x", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_vision_circle_all_t, circle_7_x) }, \
         { "circle_7_y", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_vision_circle_all_t, circle_7_y) }, \
         { "circle_7_z", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_vision_circle_all_t, circle_7_z) }, \
         { "circle_sum", NULL, MAVLINK_TYPE_UINT8_T, 0, 96, offsetof(mavlink_vision_circle_all_t, circle_sum) }, \
         { "marker_sum", NULL, MAVLINK_TYPE_UINT8_T, 0, 97, offsetof(mavlink_vision_circle_all_t, marker_sum) }, \
         { "number_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 98, offsetof(mavlink_vision_circle_all_t, number_mode) }, \
         { "circle_0_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 99, offsetof(mavlink_vision_circle_all_t, circle_0_number) }, \
         { "circle_1_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 100, offsetof(mavlink_vision_circle_all_t, circle_1_number) }, \
         { "circle_2_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 101, offsetof(mavlink_vision_circle_all_t, circle_2_number) }, \
         { "circle_3_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 102, offsetof(mavlink_vision_circle_all_t, circle_3_number) }, \
         { "circle_4_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 103, offsetof(mavlink_vision_circle_all_t, circle_4_number) }, \
         { "circle_5_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 104, offsetof(mavlink_vision_circle_all_t, circle_5_number) }, \
         { "circle_6_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 105, offsetof(mavlink_vision_circle_all_t, circle_6_number) }, \
         { "circle_7_number", NULL, MAVLINK_TYPE_UINT8_T, 0, 106, offsetof(mavlink_vision_circle_all_t, circle_7_number) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_circle_all message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param circle_sum the number of circles
 * @param marker_sum the number of makers
 * @param number_mode mode status
 * @param circle_0_number first circle's maker number
 * @param circle_0_x relative local poosition x
 * @param circle_0_y relative local poosition y
 * @param circle_0_z relative local poosition z
 * @param circle_1_number second circle's maker number
 * @param circle_1_x relative local poosition x
 * @param circle_1_y relative local poosition y
 * @param circle_1_z relative local poosition z
 * @param circle_2_number circle's maker number
 * @param circle_2_x relative local poosition x
 * @param circle_2_y relative local poosition y
 * @param circle_2_z relative local poosition z
 * @param circle_3_number circle's maker number
 * @param circle_3_x relative local poosition x
 * @param circle_3_y relative local poosition y
 * @param circle_3_z relative local poosition z
 * @param circle_4_number circle's maker number
 * @param circle_4_x relative local poosition x
 * @param circle_4_y relative local poosition y
 * @param circle_4_z relative local poosition z
 * @param circle_5_number circle's maker number
 * @param circle_5_x relative local poosition x
 * @param circle_5_y relative local poosition y
 * @param circle_5_z relative local poosition z
 * @param circle_6_number circle's maker number
 * @param circle_6_x relative local poosition x
 * @param circle_6_y relative local poosition y
 * @param circle_6_z relative local poosition z
 * @param circle_7_number circle's maker number
 * @param circle_7_x relative local poosition x
 * @param circle_7_y relative local poosition y
 * @param circle_7_z relative local poosition z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_circle_all_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t circle_sum, uint8_t marker_sum, uint8_t number_mode, uint8_t circle_0_number, float circle_0_x, float circle_0_y, float circle_0_z, uint8_t circle_1_number, float circle_1_x, float circle_1_y, float circle_1_z, uint8_t circle_2_number, float circle_2_x, float circle_2_y, float circle_2_z, uint8_t circle_3_number, float circle_3_x, float circle_3_y, float circle_3_z, uint8_t circle_4_number, float circle_4_x, float circle_4_y, float circle_4_z, uint8_t circle_5_number, float circle_5_x, float circle_5_y, float circle_5_z, uint8_t circle_6_number, float circle_6_x, float circle_6_y, float circle_6_z, uint8_t circle_7_number, float circle_7_x, float circle_7_y, float circle_7_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN];
	_mav_put_float(buf, 0, circle_0_x);
	_mav_put_float(buf, 4, circle_0_y);
	_mav_put_float(buf, 8, circle_0_z);
	_mav_put_float(buf, 12, circle_1_x);
	_mav_put_float(buf, 16, circle_1_y);
	_mav_put_float(buf, 20, circle_1_z);
	_mav_put_float(buf, 24, circle_2_x);
	_mav_put_float(buf, 28, circle_2_y);
	_mav_put_float(buf, 32, circle_2_z);
	_mav_put_float(buf, 36, circle_3_x);
	_mav_put_float(buf, 40, circle_3_y);
	_mav_put_float(buf, 44, circle_3_z);
	_mav_put_float(buf, 48, circle_4_x);
	_mav_put_float(buf, 52, circle_4_y);
	_mav_put_float(buf, 56, circle_4_z);
	_mav_put_float(buf, 60, circle_5_x);
	_mav_put_float(buf, 64, circle_5_y);
	_mav_put_float(buf, 68, circle_5_z);
	_mav_put_float(buf, 72, circle_6_x);
	_mav_put_float(buf, 76, circle_6_y);
	_mav_put_float(buf, 80, circle_6_z);
	_mav_put_float(buf, 84, circle_7_x);
	_mav_put_float(buf, 88, circle_7_y);
	_mav_put_float(buf, 92, circle_7_z);
	_mav_put_uint8_t(buf, 96, circle_sum);
	_mav_put_uint8_t(buf, 97, marker_sum);
	_mav_put_uint8_t(buf, 98, number_mode);
	_mav_put_uint8_t(buf, 99, circle_0_number);
	_mav_put_uint8_t(buf, 100, circle_1_number);
	_mav_put_uint8_t(buf, 101, circle_2_number);
	_mav_put_uint8_t(buf, 102, circle_3_number);
	_mav_put_uint8_t(buf, 103, circle_4_number);
	_mav_put_uint8_t(buf, 104, circle_5_number);
	_mav_put_uint8_t(buf, 105, circle_6_number);
	_mav_put_uint8_t(buf, 106, circle_7_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN);
#else
	mavlink_vision_circle_all_t packet;
	packet.circle_0_x = circle_0_x;
	packet.circle_0_y = circle_0_y;
	packet.circle_0_z = circle_0_z;
	packet.circle_1_x = circle_1_x;
	packet.circle_1_y = circle_1_y;
	packet.circle_1_z = circle_1_z;
	packet.circle_2_x = circle_2_x;
	packet.circle_2_y = circle_2_y;
	packet.circle_2_z = circle_2_z;
	packet.circle_3_x = circle_3_x;
	packet.circle_3_y = circle_3_y;
	packet.circle_3_z = circle_3_z;
	packet.circle_4_x = circle_4_x;
	packet.circle_4_y = circle_4_y;
	packet.circle_4_z = circle_4_z;
	packet.circle_5_x = circle_5_x;
	packet.circle_5_y = circle_5_y;
	packet.circle_5_z = circle_5_z;
	packet.circle_6_x = circle_6_x;
	packet.circle_6_y = circle_6_y;
	packet.circle_6_z = circle_6_z;
	packet.circle_7_x = circle_7_x;
	packet.circle_7_y = circle_7_y;
	packet.circle_7_z = circle_7_z;
	packet.circle_sum = circle_sum;
	packet.marker_sum = marker_sum;
	packet.number_mode = number_mode;
	packet.circle_0_number = circle_0_number;
	packet.circle_1_number = circle_1_number;
	packet.circle_2_number = circle_2_number;
	packet.circle_3_number = circle_3_number;
	packet.circle_4_number = circle_4_number;
	packet.circle_5_number = circle_5_number;
	packet.circle_6_number = circle_6_number;
	packet.circle_7_number = circle_7_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_CIRCLE_ALL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
}

/**
 * @brief Pack a vision_circle_all message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param circle_sum the number of circles
 * @param marker_sum the number of makers
 * @param number_mode mode status
 * @param circle_0_number first circle's maker number
 * @param circle_0_x relative local poosition x
 * @param circle_0_y relative local poosition y
 * @param circle_0_z relative local poosition z
 * @param circle_1_number second circle's maker number
 * @param circle_1_x relative local poosition x
 * @param circle_1_y relative local poosition y
 * @param circle_1_z relative local poosition z
 * @param circle_2_number circle's maker number
 * @param circle_2_x relative local poosition x
 * @param circle_2_y relative local poosition y
 * @param circle_2_z relative local poosition z
 * @param circle_3_number circle's maker number
 * @param circle_3_x relative local poosition x
 * @param circle_3_y relative local poosition y
 * @param circle_3_z relative local poosition z
 * @param circle_4_number circle's maker number
 * @param circle_4_x relative local poosition x
 * @param circle_4_y relative local poosition y
 * @param circle_4_z relative local poosition z
 * @param circle_5_number circle's maker number
 * @param circle_5_x relative local poosition x
 * @param circle_5_y relative local poosition y
 * @param circle_5_z relative local poosition z
 * @param circle_6_number circle's maker number
 * @param circle_6_x relative local poosition x
 * @param circle_6_y relative local poosition y
 * @param circle_6_z relative local poosition z
 * @param circle_7_number circle's maker number
 * @param circle_7_x relative local poosition x
 * @param circle_7_y relative local poosition y
 * @param circle_7_z relative local poosition z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_circle_all_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t circle_sum,uint8_t marker_sum,uint8_t number_mode,uint8_t circle_0_number,float circle_0_x,float circle_0_y,float circle_0_z,uint8_t circle_1_number,float circle_1_x,float circle_1_y,float circle_1_z,uint8_t circle_2_number,float circle_2_x,float circle_2_y,float circle_2_z,uint8_t circle_3_number,float circle_3_x,float circle_3_y,float circle_3_z,uint8_t circle_4_number,float circle_4_x,float circle_4_y,float circle_4_z,uint8_t circle_5_number,float circle_5_x,float circle_5_y,float circle_5_z,uint8_t circle_6_number,float circle_6_x,float circle_6_y,float circle_6_z,uint8_t circle_7_number,float circle_7_x,float circle_7_y,float circle_7_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN];
	_mav_put_float(buf, 0, circle_0_x);
	_mav_put_float(buf, 4, circle_0_y);
	_mav_put_float(buf, 8, circle_0_z);
	_mav_put_float(buf, 12, circle_1_x);
	_mav_put_float(buf, 16, circle_1_y);
	_mav_put_float(buf, 20, circle_1_z);
	_mav_put_float(buf, 24, circle_2_x);
	_mav_put_float(buf, 28, circle_2_y);
	_mav_put_float(buf, 32, circle_2_z);
	_mav_put_float(buf, 36, circle_3_x);
	_mav_put_float(buf, 40, circle_3_y);
	_mav_put_float(buf, 44, circle_3_z);
	_mav_put_float(buf, 48, circle_4_x);
	_mav_put_float(buf, 52, circle_4_y);
	_mav_put_float(buf, 56, circle_4_z);
	_mav_put_float(buf, 60, circle_5_x);
	_mav_put_float(buf, 64, circle_5_y);
	_mav_put_float(buf, 68, circle_5_z);
	_mav_put_float(buf, 72, circle_6_x);
	_mav_put_float(buf, 76, circle_6_y);
	_mav_put_float(buf, 80, circle_6_z);
	_mav_put_float(buf, 84, circle_7_x);
	_mav_put_float(buf, 88, circle_7_y);
	_mav_put_float(buf, 92, circle_7_z);
	_mav_put_uint8_t(buf, 96, circle_sum);
	_mav_put_uint8_t(buf, 97, marker_sum);
	_mav_put_uint8_t(buf, 98, number_mode);
	_mav_put_uint8_t(buf, 99, circle_0_number);
	_mav_put_uint8_t(buf, 100, circle_1_number);
	_mav_put_uint8_t(buf, 101, circle_2_number);
	_mav_put_uint8_t(buf, 102, circle_3_number);
	_mav_put_uint8_t(buf, 103, circle_4_number);
	_mav_put_uint8_t(buf, 104, circle_5_number);
	_mav_put_uint8_t(buf, 105, circle_6_number);
	_mav_put_uint8_t(buf, 106, circle_7_number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN);
#else
	mavlink_vision_circle_all_t packet;
	packet.circle_0_x = circle_0_x;
	packet.circle_0_y = circle_0_y;
	packet.circle_0_z = circle_0_z;
	packet.circle_1_x = circle_1_x;
	packet.circle_1_y = circle_1_y;
	packet.circle_1_z = circle_1_z;
	packet.circle_2_x = circle_2_x;
	packet.circle_2_y = circle_2_y;
	packet.circle_2_z = circle_2_z;
	packet.circle_3_x = circle_3_x;
	packet.circle_3_y = circle_3_y;
	packet.circle_3_z = circle_3_z;
	packet.circle_4_x = circle_4_x;
	packet.circle_4_y = circle_4_y;
	packet.circle_4_z = circle_4_z;
	packet.circle_5_x = circle_5_x;
	packet.circle_5_y = circle_5_y;
	packet.circle_5_z = circle_5_z;
	packet.circle_6_x = circle_6_x;
	packet.circle_6_y = circle_6_y;
	packet.circle_6_z = circle_6_z;
	packet.circle_7_x = circle_7_x;
	packet.circle_7_y = circle_7_y;
	packet.circle_7_z = circle_7_z;
	packet.circle_sum = circle_sum;
	packet.marker_sum = marker_sum;
	packet.number_mode = number_mode;
	packet.circle_0_number = circle_0_number;
	packet.circle_1_number = circle_1_number;
	packet.circle_2_number = circle_2_number;
	packet.circle_3_number = circle_3_number;
	packet.circle_4_number = circle_4_number;
	packet.circle_5_number = circle_5_number;
	packet.circle_6_number = circle_6_number;
	packet.circle_7_number = circle_7_number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_CIRCLE_ALL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
}

/**
 * @brief Encode a vision_circle_all struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_circle_all C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_circle_all_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_circle_all_t* vision_circle_all)
{
	return mavlink_msg_vision_circle_all_pack(system_id, component_id, msg, vision_circle_all->circle_sum, vision_circle_all->marker_sum, vision_circle_all->number_mode, vision_circle_all->circle_0_number, vision_circle_all->circle_0_x, vision_circle_all->circle_0_y, vision_circle_all->circle_0_z, vision_circle_all->circle_1_number, vision_circle_all->circle_1_x, vision_circle_all->circle_1_y, vision_circle_all->circle_1_z, vision_circle_all->circle_2_number, vision_circle_all->circle_2_x, vision_circle_all->circle_2_y, vision_circle_all->circle_2_z, vision_circle_all->circle_3_number, vision_circle_all->circle_3_x, vision_circle_all->circle_3_y, vision_circle_all->circle_3_z, vision_circle_all->circle_4_number, vision_circle_all->circle_4_x, vision_circle_all->circle_4_y, vision_circle_all->circle_4_z, vision_circle_all->circle_5_number, vision_circle_all->circle_5_x, vision_circle_all->circle_5_y, vision_circle_all->circle_5_z, vision_circle_all->circle_6_number, vision_circle_all->circle_6_x, vision_circle_all->circle_6_y, vision_circle_all->circle_6_z, vision_circle_all->circle_7_number, vision_circle_all->circle_7_x, vision_circle_all->circle_7_y, vision_circle_all->circle_7_z);
}

/**
 * @brief Encode a vision_circle_all struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_circle_all C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_circle_all_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_circle_all_t* vision_circle_all)
{
	return mavlink_msg_vision_circle_all_pack_chan(system_id, component_id, chan, msg, vision_circle_all->circle_sum, vision_circle_all->marker_sum, vision_circle_all->number_mode, vision_circle_all->circle_0_number, vision_circle_all->circle_0_x, vision_circle_all->circle_0_y, vision_circle_all->circle_0_z, vision_circle_all->circle_1_number, vision_circle_all->circle_1_x, vision_circle_all->circle_1_y, vision_circle_all->circle_1_z, vision_circle_all->circle_2_number, vision_circle_all->circle_2_x, vision_circle_all->circle_2_y, vision_circle_all->circle_2_z, vision_circle_all->circle_3_number, vision_circle_all->circle_3_x, vision_circle_all->circle_3_y, vision_circle_all->circle_3_z, vision_circle_all->circle_4_number, vision_circle_all->circle_4_x, vision_circle_all->circle_4_y, vision_circle_all->circle_4_z, vision_circle_all->circle_5_number, vision_circle_all->circle_5_x, vision_circle_all->circle_5_y, vision_circle_all->circle_5_z, vision_circle_all->circle_6_number, vision_circle_all->circle_6_x, vision_circle_all->circle_6_y, vision_circle_all->circle_6_z, vision_circle_all->circle_7_number, vision_circle_all->circle_7_x, vision_circle_all->circle_7_y, vision_circle_all->circle_7_z);
}

/**
 * @brief Send a vision_circle_all message
 * @param chan MAVLink channel to send the message
 *
 * @param circle_sum the number of circles
 * @param marker_sum the number of makers
 * @param number_mode mode status
 * @param circle_0_number first circle's maker number
 * @param circle_0_x relative local poosition x
 * @param circle_0_y relative local poosition y
 * @param circle_0_z relative local poosition z
 * @param circle_1_number second circle's maker number
 * @param circle_1_x relative local poosition x
 * @param circle_1_y relative local poosition y
 * @param circle_1_z relative local poosition z
 * @param circle_2_number circle's maker number
 * @param circle_2_x relative local poosition x
 * @param circle_2_y relative local poosition y
 * @param circle_2_z relative local poosition z
 * @param circle_3_number circle's maker number
 * @param circle_3_x relative local poosition x
 * @param circle_3_y relative local poosition y
 * @param circle_3_z relative local poosition z
 * @param circle_4_number circle's maker number
 * @param circle_4_x relative local poosition x
 * @param circle_4_y relative local poosition y
 * @param circle_4_z relative local poosition z
 * @param circle_5_number circle's maker number
 * @param circle_5_x relative local poosition x
 * @param circle_5_y relative local poosition y
 * @param circle_5_z relative local poosition z
 * @param circle_6_number circle's maker number
 * @param circle_6_x relative local poosition x
 * @param circle_6_y relative local poosition y
 * @param circle_6_z relative local poosition z
 * @param circle_7_number circle's maker number
 * @param circle_7_x relative local poosition x
 * @param circle_7_y relative local poosition y
 * @param circle_7_z relative local poosition z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_circle_all_send(mavlink_channel_t chan, uint8_t circle_sum, uint8_t marker_sum, uint8_t number_mode, uint8_t circle_0_number, float circle_0_x, float circle_0_y, float circle_0_z, uint8_t circle_1_number, float circle_1_x, float circle_1_y, float circle_1_z, uint8_t circle_2_number, float circle_2_x, float circle_2_y, float circle_2_z, uint8_t circle_3_number, float circle_3_x, float circle_3_y, float circle_3_z, uint8_t circle_4_number, float circle_4_x, float circle_4_y, float circle_4_z, uint8_t circle_5_number, float circle_5_x, float circle_5_y, float circle_5_z, uint8_t circle_6_number, float circle_6_x, float circle_6_y, float circle_6_z, uint8_t circle_7_number, float circle_7_x, float circle_7_y, float circle_7_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN];
	_mav_put_float(buf, 0, circle_0_x);
	_mav_put_float(buf, 4, circle_0_y);
	_mav_put_float(buf, 8, circle_0_z);
	_mav_put_float(buf, 12, circle_1_x);
	_mav_put_float(buf, 16, circle_1_y);
	_mav_put_float(buf, 20, circle_1_z);
	_mav_put_float(buf, 24, circle_2_x);
	_mav_put_float(buf, 28, circle_2_y);
	_mav_put_float(buf, 32, circle_2_z);
	_mav_put_float(buf, 36, circle_3_x);
	_mav_put_float(buf, 40, circle_3_y);
	_mav_put_float(buf, 44, circle_3_z);
	_mav_put_float(buf, 48, circle_4_x);
	_mav_put_float(buf, 52, circle_4_y);
	_mav_put_float(buf, 56, circle_4_z);
	_mav_put_float(buf, 60, circle_5_x);
	_mav_put_float(buf, 64, circle_5_y);
	_mav_put_float(buf, 68, circle_5_z);
	_mav_put_float(buf, 72, circle_6_x);
	_mav_put_float(buf, 76, circle_6_y);
	_mav_put_float(buf, 80, circle_6_z);
	_mav_put_float(buf, 84, circle_7_x);
	_mav_put_float(buf, 88, circle_7_y);
	_mav_put_float(buf, 92, circle_7_z);
	_mav_put_uint8_t(buf, 96, circle_sum);
	_mav_put_uint8_t(buf, 97, marker_sum);
	_mav_put_uint8_t(buf, 98, number_mode);
	_mav_put_uint8_t(buf, 99, circle_0_number);
	_mav_put_uint8_t(buf, 100, circle_1_number);
	_mav_put_uint8_t(buf, 101, circle_2_number);
	_mav_put_uint8_t(buf, 102, circle_3_number);
	_mav_put_uint8_t(buf, 103, circle_4_number);
	_mav_put_uint8_t(buf, 104, circle_5_number);
	_mav_put_uint8_t(buf, 105, circle_6_number);
	_mav_put_uint8_t(buf, 106, circle_7_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL, buf, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
#else
	mavlink_vision_circle_all_t packet;
	packet.circle_0_x = circle_0_x;
	packet.circle_0_y = circle_0_y;
	packet.circle_0_z = circle_0_z;
	packet.circle_1_x = circle_1_x;
	packet.circle_1_y = circle_1_y;
	packet.circle_1_z = circle_1_z;
	packet.circle_2_x = circle_2_x;
	packet.circle_2_y = circle_2_y;
	packet.circle_2_z = circle_2_z;
	packet.circle_3_x = circle_3_x;
	packet.circle_3_y = circle_3_y;
	packet.circle_3_z = circle_3_z;
	packet.circle_4_x = circle_4_x;
	packet.circle_4_y = circle_4_y;
	packet.circle_4_z = circle_4_z;
	packet.circle_5_x = circle_5_x;
	packet.circle_5_y = circle_5_y;
	packet.circle_5_z = circle_5_z;
	packet.circle_6_x = circle_6_x;
	packet.circle_6_y = circle_6_y;
	packet.circle_6_z = circle_6_z;
	packet.circle_7_x = circle_7_x;
	packet.circle_7_y = circle_7_y;
	packet.circle_7_z = circle_7_z;
	packet.circle_sum = circle_sum;
	packet.marker_sum = marker_sum;
	packet.number_mode = number_mode;
	packet.circle_0_number = circle_0_number;
	packet.circle_1_number = circle_1_number;
	packet.circle_2_number = circle_2_number;
	packet.circle_3_number = circle_3_number;
	packet.circle_4_number = circle_4_number;
	packet.circle_5_number = circle_5_number;
	packet.circle_6_number = circle_6_number;
	packet.circle_7_number = circle_7_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL, (const char *)&packet, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
#endif
}

/**
 * @brief Send a vision_circle_all message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_circle_all_send_struct(mavlink_channel_t chan, const mavlink_vision_circle_all_t* vision_circle_all)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_circle_all_send(chan, vision_circle_all->circle_sum, vision_circle_all->marker_sum, vision_circle_all->number_mode, vision_circle_all->circle_0_number, vision_circle_all->circle_0_x, vision_circle_all->circle_0_y, vision_circle_all->circle_0_z, vision_circle_all->circle_1_number, vision_circle_all->circle_1_x, vision_circle_all->circle_1_y, vision_circle_all->circle_1_z, vision_circle_all->circle_2_number, vision_circle_all->circle_2_x, vision_circle_all->circle_2_y, vision_circle_all->circle_2_z, vision_circle_all->circle_3_number, vision_circle_all->circle_3_x, vision_circle_all->circle_3_y, vision_circle_all->circle_3_z, vision_circle_all->circle_4_number, vision_circle_all->circle_4_x, vision_circle_all->circle_4_y, vision_circle_all->circle_4_z, vision_circle_all->circle_5_number, vision_circle_all->circle_5_x, vision_circle_all->circle_5_y, vision_circle_all->circle_5_z, vision_circle_all->circle_6_number, vision_circle_all->circle_6_x, vision_circle_all->circle_6_y, vision_circle_all->circle_6_z, vision_circle_all->circle_7_number, vision_circle_all->circle_7_x, vision_circle_all->circle_7_y, vision_circle_all->circle_7_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL, (const char *)vision_circle_all, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_circle_all_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t circle_sum, uint8_t marker_sum, uint8_t number_mode, uint8_t circle_0_number, float circle_0_x, float circle_0_y, float circle_0_z, uint8_t circle_1_number, float circle_1_x, float circle_1_y, float circle_1_z, uint8_t circle_2_number, float circle_2_x, float circle_2_y, float circle_2_z, uint8_t circle_3_number, float circle_3_x, float circle_3_y, float circle_3_z, uint8_t circle_4_number, float circle_4_x, float circle_4_y, float circle_4_z, uint8_t circle_5_number, float circle_5_x, float circle_5_y, float circle_5_z, uint8_t circle_6_number, float circle_6_x, float circle_6_y, float circle_6_z, uint8_t circle_7_number, float circle_7_x, float circle_7_y, float circle_7_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, circle_0_x);
	_mav_put_float(buf, 4, circle_0_y);
	_mav_put_float(buf, 8, circle_0_z);
	_mav_put_float(buf, 12, circle_1_x);
	_mav_put_float(buf, 16, circle_1_y);
	_mav_put_float(buf, 20, circle_1_z);
	_mav_put_float(buf, 24, circle_2_x);
	_mav_put_float(buf, 28, circle_2_y);
	_mav_put_float(buf, 32, circle_2_z);
	_mav_put_float(buf, 36, circle_3_x);
	_mav_put_float(buf, 40, circle_3_y);
	_mav_put_float(buf, 44, circle_3_z);
	_mav_put_float(buf, 48, circle_4_x);
	_mav_put_float(buf, 52, circle_4_y);
	_mav_put_float(buf, 56, circle_4_z);
	_mav_put_float(buf, 60, circle_5_x);
	_mav_put_float(buf, 64, circle_5_y);
	_mav_put_float(buf, 68, circle_5_z);
	_mav_put_float(buf, 72, circle_6_x);
	_mav_put_float(buf, 76, circle_6_y);
	_mav_put_float(buf, 80, circle_6_z);
	_mav_put_float(buf, 84, circle_7_x);
	_mav_put_float(buf, 88, circle_7_y);
	_mav_put_float(buf, 92, circle_7_z);
	_mav_put_uint8_t(buf, 96, circle_sum);
	_mav_put_uint8_t(buf, 97, marker_sum);
	_mav_put_uint8_t(buf, 98, number_mode);
	_mav_put_uint8_t(buf, 99, circle_0_number);
	_mav_put_uint8_t(buf, 100, circle_1_number);
	_mav_put_uint8_t(buf, 101, circle_2_number);
	_mav_put_uint8_t(buf, 102, circle_3_number);
	_mav_put_uint8_t(buf, 103, circle_4_number);
	_mav_put_uint8_t(buf, 104, circle_5_number);
	_mav_put_uint8_t(buf, 105, circle_6_number);
	_mav_put_uint8_t(buf, 106, circle_7_number);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL, buf, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
#else
	mavlink_vision_circle_all_t *packet = (mavlink_vision_circle_all_t *)msgbuf;
	packet->circle_0_x = circle_0_x;
	packet->circle_0_y = circle_0_y;
	packet->circle_0_z = circle_0_z;
	packet->circle_1_x = circle_1_x;
	packet->circle_1_y = circle_1_y;
	packet->circle_1_z = circle_1_z;
	packet->circle_2_x = circle_2_x;
	packet->circle_2_y = circle_2_y;
	packet->circle_2_z = circle_2_z;
	packet->circle_3_x = circle_3_x;
	packet->circle_3_y = circle_3_y;
	packet->circle_3_z = circle_3_z;
	packet->circle_4_x = circle_4_x;
	packet->circle_4_y = circle_4_y;
	packet->circle_4_z = circle_4_z;
	packet->circle_5_x = circle_5_x;
	packet->circle_5_y = circle_5_y;
	packet->circle_5_z = circle_5_z;
	packet->circle_6_x = circle_6_x;
	packet->circle_6_y = circle_6_y;
	packet->circle_6_z = circle_6_z;
	packet->circle_7_x = circle_7_x;
	packet->circle_7_y = circle_7_y;
	packet->circle_7_z = circle_7_z;
	packet->circle_sum = circle_sum;
	packet->marker_sum = marker_sum;
	packet->number_mode = number_mode;
	packet->circle_0_number = circle_0_number;
	packet->circle_1_number = circle_1_number;
	packet->circle_2_number = circle_2_number;
	packet->circle_3_number = circle_3_number;
	packet->circle_4_number = circle_4_number;
	packet->circle_5_number = circle_5_number;
	packet->circle_6_number = circle_6_number;
	packet->circle_7_number = circle_7_number;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_CIRCLE_ALL, (const char *)packet, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_MIN_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_CIRCLE_ALL UNPACKING


/**
 * @brief Get field circle_sum from vision_circle_all message
 *
 * @return the number of circles
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_sum(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  96);
}

/**
 * @brief Get field marker_sum from vision_circle_all message
 *
 * @return the number of makers
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_marker_sum(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  97);
}

/**
 * @brief Get field number_mode from vision_circle_all message
 *
 * @return mode status
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_number_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  98);
}

/**
 * @brief Get field circle_0_number from vision_circle_all message
 *
 * @return first circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_0_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  99);
}

/**
 * @brief Get field circle_0_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_0_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field circle_0_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_0_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field circle_0_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_0_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field circle_1_number from vision_circle_all message
 *
 * @return second circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_1_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  100);
}

/**
 * @brief Get field circle_1_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_1_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field circle_1_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_1_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field circle_1_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_1_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field circle_2_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_2_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  101);
}

/**
 * @brief Get field circle_2_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_2_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field circle_2_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_2_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field circle_2_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_2_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field circle_3_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_3_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  102);
}

/**
 * @brief Get field circle_3_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_3_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field circle_3_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_3_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field circle_3_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_3_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field circle_4_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_4_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  103);
}

/**
 * @brief Get field circle_4_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_4_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field circle_4_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_4_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field circle_4_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_4_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field circle_5_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_5_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  104);
}

/**
 * @brief Get field circle_5_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_5_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field circle_5_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_5_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field circle_5_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_5_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field circle_6_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_6_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  105);
}

/**
 * @brief Get field circle_6_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_6_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field circle_6_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_6_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field circle_6_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_6_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field circle_7_number from vision_circle_all message
 *
 * @return circle's maker number
 */
static inline uint8_t mavlink_msg_vision_circle_all_get_circle_7_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  106);
}

/**
 * @brief Get field circle_7_x from vision_circle_all message
 *
 * @return relative local poosition x
 */
static inline float mavlink_msg_vision_circle_all_get_circle_7_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field circle_7_y from vision_circle_all message
 *
 * @return relative local poosition y
 */
static inline float mavlink_msg_vision_circle_all_get_circle_7_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field circle_7_z from vision_circle_all message
 *
 * @return relative local poosition z
 */
static inline float mavlink_msg_vision_circle_all_get_circle_7_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Decode a vision_circle_all message into a struct
 *
 * @param msg The message to decode
 * @param vision_circle_all C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_circle_all_decode(const mavlink_message_t* msg, mavlink_vision_circle_all_t* vision_circle_all)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_circle_all->circle_0_x = mavlink_msg_vision_circle_all_get_circle_0_x(msg);
	vision_circle_all->circle_0_y = mavlink_msg_vision_circle_all_get_circle_0_y(msg);
	vision_circle_all->circle_0_z = mavlink_msg_vision_circle_all_get_circle_0_z(msg);
	vision_circle_all->circle_1_x = mavlink_msg_vision_circle_all_get_circle_1_x(msg);
	vision_circle_all->circle_1_y = mavlink_msg_vision_circle_all_get_circle_1_y(msg);
	vision_circle_all->circle_1_z = mavlink_msg_vision_circle_all_get_circle_1_z(msg);
	vision_circle_all->circle_2_x = mavlink_msg_vision_circle_all_get_circle_2_x(msg);
	vision_circle_all->circle_2_y = mavlink_msg_vision_circle_all_get_circle_2_y(msg);
	vision_circle_all->circle_2_z = mavlink_msg_vision_circle_all_get_circle_2_z(msg);
	vision_circle_all->circle_3_x = mavlink_msg_vision_circle_all_get_circle_3_x(msg);
	vision_circle_all->circle_3_y = mavlink_msg_vision_circle_all_get_circle_3_y(msg);
	vision_circle_all->circle_3_z = mavlink_msg_vision_circle_all_get_circle_3_z(msg);
	vision_circle_all->circle_4_x = mavlink_msg_vision_circle_all_get_circle_4_x(msg);
	vision_circle_all->circle_4_y = mavlink_msg_vision_circle_all_get_circle_4_y(msg);
	vision_circle_all->circle_4_z = mavlink_msg_vision_circle_all_get_circle_4_z(msg);
	vision_circle_all->circle_5_x = mavlink_msg_vision_circle_all_get_circle_5_x(msg);
	vision_circle_all->circle_5_y = mavlink_msg_vision_circle_all_get_circle_5_y(msg);
	vision_circle_all->circle_5_z = mavlink_msg_vision_circle_all_get_circle_5_z(msg);
	vision_circle_all->circle_6_x = mavlink_msg_vision_circle_all_get_circle_6_x(msg);
	vision_circle_all->circle_6_y = mavlink_msg_vision_circle_all_get_circle_6_y(msg);
	vision_circle_all->circle_6_z = mavlink_msg_vision_circle_all_get_circle_6_z(msg);
	vision_circle_all->circle_7_x = mavlink_msg_vision_circle_all_get_circle_7_x(msg);
	vision_circle_all->circle_7_y = mavlink_msg_vision_circle_all_get_circle_7_y(msg);
	vision_circle_all->circle_7_z = mavlink_msg_vision_circle_all_get_circle_7_z(msg);
	vision_circle_all->circle_sum = mavlink_msg_vision_circle_all_get_circle_sum(msg);
	vision_circle_all->marker_sum = mavlink_msg_vision_circle_all_get_marker_sum(msg);
	vision_circle_all->number_mode = mavlink_msg_vision_circle_all_get_number_mode(msg);
	vision_circle_all->circle_0_number = mavlink_msg_vision_circle_all_get_circle_0_number(msg);
	vision_circle_all->circle_1_number = mavlink_msg_vision_circle_all_get_circle_1_number(msg);
	vision_circle_all->circle_2_number = mavlink_msg_vision_circle_all_get_circle_2_number(msg);
	vision_circle_all->circle_3_number = mavlink_msg_vision_circle_all_get_circle_3_number(msg);
	vision_circle_all->circle_4_number = mavlink_msg_vision_circle_all_get_circle_4_number(msg);
	vision_circle_all->circle_5_number = mavlink_msg_vision_circle_all_get_circle_5_number(msg);
	vision_circle_all->circle_6_number = mavlink_msg_vision_circle_all_get_circle_6_number(msg);
	vision_circle_all->circle_7_number = mavlink_msg_vision_circle_all_get_circle_7_number(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN? msg->len : MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN;
        memset(vision_circle_all, 0, MAVLINK_MSG_ID_VISION_CIRCLE_ALL_LEN);
	memcpy(vision_circle_all, _MAV_PAYLOAD(msg), len);
#endif
}
