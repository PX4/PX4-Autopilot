// MESSAGE 8DOF_SETPOINT PACKING

#define MAVLINK_MSG_ID_8DOF_SETPOINT 148

typedef struct __mavlink_8dof_setpoint_t
{
 float val1; ///< Value 1
 float val2; ///< Value 2
 float val3; ///< Value 3
 float val4; ///< Value 4
 float val5; ///< Value 5
 float val6; ///< Value 6
 float val7; ///< Value 7
 float val8; ///< Value 8
 uint8_t target_system; ///< System ID
} mavlink_8dof_setpoint_t;

#define MAVLINK_MSG_ID_8DOF_SETPOINT_LEN 33
#define MAVLINK_MSG_ID_148_LEN 33



#define MAVLINK_MESSAGE_INFO_8DOF_SETPOINT { \
	"8DOF_SETPOINT", \
	9, \
	{  { "val1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_8dof_setpoint_t, val1) }, \
         { "val2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_8dof_setpoint_t, val2) }, \
         { "val3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_8dof_setpoint_t, val3) }, \
         { "val4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_8dof_setpoint_t, val4) }, \
         { "val5", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_8dof_setpoint_t, val5) }, \
         { "val6", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_8dof_setpoint_t, val6) }, \
         { "val7", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_8dof_setpoint_t, val7) }, \
         { "val8", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_8dof_setpoint_t, val8) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_8dof_setpoint_t, target_system) }, \
         } \
}


/**
 * @brief Pack a 8dof_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param val1 Value 1
 * @param val2 Value 2
 * @param val3 Value 3
 * @param val4 Value 4
 * @param val5 Value 5
 * @param val6 Value 6
 * @param val7 Value 7
 * @param val8 Value 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_8dof_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, float val1, float val2, float val3, float val4, float val5, float val6, float val7, float val8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_float(buf, 0, val1);
	_mav_put_float(buf, 4, val2);
	_mav_put_float(buf, 8, val3);
	_mav_put_float(buf, 12, val4);
	_mav_put_float(buf, 16, val5);
	_mav_put_float(buf, 20, val6);
	_mav_put_float(buf, 24, val7);
	_mav_put_float(buf, 28, val8);
	_mav_put_uint8_t(buf, 32, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_8dof_setpoint_t packet;
	packet.val1 = val1;
	packet.val2 = val2;
	packet.val3 = val3;
	packet.val4 = val4;
	packet.val5 = val5;
	packet.val6 = val6;
	packet.val7 = val7;
	packet.val8 = val8;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_8DOF_SETPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 33, 42);
}

/**
 * @brief Pack a 8dof_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param val1 Value 1
 * @param val2 Value 2
 * @param val3 Value 3
 * @param val4 Value 4
 * @param val5 Value 5
 * @param val6 Value 6
 * @param val7 Value 7
 * @param val8 Value 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_8dof_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,float val1,float val2,float val3,float val4,float val5,float val6,float val7,float val8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_float(buf, 0, val1);
	_mav_put_float(buf, 4, val2);
	_mav_put_float(buf, 8, val3);
	_mav_put_float(buf, 12, val4);
	_mav_put_float(buf, 16, val5);
	_mav_put_float(buf, 20, val6);
	_mav_put_float(buf, 24, val7);
	_mav_put_float(buf, 28, val8);
	_mav_put_uint8_t(buf, 32, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_8dof_setpoint_t packet;
	packet.val1 = val1;
	packet.val2 = val2;
	packet.val3 = val3;
	packet.val4 = val4;
	packet.val5 = val5;
	packet.val6 = val6;
	packet.val7 = val7;
	packet.val8 = val8;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_8DOF_SETPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 33, 42);
}

/**
 * @brief Encode a 8dof_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param 8dof_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_8dof_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_8dof_setpoint_t* 8dof_setpoint)
{
	return mavlink_msg_8dof_setpoint_pack(system_id, component_id, msg, 8dof_setpoint->target_system, 8dof_setpoint->val1, 8dof_setpoint->val2, 8dof_setpoint->val3, 8dof_setpoint->val4, 8dof_setpoint->val5, 8dof_setpoint->val6, 8dof_setpoint->val7, 8dof_setpoint->val8);
}

/**
 * @brief Send a 8dof_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param val1 Value 1
 * @param val2 Value 2
 * @param val3 Value 3
 * @param val4 Value 4
 * @param val5 Value 5
 * @param val6 Value 6
 * @param val7 Value 7
 * @param val8 Value 8
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_8dof_setpoint_send(mavlink_channel_t chan, uint8_t target_system, float val1, float val2, float val3, float val4, float val5, float val6, float val7, float val8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_float(buf, 0, val1);
	_mav_put_float(buf, 4, val2);
	_mav_put_float(buf, 8, val3);
	_mav_put_float(buf, 12, val4);
	_mav_put_float(buf, 16, val5);
	_mav_put_float(buf, 20, val6);
	_mav_put_float(buf, 24, val7);
	_mav_put_float(buf, 28, val8);
	_mav_put_uint8_t(buf, 32, target_system);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_8DOF_SETPOINT, buf, 33, 42);
#else
	mavlink_8dof_setpoint_t packet;
	packet.val1 = val1;
	packet.val2 = val2;
	packet.val3 = val3;
	packet.val4 = val4;
	packet.val5 = val5;
	packet.val6 = val6;
	packet.val7 = val7;
	packet.val8 = val8;
	packet.target_system = target_system;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_8DOF_SETPOINT, (const char *)&packet, 33, 42);
#endif
}

#endif

// MESSAGE 8DOF_SETPOINT UNPACKING


/**
 * @brief Get field target_system from 8dof_setpoint message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_8dof_setpoint_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field val1 from 8dof_setpoint message
 *
 * @return Value 1
 */
static inline float mavlink_msg_8dof_setpoint_get_val1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field val2 from 8dof_setpoint message
 *
 * @return Value 2
 */
static inline float mavlink_msg_8dof_setpoint_get_val2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field val3 from 8dof_setpoint message
 *
 * @return Value 3
 */
static inline float mavlink_msg_8dof_setpoint_get_val3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field val4 from 8dof_setpoint message
 *
 * @return Value 4
 */
static inline float mavlink_msg_8dof_setpoint_get_val4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field val5 from 8dof_setpoint message
 *
 * @return Value 5
 */
static inline float mavlink_msg_8dof_setpoint_get_val5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field val6 from 8dof_setpoint message
 *
 * @return Value 6
 */
static inline float mavlink_msg_8dof_setpoint_get_val6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field val7 from 8dof_setpoint message
 *
 * @return Value 7
 */
static inline float mavlink_msg_8dof_setpoint_get_val7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field val8 from 8dof_setpoint message
 *
 * @return Value 8
 */
static inline float mavlink_msg_8dof_setpoint_get_val8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a 8dof_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param 8dof_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_8dof_setpoint_decode(const mavlink_message_t* msg, mavlink_8dof_setpoint_t* 8dof_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	8dof_setpoint->val1 = mavlink_msg_8dof_setpoint_get_val1(msg);
	8dof_setpoint->val2 = mavlink_msg_8dof_setpoint_get_val2(msg);
	8dof_setpoint->val3 = mavlink_msg_8dof_setpoint_get_val3(msg);
	8dof_setpoint->val4 = mavlink_msg_8dof_setpoint_get_val4(msg);
	8dof_setpoint->val5 = mavlink_msg_8dof_setpoint_get_val5(msg);
	8dof_setpoint->val6 = mavlink_msg_8dof_setpoint_get_val6(msg);
	8dof_setpoint->val7 = mavlink_msg_8dof_setpoint_get_val7(msg);
	8dof_setpoint->val8 = mavlink_msg_8dof_setpoint_get_val8(msg);
	8dof_setpoint->target_system = mavlink_msg_8dof_setpoint_get_target_system(msg);
#else
	memcpy(8dof_setpoint, _MAV_PAYLOAD(msg), 33);
#endif
}
