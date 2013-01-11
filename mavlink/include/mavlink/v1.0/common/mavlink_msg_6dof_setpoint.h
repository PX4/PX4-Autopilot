// MESSAGE 6DOF_SETPOINT PACKING

#define MAVLINK_MSG_ID_6DOF_SETPOINT 149

typedef struct __mavlink_6dof_setpoint_t
{
 float trans_x; ///< Translational Component in x
 float trans_y; ///< Translational Component in y
 float trans_z; ///< Translational Component in z
 float rot_x; ///< Rotational Component in x
 float rot_y; ///< Rotational Component in y
 float rot_z; ///< Rotational Component in z
 uint8_t target_system; ///< System ID
} mavlink_6dof_setpoint_t;

#define MAVLINK_MSG_ID_6DOF_SETPOINT_LEN 25
#define MAVLINK_MSG_ID_149_LEN 25



#define MAVLINK_MESSAGE_INFO_6DOF_SETPOINT { \
	"6DOF_SETPOINT", \
	7, \
	{  { "trans_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_6dof_setpoint_t, trans_x) }, \
         { "trans_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_6dof_setpoint_t, trans_y) }, \
         { "trans_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_6dof_setpoint_t, trans_z) }, \
         { "rot_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_6dof_setpoint_t, rot_x) }, \
         { "rot_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_6dof_setpoint_t, rot_y) }, \
         { "rot_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_6dof_setpoint_t, rot_z) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_6dof_setpoint_t, target_system) }, \
         } \
}


/**
 * @brief Pack a 6dof_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_6dof_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, float trans_x, float trans_y, float trans_z, float rot_x, float rot_y, float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 25);
#else
	mavlink_6dof_setpoint_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 25);
#endif

	msg->msgid = MAVLINK_MSG_ID_6DOF_SETPOINT;
	return mavlink_finalize_message(msg, system_id, component_id, 25, 144);
}

/**
 * @brief Pack a 6dof_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_6dof_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,float trans_x,float trans_y,float trans_z,float rot_x,float rot_y,float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 25);
#else
	mavlink_6dof_setpoint_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 25);
#endif

	msg->msgid = MAVLINK_MSG_ID_6DOF_SETPOINT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 25, 144);
}

/**
 * @brief Encode a 6dof_setpoint struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param 6dof_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_6dof_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_6dof_setpoint_t* 6dof_setpoint)
{
	return mavlink_msg_6dof_setpoint_pack(system_id, component_id, msg, 6dof_setpoint->target_system, 6dof_setpoint->trans_x, 6dof_setpoint->trans_y, 6dof_setpoint->trans_z, 6dof_setpoint->rot_x, 6dof_setpoint->rot_y, 6dof_setpoint->rot_z);
}

/**
 * @brief Send a 6dof_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param trans_x Translational Component in x
 * @param trans_y Translational Component in y
 * @param trans_z Translational Component in z
 * @param rot_x Rotational Component in x
 * @param rot_y Rotational Component in y
 * @param rot_z Rotational Component in z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_6dof_setpoint_send(mavlink_channel_t chan, uint8_t target_system, float trans_x, float trans_y, float trans_z, float rot_x, float rot_y, float rot_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[25];
	_mav_put_float(buf, 0, trans_x);
	_mav_put_float(buf, 4, trans_y);
	_mav_put_float(buf, 8, trans_z);
	_mav_put_float(buf, 12, rot_x);
	_mav_put_float(buf, 16, rot_y);
	_mav_put_float(buf, 20, rot_z);
	_mav_put_uint8_t(buf, 24, target_system);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_6DOF_SETPOINT, buf, 25, 144);
#else
	mavlink_6dof_setpoint_t packet;
	packet.trans_x = trans_x;
	packet.trans_y = trans_y;
	packet.trans_z = trans_z;
	packet.rot_x = rot_x;
	packet.rot_y = rot_y;
	packet.rot_z = rot_z;
	packet.target_system = target_system;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_6DOF_SETPOINT, (const char *)&packet, 25, 144);
#endif
}

#endif

// MESSAGE 6DOF_SETPOINT UNPACKING


/**
 * @brief Get field target_system from 6dof_setpoint message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_6dof_setpoint_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field trans_x from 6dof_setpoint message
 *
 * @return Translational Component in x
 */
static inline float mavlink_msg_6dof_setpoint_get_trans_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field trans_y from 6dof_setpoint message
 *
 * @return Translational Component in y
 */
static inline float mavlink_msg_6dof_setpoint_get_trans_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field trans_z from 6dof_setpoint message
 *
 * @return Translational Component in z
 */
static inline float mavlink_msg_6dof_setpoint_get_trans_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rot_x from 6dof_setpoint message
 *
 * @return Rotational Component in x
 */
static inline float mavlink_msg_6dof_setpoint_get_rot_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rot_y from 6dof_setpoint message
 *
 * @return Rotational Component in y
 */
static inline float mavlink_msg_6dof_setpoint_get_rot_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field rot_z from 6dof_setpoint message
 *
 * @return Rotational Component in z
 */
static inline float mavlink_msg_6dof_setpoint_get_rot_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a 6dof_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param 6dof_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_6dof_setpoint_decode(const mavlink_message_t* msg, mavlink_6dof_setpoint_t* 6dof_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP
	6dof_setpoint->trans_x = mavlink_msg_6dof_setpoint_get_trans_x(msg);
	6dof_setpoint->trans_y = mavlink_msg_6dof_setpoint_get_trans_y(msg);
	6dof_setpoint->trans_z = mavlink_msg_6dof_setpoint_get_trans_z(msg);
	6dof_setpoint->rot_x = mavlink_msg_6dof_setpoint_get_rot_x(msg);
	6dof_setpoint->rot_y = mavlink_msg_6dof_setpoint_get_rot_y(msg);
	6dof_setpoint->rot_z = mavlink_msg_6dof_setpoint_get_rot_z(msg);
	6dof_setpoint->target_system = mavlink_msg_6dof_setpoint_get_target_system(msg);
#else
	memcpy(6dof_setpoint, _MAV_PAYLOAD(msg), 25);
#endif
}
