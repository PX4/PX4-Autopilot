// MESSAGE FLEXIFUNCTION_COMMAND PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND 157

typedef struct __mavlink_flexifunction_command_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t command_type; ///< Flexifunction command type
} mavlink_flexifunction_command_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN 3
#define MAVLINK_MSG_ID_157_LEN 3

#define MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC 133
#define MAVLINK_MSG_ID_157_CRC 133



#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_COMMAND { \
	"FLEXIFUNCTION_COMMAND", \
	3, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_flexifunction_command_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_flexifunction_command_t, target_component) }, \
         { "command_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_flexifunction_command_t, command_type) }, \
         } \
}


/**
 * @brief Pack a flexifunction_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#else
	mavlink_flexifunction_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, command_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#else
	mavlink_flexifunction_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command_type = command_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_command struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_command_t* flexifunction_command)
{
	return mavlink_msg_flexifunction_command_pack(system_id, component_id, msg, flexifunction_command->target_system, flexifunction_command->target_component, flexifunction_command->command_type);
}

/**
 * @brief Send a flexifunction_command message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param command_type Flexifunction command type
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_command_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t command_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, command_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif
#else
	mavlink_flexifunction_command_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.command_type = command_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif
#endif
}

#endif

// MESSAGE FLEXIFUNCTION_COMMAND UNPACKING


/**
 * @brief Get field target_system from flexifunction_command message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field target_component from flexifunction_command message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field command_type from flexifunction_command message
 *
 * @return Flexifunction command type
 */
static inline uint8_t mavlink_msg_flexifunction_command_get_command_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a flexifunction_command message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_command_decode(const mavlink_message_t* msg, mavlink_flexifunction_command_t* flexifunction_command)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_command->target_system = mavlink_msg_flexifunction_command_get_target_system(msg);
	flexifunction_command->target_component = mavlink_msg_flexifunction_command_get_target_component(msg);
	flexifunction_command->command_type = mavlink_msg_flexifunction_command_get_command_type(msg);
#else
	memcpy(flexifunction_command, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_COMMAND_LEN);
#endif
}
