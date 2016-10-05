// MESSAGE VISION_COMMAND PACKING

#define MAVLINK_MSG_ID_VISION_COMMAND 215

MAVPACKED(
typedef struct __mavlink_vision_command_t {
 uint8_t vision_cmd; /*< vision control interface for handshack */
}) mavlink_vision_command_t;

#define MAVLINK_MSG_ID_VISION_COMMAND_LEN 1
#define MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN 1
#define MAVLINK_MSG_ID_215_LEN 1
#define MAVLINK_MSG_ID_215_MIN_LEN 1

#define MAVLINK_MSG_ID_VISION_COMMAND_CRC 114
#define MAVLINK_MSG_ID_215_CRC 114



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_COMMAND { \
	215, \
	"VISION_COMMAND", \
	1, \
	{  { "vision_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_vision_command_t, vision_cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_COMMAND { \
	"VISION_COMMAND", \
	1, \
	{  { "vision_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_vision_command_t, vision_cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vision_cmd vision control interface for handshack 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t vision_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, vision_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_COMMAND_LEN);
#else
	mavlink_vision_command_t packet;
	packet.vision_cmd = vision_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
}

/**
 * @brief Pack a vision_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_cmd vision control interface for handshack 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t vision_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, vision_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_COMMAND_LEN);
#else
	mavlink_vision_command_t packet;
	packet.vision_cmd = vision_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_COMMAND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
}

/**
 * @brief Encode a vision_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_command_t* vision_command)
{
	return mavlink_msg_vision_command_pack(system_id, component_id, msg, vision_command->vision_cmd);
}

/**
 * @brief Encode a vision_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_command_t* vision_command)
{
	return mavlink_msg_vision_command_pack_chan(system_id, component_id, chan, msg, vision_command->vision_cmd);
}

/**
 * @brief Send a vision_command message
 * @param chan MAVLink channel to send the message
 *
 * @param vision_cmd vision control interface for handshack 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_command_send(mavlink_channel_t chan, uint8_t vision_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_COMMAND_LEN];
	_mav_put_uint8_t(buf, 0, vision_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_COMMAND, buf, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
#else
	mavlink_vision_command_t packet;
	packet.vision_cmd = vision_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
#endif
}

/**
 * @brief Send a vision_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_command_send_struct(mavlink_channel_t chan, const mavlink_vision_command_t* vision_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_command_send(chan, vision_command->vision_cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_COMMAND, (const char *)vision_command, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t vision_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, vision_cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_COMMAND, buf, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
#else
	mavlink_vision_command_t *packet = (mavlink_vision_command_t *)msgbuf;
	packet->vision_cmd = vision_cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_COMMAND, (const char *)packet, MAVLINK_MSG_ID_VISION_COMMAND_MIN_LEN, MAVLINK_MSG_ID_VISION_COMMAND_LEN, MAVLINK_MSG_ID_VISION_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_COMMAND UNPACKING


/**
 * @brief Get field vision_cmd from vision_command message
 *
 * @return vision control interface for handshack 
 */
static inline uint8_t mavlink_msg_vision_command_get_vision_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a vision_command message into a struct
 *
 * @param msg The message to decode
 * @param vision_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_command_decode(const mavlink_message_t* msg, mavlink_vision_command_t* vision_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_command->vision_cmd = mavlink_msg_vision_command_get_vision_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_VISION_COMMAND_LEN;
        memset(vision_command, 0, MAVLINK_MSG_ID_VISION_COMMAND_LEN);
	memcpy(vision_command, _MAV_PAYLOAD(msg), len);
#endif
}
