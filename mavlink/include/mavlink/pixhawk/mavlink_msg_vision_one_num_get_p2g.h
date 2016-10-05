// MESSAGE VISION_ONE_NUM_GET_P2G PACKING

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G 164

MAVPACKED(
typedef struct __mavlink_vision_one_num_get_p2g_t {
 uint64_t timestamp; /*< timestamp*/
 uint8_t loop_value; /*< loop number of current task but not 0, 1~5*/
 uint8_t num; /*< gotten number by vision*/
}) mavlink_vision_one_num_get_p2g_t;

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN 10
#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN 10
#define MAVLINK_MSG_ID_164_LEN 10
#define MAVLINK_MSG_ID_164_MIN_LEN 10

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC 111
#define MAVLINK_MSG_ID_164_CRC 111



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_ONE_NUM_GET_P2G { \
	164, \
	"VISION_ONE_NUM_GET_P2G", \
	3, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_one_num_get_p2g_t, timestamp) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_vision_one_num_get_p2g_t, loop_value) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_vision_one_num_get_p2g_t, num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_ONE_NUM_GET_P2G { \
	"VISION_ONE_NUM_GET_P2G", \
	3, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_one_num_get_p2g_t, timestamp) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_vision_one_num_get_p2g_t, loop_value) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_vision_one_num_get_p2g_t, num) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_one_num_get_p2g message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_one_num_get_p2g_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN);
#else
	mavlink_vision_one_num_get_p2g_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
}

/**
 * @brief Pack a vision_one_num_get_p2g message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_one_num_get_p2g_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t loop_value,uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN);
#else
	mavlink_vision_one_num_get_p2g_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
}

/**
 * @brief Encode a vision_one_num_get_p2g struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_one_num_get_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_one_num_get_p2g_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_one_num_get_p2g_t* vision_one_num_get_p2g)
{
	return mavlink_msg_vision_one_num_get_p2g_pack(system_id, component_id, msg, vision_one_num_get_p2g->timestamp, vision_one_num_get_p2g->loop_value, vision_one_num_get_p2g->num);
}

/**
 * @brief Encode a vision_one_num_get_p2g struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_one_num_get_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_one_num_get_p2g_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_one_num_get_p2g_t* vision_one_num_get_p2g)
{
	return mavlink_msg_vision_one_num_get_p2g_pack_chan(system_id, component_id, chan, msg, vision_one_num_get_p2g->timestamp, vision_one_num_get_p2g->loop_value, vision_one_num_get_p2g->num);
}

/**
 * @brief Send a vision_one_num_get_p2g message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_one_num_get_p2g_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G, buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
#else
	mavlink_vision_one_num_get_p2g_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G, (const char *)&packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
#endif
}

/**
 * @brief Send a vision_one_num_get_p2g message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_one_num_get_p2g_send_struct(mavlink_channel_t chan, const mavlink_vision_one_num_get_p2g_t* vision_one_num_get_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_one_num_get_p2g_send(chan, vision_one_num_get_p2g->timestamp, vision_one_num_get_p2g->loop_value, vision_one_num_get_p2g->num);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G, (const char *)vision_one_num_get_p2g, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_one_num_get_p2g_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G, buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
#else
	mavlink_vision_one_num_get_p2g_t *packet = (mavlink_vision_one_num_get_p2g_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->loop_value = loop_value;
	packet->num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G, (const char *)packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_ONE_NUM_GET_P2G UNPACKING


/**
 * @brief Get field timestamp from vision_one_num_get_p2g message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_vision_one_num_get_p2g_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field loop_value from vision_one_num_get_p2g message
 *
 * @return loop number of current task but not 0, 1~5
 */
static inline uint8_t mavlink_msg_vision_one_num_get_p2g_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field num from vision_one_num_get_p2g message
 *
 * @return gotten number by vision
 */
static inline uint8_t mavlink_msg_vision_one_num_get_p2g_get_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a vision_one_num_get_p2g message into a struct
 *
 * @param msg The message to decode
 * @param vision_one_num_get_p2g C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_one_num_get_p2g_decode(const mavlink_message_t* msg, mavlink_vision_one_num_get_p2g_t* vision_one_num_get_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_one_num_get_p2g->timestamp = mavlink_msg_vision_one_num_get_p2g_get_timestamp(msg);
	vision_one_num_get_p2g->loop_value = mavlink_msg_vision_one_num_get_p2g_get_loop_value(msg);
	vision_one_num_get_p2g->num = mavlink_msg_vision_one_num_get_p2g_get_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN? msg->len : MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN;
        memset(vision_one_num_get_p2g, 0, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_P2G_LEN);
	memcpy(vision_one_num_get_p2g, _MAV_PAYLOAD(msg), len);
#endif
}
