// MESSAGE VISION_ONE_NUM_GET_M2P PACKING

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P 163

MAVPACKED(
typedef struct __mavlink_vision_one_num_get_m2p_t {
 uint64_t timestamp; /*< timestamp*/
 uint8_t loop_value; /*< loop number of current task but not 0, 1~5*/
 uint8_t num; /*< gotten number by vision*/
}) mavlink_vision_one_num_get_m2p_t;

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN 10
#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN 10
#define MAVLINK_MSG_ID_163_LEN 10
#define MAVLINK_MSG_ID_163_MIN_LEN 10

#define MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC 142
#define MAVLINK_MSG_ID_163_CRC 142



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_ONE_NUM_GET_M2P { \
	163, \
	"VISION_ONE_NUM_GET_M2P", \
	3, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_one_num_get_m2p_t, timestamp) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_vision_one_num_get_m2p_t, loop_value) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_vision_one_num_get_m2p_t, num) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_ONE_NUM_GET_M2P { \
	"VISION_ONE_NUM_GET_M2P", \
	3, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_one_num_get_m2p_t, timestamp) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_vision_one_num_get_m2p_t, loop_value) }, \
         { "num", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_vision_one_num_get_m2p_t, num) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_one_num_get_m2p message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_one_num_get_m2p_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN);
#else
	mavlink_vision_one_num_get_m2p_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
}

/**
 * @brief Pack a vision_one_num_get_m2p message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_one_num_get_m2p_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t loop_value,uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN);
#else
	mavlink_vision_one_num_get_m2p_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
}

/**
 * @brief Encode a vision_one_num_get_m2p struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_one_num_get_m2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_one_num_get_m2p_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_one_num_get_m2p_t* vision_one_num_get_m2p)
{
	return mavlink_msg_vision_one_num_get_m2p_pack(system_id, component_id, msg, vision_one_num_get_m2p->timestamp, vision_one_num_get_m2p->loop_value, vision_one_num_get_m2p->num);
}

/**
 * @brief Encode a vision_one_num_get_m2p struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_one_num_get_m2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_one_num_get_m2p_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_one_num_get_m2p_t* vision_one_num_get_m2p)
{
	return mavlink_msg_vision_one_num_get_m2p_pack_chan(system_id, component_id, chan, msg, vision_one_num_get_m2p->timestamp, vision_one_num_get_m2p->loop_value, vision_one_num_get_m2p->num);
}

/**
 * @brief Send a vision_one_num_get_m2p message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param loop_value loop number of current task but not 0, 1~5
 * @param num gotten number by vision
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_one_num_get_m2p_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P, buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
#else
	mavlink_vision_one_num_get_m2p_t packet;
	packet.timestamp = timestamp;
	packet.loop_value = loop_value;
	packet.num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P, (const char *)&packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
#endif
}

/**
 * @brief Send a vision_one_num_get_m2p message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_one_num_get_m2p_send_struct(mavlink_channel_t chan, const mavlink_vision_one_num_get_m2p_t* vision_one_num_get_m2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_one_num_get_m2p_send(chan, vision_one_num_get_m2p->timestamp, vision_one_num_get_m2p->loop_value, vision_one_num_get_m2p->num);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P, (const char *)vision_one_num_get_m2p, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_one_num_get_m2p_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t loop_value, uint8_t num)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint8_t(buf, 8, loop_value);
	_mav_put_uint8_t(buf, 9, num);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P, buf, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
#else
	mavlink_vision_one_num_get_m2p_t *packet = (mavlink_vision_one_num_get_m2p_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->loop_value = loop_value;
	packet->num = num;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P, (const char *)packet, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_ONE_NUM_GET_M2P UNPACKING


/**
 * @brief Get field timestamp from vision_one_num_get_m2p message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_vision_one_num_get_m2p_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field loop_value from vision_one_num_get_m2p message
 *
 * @return loop number of current task but not 0, 1~5
 */
static inline uint8_t mavlink_msg_vision_one_num_get_m2p_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field num from vision_one_num_get_m2p message
 *
 * @return gotten number by vision
 */
static inline uint8_t mavlink_msg_vision_one_num_get_m2p_get_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a vision_one_num_get_m2p message into a struct
 *
 * @param msg The message to decode
 * @param vision_one_num_get_m2p C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_one_num_get_m2p_decode(const mavlink_message_t* msg, mavlink_vision_one_num_get_m2p_t* vision_one_num_get_m2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_one_num_get_m2p->timestamp = mavlink_msg_vision_one_num_get_m2p_get_timestamp(msg);
	vision_one_num_get_m2p->loop_value = mavlink_msg_vision_one_num_get_m2p_get_loop_value(msg);
	vision_one_num_get_m2p->num = mavlink_msg_vision_one_num_get_m2p_get_num(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN? msg->len : MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN;
        memset(vision_one_num_get_m2p, 0, MAVLINK_MSG_ID_VISION_ONE_NUM_GET_M2P_LEN);
	memcpy(vision_one_num_get_m2p, _MAV_PAYLOAD(msg), len);
#endif
}
