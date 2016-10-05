// MESSAGE VISION_NUM_SCAN_M2P PACKING

#define MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P 161

MAVPACKED(
typedef struct __mavlink_vision_num_scan_m2p_t {
 uint64_t timestamp; /*< timestamp*/
 float board_x; /*< current board number X position (NED), in m*/
 float board_y; /*< current board number Y position (NED), in m*/
 float board_z; /*< current board number Z position (NED), in m*/
 uint8_t board_num; /*< current board number*/
 uint8_t board_valid; /*< whether this number is gotten*/
}) mavlink_vision_num_scan_m2p_t;

#define MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN 22
#define MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN 22
#define MAVLINK_MSG_ID_161_LEN 22
#define MAVLINK_MSG_ID_161_MIN_LEN 22

#define MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC 68
#define MAVLINK_MSG_ID_161_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_VISION_NUM_SCAN_M2P { \
	161, \
	"VISION_NUM_SCAN_M2P", \
	6, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_num_scan_m2p_t, timestamp) }, \
         { "board_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_num_scan_m2p_t, board_x) }, \
         { "board_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_num_scan_m2p_t, board_y) }, \
         { "board_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_num_scan_m2p_t, board_z) }, \
         { "board_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_vision_num_scan_m2p_t, board_num) }, \
         { "board_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_vision_num_scan_m2p_t, board_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_VISION_NUM_SCAN_M2P { \
	"VISION_NUM_SCAN_M2P", \
	6, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_vision_num_scan_m2p_t, timestamp) }, \
         { "board_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_vision_num_scan_m2p_t, board_x) }, \
         { "board_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_vision_num_scan_m2p_t, board_y) }, \
         { "board_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_vision_num_scan_m2p_t, board_z) }, \
         { "board_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_vision_num_scan_m2p_t, board_num) }, \
         { "board_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_vision_num_scan_m2p_t, board_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a vision_num_scan_m2p message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param board_num current board number
 * @param board_x current board number X position (NED), in m
 * @param board_y current board number Y position (NED), in m
 * @param board_z current board number Z position (NED), in m
 * @param board_valid whether this number is gotten
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_num_scan_m2p_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t board_num, float board_x, float board_y, float board_z, uint8_t board_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, board_x);
	_mav_put_float(buf, 12, board_y);
	_mav_put_float(buf, 16, board_z);
	_mav_put_uint8_t(buf, 20, board_num);
	_mav_put_uint8_t(buf, 21, board_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN);
#else
	mavlink_vision_num_scan_m2p_t packet;
	packet.timestamp = timestamp;
	packet.board_x = board_x;
	packet.board_y = board_y;
	packet.board_z = board_z;
	packet.board_num = board_num;
	packet.board_valid = board_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
}

/**
 * @brief Pack a vision_num_scan_m2p message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param board_num current board number
 * @param board_x current board number X position (NED), in m
 * @param board_y current board number Y position (NED), in m
 * @param board_z current board number Z position (NED), in m
 * @param board_valid whether this number is gotten
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vision_num_scan_m2p_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t board_num,float board_x,float board_y,float board_z,uint8_t board_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, board_x);
	_mav_put_float(buf, 12, board_y);
	_mav_put_float(buf, 16, board_z);
	_mav_put_uint8_t(buf, 20, board_num);
	_mav_put_uint8_t(buf, 21, board_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN);
#else
	mavlink_vision_num_scan_m2p_t packet;
	packet.timestamp = timestamp;
	packet.board_x = board_x;
	packet.board_y = board_y;
	packet.board_z = board_z;
	packet.board_num = board_num;
	packet.board_valid = board_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
}

/**
 * @brief Encode a vision_num_scan_m2p struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vision_num_scan_m2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_num_scan_m2p_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vision_num_scan_m2p_t* vision_num_scan_m2p)
{
	return mavlink_msg_vision_num_scan_m2p_pack(system_id, component_id, msg, vision_num_scan_m2p->timestamp, vision_num_scan_m2p->board_num, vision_num_scan_m2p->board_x, vision_num_scan_m2p->board_y, vision_num_scan_m2p->board_z, vision_num_scan_m2p->board_valid);
}

/**
 * @brief Encode a vision_num_scan_m2p struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vision_num_scan_m2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vision_num_scan_m2p_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vision_num_scan_m2p_t* vision_num_scan_m2p)
{
	return mavlink_msg_vision_num_scan_m2p_pack_chan(system_id, component_id, chan, msg, vision_num_scan_m2p->timestamp, vision_num_scan_m2p->board_num, vision_num_scan_m2p->board_x, vision_num_scan_m2p->board_y, vision_num_scan_m2p->board_z, vision_num_scan_m2p->board_valid);
}

/**
 * @brief Send a vision_num_scan_m2p message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param board_num current board number
 * @param board_x current board number X position (NED), in m
 * @param board_y current board number Y position (NED), in m
 * @param board_z current board number Z position (NED), in m
 * @param board_valid whether this number is gotten
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vision_num_scan_m2p_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t board_num, float board_x, float board_y, float board_z, uint8_t board_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, board_x);
	_mav_put_float(buf, 12, board_y);
	_mav_put_float(buf, 16, board_z);
	_mav_put_uint8_t(buf, 20, board_num);
	_mav_put_uint8_t(buf, 21, board_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P, buf, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
#else
	mavlink_vision_num_scan_m2p_t packet;
	packet.timestamp = timestamp;
	packet.board_x = board_x;
	packet.board_y = board_y;
	packet.board_z = board_z;
	packet.board_num = board_num;
	packet.board_valid = board_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P, (const char *)&packet, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
#endif
}

/**
 * @brief Send a vision_num_scan_m2p message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_vision_num_scan_m2p_send_struct(mavlink_channel_t chan, const mavlink_vision_num_scan_m2p_t* vision_num_scan_m2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_vision_num_scan_m2p_send(chan, vision_num_scan_m2p->timestamp, vision_num_scan_m2p->board_num, vision_num_scan_m2p->board_x, vision_num_scan_m2p->board_y, vision_num_scan_m2p->board_z, vision_num_scan_m2p->board_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P, (const char *)vision_num_scan_m2p, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
#endif
}

#if MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vision_num_scan_m2p_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t board_num, float board_x, float board_y, float board_z, uint8_t board_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, board_x);
	_mav_put_float(buf, 12, board_y);
	_mav_put_float(buf, 16, board_z);
	_mav_put_uint8_t(buf, 20, board_num);
	_mav_put_uint8_t(buf, 21, board_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P, buf, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
#else
	mavlink_vision_num_scan_m2p_t *packet = (mavlink_vision_num_scan_m2p_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->board_x = board_x;
	packet->board_y = board_y;
	packet->board_z = board_z;
	packet->board_num = board_num;
	packet->board_valid = board_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P, (const char *)packet, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_MIN_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_CRC);
#endif
}
#endif

#endif

// MESSAGE VISION_NUM_SCAN_M2P UNPACKING


/**
 * @brief Get field timestamp from vision_num_scan_m2p message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_vision_num_scan_m2p_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field board_num from vision_num_scan_m2p message
 *
 * @return current board number
 */
static inline uint8_t mavlink_msg_vision_num_scan_m2p_get_board_num(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field board_x from vision_num_scan_m2p message
 *
 * @return current board number X position (NED), in m
 */
static inline float mavlink_msg_vision_num_scan_m2p_get_board_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field board_y from vision_num_scan_m2p message
 *
 * @return current board number Y position (NED), in m
 */
static inline float mavlink_msg_vision_num_scan_m2p_get_board_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field board_z from vision_num_scan_m2p message
 *
 * @return current board number Z position (NED), in m
 */
static inline float mavlink_msg_vision_num_scan_m2p_get_board_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field board_valid from vision_num_scan_m2p message
 *
 * @return whether this number is gotten
 */
static inline uint8_t mavlink_msg_vision_num_scan_m2p_get_board_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a vision_num_scan_m2p message into a struct
 *
 * @param msg The message to decode
 * @param vision_num_scan_m2p C-struct to decode the message contents into
 */
static inline void mavlink_msg_vision_num_scan_m2p_decode(const mavlink_message_t* msg, mavlink_vision_num_scan_m2p_t* vision_num_scan_m2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	vision_num_scan_m2p->timestamp = mavlink_msg_vision_num_scan_m2p_get_timestamp(msg);
	vision_num_scan_m2p->board_x = mavlink_msg_vision_num_scan_m2p_get_board_x(msg);
	vision_num_scan_m2p->board_y = mavlink_msg_vision_num_scan_m2p_get_board_y(msg);
	vision_num_scan_m2p->board_z = mavlink_msg_vision_num_scan_m2p_get_board_z(msg);
	vision_num_scan_m2p->board_num = mavlink_msg_vision_num_scan_m2p_get_board_num(msg);
	vision_num_scan_m2p->board_valid = mavlink_msg_vision_num_scan_m2p_get_board_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN? msg->len : MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN;
        memset(vision_num_scan_m2p, 0, MAVLINK_MSG_ID_VISION_NUM_SCAN_M2P_LEN);
	memcpy(vision_num_scan_m2p, _MAV_PAYLOAD(msg), len);
#endif
}
