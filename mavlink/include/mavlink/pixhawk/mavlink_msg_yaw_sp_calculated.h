// MESSAGE YAW_SP_CALCULATED PACKING

#define MAVLINK_MSG_ID_YAW_SP_CALCULATED 223

MAVPACKED(
typedef struct __mavlink_yaw_sp_calculated_t {
 uint64_t timestamp; /*< timestamp*/
 float yaw_sp; /*< the calculated yaw set point, in rad (-pi ... pi)*/
}) mavlink_yaw_sp_calculated_t;

#define MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN 12
#define MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN 12
#define MAVLINK_MSG_ID_223_LEN 12
#define MAVLINK_MSG_ID_223_MIN_LEN 12

#define MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC 81
#define MAVLINK_MSG_ID_223_CRC 81



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_YAW_SP_CALCULATED { \
	223, \
	"YAW_SP_CALCULATED", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_yaw_sp_calculated_t, timestamp) }, \
         { "yaw_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_yaw_sp_calculated_t, yaw_sp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_YAW_SP_CALCULATED { \
	"YAW_SP_CALCULATED", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_yaw_sp_calculated_t, timestamp) }, \
         { "yaw_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_yaw_sp_calculated_t, yaw_sp) }, \
         } \
}
#endif

/**
 * @brief Pack a yaw_sp_calculated message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param yaw_sp the calculated yaw set point, in rad (-pi ... pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yaw_sp_calculated_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float yaw_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, yaw_sp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN);
#else
	mavlink_yaw_sp_calculated_t packet;
	packet.timestamp = timestamp;
	packet.yaw_sp = yaw_sp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YAW_SP_CALCULATED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
}

/**
 * @brief Pack a yaw_sp_calculated message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param yaw_sp the calculated yaw set point, in rad (-pi ... pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yaw_sp_calculated_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float yaw_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, yaw_sp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN);
#else
	mavlink_yaw_sp_calculated_t packet;
	packet.timestamp = timestamp;
	packet.yaw_sp = yaw_sp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YAW_SP_CALCULATED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
}

/**
 * @brief Encode a yaw_sp_calculated struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param yaw_sp_calculated C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yaw_sp_calculated_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_yaw_sp_calculated_t* yaw_sp_calculated)
{
	return mavlink_msg_yaw_sp_calculated_pack(system_id, component_id, msg, yaw_sp_calculated->timestamp, yaw_sp_calculated->yaw_sp);
}

/**
 * @brief Encode a yaw_sp_calculated struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yaw_sp_calculated C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yaw_sp_calculated_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_yaw_sp_calculated_t* yaw_sp_calculated)
{
	return mavlink_msg_yaw_sp_calculated_pack_chan(system_id, component_id, chan, msg, yaw_sp_calculated->timestamp, yaw_sp_calculated->yaw_sp);
}

/**
 * @brief Send a yaw_sp_calculated message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param yaw_sp the calculated yaw set point, in rad (-pi ... pi)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_yaw_sp_calculated_send(mavlink_channel_t chan, uint64_t timestamp, float yaw_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, yaw_sp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED, buf, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
#else
	mavlink_yaw_sp_calculated_t packet;
	packet.timestamp = timestamp;
	packet.yaw_sp = yaw_sp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED, (const char *)&packet, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
#endif
}

/**
 * @brief Send a yaw_sp_calculated message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_yaw_sp_calculated_send_struct(mavlink_channel_t chan, const mavlink_yaw_sp_calculated_t* yaw_sp_calculated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_yaw_sp_calculated_send(chan, yaw_sp_calculated->timestamp, yaw_sp_calculated->yaw_sp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED, (const char *)yaw_sp_calculated, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
#endif
}

#if MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_yaw_sp_calculated_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float yaw_sp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, yaw_sp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED, buf, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
#else
	mavlink_yaw_sp_calculated_t *packet = (mavlink_yaw_sp_calculated_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->yaw_sp = yaw_sp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YAW_SP_CALCULATED, (const char *)packet, MAVLINK_MSG_ID_YAW_SP_CALCULATED_MIN_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN, MAVLINK_MSG_ID_YAW_SP_CALCULATED_CRC);
#endif
}
#endif

#endif

// MESSAGE YAW_SP_CALCULATED UNPACKING


/**
 * @brief Get field timestamp from yaw_sp_calculated message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_yaw_sp_calculated_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field yaw_sp from yaw_sp_calculated message
 *
 * @return the calculated yaw set point, in rad (-pi ... pi)
 */
static inline float mavlink_msg_yaw_sp_calculated_get_yaw_sp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a yaw_sp_calculated message into a struct
 *
 * @param msg The message to decode
 * @param yaw_sp_calculated C-struct to decode the message contents into
 */
static inline void mavlink_msg_yaw_sp_calculated_decode(const mavlink_message_t* msg, mavlink_yaw_sp_calculated_t* yaw_sp_calculated)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	yaw_sp_calculated->timestamp = mavlink_msg_yaw_sp_calculated_get_timestamp(msg);
	yaw_sp_calculated->yaw_sp = mavlink_msg_yaw_sp_calculated_get_yaw_sp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN? msg->len : MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN;
        memset(yaw_sp_calculated, 0, MAVLINK_MSG_ID_YAW_SP_CALCULATED_LEN);
	memcpy(yaw_sp_calculated, _MAV_PAYLOAD(msg), len);
#endif
}
