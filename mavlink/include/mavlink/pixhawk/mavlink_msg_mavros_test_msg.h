// MESSAGE MAVROS_TEST_MSG PACKING

#define MAVLINK_MSG_ID_MAVROS_TEST_MSG 182

MAVPACKED(
typedef struct __mavlink_mavros_test_msg_t {
 uint64_t timestamp; /*< Timestamp (milliseconds since system boot)*/
 float test; /*< data*/
}) mavlink_mavros_test_msg_t;

#define MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN 12
#define MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN 12
#define MAVLINK_MSG_ID_182_LEN 12
#define MAVLINK_MSG_ID_182_MIN_LEN 12

#define MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC 212
#define MAVLINK_MSG_ID_182_CRC 212



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAVROS_TEST_MSG { \
	182, \
	"MAVROS_TEST_MSG", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mavros_test_msg_t, timestamp) }, \
         { "test", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mavros_test_msg_t, test) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAVROS_TEST_MSG { \
	"MAVROS_TEST_MSG", \
	2, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mavros_test_msg_t, timestamp) }, \
         { "test", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mavros_test_msg_t, test) }, \
         } \
}
#endif

/**
 * @brief Pack a mavros_test_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp (milliseconds since system boot)
 * @param test data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mavros_test_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, test);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN);
#else
	mavlink_mavros_test_msg_t packet;
	packet.timestamp = timestamp;
	packet.test = test;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAVROS_TEST_MSG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
}

/**
 * @brief Pack a mavros_test_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp (milliseconds since system boot)
 * @param test data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mavros_test_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, test);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN);
#else
	mavlink_mavros_test_msg_t packet;
	packet.timestamp = timestamp;
	packet.test = test;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAVROS_TEST_MSG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
}

/**
 * @brief Encode a mavros_test_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mavros_test_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mavros_test_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mavros_test_msg_t* mavros_test_msg)
{
	return mavlink_msg_mavros_test_msg_pack(system_id, component_id, msg, mavros_test_msg->timestamp, mavros_test_msg->test);
}

/**
 * @brief Encode a mavros_test_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mavros_test_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mavros_test_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mavros_test_msg_t* mavros_test_msg)
{
	return mavlink_msg_mavros_test_msg_pack_chan(system_id, component_id, chan, msg, mavros_test_msg->timestamp, mavros_test_msg->test);
}

/**
 * @brief Send a mavros_test_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp (milliseconds since system boot)
 * @param test data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mavros_test_msg_send(mavlink_channel_t chan, uint64_t timestamp, float test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, test);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG, buf, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
#else
	mavlink_mavros_test_msg_t packet;
	packet.timestamp = timestamp;
	packet.test = test;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG, (const char *)&packet, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
#endif
}

/**
 * @brief Send a mavros_test_msg message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mavros_test_msg_send_struct(mavlink_channel_t chan, const mavlink_mavros_test_msg_t* mavros_test_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mavros_test_msg_send(chan, mavros_test_msg->timestamp, mavros_test_msg->test);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG, (const char *)mavros_test_msg, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mavros_test_msg_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float test)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, test);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG, buf, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
#else
	mavlink_mavros_test_msg_t *packet = (mavlink_mavros_test_msg_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->test = test;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAVROS_TEST_MSG, (const char *)packet, MAVLINK_MSG_ID_MAVROS_TEST_MSG_MIN_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN, MAVLINK_MSG_ID_MAVROS_TEST_MSG_CRC);
#endif
}
#endif

#endif

// MESSAGE MAVROS_TEST_MSG UNPACKING


/**
 * @brief Get field timestamp from mavros_test_msg message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint64_t mavlink_msg_mavros_test_msg_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field test from mavros_test_msg message
 *
 * @return data
 */
static inline float mavlink_msg_mavros_test_msg_get_test(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a mavros_test_msg message into a struct
 *
 * @param msg The message to decode
 * @param mavros_test_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_mavros_test_msg_decode(const mavlink_message_t* msg, mavlink_mavros_test_msg_t* mavros_test_msg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mavros_test_msg->timestamp = mavlink_msg_mavros_test_msg_get_timestamp(msg);
	mavros_test_msg->test = mavlink_msg_mavros_test_msg_get_test(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN? msg->len : MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN;
        memset(mavros_test_msg, 0, MAVLINK_MSG_ID_MAVROS_TEST_MSG_LEN);
	memcpy(mavros_test_msg, _MAV_PAYLOAD(msg), len);
#endif
}
