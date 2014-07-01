// MESSAGE PATTERN_DETECTED PACKING

#define MAVLINK_MSG_ID_PATTERN_DETECTED 190

typedef struct __mavlink_pattern_detected_t
{
 float confidence; ///< Confidence of detection
 uint8_t type; ///< 0: Pattern, 1: Letter
 char file[100]; ///< Pattern file name
 uint8_t detected; ///< Accepted as true detection, 0 no, 1 yes
} mavlink_pattern_detected_t;

#define MAVLINK_MSG_ID_PATTERN_DETECTED_LEN 106
#define MAVLINK_MSG_ID_190_LEN 106

#define MAVLINK_MSG_ID_PATTERN_DETECTED_CRC 90
#define MAVLINK_MSG_ID_190_CRC 90

#define MAVLINK_MSG_PATTERN_DETECTED_FIELD_FILE_LEN 100

#define MAVLINK_MESSAGE_INFO_PATTERN_DETECTED { \
	"PATTERN_DETECTED", \
	4, \
	{  { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pattern_detected_t, confidence) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_pattern_detected_t, type) }, \
         { "file", NULL, MAVLINK_TYPE_CHAR, 100, 5, offsetof(mavlink_pattern_detected_t, file) }, \
         { "detected", NULL, MAVLINK_TYPE_UINT8_T, 0, 105, offsetof(mavlink_pattern_detected_t, detected) }, \
         } \
}


/**
 * @brief Pack a pattern_detected message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param type 0: Pattern, 1: Letter
 * @param confidence Confidence of detection
 * @param file Pattern file name
 * @param detected Accepted as true detection, 0 no, 1 yes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pattern_detected_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t type, float confidence, const char *file, uint8_t detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PATTERN_DETECTED_LEN];
	_mav_put_float(buf, 0, confidence);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 105, detected);
	_mav_put_char_array(buf, 5, file, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#else
	mavlink_pattern_detected_t packet;
	packet.confidence = confidence;
	packet.type = type;
	packet.detected = detected;
	mav_array_memcpy(packet.file, file, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PATTERN_DETECTED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
}

/**
 * @brief Pack a pattern_detected message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param type 0: Pattern, 1: Letter
 * @param confidence Confidence of detection
 * @param file Pattern file name
 * @param detected Accepted as true detection, 0 no, 1 yes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pattern_detected_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t type,float confidence,const char *file,uint8_t detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PATTERN_DETECTED_LEN];
	_mav_put_float(buf, 0, confidence);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 105, detected);
	_mav_put_char_array(buf, 5, file, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#else
	mavlink_pattern_detected_t packet;
	packet.confidence = confidence;
	packet.type = type;
	packet.detected = detected;
	mav_array_memcpy(packet.file, file, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PATTERN_DETECTED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
}

/**
 * @brief Encode a pattern_detected struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pattern_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pattern_detected_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pattern_detected_t* pattern_detected)
{
	return mavlink_msg_pattern_detected_pack(system_id, component_id, msg, pattern_detected->type, pattern_detected->confidence, pattern_detected->file, pattern_detected->detected);
}

/**
 * @brief Encode a pattern_detected struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pattern_detected C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pattern_detected_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pattern_detected_t* pattern_detected)
{
	return mavlink_msg_pattern_detected_pack_chan(system_id, component_id, chan, msg, pattern_detected->type, pattern_detected->confidence, pattern_detected->file, pattern_detected->detected);
}

/**
 * @brief Send a pattern_detected message
 * @param chan MAVLink channel to send the message
 *
 * @param type 0: Pattern, 1: Letter
 * @param confidence Confidence of detection
 * @param file Pattern file name
 * @param detected Accepted as true detection, 0 no, 1 yes
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pattern_detected_send(mavlink_channel_t chan, uint8_t type, float confidence, const char *file, uint8_t detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PATTERN_DETECTED_LEN];
	_mav_put_float(buf, 0, confidence);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 105, detected);
	_mav_put_char_array(buf, 5, file, 100);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
#else
	mavlink_pattern_detected_t packet;
	packet.confidence = confidence;
	packet.type = type;
	packet.detected = detected;
	mav_array_memcpy(packet.file, file, sizeof(char)*100);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, (const char *)&packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, (const char *)&packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PATTERN_DETECTED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pattern_detected_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t type, float confidence, const char *file, uint8_t detected)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, confidence);
	_mav_put_uint8_t(buf, 4, type);
	_mav_put_uint8_t(buf, 105, detected);
	_mav_put_char_array(buf, 5, file, 100);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, buf, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
#else
	mavlink_pattern_detected_t *packet = (mavlink_pattern_detected_t *)msgbuf;
	packet->confidence = confidence;
	packet->type = type;
	packet->detected = detected;
	mav_array_memcpy(packet->file, file, sizeof(char)*100);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, (const char *)packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN, MAVLINK_MSG_ID_PATTERN_DETECTED_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PATTERN_DETECTED, (const char *)packet, MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PATTERN_DETECTED UNPACKING


/**
 * @brief Get field type from pattern_detected message
 *
 * @return 0: Pattern, 1: Letter
 */
static inline uint8_t mavlink_msg_pattern_detected_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field confidence from pattern_detected message
 *
 * @return Confidence of detection
 */
static inline float mavlink_msg_pattern_detected_get_confidence(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field file from pattern_detected message
 *
 * @return Pattern file name
 */
static inline uint16_t mavlink_msg_pattern_detected_get_file(const mavlink_message_t* msg, char *file)
{
	return _MAV_RETURN_char_array(msg, file, 100,  5);
}

/**
 * @brief Get field detected from pattern_detected message
 *
 * @return Accepted as true detection, 0 no, 1 yes
 */
static inline uint8_t mavlink_msg_pattern_detected_get_detected(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  105);
}

/**
 * @brief Decode a pattern_detected message into a struct
 *
 * @param msg The message to decode
 * @param pattern_detected C-struct to decode the message contents into
 */
static inline void mavlink_msg_pattern_detected_decode(const mavlink_message_t* msg, mavlink_pattern_detected_t* pattern_detected)
{
#if MAVLINK_NEED_BYTE_SWAP
	pattern_detected->confidence = mavlink_msg_pattern_detected_get_confidence(msg);
	pattern_detected->type = mavlink_msg_pattern_detected_get_type(msg);
	mavlink_msg_pattern_detected_get_file(msg, pattern_detected->file);
	pattern_detected->detected = mavlink_msg_pattern_detected_get_detected(msg);
#else
	memcpy(pattern_detected, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PATTERN_DETECTED_LEN);
#endif
}
