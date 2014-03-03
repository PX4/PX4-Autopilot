// MESSAGE STATUSTEXT PACKING

#define MAVLINK_MSG_ID_STATUSTEXT 253

#define MAVLINK_MSG_ID_STATUSTEXT_LEN 51
#define MAVLINK_MSG_ID_253_LEN MAVLINK_MSG_ID_STATUSTEXT_LEN

#define MAVLINK_MSG_ID_STATUSTEXT_CRC 83
#define MAVLINK_MSG_ID_253_CRC MAVLINK_MSG_ID_STATUSTEXT_CRC

#define MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN 49		///< Length in bytes

#define MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES 1

typedef struct __mavlink_statustext_t
{
 uint8_t severity; ///< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 char text[49]; ///< Status text message, without null termination character
 uint8_t _field_len_text; ///< Length of status text message
} mavlink_statustext_t;

#define MAVLINK_MESSAGE_INFO_STATUSTEXT { \
	"STATUSTEXT", \
	3, \
	{  { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_statustext_t, severity) }, \
         { "text", NULL, MAVLINK_TYPE_CHAR, -1, -1, -1 }, \
         { "_field_len_text", NULL, MAVLINK_TYPE_UINT8_T, 0, -1, 1 } \
         } \
}


/**
 * @brief Pack a statustext message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t severity, const char *text)
{
	uint8_t _field_len_text;
	char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
	_mav_put_uint8_t(buf, 0, severity);
	_field_len_text = mav_string_copy(&buf[1], text, sizeof(char)*49);
	_mav_put_uint8_t(buf, 1 + _field_len_text, _field_len_text);
	uint16_t msglen = 1 + _field_len_text + MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, msglen);

	msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
	return mavlink_finalize_message(msg, system_id, component_id, msglen, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Pack a statustext message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_statustext_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t severity,const char *text)
{
	uint8_t _field_len_text;
	char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
	_mav_put_uint8_t(buf, 0, severity);
	_field_len_text = mav_string_copy(&buf[1], text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
	_mav_put_uint8_t(buf, 1 + _field_len_text, _field_len_text);
	uint16_t msglen = 1 + _field_len_text + MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, msglen);

	msg->msgid = MAVLINK_MSG_ID_STATUSTEXT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, msglen, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

/**
 * @brief Encode a statustext struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
	return mavlink_msg_statustext_pack(system_id, component_id, msg, statustext->severity, statustext->text);
}

/**
 * @brief Encode a statustext struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param statustext C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_statustext_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_statustext_t* statustext)
{
	return mavlink_msg_statustext_pack_chan(system_id, component_id, chan, msg, statustext->severity, statustext->text);
}

/**
 * @brief Send a statustext message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param text Status text message, without null termination character
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_statustext_send(mavlink_channel_t chan, uint8_t severity, const char *text)
{
	uint8_t _field_len_text;
	char buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];
	_mav_put_uint8_t(buf, 0, severity);
	_field_len_text = mav_string_copy(&buf[1], text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
	_mav_put_uint8_t(buf, 1 + _field_len_text, _field_len_text);
	uint16_t msglen = 1 + _field_len_text + MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, msglen);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUSTEXT, buf, msglen, MAVLINK_MSG_ID_STATUSTEXT_CRC);
}

#endif

// MESSAGE STATUSTEXT UNPACKING


/**
 * @brief Get field severity from statustext message
 *
 * @return Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 */
static inline uint8_t mavlink_msg_statustext_get_severity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field text from statustext message
 *
 * @return Status text message, without null termination character
 */
static inline uint16_t mavlink_msg_statustext_get_text(const mavlink_message_t* msg, char *text)
{
	uint8_t len = _MAV_RETURN_uint8_t(msg, msg->len - MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES + 0);
	return _MAV_RETURN_char_array(msg, text, len,  1);
}

/**
 * @brief Decode a statustext message into a struct
 *
 * @param msg The message to decode
 * @param statustext C-struct to decode the message contents into
 */
static inline void mavlink_msg_statustext_decode(const mavlink_message_t* msg, mavlink_statustext_t* statustext)
{
	statustext->_field_len_text = _MAV_RETURN_uint8_t(msg, msg->len - MAVLINK_MSG_STATUSTEXT_VARLEN_BYTES + 0);
	statustext->severity = mavlink_msg_statustext_get_severity(msg);
	/* copy the string, but at maximum to the full field length or the announced string length, depending on which is shorter */
	strncpy(statustext->text, _MAV_PAYLOAD(msg+1), (statustext->_field_len_text < MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN) ? statustext->_field_len_text : MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
}
