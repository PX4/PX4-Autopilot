// MESSAGE IMAGE_TRIGGER_CONTROL PACKING

#define MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL 153

typedef struct __mavlink_image_trigger_control_t
{
 uint8_t enable; ///< 0 to disable, 1 to enable
} mavlink_image_trigger_control_t;

#define MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN 1
#define MAVLINK_MSG_ID_153_LEN 1

#define MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC 95
#define MAVLINK_MSG_ID_153_CRC 95



#define MAVLINK_MESSAGE_INFO_IMAGE_TRIGGER_CONTROL { \
	"IMAGE_TRIGGER_CONTROL", \
	1, \
	{  { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_image_trigger_control_t, enable) }, \
         } \
}


/**
 * @brief Pack a image_trigger_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_trigger_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#else
	mavlink_image_trigger_control_t packet;
	packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a image_trigger_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable 0 to disable, 1 to enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_image_trigger_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#else
	mavlink_image_trigger_control_t packet;
	packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a image_trigger_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param image_trigger_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_trigger_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_image_trigger_control_t* image_trigger_control)
{
	return mavlink_msg_image_trigger_control_pack(system_id, component_id, msg, image_trigger_control->enable);
}

/**
 * @brief Encode a image_trigger_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param image_trigger_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_image_trigger_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_image_trigger_control_t* image_trigger_control)
{
	return mavlink_msg_image_trigger_control_pack_chan(system_id, component_id, chan, msg, image_trigger_control->enable);
}

/**
 * @brief Send a image_trigger_control message
 * @param chan MAVLink channel to send the message
 *
 * @param enable 0 to disable, 1 to enable
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_image_trigger_control_send(mavlink_channel_t chan, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN];
	_mav_put_uint8_t(buf, 0, enable);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
#else
	mavlink_image_trigger_control_t packet;
	packet.enable = enable;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_image_trigger_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, enable);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, buf, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
#else
	mavlink_image_trigger_control_t *packet = (mavlink_image_trigger_control_t *)msgbuf;
	packet->enable = enable;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, (const char *)packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL, (const char *)packet, MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE IMAGE_TRIGGER_CONTROL UNPACKING


/**
 * @brief Get field enable from image_trigger_control message
 *
 * @return 0 to disable, 1 to enable
 */
static inline uint8_t mavlink_msg_image_trigger_control_get_enable(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a image_trigger_control message into a struct
 *
 * @param msg The message to decode
 * @param image_trigger_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_image_trigger_control_decode(const mavlink_message_t* msg, mavlink_image_trigger_control_t* image_trigger_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	image_trigger_control->enable = mavlink_msg_image_trigger_control_get_enable(msg);
#else
	memcpy(image_trigger_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IMAGE_TRIGGER_CONTROL_LEN);
#endif
}
