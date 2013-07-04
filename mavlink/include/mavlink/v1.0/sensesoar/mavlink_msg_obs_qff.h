// MESSAGE OBS_QFF PACKING

#define MAVLINK_MSG_ID_OBS_QFF 182

typedef struct __mavlink_obs_qff_t
{
 float qff; ///< 
                
            
} mavlink_obs_qff_t;

#define MAVLINK_MSG_ID_OBS_QFF_LEN 4
#define MAVLINK_MSG_ID_182_LEN 4

#define MAVLINK_MSG_ID_OBS_QFF_CRC 24
#define MAVLINK_MSG_ID_182_CRC 24



#define MAVLINK_MESSAGE_INFO_OBS_QFF { \
	"OBS_QFF", \
	1, \
	{  { "qff", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_obs_qff_t, qff) }, \
         } \
}


/**
 * @brief Pack a obs_qff message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param qff 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_qff_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float qff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_QFF_LEN];
	_mav_put_float(buf, 0, qff);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_QFF_LEN);
#else
	mavlink_obs_qff_t packet;
	packet.qff = qff;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_QFF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_QFF_LEN, MAVLINK_MSG_ID_OBS_QFF_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif
}

/**
 * @brief Pack a obs_qff message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param qff 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_qff_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float qff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_QFF_LEN];
	_mav_put_float(buf, 0, qff);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_QFF_LEN);
#else
	mavlink_obs_qff_t packet;
	packet.qff = qff;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_QFF;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_QFF_LEN, MAVLINK_MSG_ID_OBS_QFF_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif
}

/**
 * @brief Encode a obs_qff struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_qff C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_qff_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_qff_t* obs_qff)
{
	return mavlink_msg_obs_qff_pack(system_id, component_id, msg, obs_qff->qff);
}

/**
 * @brief Send a obs_qff message
 * @param chan MAVLink channel to send the message
 *
 * @param qff 
                
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_qff_send(mavlink_channel_t chan, float qff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_QFF_LEN];
	_mav_put_float(buf, 0, qff);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_QFF, buf, MAVLINK_MSG_ID_OBS_QFF_LEN, MAVLINK_MSG_ID_OBS_QFF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_QFF, buf, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif
#else
	mavlink_obs_qff_t packet;
	packet.qff = qff;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_QFF, (const char *)&packet, MAVLINK_MSG_ID_OBS_QFF_LEN, MAVLINK_MSG_ID_OBS_QFF_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_QFF, (const char *)&packet, MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif
#endif
}

#endif

// MESSAGE OBS_QFF UNPACKING


/**
 * @brief Get field qff from obs_qff message
 *
 * @return 
                
            
 */
static inline float mavlink_msg_obs_qff_get_qff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a obs_qff message into a struct
 *
 * @param msg The message to decode
 * @param obs_qff C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_qff_decode(const mavlink_message_t* msg, mavlink_obs_qff_t* obs_qff)
{
#if MAVLINK_NEED_BYTE_SWAP
	obs_qff->qff = mavlink_msg_obs_qff_get_qff(msg);
#else
	memcpy(obs_qff, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OBS_QFF_LEN);
#endif
}
