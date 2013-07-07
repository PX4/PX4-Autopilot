// MESSAGE OBS_WIND PACKING

#define MAVLINK_MSG_ID_OBS_WIND 176

typedef struct __mavlink_obs_wind_t
{
 float wind[3]; ///< 
                
            
} mavlink_obs_wind_t;

#define MAVLINK_MSG_ID_OBS_WIND_LEN 12
#define MAVLINK_MSG_ID_176_LEN 12

#define MAVLINK_MSG_ID_OBS_WIND_CRC 16
#define MAVLINK_MSG_ID_176_CRC 16

#define MAVLINK_MSG_OBS_WIND_FIELD_WIND_LEN 3

#define MAVLINK_MESSAGE_INFO_OBS_WIND { \
	"OBS_WIND", \
	1, \
	{  { "wind", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_obs_wind_t, wind) }, \
         } \
}


/**
 * @brief Pack a obs_wind message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wind 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_wind_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_WIND_LEN];

	_mav_put_float_array(buf, 0, wind, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_WIND_LEN);
#else
	mavlink_obs_wind_t packet;

	mav_array_memcpy(packet.wind, wind, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_WIND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_WIND_LEN, MAVLINK_MSG_ID_OBS_WIND_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif
}

/**
 * @brief Pack a obs_wind message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param wind 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_wind_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_WIND_LEN];

	_mav_put_float_array(buf, 0, wind, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_WIND_LEN);
#else
	mavlink_obs_wind_t packet;

	mav_array_memcpy(packet.wind, wind, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_WIND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_WIND_LEN, MAVLINK_MSG_ID_OBS_WIND_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif
}

/**
 * @brief Encode a obs_wind struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_wind C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_wind_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_wind_t* obs_wind)
{
	return mavlink_msg_obs_wind_pack(system_id, component_id, msg, obs_wind->wind);
}

/**
 * @brief Send a obs_wind message
 * @param chan MAVLink channel to send the message
 *
 * @param wind 
                
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_wind_send(mavlink_channel_t chan, const float *wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_WIND_LEN];

	_mav_put_float_array(buf, 0, wind, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_WIND, buf, MAVLINK_MSG_ID_OBS_WIND_LEN, MAVLINK_MSG_ID_OBS_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_WIND, buf, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif
#else
	mavlink_obs_wind_t packet;

	mav_array_memcpy(packet.wind, wind, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_WIND, (const char *)&packet, MAVLINK_MSG_ID_OBS_WIND_LEN, MAVLINK_MSG_ID_OBS_WIND_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_WIND, (const char *)&packet, MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif
#endif
}

#endif

// MESSAGE OBS_WIND UNPACKING


/**
 * @brief Get field wind from obs_wind message
 *
 * @return 
                
            
 */
static inline uint16_t mavlink_msg_obs_wind_get_wind(const mavlink_message_t* msg, float *wind)
{
	return _MAV_RETURN_float_array(msg, wind, 3,  0);
}

/**
 * @brief Decode a obs_wind message into a struct
 *
 * @param msg The message to decode
 * @param obs_wind C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_wind_decode(const mavlink_message_t* msg, mavlink_obs_wind_t* obs_wind)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_obs_wind_get_wind(msg, obs_wind->wind);
#else
	memcpy(obs_wind, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OBS_WIND_LEN);
#endif
}
