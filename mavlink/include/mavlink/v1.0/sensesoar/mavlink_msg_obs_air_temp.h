// MESSAGE OBS_AIR_TEMP PACKING

#define MAVLINK_MSG_ID_OBS_AIR_TEMP 183

typedef struct __mavlink_obs_air_temp_t
{
 float airT; ///< 
                
            
} mavlink_obs_air_temp_t;

#define MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN 4
#define MAVLINK_MSG_ID_183_LEN 4

#define MAVLINK_MSG_ID_OBS_AIR_TEMP_CRC 248
#define MAVLINK_MSG_ID_183_CRC 248



#define MAVLINK_MESSAGE_INFO_OBS_AIR_TEMP { \
	"OBS_AIR_TEMP", \
	1, \
	{  { "airT", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_obs_air_temp_t, airT) }, \
         } \
}


/**
 * @brief Pack a obs_air_temp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airT 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_air_temp_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float airT)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN];
	_mav_put_float(buf, 0, airT);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#else
	mavlink_obs_air_temp_t packet;
	packet.airT = airT;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_AIR_TEMP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN, MAVLINK_MSG_ID_OBS_AIR_TEMP_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif
}

/**
 * @brief Pack a obs_air_temp message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param airT 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_air_temp_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float airT)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN];
	_mav_put_float(buf, 0, airT);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#else
	mavlink_obs_air_temp_t packet;
	packet.airT = airT;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_AIR_TEMP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN, MAVLINK_MSG_ID_OBS_AIR_TEMP_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif
}

/**
 * @brief Encode a obs_air_temp struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_air_temp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_air_temp_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_air_temp_t* obs_air_temp)
{
	return mavlink_msg_obs_air_temp_pack(system_id, component_id, msg, obs_air_temp->airT);
}

/**
 * @brief Send a obs_air_temp message
 * @param chan MAVLink channel to send the message
 *
 * @param airT 
                
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_air_temp_send(mavlink_channel_t chan, float airT)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN];
	_mav_put_float(buf, 0, airT);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_AIR_TEMP, buf, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN, MAVLINK_MSG_ID_OBS_AIR_TEMP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_AIR_TEMP, buf, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif
#else
	mavlink_obs_air_temp_t packet;
	packet.airT = airT;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_AIR_TEMP, (const char *)&packet, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN, MAVLINK_MSG_ID_OBS_AIR_TEMP_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_AIR_TEMP, (const char *)&packet, MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif
#endif
}

#endif

// MESSAGE OBS_AIR_TEMP UNPACKING


/**
 * @brief Get field airT from obs_air_temp message
 *
 * @return 
                
            
 */
static inline float mavlink_msg_obs_air_temp_get_airT(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a obs_air_temp message into a struct
 *
 * @param msg The message to decode
 * @param obs_air_temp C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_air_temp_decode(const mavlink_message_t* msg, mavlink_obs_air_temp_t* obs_air_temp)
{
#if MAVLINK_NEED_BYTE_SWAP
	obs_air_temp->airT = mavlink_msg_obs_air_temp_get_airT(msg);
#else
	memcpy(obs_air_temp, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OBS_AIR_TEMP_LEN);
#endif
}
