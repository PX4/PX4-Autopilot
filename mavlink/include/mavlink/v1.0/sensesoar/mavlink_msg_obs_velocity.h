// MESSAGE OBS_VELOCITY PACKING

#define MAVLINK_MSG_ID_OBS_VELOCITY 172

typedef struct __mavlink_obs_velocity_t
{
 float vel[3]; ///< 
                
            
} mavlink_obs_velocity_t;

#define MAVLINK_MSG_ID_OBS_VELOCITY_LEN 12
#define MAVLINK_MSG_ID_172_LEN 12

#define MAVLINK_MSG_ID_OBS_VELOCITY_CRC 108
#define MAVLINK_MSG_ID_172_CRC 108

#define MAVLINK_MSG_OBS_VELOCITY_FIELD_VEL_LEN 3

#define MAVLINK_MESSAGE_INFO_OBS_VELOCITY { \
	"OBS_VELOCITY", \
	1, \
	{  { "vel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_obs_velocity_t, vel) }, \
         } \
}


/**
 * @brief Pack a obs_velocity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vel 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_velocity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_VELOCITY_LEN];

	_mav_put_float_array(buf, 0, vel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#else
	mavlink_obs_velocity_t packet;

	mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_VELOCITY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_VELOCITY_LEN, MAVLINK_MSG_ID_OBS_VELOCITY_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif
}

/**
 * @brief Pack a obs_velocity message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param vel 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_obs_velocity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_VELOCITY_LEN];

	_mav_put_float_array(buf, 0, vel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#else
	mavlink_obs_velocity_t packet;

	mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OBS_VELOCITY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_VELOCITY_LEN, MAVLINK_MSG_ID_OBS_VELOCITY_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif
}

/**
 * @brief Encode a obs_velocity struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param obs_velocity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_obs_velocity_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_obs_velocity_t* obs_velocity)
{
	return mavlink_msg_obs_velocity_pack(system_id, component_id, msg, obs_velocity->vel);
}

/**
 * @brief Send a obs_velocity message
 * @param chan MAVLink channel to send the message
 *
 * @param vel 
                
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_obs_velocity_send(mavlink_channel_t chan, const float *vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OBS_VELOCITY_LEN];

	_mav_put_float_array(buf, 0, vel, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_VELOCITY, buf, MAVLINK_MSG_ID_OBS_VELOCITY_LEN, MAVLINK_MSG_ID_OBS_VELOCITY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_VELOCITY, buf, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif
#else
	mavlink_obs_velocity_t packet;

	mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_VELOCITY, (const char *)&packet, MAVLINK_MSG_ID_OBS_VELOCITY_LEN, MAVLINK_MSG_ID_OBS_VELOCITY_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OBS_VELOCITY, (const char *)&packet, MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif
#endif
}

#endif

// MESSAGE OBS_VELOCITY UNPACKING


/**
 * @brief Get field vel from obs_velocity message
 *
 * @return 
                
            
 */
static inline uint16_t mavlink_msg_obs_velocity_get_vel(const mavlink_message_t* msg, float *vel)
{
	return _MAV_RETURN_float_array(msg, vel, 3,  0);
}

/**
 * @brief Decode a obs_velocity message into a struct
 *
 * @param msg The message to decode
 * @param obs_velocity C-struct to decode the message contents into
 */
static inline void mavlink_msg_obs_velocity_decode(const mavlink_message_t* msg, mavlink_obs_velocity_t* obs_velocity)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_obs_velocity_get_vel(msg, obs_velocity->vel);
#else
	memcpy(obs_velocity, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OBS_VELOCITY_LEN);
#endif
}
