// MESSAGE FILT_ROT_VEL PACKING

#define MAVLINK_MSG_ID_FILT_ROT_VEL 184

typedef struct __mavlink_filt_rot_vel_t
{
 float rotVel[3]; ///< 
                
            
} mavlink_filt_rot_vel_t;

#define MAVLINK_MSG_ID_FILT_ROT_VEL_LEN 12
#define MAVLINK_MSG_ID_184_LEN 12

#define MAVLINK_MSG_ID_FILT_ROT_VEL_CRC 79
#define MAVLINK_MSG_ID_184_CRC 79

#define MAVLINK_MSG_FILT_ROT_VEL_FIELD_ROTVEL_LEN 3

#define MAVLINK_MESSAGE_INFO_FILT_ROT_VEL { \
	"FILT_ROT_VEL", \
	1, \
	{  { "rotVel", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_filt_rot_vel_t, rotVel) }, \
         } \
}


/**
 * @brief Pack a filt_rot_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rotVel 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_filt_rot_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *rotVel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILT_ROT_VEL_LEN];

	_mav_put_float_array(buf, 0, rotVel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#else
	mavlink_filt_rot_vel_t packet;

	mav_array_memcpy(packet.rotVel, rotVel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILT_ROT_VEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN, MAVLINK_MSG_ID_FILT_ROT_VEL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif
}

/**
 * @brief Pack a filt_rot_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param rotVel 
                
            
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_filt_rot_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *rotVel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILT_ROT_VEL_LEN];

	_mav_put_float_array(buf, 0, rotVel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#else
	mavlink_filt_rot_vel_t packet;

	mav_array_memcpy(packet.rotVel, rotVel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FILT_ROT_VEL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN, MAVLINK_MSG_ID_FILT_ROT_VEL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif
}

/**
 * @brief Encode a filt_rot_vel struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param filt_rot_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_filt_rot_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_filt_rot_vel_t* filt_rot_vel)
{
	return mavlink_msg_filt_rot_vel_pack(system_id, component_id, msg, filt_rot_vel->rotVel);
}

/**
 * @brief Send a filt_rot_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param rotVel 
                
            
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_filt_rot_vel_send(mavlink_channel_t chan, const float *rotVel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FILT_ROT_VEL_LEN];

	_mav_put_float_array(buf, 0, rotVel, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILT_ROT_VEL, buf, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN, MAVLINK_MSG_ID_FILT_ROT_VEL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILT_ROT_VEL, buf, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif
#else
	mavlink_filt_rot_vel_t packet;

	mav_array_memcpy(packet.rotVel, rotVel, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILT_ROT_VEL, (const char *)&packet, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN, MAVLINK_MSG_ID_FILT_ROT_VEL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FILT_ROT_VEL, (const char *)&packet, MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif
#endif
}

#endif

// MESSAGE FILT_ROT_VEL UNPACKING


/**
 * @brief Get field rotVel from filt_rot_vel message
 *
 * @return 
                
            
 */
static inline uint16_t mavlink_msg_filt_rot_vel_get_rotVel(const mavlink_message_t* msg, float *rotVel)
{
	return _MAV_RETURN_float_array(msg, rotVel, 3,  0);
}

/**
 * @brief Decode a filt_rot_vel message into a struct
 *
 * @param msg The message to decode
 * @param filt_rot_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_filt_rot_vel_decode(const mavlink_message_t* msg, mavlink_filt_rot_vel_t* filt_rot_vel)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_filt_rot_vel_get_rotVel(msg, filt_rot_vel->rotVel);
#else
	memcpy(filt_rot_vel, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FILT_ROT_VEL_LEN);
#endif
}
