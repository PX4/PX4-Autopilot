// MESSAGE FIXED_TARGET_POSITION_P2M PACKING

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M 152

MAVPACKED(
typedef struct __mavlink_fixed_target_position_p2m_t {
 float home_x; /*< home point X position (NED), in m*/
 float home_y; /*< home point Y position (NED), in m*/
 float home_z; /*< home point Z position (NED), in m*/
 float observe_x; /*< observe point X position (NED), in m*/
 float observe_y; /*< observe point Y position (NED), in m*/
 float observe_z; /*< observe point Z position (NED), in m*/
 float spray_left_x; /*< spray point X position (left) (NED), in m*/
 float spray_left_y; /*< spray point Y position (left) (NED), in m*/
 float spray_left_z; /*< spray point Z position (left) (NED), in m*/
 float spray_right_x; /*< spray point X position (right) (NED), in m*/
 float spray_right_y; /*< spray point Y position (right) (NED), in m*/
 float spray_right_z; /*< spray point Z position (right) (NED), in m*/
}) mavlink_fixed_target_position_p2m_t;

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN 48
#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN 48
#define MAVLINK_MSG_ID_152_LEN 48
#define MAVLINK_MSG_ID_152_MIN_LEN 48

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC 99
#define MAVLINK_MSG_ID_152_CRC 99



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FIXED_TARGET_POSITION_P2M { \
	152, \
	"FIXED_TARGET_POSITION_P2M", \
	12, \
	{  { "home_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fixed_target_position_p2m_t, home_x) }, \
         { "home_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fixed_target_position_p2m_t, home_y) }, \
         { "home_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fixed_target_position_p2m_t, home_z) }, \
         { "observe_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fixed_target_position_p2m_t, observe_x) }, \
         { "observe_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fixed_target_position_p2m_t, observe_y) }, \
         { "observe_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fixed_target_position_p2m_t, observe_z) }, \
         { "spray_left_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_x) }, \
         { "spray_left_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_y) }, \
         { "spray_left_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_z) }, \
         { "spray_right_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_x) }, \
         { "spray_right_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_y) }, \
         { "spray_right_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FIXED_TARGET_POSITION_P2M { \
	"FIXED_TARGET_POSITION_P2M", \
	12, \
	{  { "home_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fixed_target_position_p2m_t, home_x) }, \
         { "home_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fixed_target_position_p2m_t, home_y) }, \
         { "home_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_fixed_target_position_p2m_t, home_z) }, \
         { "observe_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_fixed_target_position_p2m_t, observe_x) }, \
         { "observe_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_fixed_target_position_p2m_t, observe_y) }, \
         { "observe_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_fixed_target_position_p2m_t, observe_z) }, \
         { "spray_left_x", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_x) }, \
         { "spray_left_y", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_y) }, \
         { "spray_left_z", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_fixed_target_position_p2m_t, spray_left_z) }, \
         { "spray_right_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_x) }, \
         { "spray_right_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_y) }, \
         { "spray_right_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_fixed_target_position_p2m_t, spray_right_z) }, \
         } \
}
#endif

/**
 * @brief Pack a fixed_target_position_p2m message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param home_x home point X position (NED), in m
 * @param home_y home point Y position (NED), in m
 * @param home_z home point Z position (NED), in m
 * @param observe_x observe point X position (NED), in m
 * @param observe_y observe point Y position (NED), in m
 * @param observe_z observe point Z position (NED), in m
 * @param spray_left_x spray point X position (left) (NED), in m
 * @param spray_left_y spray point Y position (left) (NED), in m
 * @param spray_left_z spray point Z position (left) (NED), in m
 * @param spray_right_x spray point X position (right) (NED), in m
 * @param spray_right_y spray point Y position (right) (NED), in m
 * @param spray_right_z spray point Z position (right) (NED), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fixed_target_position_p2m_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float home_x, float home_y, float home_z, float observe_x, float observe_y, float observe_z, float spray_left_x, float spray_left_y, float spray_left_z, float spray_right_x, float spray_right_y, float spray_right_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN];
	_mav_put_float(buf, 0, home_x);
	_mav_put_float(buf, 4, home_y);
	_mav_put_float(buf, 8, home_z);
	_mav_put_float(buf, 12, observe_x);
	_mav_put_float(buf, 16, observe_y);
	_mav_put_float(buf, 20, observe_z);
	_mav_put_float(buf, 24, spray_left_x);
	_mav_put_float(buf, 28, spray_left_y);
	_mav_put_float(buf, 32, spray_left_z);
	_mav_put_float(buf, 36, spray_right_x);
	_mav_put_float(buf, 40, spray_right_y);
	_mav_put_float(buf, 44, spray_right_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN);
#else
	mavlink_fixed_target_position_p2m_t packet;
	packet.home_x = home_x;
	packet.home_y = home_y;
	packet.home_z = home_z;
	packet.observe_x = observe_x;
	packet.observe_y = observe_y;
	packet.observe_z = observe_z;
	packet.spray_left_x = spray_left_x;
	packet.spray_left_y = spray_left_y;
	packet.spray_left_z = spray_left_z;
	packet.spray_right_x = spray_right_x;
	packet.spray_right_y = spray_right_y;
	packet.spray_right_z = spray_right_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
}

/**
 * @brief Pack a fixed_target_position_p2m message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param home_x home point X position (NED), in m
 * @param home_y home point Y position (NED), in m
 * @param home_z home point Z position (NED), in m
 * @param observe_x observe point X position (NED), in m
 * @param observe_y observe point Y position (NED), in m
 * @param observe_z observe point Z position (NED), in m
 * @param spray_left_x spray point X position (left) (NED), in m
 * @param spray_left_y spray point Y position (left) (NED), in m
 * @param spray_left_z spray point Z position (left) (NED), in m
 * @param spray_right_x spray point X position (right) (NED), in m
 * @param spray_right_y spray point Y position (right) (NED), in m
 * @param spray_right_z spray point Z position (right) (NED), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fixed_target_position_p2m_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float home_x,float home_y,float home_z,float observe_x,float observe_y,float observe_z,float spray_left_x,float spray_left_y,float spray_left_z,float spray_right_x,float spray_right_y,float spray_right_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN];
	_mav_put_float(buf, 0, home_x);
	_mav_put_float(buf, 4, home_y);
	_mav_put_float(buf, 8, home_z);
	_mav_put_float(buf, 12, observe_x);
	_mav_put_float(buf, 16, observe_y);
	_mav_put_float(buf, 20, observe_z);
	_mav_put_float(buf, 24, spray_left_x);
	_mav_put_float(buf, 28, spray_left_y);
	_mav_put_float(buf, 32, spray_left_z);
	_mav_put_float(buf, 36, spray_right_x);
	_mav_put_float(buf, 40, spray_right_y);
	_mav_put_float(buf, 44, spray_right_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN);
#else
	mavlink_fixed_target_position_p2m_t packet;
	packet.home_x = home_x;
	packet.home_y = home_y;
	packet.home_z = home_z;
	packet.observe_x = observe_x;
	packet.observe_y = observe_y;
	packet.observe_z = observe_z;
	packet.spray_left_x = spray_left_x;
	packet.spray_left_y = spray_left_y;
	packet.spray_left_z = spray_left_z;
	packet.spray_right_x = spray_right_x;
	packet.spray_right_y = spray_right_y;
	packet.spray_right_z = spray_right_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
}

/**
 * @brief Encode a fixed_target_position_p2m struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fixed_target_position_p2m C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fixed_target_position_p2m_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fixed_target_position_p2m_t* fixed_target_position_p2m)
{
	return mavlink_msg_fixed_target_position_p2m_pack(system_id, component_id, msg, fixed_target_position_p2m->home_x, fixed_target_position_p2m->home_y, fixed_target_position_p2m->home_z, fixed_target_position_p2m->observe_x, fixed_target_position_p2m->observe_y, fixed_target_position_p2m->observe_z, fixed_target_position_p2m->spray_left_x, fixed_target_position_p2m->spray_left_y, fixed_target_position_p2m->spray_left_z, fixed_target_position_p2m->spray_right_x, fixed_target_position_p2m->spray_right_y, fixed_target_position_p2m->spray_right_z);
}

/**
 * @brief Encode a fixed_target_position_p2m struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fixed_target_position_p2m C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fixed_target_position_p2m_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fixed_target_position_p2m_t* fixed_target_position_p2m)
{
	return mavlink_msg_fixed_target_position_p2m_pack_chan(system_id, component_id, chan, msg, fixed_target_position_p2m->home_x, fixed_target_position_p2m->home_y, fixed_target_position_p2m->home_z, fixed_target_position_p2m->observe_x, fixed_target_position_p2m->observe_y, fixed_target_position_p2m->observe_z, fixed_target_position_p2m->spray_left_x, fixed_target_position_p2m->spray_left_y, fixed_target_position_p2m->spray_left_z, fixed_target_position_p2m->spray_right_x, fixed_target_position_p2m->spray_right_y, fixed_target_position_p2m->spray_right_z);
}

/**
 * @brief Send a fixed_target_position_p2m message
 * @param chan MAVLink channel to send the message
 *
 * @param home_x home point X position (NED), in m
 * @param home_y home point Y position (NED), in m
 * @param home_z home point Z position (NED), in m
 * @param observe_x observe point X position (NED), in m
 * @param observe_y observe point Y position (NED), in m
 * @param observe_z observe point Z position (NED), in m
 * @param spray_left_x spray point X position (left) (NED), in m
 * @param spray_left_y spray point Y position (left) (NED), in m
 * @param spray_left_z spray point Z position (left) (NED), in m
 * @param spray_right_x spray point X position (right) (NED), in m
 * @param spray_right_y spray point Y position (right) (NED), in m
 * @param spray_right_z spray point Z position (right) (NED), in m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fixed_target_position_p2m_send(mavlink_channel_t chan, float home_x, float home_y, float home_z, float observe_x, float observe_y, float observe_z, float spray_left_x, float spray_left_y, float spray_left_z, float spray_right_x, float spray_right_y, float spray_right_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN];
	_mav_put_float(buf, 0, home_x);
	_mav_put_float(buf, 4, home_y);
	_mav_put_float(buf, 8, home_z);
	_mav_put_float(buf, 12, observe_x);
	_mav_put_float(buf, 16, observe_y);
	_mav_put_float(buf, 20, observe_z);
	_mav_put_float(buf, 24, spray_left_x);
	_mav_put_float(buf, 28, spray_left_y);
	_mav_put_float(buf, 32, spray_left_z);
	_mav_put_float(buf, 36, spray_right_x);
	_mav_put_float(buf, 40, spray_right_y);
	_mav_put_float(buf, 44, spray_right_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M, buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
#else
	mavlink_fixed_target_position_p2m_t packet;
	packet.home_x = home_x;
	packet.home_y = home_y;
	packet.home_z = home_z;
	packet.observe_x = observe_x;
	packet.observe_y = observe_y;
	packet.observe_z = observe_z;
	packet.spray_left_x = spray_left_x;
	packet.spray_left_y = spray_left_y;
	packet.spray_left_z = spray_left_z;
	packet.spray_right_x = spray_right_x;
	packet.spray_right_y = spray_right_y;
	packet.spray_right_z = spray_right_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M, (const char *)&packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
#endif
}

/**
 * @brief Send a fixed_target_position_p2m message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fixed_target_position_p2m_send_struct(mavlink_channel_t chan, const mavlink_fixed_target_position_p2m_t* fixed_target_position_p2m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fixed_target_position_p2m_send(chan, fixed_target_position_p2m->home_x, fixed_target_position_p2m->home_y, fixed_target_position_p2m->home_z, fixed_target_position_p2m->observe_x, fixed_target_position_p2m->observe_y, fixed_target_position_p2m->observe_z, fixed_target_position_p2m->spray_left_x, fixed_target_position_p2m->spray_left_y, fixed_target_position_p2m->spray_left_z, fixed_target_position_p2m->spray_right_x, fixed_target_position_p2m->spray_right_y, fixed_target_position_p2m->spray_right_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M, (const char *)fixed_target_position_p2m, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
#endif
}

#if MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fixed_target_position_p2m_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float home_x, float home_y, float home_z, float observe_x, float observe_y, float observe_z, float spray_left_x, float spray_left_y, float spray_left_z, float spray_right_x, float spray_right_y, float spray_right_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, home_x);
	_mav_put_float(buf, 4, home_y);
	_mav_put_float(buf, 8, home_z);
	_mav_put_float(buf, 12, observe_x);
	_mav_put_float(buf, 16, observe_y);
	_mav_put_float(buf, 20, observe_z);
	_mav_put_float(buf, 24, spray_left_x);
	_mav_put_float(buf, 28, spray_left_y);
	_mav_put_float(buf, 32, spray_left_z);
	_mav_put_float(buf, 36, spray_right_x);
	_mav_put_float(buf, 40, spray_right_y);
	_mav_put_float(buf, 44, spray_right_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M, buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
#else
	mavlink_fixed_target_position_p2m_t *packet = (mavlink_fixed_target_position_p2m_t *)msgbuf;
	packet->home_x = home_x;
	packet->home_y = home_y;
	packet->home_z = home_z;
	packet->observe_x = observe_x;
	packet->observe_y = observe_y;
	packet->observe_z = observe_z;
	packet->spray_left_x = spray_left_x;
	packet->spray_left_y = spray_left_y;
	packet->spray_left_z = spray_left_z;
	packet->spray_right_x = spray_right_x;
	packet->spray_right_y = spray_right_y;
	packet->spray_right_z = spray_right_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M, (const char *)packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_CRC);
#endif
}
#endif

#endif

// MESSAGE FIXED_TARGET_POSITION_P2M UNPACKING


/**
 * @brief Get field home_x from fixed_target_position_p2m message
 *
 * @return home point X position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_home_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field home_y from fixed_target_position_p2m message
 *
 * @return home point Y position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_home_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field home_z from fixed_target_position_p2m message
 *
 * @return home point Z position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_home_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field observe_x from fixed_target_position_p2m message
 *
 * @return observe point X position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_observe_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field observe_y from fixed_target_position_p2m message
 *
 * @return observe point Y position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_observe_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field observe_z from fixed_target_position_p2m message
 *
 * @return observe point Z position (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_observe_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field spray_left_x from fixed_target_position_p2m message
 *
 * @return spray point X position (left) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_left_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field spray_left_y from fixed_target_position_p2m message
 *
 * @return spray point Y position (left) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_left_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field spray_left_z from fixed_target_position_p2m message
 *
 * @return spray point Z position (left) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_left_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field spray_right_x from fixed_target_position_p2m message
 *
 * @return spray point X position (right) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_right_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field spray_right_y from fixed_target_position_p2m message
 *
 * @return spray point Y position (right) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_right_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field spray_right_z from fixed_target_position_p2m message
 *
 * @return spray point Z position (right) (NED), in m
 */
static inline float mavlink_msg_fixed_target_position_p2m_get_spray_right_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a fixed_target_position_p2m message into a struct
 *
 * @param msg The message to decode
 * @param fixed_target_position_p2m C-struct to decode the message contents into
 */
static inline void mavlink_msg_fixed_target_position_p2m_decode(const mavlink_message_t* msg, mavlink_fixed_target_position_p2m_t* fixed_target_position_p2m)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	fixed_target_position_p2m->home_x = mavlink_msg_fixed_target_position_p2m_get_home_x(msg);
	fixed_target_position_p2m->home_y = mavlink_msg_fixed_target_position_p2m_get_home_y(msg);
	fixed_target_position_p2m->home_z = mavlink_msg_fixed_target_position_p2m_get_home_z(msg);
	fixed_target_position_p2m->observe_x = mavlink_msg_fixed_target_position_p2m_get_observe_x(msg);
	fixed_target_position_p2m->observe_y = mavlink_msg_fixed_target_position_p2m_get_observe_y(msg);
	fixed_target_position_p2m->observe_z = mavlink_msg_fixed_target_position_p2m_get_observe_z(msg);
	fixed_target_position_p2m->spray_left_x = mavlink_msg_fixed_target_position_p2m_get_spray_left_x(msg);
	fixed_target_position_p2m->spray_left_y = mavlink_msg_fixed_target_position_p2m_get_spray_left_y(msg);
	fixed_target_position_p2m->spray_left_z = mavlink_msg_fixed_target_position_p2m_get_spray_left_z(msg);
	fixed_target_position_p2m->spray_right_x = mavlink_msg_fixed_target_position_p2m_get_spray_right_x(msg);
	fixed_target_position_p2m->spray_right_y = mavlink_msg_fixed_target_position_p2m_get_spray_right_y(msg);
	fixed_target_position_p2m->spray_right_z = mavlink_msg_fixed_target_position_p2m_get_spray_right_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN? msg->len : MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN;
        memset(fixed_target_position_p2m, 0, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_P2M_LEN);
	memcpy(fixed_target_position_p2m, _MAV_PAYLOAD(msg), len);
#endif
}
