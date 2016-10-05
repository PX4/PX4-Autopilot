// MESSAGE FIXED_TARGET_POSITION PACKING

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION 221

MAVPACKED(
typedef struct __mavlink_fixed_target_position_t {
 double home_lat; /*< home point latitude, in deg*/
 double home_lon; /*< home point longitude, in deg*/
 double observe_lat; /*< observe point latitude, in deg*/
 double observe_lon; /*< observe point longitude, in deg*/
 double spray_left_lat; /*< spray point latitude (left), in deg*/
 double spray_left_lon; /*< spray point longitude (left), in deg*/
 double spray_right_lat; /*< spray point latitude (right), in deg*/
 double spray_right_lon; /*< spray point longitude (right), in deg*/
 float home_alt; /*< home point altitude, in m*/
 float observe_alt; /*< observe point altitude, in m*/
 float spray_left_alt; /*< spray point altitude (left), in m*/
 float spray_right_alt; /*< spray point altitude (right), in m*/
}) mavlink_fixed_target_position_t;

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN 80
#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN 80
#define MAVLINK_MSG_ID_221_LEN 80
#define MAVLINK_MSG_ID_221_MIN_LEN 80

#define MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC 173
#define MAVLINK_MSG_ID_221_CRC 173



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FIXED_TARGET_POSITION { \
	221, \
	"FIXED_TARGET_POSITION", \
	12, \
	{  { "home_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_fixed_target_position_t, home_lat) }, \
         { "home_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_fixed_target_position_t, home_lon) }, \
         { "observe_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_fixed_target_position_t, observe_lat) }, \
         { "observe_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_fixed_target_position_t, observe_lon) }, \
         { "spray_left_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_fixed_target_position_t, spray_left_lat) }, \
         { "spray_left_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_fixed_target_position_t, spray_left_lon) }, \
         { "spray_right_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_fixed_target_position_t, spray_right_lat) }, \
         { "spray_right_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_fixed_target_position_t, spray_right_lon) }, \
         { "home_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fixed_target_position_t, home_alt) }, \
         { "observe_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fixed_target_position_t, observe_alt) }, \
         { "spray_left_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_fixed_target_position_t, spray_left_alt) }, \
         { "spray_right_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_fixed_target_position_t, spray_right_alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FIXED_TARGET_POSITION { \
	"FIXED_TARGET_POSITION", \
	12, \
	{  { "home_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_fixed_target_position_t, home_lat) }, \
         { "home_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_fixed_target_position_t, home_lon) }, \
         { "observe_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_fixed_target_position_t, observe_lat) }, \
         { "observe_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_fixed_target_position_t, observe_lon) }, \
         { "spray_left_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_fixed_target_position_t, spray_left_lat) }, \
         { "spray_left_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_fixed_target_position_t, spray_left_lon) }, \
         { "spray_right_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_fixed_target_position_t, spray_right_lat) }, \
         { "spray_right_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_fixed_target_position_t, spray_right_lon) }, \
         { "home_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_fixed_target_position_t, home_alt) }, \
         { "observe_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_fixed_target_position_t, observe_alt) }, \
         { "spray_left_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_fixed_target_position_t, spray_left_alt) }, \
         { "spray_right_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_fixed_target_position_t, spray_right_alt) }, \
         } \
}
#endif

/**
 * @brief Pack a fixed_target_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param home_lat home point latitude, in deg
 * @param home_lon home point longitude, in deg
 * @param home_alt home point altitude, in m
 * @param observe_lat observe point latitude, in deg
 * @param observe_lon observe point longitude, in deg
 * @param observe_alt observe point altitude, in m
 * @param spray_left_lat spray point latitude (left), in deg
 * @param spray_left_lon spray point longitude (left), in deg
 * @param spray_left_alt spray point altitude (left), in m
 * @param spray_right_lat spray point latitude (right), in deg
 * @param spray_right_lon spray point longitude (right), in deg
 * @param spray_right_alt spray point altitude (right), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fixed_target_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       double home_lat, double home_lon, float home_alt, double observe_lat, double observe_lon, float observe_alt, double spray_left_lat, double spray_left_lon, float spray_left_alt, double spray_right_lat, double spray_right_lon, float spray_right_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN];
	_mav_put_double(buf, 0, home_lat);
	_mav_put_double(buf, 8, home_lon);
	_mav_put_double(buf, 16, observe_lat);
	_mav_put_double(buf, 24, observe_lon);
	_mav_put_double(buf, 32, spray_left_lat);
	_mav_put_double(buf, 40, spray_left_lon);
	_mav_put_double(buf, 48, spray_right_lat);
	_mav_put_double(buf, 56, spray_right_lon);
	_mav_put_float(buf, 64, home_alt);
	_mav_put_float(buf, 68, observe_alt);
	_mav_put_float(buf, 72, spray_left_alt);
	_mav_put_float(buf, 76, spray_right_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN);
#else
	mavlink_fixed_target_position_t packet;
	packet.home_lat = home_lat;
	packet.home_lon = home_lon;
	packet.observe_lat = observe_lat;
	packet.observe_lon = observe_lon;
	packet.spray_left_lat = spray_left_lat;
	packet.spray_left_lon = spray_left_lon;
	packet.spray_right_lat = spray_right_lat;
	packet.spray_right_lon = spray_right_lon;
	packet.home_alt = home_alt;
	packet.observe_alt = observe_alt;
	packet.spray_left_alt = spray_left_alt;
	packet.spray_right_alt = spray_right_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FIXED_TARGET_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
}

/**
 * @brief Pack a fixed_target_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param home_lat home point latitude, in deg
 * @param home_lon home point longitude, in deg
 * @param home_alt home point altitude, in m
 * @param observe_lat observe point latitude, in deg
 * @param observe_lon observe point longitude, in deg
 * @param observe_alt observe point altitude, in m
 * @param spray_left_lat spray point latitude (left), in deg
 * @param spray_left_lon spray point longitude (left), in deg
 * @param spray_left_alt spray point altitude (left), in m
 * @param spray_right_lat spray point latitude (right), in deg
 * @param spray_right_lon spray point longitude (right), in deg
 * @param spray_right_alt spray point altitude (right), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fixed_target_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           double home_lat,double home_lon,float home_alt,double observe_lat,double observe_lon,float observe_alt,double spray_left_lat,double spray_left_lon,float spray_left_alt,double spray_right_lat,double spray_right_lon,float spray_right_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN];
	_mav_put_double(buf, 0, home_lat);
	_mav_put_double(buf, 8, home_lon);
	_mav_put_double(buf, 16, observe_lat);
	_mav_put_double(buf, 24, observe_lon);
	_mav_put_double(buf, 32, spray_left_lat);
	_mav_put_double(buf, 40, spray_left_lon);
	_mav_put_double(buf, 48, spray_right_lat);
	_mav_put_double(buf, 56, spray_right_lon);
	_mav_put_float(buf, 64, home_alt);
	_mav_put_float(buf, 68, observe_alt);
	_mav_put_float(buf, 72, spray_left_alt);
	_mav_put_float(buf, 76, spray_right_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN);
#else
	mavlink_fixed_target_position_t packet;
	packet.home_lat = home_lat;
	packet.home_lon = home_lon;
	packet.observe_lat = observe_lat;
	packet.observe_lon = observe_lon;
	packet.spray_left_lat = spray_left_lat;
	packet.spray_left_lon = spray_left_lon;
	packet.spray_right_lat = spray_right_lat;
	packet.spray_right_lon = spray_right_lon;
	packet.home_alt = home_alt;
	packet.observe_alt = observe_alt;
	packet.spray_left_alt = spray_left_alt;
	packet.spray_right_alt = spray_right_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FIXED_TARGET_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
}

/**
 * @brief Encode a fixed_target_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fixed_target_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fixed_target_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fixed_target_position_t* fixed_target_position)
{
	return mavlink_msg_fixed_target_position_pack(system_id, component_id, msg, fixed_target_position->home_lat, fixed_target_position->home_lon, fixed_target_position->home_alt, fixed_target_position->observe_lat, fixed_target_position->observe_lon, fixed_target_position->observe_alt, fixed_target_position->spray_left_lat, fixed_target_position->spray_left_lon, fixed_target_position->spray_left_alt, fixed_target_position->spray_right_lat, fixed_target_position->spray_right_lon, fixed_target_position->spray_right_alt);
}

/**
 * @brief Encode a fixed_target_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fixed_target_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fixed_target_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fixed_target_position_t* fixed_target_position)
{
	return mavlink_msg_fixed_target_position_pack_chan(system_id, component_id, chan, msg, fixed_target_position->home_lat, fixed_target_position->home_lon, fixed_target_position->home_alt, fixed_target_position->observe_lat, fixed_target_position->observe_lon, fixed_target_position->observe_alt, fixed_target_position->spray_left_lat, fixed_target_position->spray_left_lon, fixed_target_position->spray_left_alt, fixed_target_position->spray_right_lat, fixed_target_position->spray_right_lon, fixed_target_position->spray_right_alt);
}

/**
 * @brief Send a fixed_target_position message
 * @param chan MAVLink channel to send the message
 *
 * @param home_lat home point latitude, in deg
 * @param home_lon home point longitude, in deg
 * @param home_alt home point altitude, in m
 * @param observe_lat observe point latitude, in deg
 * @param observe_lon observe point longitude, in deg
 * @param observe_alt observe point altitude, in m
 * @param spray_left_lat spray point latitude (left), in deg
 * @param spray_left_lon spray point longitude (left), in deg
 * @param spray_left_alt spray point altitude (left), in m
 * @param spray_right_lat spray point latitude (right), in deg
 * @param spray_right_lon spray point longitude (right), in deg
 * @param spray_right_alt spray point altitude (right), in m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fixed_target_position_send(mavlink_channel_t chan, double home_lat, double home_lon, float home_alt, double observe_lat, double observe_lon, float observe_alt, double spray_left_lat, double spray_left_lon, float spray_left_alt, double spray_right_lat, double spray_right_lon, float spray_right_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN];
	_mav_put_double(buf, 0, home_lat);
	_mav_put_double(buf, 8, home_lon);
	_mav_put_double(buf, 16, observe_lat);
	_mav_put_double(buf, 24, observe_lon);
	_mav_put_double(buf, 32, spray_left_lat);
	_mav_put_double(buf, 40, spray_left_lon);
	_mav_put_double(buf, 48, spray_right_lat);
	_mav_put_double(buf, 56, spray_right_lon);
	_mav_put_float(buf, 64, home_alt);
	_mav_put_float(buf, 68, observe_alt);
	_mav_put_float(buf, 72, spray_left_alt);
	_mav_put_float(buf, 76, spray_right_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION, buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
#else
	mavlink_fixed_target_position_t packet;
	packet.home_lat = home_lat;
	packet.home_lon = home_lon;
	packet.observe_lat = observe_lat;
	packet.observe_lon = observe_lon;
	packet.spray_left_lat = spray_left_lat;
	packet.spray_left_lon = spray_left_lon;
	packet.spray_right_lat = spray_right_lat;
	packet.spray_right_lon = spray_right_lon;
	packet.home_alt = home_alt;
	packet.observe_alt = observe_alt;
	packet.spray_left_alt = spray_left_alt;
	packet.spray_right_alt = spray_right_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION, (const char *)&packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
#endif
}

/**
 * @brief Send a fixed_target_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fixed_target_position_send_struct(mavlink_channel_t chan, const mavlink_fixed_target_position_t* fixed_target_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fixed_target_position_send(chan, fixed_target_position->home_lat, fixed_target_position->home_lon, fixed_target_position->home_alt, fixed_target_position->observe_lat, fixed_target_position->observe_lon, fixed_target_position->observe_alt, fixed_target_position->spray_left_lat, fixed_target_position->spray_left_lon, fixed_target_position->spray_left_alt, fixed_target_position->spray_right_lat, fixed_target_position->spray_right_lon, fixed_target_position->spray_right_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION, (const char *)fixed_target_position, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fixed_target_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double home_lat, double home_lon, float home_alt, double observe_lat, double observe_lon, float observe_alt, double spray_left_lat, double spray_left_lon, float spray_left_alt, double spray_right_lat, double spray_right_lon, float spray_right_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_double(buf, 0, home_lat);
	_mav_put_double(buf, 8, home_lon);
	_mav_put_double(buf, 16, observe_lat);
	_mav_put_double(buf, 24, observe_lon);
	_mav_put_double(buf, 32, spray_left_lat);
	_mav_put_double(buf, 40, spray_left_lon);
	_mav_put_double(buf, 48, spray_right_lat);
	_mav_put_double(buf, 56, spray_right_lon);
	_mav_put_float(buf, 64, home_alt);
	_mav_put_float(buf, 68, observe_alt);
	_mav_put_float(buf, 72, spray_left_alt);
	_mav_put_float(buf, 76, spray_right_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION, buf, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
#else
	mavlink_fixed_target_position_t *packet = (mavlink_fixed_target_position_t *)msgbuf;
	packet->home_lat = home_lat;
	packet->home_lon = home_lon;
	packet->observe_lat = observe_lat;
	packet->observe_lon = observe_lon;
	packet->spray_left_lat = spray_left_lat;
	packet->spray_left_lon = spray_left_lon;
	packet->spray_right_lat = spray_right_lat;
	packet->spray_right_lon = spray_right_lon;
	packet->home_alt = home_alt;
	packet->observe_alt = observe_alt;
	packet->spray_left_alt = spray_left_alt;
	packet->spray_right_alt = spray_right_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FIXED_TARGET_POSITION, (const char *)packet, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_MIN_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE FIXED_TARGET_POSITION UNPACKING


/**
 * @brief Get field home_lat from fixed_target_position message
 *
 * @return home point latitude, in deg
 */
static inline double mavlink_msg_fixed_target_position_get_home_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field home_lon from fixed_target_position message
 *
 * @return home point longitude, in deg
 */
static inline double mavlink_msg_fixed_target_position_get_home_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field home_alt from fixed_target_position message
 *
 * @return home point altitude, in m
 */
static inline float mavlink_msg_fixed_target_position_get_home_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field observe_lat from fixed_target_position message
 *
 * @return observe point latitude, in deg
 */
static inline double mavlink_msg_fixed_target_position_get_observe_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field observe_lon from fixed_target_position message
 *
 * @return observe point longitude, in deg
 */
static inline double mavlink_msg_fixed_target_position_get_observe_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field observe_alt from fixed_target_position message
 *
 * @return observe point altitude, in m
 */
static inline float mavlink_msg_fixed_target_position_get_observe_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field spray_left_lat from fixed_target_position message
 *
 * @return spray point latitude (left), in deg
 */
static inline double mavlink_msg_fixed_target_position_get_spray_left_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  32);
}

/**
 * @brief Get field spray_left_lon from fixed_target_position message
 *
 * @return spray point longitude (left), in deg
 */
static inline double mavlink_msg_fixed_target_position_get_spray_left_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  40);
}

/**
 * @brief Get field spray_left_alt from fixed_target_position message
 *
 * @return spray point altitude (left), in m
 */
static inline float mavlink_msg_fixed_target_position_get_spray_left_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field spray_right_lat from fixed_target_position message
 *
 * @return spray point latitude (right), in deg
 */
static inline double mavlink_msg_fixed_target_position_get_spray_right_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  48);
}

/**
 * @brief Get field spray_right_lon from fixed_target_position message
 *
 * @return spray point longitude (right), in deg
 */
static inline double mavlink_msg_fixed_target_position_get_spray_right_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  56);
}

/**
 * @brief Get field spray_right_alt from fixed_target_position message
 *
 * @return spray point altitude (right), in m
 */
static inline float mavlink_msg_fixed_target_position_get_spray_right_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Decode a fixed_target_position message into a struct
 *
 * @param msg The message to decode
 * @param fixed_target_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_fixed_target_position_decode(const mavlink_message_t* msg, mavlink_fixed_target_position_t* fixed_target_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	fixed_target_position->home_lat = mavlink_msg_fixed_target_position_get_home_lat(msg);
	fixed_target_position->home_lon = mavlink_msg_fixed_target_position_get_home_lon(msg);
	fixed_target_position->observe_lat = mavlink_msg_fixed_target_position_get_observe_lat(msg);
	fixed_target_position->observe_lon = mavlink_msg_fixed_target_position_get_observe_lon(msg);
	fixed_target_position->spray_left_lat = mavlink_msg_fixed_target_position_get_spray_left_lat(msg);
	fixed_target_position->spray_left_lon = mavlink_msg_fixed_target_position_get_spray_left_lon(msg);
	fixed_target_position->spray_right_lat = mavlink_msg_fixed_target_position_get_spray_right_lat(msg);
	fixed_target_position->spray_right_lon = mavlink_msg_fixed_target_position_get_spray_right_lon(msg);
	fixed_target_position->home_alt = mavlink_msg_fixed_target_position_get_home_alt(msg);
	fixed_target_position->observe_alt = mavlink_msg_fixed_target_position_get_observe_alt(msg);
	fixed_target_position->spray_left_alt = mavlink_msg_fixed_target_position_get_spray_left_alt(msg);
	fixed_target_position->spray_right_alt = mavlink_msg_fixed_target_position_get_spray_right_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN? msg->len : MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN;
        memset(fixed_target_position, 0, MAVLINK_MSG_ID_FIXED_TARGET_POSITION_LEN);
	memcpy(fixed_target_position, _MAV_PAYLOAD(msg), len);
#endif
}
