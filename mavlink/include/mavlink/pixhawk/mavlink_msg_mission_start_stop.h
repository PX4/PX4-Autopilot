// MESSAGE MISSION_START_STOP PACKING

#define MAVLINK_MSG_ID_MISSION_START_STOP 214

MAVPACKED(
typedef struct __mavlink_mission_start_stop_t {
 uint8_t start_flag; /*< start flag*/
 uint8_t main_commander; /*< main commander for task*/
 uint8_t sub_commander; /*< sub commander for task*/
}) mavlink_mission_start_stop_t;

#define MAVLINK_MSG_ID_MISSION_START_STOP_LEN 3
#define MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN 3
#define MAVLINK_MSG_ID_214_LEN 3
#define MAVLINK_MSG_ID_214_MIN_LEN 3

#define MAVLINK_MSG_ID_MISSION_START_STOP_CRC 170
#define MAVLINK_MSG_ID_214_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_START_STOP { \
	214, \
	"MISSION_START_STOP", \
	3, \
	{  { "start_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_start_stop_t, start_flag) }, \
         { "main_commander", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_start_stop_t, main_commander) }, \
         { "sub_commander", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_start_stop_t, sub_commander) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_START_STOP { \
	"MISSION_START_STOP", \
	3, \
	{  { "start_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_start_stop_t, start_flag) }, \
         { "main_commander", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_start_stop_t, main_commander) }, \
         { "sub_commander", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_start_stop_t, sub_commander) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_start_stop message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param start_flag start flag
 * @param main_commander main commander for task
 * @param sub_commander sub commander for task
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_start_stop_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t start_flag, uint8_t main_commander, uint8_t sub_commander)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_START_STOP_LEN];
	_mav_put_uint8_t(buf, 0, start_flag);
	_mav_put_uint8_t(buf, 1, main_commander);
	_mav_put_uint8_t(buf, 2, sub_commander);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_START_STOP_LEN);
#else
	mavlink_mission_start_stop_t packet;
	packet.start_flag = start_flag;
	packet.main_commander = main_commander;
	packet.sub_commander = sub_commander;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_START_STOP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_START_STOP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
}

/**
 * @brief Pack a mission_start_stop message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param start_flag start flag
 * @param main_commander main commander for task
 * @param sub_commander sub commander for task
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_start_stop_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t start_flag,uint8_t main_commander,uint8_t sub_commander)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_START_STOP_LEN];
	_mav_put_uint8_t(buf, 0, start_flag);
	_mav_put_uint8_t(buf, 1, main_commander);
	_mav_put_uint8_t(buf, 2, sub_commander);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_START_STOP_LEN);
#else
	mavlink_mission_start_stop_t packet;
	packet.start_flag = start_flag;
	packet.main_commander = main_commander;
	packet.sub_commander = sub_commander;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_START_STOP_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_START_STOP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
}

/**
 * @brief Encode a mission_start_stop struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_start_stop C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_start_stop_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_start_stop_t* mission_start_stop)
{
	return mavlink_msg_mission_start_stop_pack(system_id, component_id, msg, mission_start_stop->start_flag, mission_start_stop->main_commander, mission_start_stop->sub_commander);
}

/**
 * @brief Encode a mission_start_stop struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_start_stop C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_start_stop_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_start_stop_t* mission_start_stop)
{
	return mavlink_msg_mission_start_stop_pack_chan(system_id, component_id, chan, msg, mission_start_stop->start_flag, mission_start_stop->main_commander, mission_start_stop->sub_commander);
}

/**
 * @brief Send a mission_start_stop message
 * @param chan MAVLink channel to send the message
 *
 * @param start_flag start flag
 * @param main_commander main commander for task
 * @param sub_commander sub commander for task
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_start_stop_send(mavlink_channel_t chan, uint8_t start_flag, uint8_t main_commander, uint8_t sub_commander)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_START_STOP_LEN];
	_mav_put_uint8_t(buf, 0, start_flag);
	_mav_put_uint8_t(buf, 1, main_commander);
	_mav_put_uint8_t(buf, 2, sub_commander);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_START_STOP, buf, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
#else
	mavlink_mission_start_stop_t packet;
	packet.start_flag = start_flag;
	packet.main_commander = main_commander;
	packet.sub_commander = sub_commander;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_START_STOP, (const char *)&packet, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
#endif
}

/**
 * @brief Send a mission_start_stop message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_start_stop_send_struct(mavlink_channel_t chan, const mavlink_mission_start_stop_t* mission_start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_start_stop_send(chan, mission_start_stop->start_flag, mission_start_stop->main_commander, mission_start_stop->sub_commander);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_START_STOP, (const char *)mission_start_stop, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_START_STOP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_start_stop_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t start_flag, uint8_t main_commander, uint8_t sub_commander)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, start_flag);
	_mav_put_uint8_t(buf, 1, main_commander);
	_mav_put_uint8_t(buf, 2, sub_commander);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_START_STOP, buf, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
#else
	mavlink_mission_start_stop_t *packet = (mavlink_mission_start_stop_t *)msgbuf;
	packet->start_flag = start_flag;
	packet->main_commander = main_commander;
	packet->sub_commander = sub_commander;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_START_STOP, (const char *)packet, MAVLINK_MSG_ID_MISSION_START_STOP_MIN_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_LEN, MAVLINK_MSG_ID_MISSION_START_STOP_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_START_STOP UNPACKING


/**
 * @brief Get field start_flag from mission_start_stop message
 *
 * @return start flag
 */
static inline uint8_t mavlink_msg_mission_start_stop_get_start_flag(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field main_commander from mission_start_stop message
 *
 * @return main commander for task
 */
static inline uint8_t mavlink_msg_mission_start_stop_get_main_commander(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field sub_commander from mission_start_stop message
 *
 * @return sub commander for task
 */
static inline uint8_t mavlink_msg_mission_start_stop_get_sub_commander(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a mission_start_stop message into a struct
 *
 * @param msg The message to decode
 * @param mission_start_stop C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_start_stop_decode(const mavlink_message_t* msg, mavlink_mission_start_stop_t* mission_start_stop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mission_start_stop->start_flag = mavlink_msg_mission_start_stop_get_start_flag(msg);
	mission_start_stop->main_commander = mavlink_msg_mission_start_stop_get_main_commander(msg);
	mission_start_stop->sub_commander = mavlink_msg_mission_start_stop_get_sub_commander(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_START_STOP_LEN? msg->len : MAVLINK_MSG_ID_MISSION_START_STOP_LEN;
        memset(mission_start_stop, 0, MAVLINK_MSG_ID_MISSION_START_STOP_LEN);
	memcpy(mission_start_stop, _MAV_PAYLOAD(msg), len);
#endif
}
