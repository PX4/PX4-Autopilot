// MESSAGE TASK_STATUS_MONITOR_P2G PACKING

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G 160

MAVPACKED(
typedef struct __mavlink_task_status_monitor_p2g_t {
 uint64_t timestamp; /*< timestamp*/
 float spray_duration; /*< spray duration in task set by GCS, a constant value*/
 float target_x; /*< target X position (NED), in m*/
 float target_y; /*< target Y position (NED), in m*/
 float target_z; /*< target Z position (NED), in m*/
 uint8_t task_status; /*< task status NO. about Task State Machine, 0~17*/
 uint8_t loop_value; /*< loop number of current task, 0~5*/
}) mavlink_task_status_monitor_p2g_t;

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN 26
#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN 26
#define MAVLINK_MSG_ID_160_LEN 26
#define MAVLINK_MSG_ID_160_MIN_LEN 26

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC 214
#define MAVLINK_MSG_ID_160_CRC 214



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_MONITOR_P2G { \
	160, \
	"TASK_STATUS_MONITOR_P2G", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_status_monitor_p2g_t, timestamp) }, \
         { "spray_duration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_status_monitor_p2g_t, spray_duration) }, \
         { "target_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_status_monitor_p2g_t, target_x) }, \
         { "target_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_status_monitor_p2g_t, target_y) }, \
         { "target_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_status_monitor_p2g_t, target_z) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_task_status_monitor_p2g_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_task_status_monitor_p2g_t, loop_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_MONITOR_P2G { \
	"TASK_STATUS_MONITOR_P2G", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_status_monitor_p2g_t, timestamp) }, \
         { "spray_duration", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_status_monitor_p2g_t, spray_duration) }, \
         { "target_x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_status_monitor_p2g_t, target_x) }, \
         { "target_y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_status_monitor_p2g_t, target_y) }, \
         { "target_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_status_monitor_p2g_t, target_z) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_task_status_monitor_p2g_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_task_status_monitor_p2g_t, loop_value) }, \
         } \
}
#endif

/**
 * @brief Pack a task_status_monitor_p2g message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param spray_duration spray duration in task set by GCS, a constant value
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 * @param target_x target X position (NED), in m
 * @param target_y target Y position (NED), in m
 * @param target_z target Z position (NED), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_monitor_p2g_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, float spray_duration, uint8_t task_status, uint8_t loop_value, float target_x, float target_y, float target_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, spray_duration);
	_mav_put_float(buf, 12, target_x);
	_mav_put_float(buf, 16, target_y);
	_mav_put_float(buf, 20, target_z);
	_mav_put_uint8_t(buf, 24, task_status);
	_mav_put_uint8_t(buf, 25, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN);
#else
	mavlink_task_status_monitor_p2g_t packet;
	packet.timestamp = timestamp;
	packet.spray_duration = spray_duration;
	packet.target_x = target_x;
	packet.target_y = target_y;
	packet.target_z = target_z;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
}

/**
 * @brief Pack a task_status_monitor_p2g message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param spray_duration spray duration in task set by GCS, a constant value
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 * @param target_x target X position (NED), in m
 * @param target_y target Y position (NED), in m
 * @param target_z target Z position (NED), in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_monitor_p2g_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,float spray_duration,uint8_t task_status,uint8_t loop_value,float target_x,float target_y,float target_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, spray_duration);
	_mav_put_float(buf, 12, target_x);
	_mav_put_float(buf, 16, target_y);
	_mav_put_float(buf, 20, target_z);
	_mav_put_uint8_t(buf, 24, task_status);
	_mav_put_uint8_t(buf, 25, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN);
#else
	mavlink_task_status_monitor_p2g_t packet;
	packet.timestamp = timestamp;
	packet.spray_duration = spray_duration;
	packet.target_x = target_x;
	packet.target_y = target_y;
	packet.target_z = target_z;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
}

/**
 * @brief Encode a task_status_monitor_p2g struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_status_monitor_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_monitor_p2g_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_status_monitor_p2g_t* task_status_monitor_p2g)
{
	return mavlink_msg_task_status_monitor_p2g_pack(system_id, component_id, msg, task_status_monitor_p2g->timestamp, task_status_monitor_p2g->spray_duration, task_status_monitor_p2g->task_status, task_status_monitor_p2g->loop_value, task_status_monitor_p2g->target_x, task_status_monitor_p2g->target_y, task_status_monitor_p2g->target_z);
}

/**
 * @brief Encode a task_status_monitor_p2g struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_status_monitor_p2g C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_monitor_p2g_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_status_monitor_p2g_t* task_status_monitor_p2g)
{
	return mavlink_msg_task_status_monitor_p2g_pack_chan(system_id, component_id, chan, msg, task_status_monitor_p2g->timestamp, task_status_monitor_p2g->spray_duration, task_status_monitor_p2g->task_status, task_status_monitor_p2g->loop_value, task_status_monitor_p2g->target_x, task_status_monitor_p2g->target_y, task_status_monitor_p2g->target_z);
}

/**
 * @brief Send a task_status_monitor_p2g message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param spray_duration spray duration in task set by GCS, a constant value
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 * @param target_x target X position (NED), in m
 * @param target_y target Y position (NED), in m
 * @param target_z target Z position (NED), in m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_status_monitor_p2g_send(mavlink_channel_t chan, uint64_t timestamp, float spray_duration, uint8_t task_status, uint8_t loop_value, float target_x, float target_y, float target_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, spray_duration);
	_mav_put_float(buf, 12, target_x);
	_mav_put_float(buf, 16, target_y);
	_mav_put_float(buf, 20, target_z);
	_mav_put_uint8_t(buf, 24, task_status);
	_mav_put_uint8_t(buf, 25, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G, buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
#else
	mavlink_task_status_monitor_p2g_t packet;
	packet.timestamp = timestamp;
	packet.spray_duration = spray_duration;
	packet.target_x = target_x;
	packet.target_y = target_y;
	packet.target_z = target_z;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
#endif
}

/**
 * @brief Send a task_status_monitor_p2g message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_status_monitor_p2g_send_struct(mavlink_channel_t chan, const mavlink_task_status_monitor_p2g_t* task_status_monitor_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_status_monitor_p2g_send(chan, task_status_monitor_p2g->timestamp, task_status_monitor_p2g->spray_duration, task_status_monitor_p2g->task_status, task_status_monitor_p2g->loop_value, task_status_monitor_p2g->target_x, task_status_monitor_p2g->target_y, task_status_monitor_p2g->target_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G, (const char *)task_status_monitor_p2g, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_status_monitor_p2g_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float spray_duration, uint8_t task_status, uint8_t loop_value, float target_x, float target_y, float target_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, spray_duration);
	_mav_put_float(buf, 12, target_x);
	_mav_put_float(buf, 16, target_y);
	_mav_put_float(buf, 20, target_z);
	_mav_put_uint8_t(buf, 24, task_status);
	_mav_put_uint8_t(buf, 25, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G, buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
#else
	mavlink_task_status_monitor_p2g_t *packet = (mavlink_task_status_monitor_p2g_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->spray_duration = spray_duration;
	packet->target_x = target_x;
	packet->target_y = target_y;
	packet->target_z = target_z;
	packet->task_status = task_status;
	packet->loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G, (const char *)packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATUS_MONITOR_P2G UNPACKING


/**
 * @brief Get field timestamp from task_status_monitor_p2g message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_task_status_monitor_p2g_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field spray_duration from task_status_monitor_p2g message
 *
 * @return spray duration in task set by GCS, a constant value
 */
static inline float mavlink_msg_task_status_monitor_p2g_get_spray_duration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field task_status from task_status_monitor_p2g message
 *
 * @return task status NO. about Task State Machine, 0~17
 */
static inline uint8_t mavlink_msg_task_status_monitor_p2g_get_task_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field loop_value from task_status_monitor_p2g message
 *
 * @return loop number of current task, 0~5
 */
static inline uint8_t mavlink_msg_task_status_monitor_p2g_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field target_x from task_status_monitor_p2g message
 *
 * @return target X position (NED), in m
 */
static inline float mavlink_msg_task_status_monitor_p2g_get_target_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field target_y from task_status_monitor_p2g message
 *
 * @return target Y position (NED), in m
 */
static inline float mavlink_msg_task_status_monitor_p2g_get_target_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field target_z from task_status_monitor_p2g message
 *
 * @return target Z position (NED), in m
 */
static inline float mavlink_msg_task_status_monitor_p2g_get_target_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a task_status_monitor_p2g message into a struct
 *
 * @param msg The message to decode
 * @param task_status_monitor_p2g C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_status_monitor_p2g_decode(const mavlink_message_t* msg, mavlink_task_status_monitor_p2g_t* task_status_monitor_p2g)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	task_status_monitor_p2g->timestamp = mavlink_msg_task_status_monitor_p2g_get_timestamp(msg);
	task_status_monitor_p2g->spray_duration = mavlink_msg_task_status_monitor_p2g_get_spray_duration(msg);
	task_status_monitor_p2g->target_x = mavlink_msg_task_status_monitor_p2g_get_target_x(msg);
	task_status_monitor_p2g->target_y = mavlink_msg_task_status_monitor_p2g_get_target_y(msg);
	task_status_monitor_p2g->target_z = mavlink_msg_task_status_monitor_p2g_get_target_z(msg);
	task_status_monitor_p2g->task_status = mavlink_msg_task_status_monitor_p2g_get_task_status(msg);
	task_status_monitor_p2g->loop_value = mavlink_msg_task_status_monitor_p2g_get_loop_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN;
        memset(task_status_monitor_p2g, 0, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_P2G_LEN);
	memcpy(task_status_monitor_p2g, _MAV_PAYLOAD(msg), len);
#endif
}
