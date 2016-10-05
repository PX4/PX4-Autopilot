// MESSAGE TASK_STATUS_MONITOR PACKING

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR 225

MAVPACKED(
typedef struct __mavlink_task_status_monitor_t {
 uint64_t timestamp; /*< timestamp*/
 double target_lat; /*< target latitude, in deg*/
 double target_lon; /*< target longitude, in deg*/
 float target_alt; /*< target altitude, in m*/
 uint8_t num_odd_even; /*< 0, 1 or 2 represent spray all number, odd number or even number*/
 uint8_t task_status; /*< task status NO. about Task State Machine, 0~16~30*/
 uint8_t loop_value; /*< loop number of current task, 0~5*/
}) mavlink_task_status_monitor_t;

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN 31
#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN 31
#define MAVLINK_MSG_ID_225_LEN 31
#define MAVLINK_MSG_ID_225_MIN_LEN 31

#define MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC 54
#define MAVLINK_MSG_ID_225_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_MONITOR { \
	225, \
	"TASK_STATUS_MONITOR", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_status_monitor_t, timestamp) }, \
         { "target_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_task_status_monitor_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_task_status_monitor_t, target_lon) }, \
         { "target_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_status_monitor_t, target_alt) }, \
         { "num_odd_even", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_task_status_monitor_t, num_odd_even) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_task_status_monitor_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_task_status_monitor_t, loop_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_MONITOR { \
	"TASK_STATUS_MONITOR", \
	7, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_status_monitor_t, timestamp) }, \
         { "target_lat", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_task_status_monitor_t, target_lat) }, \
         { "target_lon", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_task_status_monitor_t, target_lon) }, \
         { "target_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_status_monitor_t, target_alt) }, \
         { "num_odd_even", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_task_status_monitor_t, num_odd_even) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_task_status_monitor_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_task_status_monitor_t, loop_value) }, \
         } \
}
#endif

/**
 * @brief Pack a task_status_monitor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp timestamp
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 * @param target_lat target latitude, in deg
 * @param target_lon target longitude, in deg
 * @param target_alt target altitude, in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_monitor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value, double target_lat, double target_lon, float target_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_double(buf, 8, target_lat);
	_mav_put_double(buf, 16, target_lon);
	_mav_put_float(buf, 24, target_alt);
	_mav_put_uint8_t(buf, 28, num_odd_even);
	_mav_put_uint8_t(buf, 29, task_status);
	_mav_put_uint8_t(buf, 30, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN);
#else
	mavlink_task_status_monitor_t packet;
	packet.timestamp = timestamp;
	packet.target_lat = target_lat;
	packet.target_lon = target_lon;
	packet.target_alt = target_alt;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_MONITOR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
}

/**
 * @brief Pack a task_status_monitor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp timestamp
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 * @param target_lat target latitude, in deg
 * @param target_lon target longitude, in deg
 * @param target_alt target altitude, in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_monitor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t num_odd_even,uint8_t task_status,uint8_t loop_value,double target_lat,double target_lon,float target_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_double(buf, 8, target_lat);
	_mav_put_double(buf, 16, target_lon);
	_mav_put_float(buf, 24, target_alt);
	_mav_put_uint8_t(buf, 28, num_odd_even);
	_mav_put_uint8_t(buf, 29, task_status);
	_mav_put_uint8_t(buf, 30, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN);
#else
	mavlink_task_status_monitor_t packet;
	packet.timestamp = timestamp;
	packet.target_lat = target_lat;
	packet.target_lon = target_lon;
	packet.target_alt = target_alt;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_MONITOR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
}

/**
 * @brief Encode a task_status_monitor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_status_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_monitor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_status_monitor_t* task_status_monitor)
{
	return mavlink_msg_task_status_monitor_pack(system_id, component_id, msg, task_status_monitor->timestamp, task_status_monitor->num_odd_even, task_status_monitor->task_status, task_status_monitor->loop_value, task_status_monitor->target_lat, task_status_monitor->target_lon, task_status_monitor->target_alt);
}

/**
 * @brief Encode a task_status_monitor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_status_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_monitor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_status_monitor_t* task_status_monitor)
{
	return mavlink_msg_task_status_monitor_pack_chan(system_id, component_id, chan, msg, task_status_monitor->timestamp, task_status_monitor->num_odd_even, task_status_monitor->task_status, task_status_monitor->loop_value, task_status_monitor->target_lat, task_status_monitor->target_lon, task_status_monitor->target_alt);
}

/**
 * @brief Send a task_status_monitor message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp timestamp
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 * @param target_lat target latitude, in deg
 * @param target_lon target longitude, in deg
 * @param target_alt target altitude, in m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_status_monitor_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value, double target_lat, double target_lon, float target_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_double(buf, 8, target_lat);
	_mav_put_double(buf, 16, target_lon);
	_mav_put_float(buf, 24, target_alt);
	_mav_put_uint8_t(buf, 28, num_odd_even);
	_mav_put_uint8_t(buf, 29, task_status);
	_mav_put_uint8_t(buf, 30, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR, buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
#else
	mavlink_task_status_monitor_t packet;
	packet.timestamp = timestamp;
	packet.target_lat = target_lat;
	packet.target_lon = target_lon;
	packet.target_alt = target_alt;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
#endif
}

/**
 * @brief Send a task_status_monitor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_status_monitor_send_struct(mavlink_channel_t chan, const mavlink_task_status_monitor_t* task_status_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_status_monitor_send(chan, task_status_monitor->timestamp, task_status_monitor->num_odd_even, task_status_monitor->task_status, task_status_monitor->loop_value, task_status_monitor->target_lat, task_status_monitor->target_lon, task_status_monitor->target_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR, (const char *)task_status_monitor, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_status_monitor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value, double target_lat, double target_lon, float target_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_double(buf, 8, target_lat);
	_mav_put_double(buf, 16, target_lon);
	_mav_put_float(buf, 24, target_alt);
	_mav_put_uint8_t(buf, 28, num_odd_even);
	_mav_put_uint8_t(buf, 29, task_status);
	_mav_put_uint8_t(buf, 30, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR, buf, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
#else
	mavlink_task_status_monitor_t *packet = (mavlink_task_status_monitor_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->target_lat = target_lat;
	packet->target_lon = target_lon;
	packet->target_alt = target_alt;
	packet->num_odd_even = num_odd_even;
	packet->task_status = task_status;
	packet->loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_MONITOR, (const char *)packet, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATUS_MONITOR UNPACKING


/**
 * @brief Get field timestamp from task_status_monitor message
 *
 * @return timestamp
 */
static inline uint64_t mavlink_msg_task_status_monitor_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field num_odd_even from task_status_monitor message
 *
 * @return 0, 1 or 2 represent spray all number, odd number or even number
 */
static inline uint8_t mavlink_msg_task_status_monitor_get_num_odd_even(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field task_status from task_status_monitor message
 *
 * @return task status NO. about Task State Machine, 0~16~30
 */
static inline uint8_t mavlink_msg_task_status_monitor_get_task_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field loop_value from task_status_monitor message
 *
 * @return loop number of current task, 0~5
 */
static inline uint8_t mavlink_msg_task_status_monitor_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field target_lat from task_status_monitor message
 *
 * @return target latitude, in deg
 */
static inline double mavlink_msg_task_status_monitor_get_target_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field target_lon from task_status_monitor message
 *
 * @return target longitude, in deg
 */
static inline double mavlink_msg_task_status_monitor_get_target_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field target_alt from task_status_monitor message
 *
 * @return target altitude, in m
 */
static inline float mavlink_msg_task_status_monitor_get_target_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a task_status_monitor message into a struct
 *
 * @param msg The message to decode
 * @param task_status_monitor C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_status_monitor_decode(const mavlink_message_t* msg, mavlink_task_status_monitor_t* task_status_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	task_status_monitor->timestamp = mavlink_msg_task_status_monitor_get_timestamp(msg);
	task_status_monitor->target_lat = mavlink_msg_task_status_monitor_get_target_lat(msg);
	task_status_monitor->target_lon = mavlink_msg_task_status_monitor_get_target_lon(msg);
	task_status_monitor->target_alt = mavlink_msg_task_status_monitor_get_target_alt(msg);
	task_status_monitor->num_odd_even = mavlink_msg_task_status_monitor_get_num_odd_even(msg);
	task_status_monitor->task_status = mavlink_msg_task_status_monitor_get_task_status(msg);
	task_status_monitor->loop_value = mavlink_msg_task_status_monitor_get_loop_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN;
        memset(task_status_monitor, 0, MAVLINK_MSG_ID_TASK_STATUS_MONITOR_LEN);
	memcpy(task_status_monitor, _MAV_PAYLOAD(msg), len);
#endif
}
