// MESSAGE TASK_STATUS_CHANGE_G2P PACKING

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P 157

MAVPACKED(
typedef struct __mavlink_task_status_change_g2p_t {
 float spray_duration; /*< spray duration in task set by GCS*/
 uint8_t task_status; /*< task status NO. about Task State Machine, 0~17*/
 uint8_t loop_value; /*< loop number of current task, 0~5*/
}) mavlink_task_status_change_g2p_t;

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN 6
#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN 6
#define MAVLINK_MSG_ID_157_LEN 6
#define MAVLINK_MSG_ID_157_MIN_LEN 6

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC 74
#define MAVLINK_MSG_ID_157_CRC 74



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_CHANGE_G2P { \
	157, \
	"TASK_STATUS_CHANGE_G2P", \
	3, \
	{  { "spray_duration", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_task_status_change_g2p_t, spray_duration) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_task_status_change_g2p_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_task_status_change_g2p_t, loop_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_CHANGE_G2P { \
	"TASK_STATUS_CHANGE_G2P", \
	3, \
	{  { "spray_duration", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_task_status_change_g2p_t, spray_duration) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_task_status_change_g2p_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_task_status_change_g2p_t, loop_value) }, \
         } \
}
#endif

/**
 * @brief Pack a task_status_change_g2p message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param spray_duration spray duration in task set by GCS
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_change_g2p_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float spray_duration, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN];
	_mav_put_float(buf, 0, spray_duration);
	_mav_put_uint8_t(buf, 4, task_status);
	_mav_put_uint8_t(buf, 5, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN);
#else
	mavlink_task_status_change_g2p_t packet;
	packet.spray_duration = spray_duration;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
}

/**
 * @brief Pack a task_status_change_g2p message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param spray_duration spray duration in task set by GCS
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_change_g2p_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float spray_duration,uint8_t task_status,uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN];
	_mav_put_float(buf, 0, spray_duration);
	_mav_put_uint8_t(buf, 4, task_status);
	_mav_put_uint8_t(buf, 5, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN);
#else
	mavlink_task_status_change_g2p_t packet;
	packet.spray_duration = spray_duration;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
}

/**
 * @brief Encode a task_status_change_g2p struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_status_change_g2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_change_g2p_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_status_change_g2p_t* task_status_change_g2p)
{
	return mavlink_msg_task_status_change_g2p_pack(system_id, component_id, msg, task_status_change_g2p->spray_duration, task_status_change_g2p->task_status, task_status_change_g2p->loop_value);
}

/**
 * @brief Encode a task_status_change_g2p struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_status_change_g2p C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_change_g2p_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_status_change_g2p_t* task_status_change_g2p)
{
	return mavlink_msg_task_status_change_g2p_pack_chan(system_id, component_id, chan, msg, task_status_change_g2p->spray_duration, task_status_change_g2p->task_status, task_status_change_g2p->loop_value);
}

/**
 * @brief Send a task_status_change_g2p message
 * @param chan MAVLink channel to send the message
 *
 * @param spray_duration spray duration in task set by GCS
 * @param task_status task status NO. about Task State Machine, 0~17
 * @param loop_value loop number of current task, 0~5
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_status_change_g2p_send(mavlink_channel_t chan, float spray_duration, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN];
	_mav_put_float(buf, 0, spray_duration);
	_mav_put_uint8_t(buf, 4, task_status);
	_mav_put_uint8_t(buf, 5, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P, buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
#else
	mavlink_task_status_change_g2p_t packet;
	packet.spray_duration = spray_duration;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
#endif
}

/**
 * @brief Send a task_status_change_g2p message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_status_change_g2p_send_struct(mavlink_channel_t chan, const mavlink_task_status_change_g2p_t* task_status_change_g2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_status_change_g2p_send(chan, task_status_change_g2p->spray_duration, task_status_change_g2p->task_status, task_status_change_g2p->loop_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P, (const char *)task_status_change_g2p, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_status_change_g2p_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float spray_duration, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, spray_duration);
	_mav_put_uint8_t(buf, 4, task_status);
	_mav_put_uint8_t(buf, 5, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P, buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
#else
	mavlink_task_status_change_g2p_t *packet = (mavlink_task_status_change_g2p_t *)msgbuf;
	packet->spray_duration = spray_duration;
	packet->task_status = task_status;
	packet->loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P, (const char *)packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATUS_CHANGE_G2P UNPACKING


/**
 * @brief Get field spray_duration from task_status_change_g2p message
 *
 * @return spray duration in task set by GCS
 */
static inline float mavlink_msg_task_status_change_g2p_get_spray_duration(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field task_status from task_status_change_g2p message
 *
 * @return task status NO. about Task State Machine, 0~17
 */
static inline uint8_t mavlink_msg_task_status_change_g2p_get_task_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field loop_value from task_status_change_g2p message
 *
 * @return loop number of current task, 0~5
 */
static inline uint8_t mavlink_msg_task_status_change_g2p_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a task_status_change_g2p message into a struct
 *
 * @param msg The message to decode
 * @param task_status_change_g2p C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_status_change_g2p_decode(const mavlink_message_t* msg, mavlink_task_status_change_g2p_t* task_status_change_g2p)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	task_status_change_g2p->spray_duration = mavlink_msg_task_status_change_g2p_get_spray_duration(msg);
	task_status_change_g2p->task_status = mavlink_msg_task_status_change_g2p_get_task_status(msg);
	task_status_change_g2p->loop_value = mavlink_msg_task_status_change_g2p_get_loop_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN;
        memset(task_status_change_g2p, 0, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_G2P_LEN);
	memcpy(task_status_change_g2p, _MAV_PAYLOAD(msg), len);
#endif
}
