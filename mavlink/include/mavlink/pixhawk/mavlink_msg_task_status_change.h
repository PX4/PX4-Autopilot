// MESSAGE TASK_STATUS_CHANGE PACKING

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE 224

MAVPACKED(
typedef struct __mavlink_task_status_change_t {
 uint8_t num_odd_even; /*< 0, 1 or 2 represent spray all number, odd number or even number*/
 uint8_t task_status; /*< task status NO. about Task State Machine, 0~16~30*/
 uint8_t loop_value; /*< loop number of current task, 0~5*/
}) mavlink_task_status_change_t;

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN 3
#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN 3
#define MAVLINK_MSG_ID_224_LEN 3
#define MAVLINK_MSG_ID_224_MIN_LEN 3

#define MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC 251
#define MAVLINK_MSG_ID_224_CRC 251



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_CHANGE { \
	224, \
	"TASK_STATUS_CHANGE", \
	3, \
	{  { "num_odd_even", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_task_status_change_t, num_odd_even) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_task_status_change_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_task_status_change_t, loop_value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATUS_CHANGE { \
	"TASK_STATUS_CHANGE", \
	3, \
	{  { "num_odd_even", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_task_status_change_t, num_odd_even) }, \
         { "task_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_task_status_change_t, task_status) }, \
         { "loop_value", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_task_status_change_t, loop_value) }, \
         } \
}
#endif

/**
 * @brief Pack a task_status_change message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_change_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN];
	_mav_put_uint8_t(buf, 0, num_odd_even);
	_mav_put_uint8_t(buf, 1, task_status);
	_mav_put_uint8_t(buf, 2, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN);
#else
	mavlink_task_status_change_t packet;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_CHANGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
}

/**
 * @brief Pack a task_status_change message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_status_change_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t num_odd_even,uint8_t task_status,uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN];
	_mav_put_uint8_t(buf, 0, num_odd_even);
	_mav_put_uint8_t(buf, 1, task_status);
	_mav_put_uint8_t(buf, 2, loop_value);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN);
#else
	mavlink_task_status_change_t packet;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TASK_STATUS_CHANGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
}

/**
 * @brief Encode a task_status_change struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_status_change C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_change_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_status_change_t* task_status_change)
{
	return mavlink_msg_task_status_change_pack(system_id, component_id, msg, task_status_change->num_odd_even, task_status_change->task_status, task_status_change->loop_value);
}

/**
 * @brief Encode a task_status_change struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_status_change C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_status_change_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_status_change_t* task_status_change)
{
	return mavlink_msg_task_status_change_pack_chan(system_id, component_id, chan, msg, task_status_change->num_odd_even, task_status_change->task_status, task_status_change->loop_value);
}

/**
 * @brief Send a task_status_change message
 * @param chan MAVLink channel to send the message
 *
 * @param num_odd_even 0, 1 or 2 represent spray all number, odd number or even number
 * @param task_status task status NO. about Task State Machine, 0~16~30
 * @param loop_value loop number of current task, 0~5
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_status_change_send(mavlink_channel_t chan, uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN];
	_mav_put_uint8_t(buf, 0, num_odd_even);
	_mav_put_uint8_t(buf, 1, task_status);
	_mav_put_uint8_t(buf, 2, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE, buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
#else
	mavlink_task_status_change_t packet;
	packet.num_odd_even = num_odd_even;
	packet.task_status = task_status;
	packet.loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
#endif
}

/**
 * @brief Send a task_status_change message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_status_change_send_struct(mavlink_channel_t chan, const mavlink_task_status_change_t* task_status_change)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_status_change_send(chan, task_status_change->num_odd_even, task_status_change->task_status, task_status_change->loop_value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE, (const char *)task_status_change, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_status_change_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t num_odd_even, uint8_t task_status, uint8_t loop_value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, num_odd_even);
	_mav_put_uint8_t(buf, 1, task_status);
	_mav_put_uint8_t(buf, 2, loop_value);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE, buf, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
#else
	mavlink_task_status_change_t *packet = (mavlink_task_status_change_t *)msgbuf;
	packet->num_odd_even = num_odd_even;
	packet->task_status = task_status;
	packet->loop_value = loop_value;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATUS_CHANGE, (const char *)packet, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_MIN_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATUS_CHANGE UNPACKING


/**
 * @brief Get field num_odd_even from task_status_change message
 *
 * @return 0, 1 or 2 represent spray all number, odd number or even number
 */
static inline uint8_t mavlink_msg_task_status_change_get_num_odd_even(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field task_status from task_status_change message
 *
 * @return task status NO. about Task State Machine, 0~16~30
 */
static inline uint8_t mavlink_msg_task_status_change_get_task_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field loop_value from task_status_change message
 *
 * @return loop number of current task, 0~5
 */
static inline uint8_t mavlink_msg_task_status_change_get_loop_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a task_status_change message into a struct
 *
 * @param msg The message to decode
 * @param task_status_change C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_status_change_decode(const mavlink_message_t* msg, mavlink_task_status_change_t* task_status_change)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	task_status_change->num_odd_even = mavlink_msg_task_status_change_get_num_odd_even(msg);
	task_status_change->task_status = mavlink_msg_task_status_change_get_task_status(msg);
	task_status_change->loop_value = mavlink_msg_task_status_change_get_loop_value(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN;
        memset(task_status_change, 0, MAVLINK_MSG_ID_TASK_STATUS_CHANGE_LEN);
	memcpy(task_status_change, _MAV_PAYLOAD(msg), len);
#endif
}
