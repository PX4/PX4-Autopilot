// MESSAGE HC_STATE_STATUS PACKING

#define MAVLINK_MSG_ID_HC_STATE_STATUS 218

MAVPACKED(
typedef struct __mavlink_hc_state_status_t {
 uint64_t timestamp; /*< Timestamp*/
 float pos_sp_x; /*< pos setpoint x*/
 float pos_sp_y; /*< pos setpoint y*/
 float pos_sp_z; /*< pos setpoint z*/
 float vel_sp_x; /*< vel setpoint x*/
 float vel_sp_y; /*< vel setpoint y*/
 float vel_sp_z; /*< vel setpoint z*/
 uint8_t buckets_vaild; /*< buckets pos valid status.*/
 uint8_t cmd_recevied; /*< commander receiver status.*/
 uint8_t poll_recevied; /*< poll commander received status.*/
 uint8_t main_state; /*< state machine current main state, return cmd.*/
 uint8_t sec_state; /*< state machine current sec state, return cmd.*/
 uint8_t pos_en; /*< pos controller enable*/
 uint8_t vel_en; /*< vel controller enable*/
}) mavlink_hc_state_status_t;

#define MAVLINK_MSG_ID_HC_STATE_STATUS_LEN 39
#define MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN 39
#define MAVLINK_MSG_ID_218_LEN 39
#define MAVLINK_MSG_ID_218_MIN_LEN 39

#define MAVLINK_MSG_ID_HC_STATE_STATUS_CRC 15
#define MAVLINK_MSG_ID_218_CRC 15



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HC_STATE_STATUS { \
	218, \
	"HC_STATE_STATUS", \
	14, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hc_state_status_t, timestamp) }, \
         { "pos_sp_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hc_state_status_t, pos_sp_x) }, \
         { "pos_sp_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hc_state_status_t, pos_sp_y) }, \
         { "pos_sp_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hc_state_status_t, pos_sp_z) }, \
         { "vel_sp_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hc_state_status_t, vel_sp_x) }, \
         { "vel_sp_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hc_state_status_t, vel_sp_y) }, \
         { "vel_sp_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hc_state_status_t, vel_sp_z) }, \
         { "buckets_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_hc_state_status_t, buckets_vaild) }, \
         { "cmd_recevied", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_hc_state_status_t, cmd_recevied) }, \
         { "poll_recevied", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_hc_state_status_t, poll_recevied) }, \
         { "main_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_hc_state_status_t, main_state) }, \
         { "sec_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_hc_state_status_t, sec_state) }, \
         { "pos_en", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_hc_state_status_t, pos_en) }, \
         { "vel_en", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_hc_state_status_t, vel_en) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HC_STATE_STATUS { \
	"HC_STATE_STATUS", \
	14, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hc_state_status_t, timestamp) }, \
         { "pos_sp_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hc_state_status_t, pos_sp_x) }, \
         { "pos_sp_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hc_state_status_t, pos_sp_y) }, \
         { "pos_sp_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hc_state_status_t, pos_sp_z) }, \
         { "vel_sp_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hc_state_status_t, vel_sp_x) }, \
         { "vel_sp_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hc_state_status_t, vel_sp_y) }, \
         { "vel_sp_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hc_state_status_t, vel_sp_z) }, \
         { "buckets_vaild", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_hc_state_status_t, buckets_vaild) }, \
         { "cmd_recevied", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_hc_state_status_t, cmd_recevied) }, \
         { "poll_recevied", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_hc_state_status_t, poll_recevied) }, \
         { "main_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_hc_state_status_t, main_state) }, \
         { "sec_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_hc_state_status_t, sec_state) }, \
         { "pos_en", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_hc_state_status_t, pos_en) }, \
         { "vel_en", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_hc_state_status_t, vel_en) }, \
         } \
}
#endif

/**
 * @brief Pack a hc_state_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param buckets_vaild buckets pos valid status.
 * @param cmd_recevied commander receiver status.
 * @param poll_recevied poll commander received status.
 * @param main_state state machine current main state, return cmd.
 * @param sec_state state machine current sec state, return cmd.
 * @param pos_sp_x pos setpoint x
 * @param pos_sp_y pos setpoint y
 * @param pos_sp_z pos setpoint z
 * @param vel_sp_x vel setpoint x
 * @param vel_sp_y vel setpoint y
 * @param vel_sp_z vel setpoint z
 * @param pos_en pos controller enable
 * @param vel_en vel controller enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hc_state_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint8_t buckets_vaild, uint8_t cmd_recevied, uint8_t poll_recevied, uint8_t main_state, uint8_t sec_state, float pos_sp_x, float pos_sp_y, float pos_sp_z, float vel_sp_x, float vel_sp_y, float vel_sp_z, uint8_t pos_en, uint8_t vel_en)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HC_STATE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, pos_sp_x);
	_mav_put_float(buf, 12, pos_sp_y);
	_mav_put_float(buf, 16, pos_sp_z);
	_mav_put_float(buf, 20, vel_sp_x);
	_mav_put_float(buf, 24, vel_sp_y);
	_mav_put_float(buf, 28, vel_sp_z);
	_mav_put_uint8_t(buf, 32, buckets_vaild);
	_mav_put_uint8_t(buf, 33, cmd_recevied);
	_mav_put_uint8_t(buf, 34, poll_recevied);
	_mav_put_uint8_t(buf, 35, main_state);
	_mav_put_uint8_t(buf, 36, sec_state);
	_mav_put_uint8_t(buf, 37, pos_en);
	_mav_put_uint8_t(buf, 38, vel_en);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN);
#else
	mavlink_hc_state_status_t packet;
	packet.timestamp = timestamp;
	packet.pos_sp_x = pos_sp_x;
	packet.pos_sp_y = pos_sp_y;
	packet.pos_sp_z = pos_sp_z;
	packet.vel_sp_x = vel_sp_x;
	packet.vel_sp_y = vel_sp_y;
	packet.vel_sp_z = vel_sp_z;
	packet.buckets_vaild = buckets_vaild;
	packet.cmd_recevied = cmd_recevied;
	packet.poll_recevied = poll_recevied;
	packet.main_state = main_state;
	packet.sec_state = sec_state;
	packet.pos_en = pos_en;
	packet.vel_en = vel_en;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HC_STATE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
}

/**
 * @brief Pack a hc_state_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param buckets_vaild buckets pos valid status.
 * @param cmd_recevied commander receiver status.
 * @param poll_recevied poll commander received status.
 * @param main_state state machine current main state, return cmd.
 * @param sec_state state machine current sec state, return cmd.
 * @param pos_sp_x pos setpoint x
 * @param pos_sp_y pos setpoint y
 * @param pos_sp_z pos setpoint z
 * @param vel_sp_x vel setpoint x
 * @param vel_sp_y vel setpoint y
 * @param vel_sp_z vel setpoint z
 * @param pos_en pos controller enable
 * @param vel_en vel controller enable
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hc_state_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint8_t buckets_vaild,uint8_t cmd_recevied,uint8_t poll_recevied,uint8_t main_state,uint8_t sec_state,float pos_sp_x,float pos_sp_y,float pos_sp_z,float vel_sp_x,float vel_sp_y,float vel_sp_z,uint8_t pos_en,uint8_t vel_en)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HC_STATE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, pos_sp_x);
	_mav_put_float(buf, 12, pos_sp_y);
	_mav_put_float(buf, 16, pos_sp_z);
	_mav_put_float(buf, 20, vel_sp_x);
	_mav_put_float(buf, 24, vel_sp_y);
	_mav_put_float(buf, 28, vel_sp_z);
	_mav_put_uint8_t(buf, 32, buckets_vaild);
	_mav_put_uint8_t(buf, 33, cmd_recevied);
	_mav_put_uint8_t(buf, 34, poll_recevied);
	_mav_put_uint8_t(buf, 35, main_state);
	_mav_put_uint8_t(buf, 36, sec_state);
	_mav_put_uint8_t(buf, 37, pos_en);
	_mav_put_uint8_t(buf, 38, vel_en);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN);
#else
	mavlink_hc_state_status_t packet;
	packet.timestamp = timestamp;
	packet.pos_sp_x = pos_sp_x;
	packet.pos_sp_y = pos_sp_y;
	packet.pos_sp_z = pos_sp_z;
	packet.vel_sp_x = vel_sp_x;
	packet.vel_sp_y = vel_sp_y;
	packet.vel_sp_z = vel_sp_z;
	packet.buckets_vaild = buckets_vaild;
	packet.cmd_recevied = cmd_recevied;
	packet.poll_recevied = poll_recevied;
	packet.main_state = main_state;
	packet.sec_state = sec_state;
	packet.pos_en = pos_en;
	packet.vel_en = vel_en;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_HC_STATE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
}

/**
 * @brief Encode a hc_state_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hc_state_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hc_state_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hc_state_status_t* hc_state_status)
{
	return mavlink_msg_hc_state_status_pack(system_id, component_id, msg, hc_state_status->timestamp, hc_state_status->buckets_vaild, hc_state_status->cmd_recevied, hc_state_status->poll_recevied, hc_state_status->main_state, hc_state_status->sec_state, hc_state_status->pos_sp_x, hc_state_status->pos_sp_y, hc_state_status->pos_sp_z, hc_state_status->vel_sp_x, hc_state_status->vel_sp_y, hc_state_status->vel_sp_z, hc_state_status->pos_en, hc_state_status->vel_en);
}

/**
 * @brief Encode a hc_state_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hc_state_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hc_state_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hc_state_status_t* hc_state_status)
{
	return mavlink_msg_hc_state_status_pack_chan(system_id, component_id, chan, msg, hc_state_status->timestamp, hc_state_status->buckets_vaild, hc_state_status->cmd_recevied, hc_state_status->poll_recevied, hc_state_status->main_state, hc_state_status->sec_state, hc_state_status->pos_sp_x, hc_state_status->pos_sp_y, hc_state_status->pos_sp_z, hc_state_status->vel_sp_x, hc_state_status->vel_sp_y, hc_state_status->vel_sp_z, hc_state_status->pos_en, hc_state_status->vel_en);
}

/**
 * @brief Send a hc_state_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param buckets_vaild buckets pos valid status.
 * @param cmd_recevied commander receiver status.
 * @param poll_recevied poll commander received status.
 * @param main_state state machine current main state, return cmd.
 * @param sec_state state machine current sec state, return cmd.
 * @param pos_sp_x pos setpoint x
 * @param pos_sp_y pos setpoint y
 * @param pos_sp_z pos setpoint z
 * @param vel_sp_x vel setpoint x
 * @param vel_sp_y vel setpoint y
 * @param vel_sp_z vel setpoint z
 * @param pos_en pos controller enable
 * @param vel_en vel controller enable
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hc_state_status_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t buckets_vaild, uint8_t cmd_recevied, uint8_t poll_recevied, uint8_t main_state, uint8_t sec_state, float pos_sp_x, float pos_sp_y, float pos_sp_z, float vel_sp_x, float vel_sp_y, float vel_sp_z, uint8_t pos_en, uint8_t vel_en)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_HC_STATE_STATUS_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, pos_sp_x);
	_mav_put_float(buf, 12, pos_sp_y);
	_mav_put_float(buf, 16, pos_sp_z);
	_mav_put_float(buf, 20, vel_sp_x);
	_mav_put_float(buf, 24, vel_sp_y);
	_mav_put_float(buf, 28, vel_sp_z);
	_mav_put_uint8_t(buf, 32, buckets_vaild);
	_mav_put_uint8_t(buf, 33, cmd_recevied);
	_mav_put_uint8_t(buf, 34, poll_recevied);
	_mav_put_uint8_t(buf, 35, main_state);
	_mav_put_uint8_t(buf, 36, sec_state);
	_mav_put_uint8_t(buf, 37, pos_en);
	_mav_put_uint8_t(buf, 38, vel_en);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HC_STATE_STATUS, buf, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
#else
	mavlink_hc_state_status_t packet;
	packet.timestamp = timestamp;
	packet.pos_sp_x = pos_sp_x;
	packet.pos_sp_y = pos_sp_y;
	packet.pos_sp_z = pos_sp_z;
	packet.vel_sp_x = vel_sp_x;
	packet.vel_sp_y = vel_sp_y;
	packet.vel_sp_z = vel_sp_z;
	packet.buckets_vaild = buckets_vaild;
	packet.cmd_recevied = cmd_recevied;
	packet.poll_recevied = poll_recevied;
	packet.main_state = main_state;
	packet.sec_state = sec_state;
	packet.pos_en = pos_en;
	packet.vel_en = vel_en;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HC_STATE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
#endif
}

/**
 * @brief Send a hc_state_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hc_state_status_send_struct(mavlink_channel_t chan, const mavlink_hc_state_status_t* hc_state_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hc_state_status_send(chan, hc_state_status->timestamp, hc_state_status->buckets_vaild, hc_state_status->cmd_recevied, hc_state_status->poll_recevied, hc_state_status->main_state, hc_state_status->sec_state, hc_state_status->pos_sp_x, hc_state_status->pos_sp_y, hc_state_status->pos_sp_z, hc_state_status->vel_sp_x, hc_state_status->vel_sp_y, hc_state_status->vel_sp_z, hc_state_status->pos_en, hc_state_status->vel_en);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HC_STATE_STATUS, (const char *)hc_state_status, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_HC_STATE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hc_state_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t buckets_vaild, uint8_t cmd_recevied, uint8_t poll_recevied, uint8_t main_state, uint8_t sec_state, float pos_sp_x, float pos_sp_y, float pos_sp_z, float vel_sp_x, float vel_sp_y, float vel_sp_z, uint8_t pos_en, uint8_t vel_en)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_float(buf, 8, pos_sp_x);
	_mav_put_float(buf, 12, pos_sp_y);
	_mav_put_float(buf, 16, pos_sp_z);
	_mav_put_float(buf, 20, vel_sp_x);
	_mav_put_float(buf, 24, vel_sp_y);
	_mav_put_float(buf, 28, vel_sp_z);
	_mav_put_uint8_t(buf, 32, buckets_vaild);
	_mav_put_uint8_t(buf, 33, cmd_recevied);
	_mav_put_uint8_t(buf, 34, poll_recevied);
	_mav_put_uint8_t(buf, 35, main_state);
	_mav_put_uint8_t(buf, 36, sec_state);
	_mav_put_uint8_t(buf, 37, pos_en);
	_mav_put_uint8_t(buf, 38, vel_en);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HC_STATE_STATUS, buf, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
#else
	mavlink_hc_state_status_t *packet = (mavlink_hc_state_status_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->pos_sp_x = pos_sp_x;
	packet->pos_sp_y = pos_sp_y;
	packet->pos_sp_z = pos_sp_z;
	packet->vel_sp_x = vel_sp_x;
	packet->vel_sp_y = vel_sp_y;
	packet->vel_sp_z = vel_sp_z;
	packet->buckets_vaild = buckets_vaild;
	packet->cmd_recevied = cmd_recevied;
	packet->poll_recevied = poll_recevied;
	packet->main_state = main_state;
	packet->sec_state = sec_state;
	packet->pos_en = pos_en;
	packet->vel_en = vel_en;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HC_STATE_STATUS, (const char *)packet, MAVLINK_MSG_ID_HC_STATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN, MAVLINK_MSG_ID_HC_STATE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE HC_STATE_STATUS UNPACKING


/**
 * @brief Get field timestamp from hc_state_status message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_hc_state_status_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field buckets_vaild from hc_state_status message
 *
 * @return buckets pos valid status.
 */
static inline uint8_t mavlink_msg_hc_state_status_get_buckets_vaild(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field cmd_recevied from hc_state_status message
 *
 * @return commander receiver status.
 */
static inline uint8_t mavlink_msg_hc_state_status_get_cmd_recevied(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field poll_recevied from hc_state_status message
 *
 * @return poll commander received status.
 */
static inline uint8_t mavlink_msg_hc_state_status_get_poll_recevied(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field main_state from hc_state_status message
 *
 * @return state machine current main state, return cmd.
 */
static inline uint8_t mavlink_msg_hc_state_status_get_main_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field sec_state from hc_state_status message
 *
 * @return state machine current sec state, return cmd.
 */
static inline uint8_t mavlink_msg_hc_state_status_get_sec_state(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field pos_sp_x from hc_state_status message
 *
 * @return pos setpoint x
 */
static inline float mavlink_msg_hc_state_status_get_pos_sp_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_sp_y from hc_state_status message
 *
 * @return pos setpoint y
 */
static inline float mavlink_msg_hc_state_status_get_pos_sp_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pos_sp_z from hc_state_status message
 *
 * @return pos setpoint z
 */
static inline float mavlink_msg_hc_state_status_get_pos_sp_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vel_sp_x from hc_state_status message
 *
 * @return vel setpoint x
 */
static inline float mavlink_msg_hc_state_status_get_vel_sp_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vel_sp_y from hc_state_status message
 *
 * @return vel setpoint y
 */
static inline float mavlink_msg_hc_state_status_get_vel_sp_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vel_sp_z from hc_state_status message
 *
 * @return vel setpoint z
 */
static inline float mavlink_msg_hc_state_status_get_vel_sp_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pos_en from hc_state_status message
 *
 * @return pos controller enable
 */
static inline uint8_t mavlink_msg_hc_state_status_get_pos_en(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field vel_en from hc_state_status message
 *
 * @return vel controller enable
 */
static inline uint8_t mavlink_msg_hc_state_status_get_vel_en(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Decode a hc_state_status message into a struct
 *
 * @param msg The message to decode
 * @param hc_state_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_hc_state_status_decode(const mavlink_message_t* msg, mavlink_hc_state_status_t* hc_state_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	hc_state_status->timestamp = mavlink_msg_hc_state_status_get_timestamp(msg);
	hc_state_status->pos_sp_x = mavlink_msg_hc_state_status_get_pos_sp_x(msg);
	hc_state_status->pos_sp_y = mavlink_msg_hc_state_status_get_pos_sp_y(msg);
	hc_state_status->pos_sp_z = mavlink_msg_hc_state_status_get_pos_sp_z(msg);
	hc_state_status->vel_sp_x = mavlink_msg_hc_state_status_get_vel_sp_x(msg);
	hc_state_status->vel_sp_y = mavlink_msg_hc_state_status_get_vel_sp_y(msg);
	hc_state_status->vel_sp_z = mavlink_msg_hc_state_status_get_vel_sp_z(msg);
	hc_state_status->buckets_vaild = mavlink_msg_hc_state_status_get_buckets_vaild(msg);
	hc_state_status->cmd_recevied = mavlink_msg_hc_state_status_get_cmd_recevied(msg);
	hc_state_status->poll_recevied = mavlink_msg_hc_state_status_get_poll_recevied(msg);
	hc_state_status->main_state = mavlink_msg_hc_state_status_get_main_state(msg);
	hc_state_status->sec_state = mavlink_msg_hc_state_status_get_sec_state(msg);
	hc_state_status->pos_en = mavlink_msg_hc_state_status_get_pos_en(msg);
	hc_state_status->vel_en = mavlink_msg_hc_state_status_get_vel_en(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HC_STATE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_HC_STATE_STATUS_LEN;
        memset(hc_state_status, 0, MAVLINK_MSG_ID_HC_STATE_STATUS_LEN);
	memcpy(hc_state_status, _MAV_PAYLOAD(msg), len);
#endif
}
