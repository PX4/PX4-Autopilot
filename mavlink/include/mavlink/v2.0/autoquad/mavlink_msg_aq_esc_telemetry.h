#pragma once
// MESSAGE AQ_ESC_TELEMETRY PACKING

#define MAVLINK_MSG_ID_AQ_ESC_TELEMETRY 152

MAVPACKED(
typedef struct __mavlink_aq_esc_telemetry_t {
 uint32_t time_boot_ms; /*< Timestamp of the component clock since boot time in ms.*/
 uint32_t data0[4]; /*< Data bits 1-32 for each ESC.*/
 uint32_t data1[4]; /*< Data bits 33-64 for each ESC.*/
 uint16_t status_age[4]; /*< Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data.*/
 uint8_t seq; /*< Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).*/
 uint8_t num_motors; /*< Total number of active ESCs/motors on the system.*/
 uint8_t num_in_seq; /*< Number of active ESCs in this sequence (1 through this many array members will be populated with data)*/
 uint8_t escid[4]; /*< ESC/Motor ID*/
 uint8_t data_version[4]; /*< Version of data structure (determines contents).*/
}) mavlink_aq_esc_telemetry_t;

#define MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN 55
#define MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN 55
#define MAVLINK_MSG_ID_152_LEN 55
#define MAVLINK_MSG_ID_152_MIN_LEN 55

#define MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC 115
#define MAVLINK_MSG_ID_152_CRC 115

#define MAVLINK_MSG_AQ_ESC_TELEMETRY_FIELD_DATA0_LEN 4
#define MAVLINK_MSG_AQ_ESC_TELEMETRY_FIELD_DATA1_LEN 4
#define MAVLINK_MSG_AQ_ESC_TELEMETRY_FIELD_STATUS_AGE_LEN 4
#define MAVLINK_MSG_AQ_ESC_TELEMETRY_FIELD_ESCID_LEN 4
#define MAVLINK_MSG_AQ_ESC_TELEMETRY_FIELD_DATA_VERSION_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AQ_ESC_TELEMETRY { \
    152, \
    "AQ_ESC_TELEMETRY", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_aq_esc_telemetry_t, time_boot_ms) }, \
         { "data0", NULL, MAVLINK_TYPE_UINT32_T, 4, 4, offsetof(mavlink_aq_esc_telemetry_t, data0) }, \
         { "data1", NULL, MAVLINK_TYPE_UINT32_T, 4, 20, offsetof(mavlink_aq_esc_telemetry_t, data1) }, \
         { "status_age", NULL, MAVLINK_TYPE_UINT16_T, 4, 36, offsetof(mavlink_aq_esc_telemetry_t, status_age) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_aq_esc_telemetry_t, seq) }, \
         { "num_motors", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_aq_esc_telemetry_t, num_motors) }, \
         { "num_in_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_aq_esc_telemetry_t, num_in_seq) }, \
         { "escid", NULL, MAVLINK_TYPE_UINT8_T, 4, 47, offsetof(mavlink_aq_esc_telemetry_t, escid) }, \
         { "data_version", NULL, MAVLINK_TYPE_UINT8_T, 4, 51, offsetof(mavlink_aq_esc_telemetry_t, data_version) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AQ_ESC_TELEMETRY { \
    "AQ_ESC_TELEMETRY", \
    9, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_aq_esc_telemetry_t, time_boot_ms) }, \
         { "data0", NULL, MAVLINK_TYPE_UINT32_T, 4, 4, offsetof(mavlink_aq_esc_telemetry_t, data0) }, \
         { "data1", NULL, MAVLINK_TYPE_UINT32_T, 4, 20, offsetof(mavlink_aq_esc_telemetry_t, data1) }, \
         { "status_age", NULL, MAVLINK_TYPE_UINT16_T, 4, 36, offsetof(mavlink_aq_esc_telemetry_t, status_age) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_aq_esc_telemetry_t, seq) }, \
         { "num_motors", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_aq_esc_telemetry_t, num_motors) }, \
         { "num_in_seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_aq_esc_telemetry_t, num_in_seq) }, \
         { "escid", NULL, MAVLINK_TYPE_UINT8_T, 4, 47, offsetof(mavlink_aq_esc_telemetry_t, escid) }, \
         { "data_version", NULL, MAVLINK_TYPE_UINT8_T, 4, 51, offsetof(mavlink_aq_esc_telemetry_t, data_version) }, \
         } \
}
#endif

/**
 * @brief Pack a aq_esc_telemetry message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp of the component clock since boot time in ms.
 * @param seq Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
 * @param num_motors Total number of active ESCs/motors on the system.
 * @param num_in_seq Number of active ESCs in this sequence (1 through this many array members will be populated with data)
 * @param escid ESC/Motor ID
 * @param status_age Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data.
 * @param data_version Version of data structure (determines contents).
 * @param data0 Data bits 1-32 for each ESC.
 * @param data1 Data bits 33-64 for each ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, uint8_t seq, uint8_t num_motors, uint8_t num_in_seq, const uint8_t *escid, const uint16_t *status_age, const uint8_t *data_version, const uint32_t *data0, const uint32_t *data1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 44, seq);
    _mav_put_uint8_t(buf, 45, num_motors);
    _mav_put_uint8_t(buf, 46, num_in_seq);
    _mav_put_uint32_t_array(buf, 4, data0, 4);
    _mav_put_uint32_t_array(buf, 20, data1, 4);
    _mav_put_uint16_t_array(buf, 36, status_age, 4);
    _mav_put_uint8_t_array(buf, 47, escid, 4);
    _mav_put_uint8_t_array(buf, 51, data_version, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN);
#else
    mavlink_aq_esc_telemetry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.seq = seq;
    packet.num_motors = num_motors;
    packet.num_in_seq = num_in_seq;
    mav_array_memcpy(packet.data0, data0, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.data1, data1, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.status_age, status_age, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.escid, escid, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.data_version, data_version, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AQ_ESC_TELEMETRY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
}

/**
 * @brief Pack a aq_esc_telemetry message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp of the component clock since boot time in ms.
 * @param seq Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
 * @param num_motors Total number of active ESCs/motors on the system.
 * @param num_in_seq Number of active ESCs in this sequence (1 through this many array members will be populated with data)
 * @param escid ESC/Motor ID
 * @param status_age Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data.
 * @param data_version Version of data structure (determines contents).
 * @param data0 Data bits 1-32 for each ESC.
 * @param data1 Data bits 33-64 for each ESC.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,uint8_t seq,uint8_t num_motors,uint8_t num_in_seq,const uint8_t *escid,const uint16_t *status_age,const uint8_t *data_version,const uint32_t *data0,const uint32_t *data1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 44, seq);
    _mav_put_uint8_t(buf, 45, num_motors);
    _mav_put_uint8_t(buf, 46, num_in_seq);
    _mav_put_uint32_t_array(buf, 4, data0, 4);
    _mav_put_uint32_t_array(buf, 20, data1, 4);
    _mav_put_uint16_t_array(buf, 36, status_age, 4);
    _mav_put_uint8_t_array(buf, 47, escid, 4);
    _mav_put_uint8_t_array(buf, 51, data_version, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN);
#else
    mavlink_aq_esc_telemetry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.seq = seq;
    packet.num_motors = num_motors;
    packet.num_in_seq = num_in_seq;
    mav_array_memcpy(packet.data0, data0, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.data1, data1, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.status_age, status_age, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.escid, escid, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.data_version, data_version, sizeof(uint8_t)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AQ_ESC_TELEMETRY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
}

/**
 * @brief Encode a aq_esc_telemetry struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param aq_esc_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_aq_esc_telemetry_t* aq_esc_telemetry)
{
    return mavlink_msg_aq_esc_telemetry_pack(system_id, component_id, msg, aq_esc_telemetry->time_boot_ms, aq_esc_telemetry->seq, aq_esc_telemetry->num_motors, aq_esc_telemetry->num_in_seq, aq_esc_telemetry->escid, aq_esc_telemetry->status_age, aq_esc_telemetry->data_version, aq_esc_telemetry->data0, aq_esc_telemetry->data1);
}

/**
 * @brief Encode a aq_esc_telemetry struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param aq_esc_telemetry C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_aq_esc_telemetry_t* aq_esc_telemetry)
{
    return mavlink_msg_aq_esc_telemetry_pack_chan(system_id, component_id, chan, msg, aq_esc_telemetry->time_boot_ms, aq_esc_telemetry->seq, aq_esc_telemetry->num_motors, aq_esc_telemetry->num_in_seq, aq_esc_telemetry->escid, aq_esc_telemetry->status_age, aq_esc_telemetry->data_version, aq_esc_telemetry->data0, aq_esc_telemetry->data1);
}

/**
 * @brief Send a aq_esc_telemetry message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp of the component clock since boot time in ms.
 * @param seq Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
 * @param num_motors Total number of active ESCs/motors on the system.
 * @param num_in_seq Number of active ESCs in this sequence (1 through this many array members will be populated with data)
 * @param escid ESC/Motor ID
 * @param status_age Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data.
 * @param data_version Version of data structure (determines contents).
 * @param data0 Data bits 1-32 for each ESC.
 * @param data1 Data bits 33-64 for each ESC.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_aq_esc_telemetry_send(mavlink_channel_t chan, uint32_t time_boot_ms, uint8_t seq, uint8_t num_motors, uint8_t num_in_seq, const uint8_t *escid, const uint16_t *status_age, const uint8_t *data_version, const uint32_t *data0, const uint32_t *data1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 44, seq);
    _mav_put_uint8_t(buf, 45, num_motors);
    _mav_put_uint8_t(buf, 46, num_in_seq);
    _mav_put_uint32_t_array(buf, 4, data0, 4);
    _mav_put_uint32_t_array(buf, 20, data1, 4);
    _mav_put_uint16_t_array(buf, 36, status_age, 4);
    _mav_put_uint8_t_array(buf, 47, escid, 4);
    _mav_put_uint8_t_array(buf, 51, data_version, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY, buf, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
#else
    mavlink_aq_esc_telemetry_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.seq = seq;
    packet.num_motors = num_motors;
    packet.num_in_seq = num_in_seq;
    mav_array_memcpy(packet.data0, data0, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.data1, data1, sizeof(uint32_t)*4);
    mav_array_memcpy(packet.status_age, status_age, sizeof(uint16_t)*4);
    mav_array_memcpy(packet.escid, escid, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.data_version, data_version, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY, (const char *)&packet, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
#endif
}

/**
 * @brief Send a aq_esc_telemetry message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_aq_esc_telemetry_send_struct(mavlink_channel_t chan, const mavlink_aq_esc_telemetry_t* aq_esc_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_aq_esc_telemetry_send(chan, aq_esc_telemetry->time_boot_ms, aq_esc_telemetry->seq, aq_esc_telemetry->num_motors, aq_esc_telemetry->num_in_seq, aq_esc_telemetry->escid, aq_esc_telemetry->status_age, aq_esc_telemetry->data_version, aq_esc_telemetry->data0, aq_esc_telemetry->data1);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY, (const char *)aq_esc_telemetry, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
#endif
}

#if MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_aq_esc_telemetry_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, uint8_t seq, uint8_t num_motors, uint8_t num_in_seq, const uint8_t *escid, const uint16_t *status_age, const uint8_t *data_version, const uint32_t *data0, const uint32_t *data1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_uint8_t(buf, 44, seq);
    _mav_put_uint8_t(buf, 45, num_motors);
    _mav_put_uint8_t(buf, 46, num_in_seq);
    _mav_put_uint32_t_array(buf, 4, data0, 4);
    _mav_put_uint32_t_array(buf, 20, data1, 4);
    _mav_put_uint16_t_array(buf, 36, status_age, 4);
    _mav_put_uint8_t_array(buf, 47, escid, 4);
    _mav_put_uint8_t_array(buf, 51, data_version, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY, buf, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
#else
    mavlink_aq_esc_telemetry_t *packet = (mavlink_aq_esc_telemetry_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->seq = seq;
    packet->num_motors = num_motors;
    packet->num_in_seq = num_in_seq;
    mav_array_memcpy(packet->data0, data0, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->data1, data1, sizeof(uint32_t)*4);
    mav_array_memcpy(packet->status_age, status_age, sizeof(uint16_t)*4);
    mav_array_memcpy(packet->escid, escid, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->data_version, data_version, sizeof(uint8_t)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY, (const char *)packet, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_MIN_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_CRC);
#endif
}
#endif

#endif

// MESSAGE AQ_ESC_TELEMETRY UNPACKING


/**
 * @brief Get field time_boot_ms from aq_esc_telemetry message
 *
 * @return Timestamp of the component clock since boot time in ms.
 */
static inline uint32_t mavlink_msg_aq_esc_telemetry_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field seq from aq_esc_telemetry message
 *
 * @return Sequence number of message (first set of 4 motors is #1, next 4 is #2, etc).
 */
static inline uint8_t mavlink_msg_aq_esc_telemetry_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field num_motors from aq_esc_telemetry message
 *
 * @return Total number of active ESCs/motors on the system.
 */
static inline uint8_t mavlink_msg_aq_esc_telemetry_get_num_motors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field num_in_seq from aq_esc_telemetry message
 *
 * @return Number of active ESCs in this sequence (1 through this many array members will be populated with data)
 */
static inline uint8_t mavlink_msg_aq_esc_telemetry_get_num_in_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field escid from aq_esc_telemetry message
 *
 * @return ESC/Motor ID
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_get_escid(const mavlink_message_t* msg, uint8_t *escid)
{
    return _MAV_RETURN_uint8_t_array(msg, escid, 4,  47);
}

/**
 * @brief Get field status_age from aq_esc_telemetry message
 *
 * @return Age of each ESC telemetry reading in ms compared to boot time. A value of 0xFFFF means timeout/no data.
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_get_status_age(const mavlink_message_t* msg, uint16_t *status_age)
{
    return _MAV_RETURN_uint16_t_array(msg, status_age, 4,  36);
}

/**
 * @brief Get field data_version from aq_esc_telemetry message
 *
 * @return Version of data structure (determines contents).
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_get_data_version(const mavlink_message_t* msg, uint8_t *data_version)
{
    return _MAV_RETURN_uint8_t_array(msg, data_version, 4,  51);
}

/**
 * @brief Get field data0 from aq_esc_telemetry message
 *
 * @return Data bits 1-32 for each ESC.
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_get_data0(const mavlink_message_t* msg, uint32_t *data0)
{
    return _MAV_RETURN_uint32_t_array(msg, data0, 4,  4);
}

/**
 * @brief Get field data1 from aq_esc_telemetry message
 *
 * @return Data bits 33-64 for each ESC.
 */
static inline uint16_t mavlink_msg_aq_esc_telemetry_get_data1(const mavlink_message_t* msg, uint32_t *data1)
{
    return _MAV_RETURN_uint32_t_array(msg, data1, 4,  20);
}

/**
 * @brief Decode a aq_esc_telemetry message into a struct
 *
 * @param msg The message to decode
 * @param aq_esc_telemetry C-struct to decode the message contents into
 */
static inline void mavlink_msg_aq_esc_telemetry_decode(const mavlink_message_t* msg, mavlink_aq_esc_telemetry_t* aq_esc_telemetry)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    aq_esc_telemetry->time_boot_ms = mavlink_msg_aq_esc_telemetry_get_time_boot_ms(msg);
    mavlink_msg_aq_esc_telemetry_get_data0(msg, aq_esc_telemetry->data0);
    mavlink_msg_aq_esc_telemetry_get_data1(msg, aq_esc_telemetry->data1);
    mavlink_msg_aq_esc_telemetry_get_status_age(msg, aq_esc_telemetry->status_age);
    aq_esc_telemetry->seq = mavlink_msg_aq_esc_telemetry_get_seq(msg);
    aq_esc_telemetry->num_motors = mavlink_msg_aq_esc_telemetry_get_num_motors(msg);
    aq_esc_telemetry->num_in_seq = mavlink_msg_aq_esc_telemetry_get_num_in_seq(msg);
    mavlink_msg_aq_esc_telemetry_get_escid(msg, aq_esc_telemetry->escid);
    mavlink_msg_aq_esc_telemetry_get_data_version(msg, aq_esc_telemetry->data_version);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN? msg->len : MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN;
        memset(aq_esc_telemetry, 0, MAVLINK_MSG_ID_AQ_ESC_TELEMETRY_LEN);
    memcpy(aq_esc_telemetry, _MAV_PAYLOAD(msg), len);
#endif
}
