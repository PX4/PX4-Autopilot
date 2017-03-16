#pragma once
// MESSAGE CPU_LOAD PACKING

#define MAVLINK_MSG_ID_CPU_LOAD 170

MAVPACKED(
typedef struct __mavlink_cpu_load_t {
 uint16_t batVolt; /*< Battery Voltage in millivolts*/
 uint8_t sensLoad; /*< Sensor DSC Load*/
 uint8_t ctrlLoad; /*< Control DSC Load*/
}) mavlink_cpu_load_t;

#define MAVLINK_MSG_ID_CPU_LOAD_LEN 4
#define MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN 4
#define MAVLINK_MSG_ID_170_LEN 4
#define MAVLINK_MSG_ID_170_MIN_LEN 4

#define MAVLINK_MSG_ID_CPU_LOAD_CRC 75
#define MAVLINK_MSG_ID_170_CRC 75



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CPU_LOAD { \
    170, \
    "CPU_LOAD", \
    3, \
    {  { "batVolt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cpu_load_t, batVolt) }, \
         { "sensLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_cpu_load_t, sensLoad) }, \
         { "ctrlLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_cpu_load_t, ctrlLoad) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CPU_LOAD { \
    "CPU_LOAD", \
    3, \
    {  { "batVolt", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_cpu_load_t, batVolt) }, \
         { "sensLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_cpu_load_t, sensLoad) }, \
         { "ctrlLoad", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_cpu_load_t, ctrlLoad) }, \
         } \
}
#endif

/**
 * @brief Pack a cpu_load message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_load_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_LOAD_LEN];
    _mav_put_uint16_t(buf, 0, batVolt);
    _mav_put_uint8_t(buf, 2, sensLoad);
    _mav_put_uint8_t(buf, 3, ctrlLoad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPU_LOAD_LEN);
#else
    mavlink_cpu_load_t packet;
    packet.batVolt = batVolt;
    packet.sensLoad = sensLoad;
    packet.ctrlLoad = ctrlLoad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPU_LOAD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
}

/**
 * @brief Pack a cpu_load message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_cpu_load_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sensLoad,uint8_t ctrlLoad,uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_LOAD_LEN];
    _mav_put_uint16_t(buf, 0, batVolt);
    _mav_put_uint8_t(buf, 2, sensLoad);
    _mav_put_uint8_t(buf, 3, ctrlLoad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CPU_LOAD_LEN);
#else
    mavlink_cpu_load_t packet;
    packet.batVolt = batVolt;
    packet.sensLoad = sensLoad;
    packet.ctrlLoad = ctrlLoad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CPU_LOAD_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CPU_LOAD;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
}

/**
 * @brief Encode a cpu_load struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param cpu_load C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_load_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_cpu_load_t* cpu_load)
{
    return mavlink_msg_cpu_load_pack(system_id, component_id, msg, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
}

/**
 * @brief Encode a cpu_load struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cpu_load C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_cpu_load_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_cpu_load_t* cpu_load)
{
    return mavlink_msg_cpu_load_pack_chan(system_id, component_id, chan, msg, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
}

/**
 * @brief Send a cpu_load message
 * @param chan MAVLink channel to send the message
 *
 * @param sensLoad Sensor DSC Load
 * @param ctrlLoad Control DSC Load
 * @param batVolt Battery Voltage in millivolts
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_cpu_load_send(mavlink_channel_t chan, uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CPU_LOAD_LEN];
    _mav_put_uint16_t(buf, 0, batVolt);
    _mav_put_uint8_t(buf, 2, sensLoad);
    _mav_put_uint8_t(buf, 3, ctrlLoad);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, buf, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
#else
    mavlink_cpu_load_t packet;
    packet.batVolt = batVolt;
    packet.sensLoad = sensLoad;
    packet.ctrlLoad = ctrlLoad;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, (const char *)&packet, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
#endif
}

/**
 * @brief Send a cpu_load message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_cpu_load_send_struct(mavlink_channel_t chan, const mavlink_cpu_load_t* cpu_load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_cpu_load_send(chan, cpu_load->sensLoad, cpu_load->ctrlLoad, cpu_load->batVolt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, (const char *)cpu_load, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
#endif
}

#if MAVLINK_MSG_ID_CPU_LOAD_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_cpu_load_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sensLoad, uint8_t ctrlLoad, uint16_t batVolt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, batVolt);
    _mav_put_uint8_t(buf, 2, sensLoad);
    _mav_put_uint8_t(buf, 3, ctrlLoad);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, buf, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
#else
    mavlink_cpu_load_t *packet = (mavlink_cpu_load_t *)msgbuf;
    packet->batVolt = batVolt;
    packet->sensLoad = sensLoad;
    packet->ctrlLoad = ctrlLoad;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CPU_LOAD, (const char *)packet, MAVLINK_MSG_ID_CPU_LOAD_MIN_LEN, MAVLINK_MSG_ID_CPU_LOAD_LEN, MAVLINK_MSG_ID_CPU_LOAD_CRC);
#endif
}
#endif

#endif

// MESSAGE CPU_LOAD UNPACKING


/**
 * @brief Get field sensLoad from cpu_load message
 *
 * @return Sensor DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_sensLoad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ctrlLoad from cpu_load message
 *
 * @return Control DSC Load
 */
static inline uint8_t mavlink_msg_cpu_load_get_ctrlLoad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field batVolt from cpu_load message
 *
 * @return Battery Voltage in millivolts
 */
static inline uint16_t mavlink_msg_cpu_load_get_batVolt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a cpu_load message into a struct
 *
 * @param msg The message to decode
 * @param cpu_load C-struct to decode the message contents into
 */
static inline void mavlink_msg_cpu_load_decode(const mavlink_message_t* msg, mavlink_cpu_load_t* cpu_load)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    cpu_load->batVolt = mavlink_msg_cpu_load_get_batVolt(msg);
    cpu_load->sensLoad = mavlink_msg_cpu_load_get_sensLoad(msg);
    cpu_load->ctrlLoad = mavlink_msg_cpu_load_get_ctrlLoad(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CPU_LOAD_LEN? msg->len : MAVLINK_MSG_ID_CPU_LOAD_LEN;
        memset(cpu_load, 0, MAVLINK_MSG_ID_CPU_LOAD_LEN);
    memcpy(cpu_load, _MAV_PAYLOAD(msg), len);
#endif
}
