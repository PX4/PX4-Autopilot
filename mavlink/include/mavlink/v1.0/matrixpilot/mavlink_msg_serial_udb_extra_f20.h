#pragma once
// MESSAGE SERIAL_UDB_EXTRA_F20 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20 186

MAVPACKED(
typedef struct __mavlink_serial_udb_extra_f20_t {
 int16_t sue_trim_value_input_1; /*< SUE UDB PWM Trim Value on Input 1*/
 int16_t sue_trim_value_input_2; /*< SUE UDB PWM Trim Value on Input 2*/
 int16_t sue_trim_value_input_3; /*< SUE UDB PWM Trim Value on Input 3*/
 int16_t sue_trim_value_input_4; /*< SUE UDB PWM Trim Value on Input 4*/
 int16_t sue_trim_value_input_5; /*< SUE UDB PWM Trim Value on Input 5*/
 int16_t sue_trim_value_input_6; /*< SUE UDB PWM Trim Value on Input 6*/
 int16_t sue_trim_value_input_7; /*< SUE UDB PWM Trim Value on Input 7*/
 int16_t sue_trim_value_input_8; /*< SUE UDB PWM Trim Value on Input 8*/
 int16_t sue_trim_value_input_9; /*< SUE UDB PWM Trim Value on Input 9*/
 int16_t sue_trim_value_input_10; /*< SUE UDB PWM Trim Value on Input 10*/
 int16_t sue_trim_value_input_11; /*< SUE UDB PWM Trim Value on Input 11*/
 int16_t sue_trim_value_input_12; /*< SUE UDB PWM Trim Value on Input 12*/
 uint8_t sue_number_of_inputs; /*< SUE Number of Input Channels*/
}) mavlink_serial_udb_extra_f20_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN 25
#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN 25
#define MAVLINK_MSG_ID_186_LEN 25
#define MAVLINK_MSG_ID_186_MIN_LEN 25

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC 144
#define MAVLINK_MSG_ID_186_CRC 144



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F20 { \
    186, \
    "SERIAL_UDB_EXTRA_F20", \
    13, \
    {  { "sue_trim_value_input_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_1) }, \
         { "sue_trim_value_input_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_2) }, \
         { "sue_trim_value_input_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_3) }, \
         { "sue_trim_value_input_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_4) }, \
         { "sue_trim_value_input_5", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_5) }, \
         { "sue_trim_value_input_6", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_6) }, \
         { "sue_trim_value_input_7", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_7) }, \
         { "sue_trim_value_input_8", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_8) }, \
         { "sue_trim_value_input_9", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_9) }, \
         { "sue_trim_value_input_10", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_10) }, \
         { "sue_trim_value_input_11", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_11) }, \
         { "sue_trim_value_input_12", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_12) }, \
         { "sue_number_of_inputs", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_serial_udb_extra_f20_t, sue_number_of_inputs) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F20 { \
    "SERIAL_UDB_EXTRA_F20", \
    13, \
    {  { "sue_trim_value_input_1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_1) }, \
         { "sue_trim_value_input_2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_2) }, \
         { "sue_trim_value_input_3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_3) }, \
         { "sue_trim_value_input_4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_4) }, \
         { "sue_trim_value_input_5", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_5) }, \
         { "sue_trim_value_input_6", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_6) }, \
         { "sue_trim_value_input_7", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_7) }, \
         { "sue_trim_value_input_8", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_8) }, \
         { "sue_trim_value_input_9", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_9) }, \
         { "sue_trim_value_input_10", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_10) }, \
         { "sue_trim_value_input_11", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_11) }, \
         { "sue_trim_value_input_12", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_serial_udb_extra_f20_t, sue_trim_value_input_12) }, \
         { "sue_number_of_inputs", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_serial_udb_extra_f20_t, sue_number_of_inputs) }, \
         } \
}
#endif

/**
 * @brief Pack a serial_udb_extra_f20 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_number_of_inputs SUE Number of Input Channels
 * @param sue_trim_value_input_1 SUE UDB PWM Trim Value on Input 1
 * @param sue_trim_value_input_2 SUE UDB PWM Trim Value on Input 2
 * @param sue_trim_value_input_3 SUE UDB PWM Trim Value on Input 3
 * @param sue_trim_value_input_4 SUE UDB PWM Trim Value on Input 4
 * @param sue_trim_value_input_5 SUE UDB PWM Trim Value on Input 5
 * @param sue_trim_value_input_6 SUE UDB PWM Trim Value on Input 6
 * @param sue_trim_value_input_7 SUE UDB PWM Trim Value on Input 7
 * @param sue_trim_value_input_8 SUE UDB PWM Trim Value on Input 8
 * @param sue_trim_value_input_9 SUE UDB PWM Trim Value on Input 9
 * @param sue_trim_value_input_10 SUE UDB PWM Trim Value on Input 10
 * @param sue_trim_value_input_11 SUE UDB PWM Trim Value on Input 11
 * @param sue_trim_value_input_12 SUE UDB PWM Trim Value on Input 12
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f20_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sue_number_of_inputs, int16_t sue_trim_value_input_1, int16_t sue_trim_value_input_2, int16_t sue_trim_value_input_3, int16_t sue_trim_value_input_4, int16_t sue_trim_value_input_5, int16_t sue_trim_value_input_6, int16_t sue_trim_value_input_7, int16_t sue_trim_value_input_8, int16_t sue_trim_value_input_9, int16_t sue_trim_value_input_10, int16_t sue_trim_value_input_11, int16_t sue_trim_value_input_12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN];
    _mav_put_int16_t(buf, 0, sue_trim_value_input_1);
    _mav_put_int16_t(buf, 2, sue_trim_value_input_2);
    _mav_put_int16_t(buf, 4, sue_trim_value_input_3);
    _mav_put_int16_t(buf, 6, sue_trim_value_input_4);
    _mav_put_int16_t(buf, 8, sue_trim_value_input_5);
    _mav_put_int16_t(buf, 10, sue_trim_value_input_6);
    _mav_put_int16_t(buf, 12, sue_trim_value_input_7);
    _mav_put_int16_t(buf, 14, sue_trim_value_input_8);
    _mav_put_int16_t(buf, 16, sue_trim_value_input_9);
    _mav_put_int16_t(buf, 18, sue_trim_value_input_10);
    _mav_put_int16_t(buf, 20, sue_trim_value_input_11);
    _mav_put_int16_t(buf, 22, sue_trim_value_input_12);
    _mav_put_uint8_t(buf, 24, sue_number_of_inputs);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN);
#else
    mavlink_serial_udb_extra_f20_t packet;
    packet.sue_trim_value_input_1 = sue_trim_value_input_1;
    packet.sue_trim_value_input_2 = sue_trim_value_input_2;
    packet.sue_trim_value_input_3 = sue_trim_value_input_3;
    packet.sue_trim_value_input_4 = sue_trim_value_input_4;
    packet.sue_trim_value_input_5 = sue_trim_value_input_5;
    packet.sue_trim_value_input_6 = sue_trim_value_input_6;
    packet.sue_trim_value_input_7 = sue_trim_value_input_7;
    packet.sue_trim_value_input_8 = sue_trim_value_input_8;
    packet.sue_trim_value_input_9 = sue_trim_value_input_9;
    packet.sue_trim_value_input_10 = sue_trim_value_input_10;
    packet.sue_trim_value_input_11 = sue_trim_value_input_11;
    packet.sue_trim_value_input_12 = sue_trim_value_input_12;
    packet.sue_number_of_inputs = sue_number_of_inputs;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
}

/**
 * @brief Pack a serial_udb_extra_f20 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_number_of_inputs SUE Number of Input Channels
 * @param sue_trim_value_input_1 SUE UDB PWM Trim Value on Input 1
 * @param sue_trim_value_input_2 SUE UDB PWM Trim Value on Input 2
 * @param sue_trim_value_input_3 SUE UDB PWM Trim Value on Input 3
 * @param sue_trim_value_input_4 SUE UDB PWM Trim Value on Input 4
 * @param sue_trim_value_input_5 SUE UDB PWM Trim Value on Input 5
 * @param sue_trim_value_input_6 SUE UDB PWM Trim Value on Input 6
 * @param sue_trim_value_input_7 SUE UDB PWM Trim Value on Input 7
 * @param sue_trim_value_input_8 SUE UDB PWM Trim Value on Input 8
 * @param sue_trim_value_input_9 SUE UDB PWM Trim Value on Input 9
 * @param sue_trim_value_input_10 SUE UDB PWM Trim Value on Input 10
 * @param sue_trim_value_input_11 SUE UDB PWM Trim Value on Input 11
 * @param sue_trim_value_input_12 SUE UDB PWM Trim Value on Input 12
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f20_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sue_number_of_inputs,int16_t sue_trim_value_input_1,int16_t sue_trim_value_input_2,int16_t sue_trim_value_input_3,int16_t sue_trim_value_input_4,int16_t sue_trim_value_input_5,int16_t sue_trim_value_input_6,int16_t sue_trim_value_input_7,int16_t sue_trim_value_input_8,int16_t sue_trim_value_input_9,int16_t sue_trim_value_input_10,int16_t sue_trim_value_input_11,int16_t sue_trim_value_input_12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN];
    _mav_put_int16_t(buf, 0, sue_trim_value_input_1);
    _mav_put_int16_t(buf, 2, sue_trim_value_input_2);
    _mav_put_int16_t(buf, 4, sue_trim_value_input_3);
    _mav_put_int16_t(buf, 6, sue_trim_value_input_4);
    _mav_put_int16_t(buf, 8, sue_trim_value_input_5);
    _mav_put_int16_t(buf, 10, sue_trim_value_input_6);
    _mav_put_int16_t(buf, 12, sue_trim_value_input_7);
    _mav_put_int16_t(buf, 14, sue_trim_value_input_8);
    _mav_put_int16_t(buf, 16, sue_trim_value_input_9);
    _mav_put_int16_t(buf, 18, sue_trim_value_input_10);
    _mav_put_int16_t(buf, 20, sue_trim_value_input_11);
    _mav_put_int16_t(buf, 22, sue_trim_value_input_12);
    _mav_put_uint8_t(buf, 24, sue_number_of_inputs);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN);
#else
    mavlink_serial_udb_extra_f20_t packet;
    packet.sue_trim_value_input_1 = sue_trim_value_input_1;
    packet.sue_trim_value_input_2 = sue_trim_value_input_2;
    packet.sue_trim_value_input_3 = sue_trim_value_input_3;
    packet.sue_trim_value_input_4 = sue_trim_value_input_4;
    packet.sue_trim_value_input_5 = sue_trim_value_input_5;
    packet.sue_trim_value_input_6 = sue_trim_value_input_6;
    packet.sue_trim_value_input_7 = sue_trim_value_input_7;
    packet.sue_trim_value_input_8 = sue_trim_value_input_8;
    packet.sue_trim_value_input_9 = sue_trim_value_input_9;
    packet.sue_trim_value_input_10 = sue_trim_value_input_10;
    packet.sue_trim_value_input_11 = sue_trim_value_input_11;
    packet.sue_trim_value_input_12 = sue_trim_value_input_12;
    packet.sue_number_of_inputs = sue_number_of_inputs;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
}

/**
 * @brief Encode a serial_udb_extra_f20 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f20 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f20_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f20_t* serial_udb_extra_f20)
{
    return mavlink_msg_serial_udb_extra_f20_pack(system_id, component_id, msg, serial_udb_extra_f20->sue_number_of_inputs, serial_udb_extra_f20->sue_trim_value_input_1, serial_udb_extra_f20->sue_trim_value_input_2, serial_udb_extra_f20->sue_trim_value_input_3, serial_udb_extra_f20->sue_trim_value_input_4, serial_udb_extra_f20->sue_trim_value_input_5, serial_udb_extra_f20->sue_trim_value_input_6, serial_udb_extra_f20->sue_trim_value_input_7, serial_udb_extra_f20->sue_trim_value_input_8, serial_udb_extra_f20->sue_trim_value_input_9, serial_udb_extra_f20->sue_trim_value_input_10, serial_udb_extra_f20->sue_trim_value_input_11, serial_udb_extra_f20->sue_trim_value_input_12);
}

/**
 * @brief Encode a serial_udb_extra_f20 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f20 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f20_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f20_t* serial_udb_extra_f20)
{
    return mavlink_msg_serial_udb_extra_f20_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f20->sue_number_of_inputs, serial_udb_extra_f20->sue_trim_value_input_1, serial_udb_extra_f20->sue_trim_value_input_2, serial_udb_extra_f20->sue_trim_value_input_3, serial_udb_extra_f20->sue_trim_value_input_4, serial_udb_extra_f20->sue_trim_value_input_5, serial_udb_extra_f20->sue_trim_value_input_6, serial_udb_extra_f20->sue_trim_value_input_7, serial_udb_extra_f20->sue_trim_value_input_8, serial_udb_extra_f20->sue_trim_value_input_9, serial_udb_extra_f20->sue_trim_value_input_10, serial_udb_extra_f20->sue_trim_value_input_11, serial_udb_extra_f20->sue_trim_value_input_12);
}

/**
 * @brief Send a serial_udb_extra_f20 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_number_of_inputs SUE Number of Input Channels
 * @param sue_trim_value_input_1 SUE UDB PWM Trim Value on Input 1
 * @param sue_trim_value_input_2 SUE UDB PWM Trim Value on Input 2
 * @param sue_trim_value_input_3 SUE UDB PWM Trim Value on Input 3
 * @param sue_trim_value_input_4 SUE UDB PWM Trim Value on Input 4
 * @param sue_trim_value_input_5 SUE UDB PWM Trim Value on Input 5
 * @param sue_trim_value_input_6 SUE UDB PWM Trim Value on Input 6
 * @param sue_trim_value_input_7 SUE UDB PWM Trim Value on Input 7
 * @param sue_trim_value_input_8 SUE UDB PWM Trim Value on Input 8
 * @param sue_trim_value_input_9 SUE UDB PWM Trim Value on Input 9
 * @param sue_trim_value_input_10 SUE UDB PWM Trim Value on Input 10
 * @param sue_trim_value_input_11 SUE UDB PWM Trim Value on Input 11
 * @param sue_trim_value_input_12 SUE UDB PWM Trim Value on Input 12
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f20_send(mavlink_channel_t chan, uint8_t sue_number_of_inputs, int16_t sue_trim_value_input_1, int16_t sue_trim_value_input_2, int16_t sue_trim_value_input_3, int16_t sue_trim_value_input_4, int16_t sue_trim_value_input_5, int16_t sue_trim_value_input_6, int16_t sue_trim_value_input_7, int16_t sue_trim_value_input_8, int16_t sue_trim_value_input_9, int16_t sue_trim_value_input_10, int16_t sue_trim_value_input_11, int16_t sue_trim_value_input_12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN];
    _mav_put_int16_t(buf, 0, sue_trim_value_input_1);
    _mav_put_int16_t(buf, 2, sue_trim_value_input_2);
    _mav_put_int16_t(buf, 4, sue_trim_value_input_3);
    _mav_put_int16_t(buf, 6, sue_trim_value_input_4);
    _mav_put_int16_t(buf, 8, sue_trim_value_input_5);
    _mav_put_int16_t(buf, 10, sue_trim_value_input_6);
    _mav_put_int16_t(buf, 12, sue_trim_value_input_7);
    _mav_put_int16_t(buf, 14, sue_trim_value_input_8);
    _mav_put_int16_t(buf, 16, sue_trim_value_input_9);
    _mav_put_int16_t(buf, 18, sue_trim_value_input_10);
    _mav_put_int16_t(buf, 20, sue_trim_value_input_11);
    _mav_put_int16_t(buf, 22, sue_trim_value_input_12);
    _mav_put_uint8_t(buf, 24, sue_number_of_inputs);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
#else
    mavlink_serial_udb_extra_f20_t packet;
    packet.sue_trim_value_input_1 = sue_trim_value_input_1;
    packet.sue_trim_value_input_2 = sue_trim_value_input_2;
    packet.sue_trim_value_input_3 = sue_trim_value_input_3;
    packet.sue_trim_value_input_4 = sue_trim_value_input_4;
    packet.sue_trim_value_input_5 = sue_trim_value_input_5;
    packet.sue_trim_value_input_6 = sue_trim_value_input_6;
    packet.sue_trim_value_input_7 = sue_trim_value_input_7;
    packet.sue_trim_value_input_8 = sue_trim_value_input_8;
    packet.sue_trim_value_input_9 = sue_trim_value_input_9;
    packet.sue_trim_value_input_10 = sue_trim_value_input_10;
    packet.sue_trim_value_input_11 = sue_trim_value_input_11;
    packet.sue_trim_value_input_12 = sue_trim_value_input_12;
    packet.sue_number_of_inputs = sue_number_of_inputs;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
#endif
}

/**
 * @brief Send a serial_udb_extra_f20 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_serial_udb_extra_f20_send_struct(mavlink_channel_t chan, const mavlink_serial_udb_extra_f20_t* serial_udb_extra_f20)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_serial_udb_extra_f20_send(chan, serial_udb_extra_f20->sue_number_of_inputs, serial_udb_extra_f20->sue_trim_value_input_1, serial_udb_extra_f20->sue_trim_value_input_2, serial_udb_extra_f20->sue_trim_value_input_3, serial_udb_extra_f20->sue_trim_value_input_4, serial_udb_extra_f20->sue_trim_value_input_5, serial_udb_extra_f20->sue_trim_value_input_6, serial_udb_extra_f20->sue_trim_value_input_7, serial_udb_extra_f20->sue_trim_value_input_8, serial_udb_extra_f20->sue_trim_value_input_9, serial_udb_extra_f20->sue_trim_value_input_10, serial_udb_extra_f20->sue_trim_value_input_11, serial_udb_extra_f20->sue_trim_value_input_12);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20, (const char *)serial_udb_extra_f20, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f20_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sue_number_of_inputs, int16_t sue_trim_value_input_1, int16_t sue_trim_value_input_2, int16_t sue_trim_value_input_3, int16_t sue_trim_value_input_4, int16_t sue_trim_value_input_5, int16_t sue_trim_value_input_6, int16_t sue_trim_value_input_7, int16_t sue_trim_value_input_8, int16_t sue_trim_value_input_9, int16_t sue_trim_value_input_10, int16_t sue_trim_value_input_11, int16_t sue_trim_value_input_12)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, sue_trim_value_input_1);
    _mav_put_int16_t(buf, 2, sue_trim_value_input_2);
    _mav_put_int16_t(buf, 4, sue_trim_value_input_3);
    _mav_put_int16_t(buf, 6, sue_trim_value_input_4);
    _mav_put_int16_t(buf, 8, sue_trim_value_input_5);
    _mav_put_int16_t(buf, 10, sue_trim_value_input_6);
    _mav_put_int16_t(buf, 12, sue_trim_value_input_7);
    _mav_put_int16_t(buf, 14, sue_trim_value_input_8);
    _mav_put_int16_t(buf, 16, sue_trim_value_input_9);
    _mav_put_int16_t(buf, 18, sue_trim_value_input_10);
    _mav_put_int16_t(buf, 20, sue_trim_value_input_11);
    _mav_put_int16_t(buf, 22, sue_trim_value_input_12);
    _mav_put_uint8_t(buf, 24, sue_number_of_inputs);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
#else
    mavlink_serial_udb_extra_f20_t *packet = (mavlink_serial_udb_extra_f20_t *)msgbuf;
    packet->sue_trim_value_input_1 = sue_trim_value_input_1;
    packet->sue_trim_value_input_2 = sue_trim_value_input_2;
    packet->sue_trim_value_input_3 = sue_trim_value_input_3;
    packet->sue_trim_value_input_4 = sue_trim_value_input_4;
    packet->sue_trim_value_input_5 = sue_trim_value_input_5;
    packet->sue_trim_value_input_6 = sue_trim_value_input_6;
    packet->sue_trim_value_input_7 = sue_trim_value_input_7;
    packet->sue_trim_value_input_8 = sue_trim_value_input_8;
    packet->sue_trim_value_input_9 = sue_trim_value_input_9;
    packet->sue_trim_value_input_10 = sue_trim_value_input_10;
    packet->sue_trim_value_input_11 = sue_trim_value_input_11;
    packet->sue_trim_value_input_12 = sue_trim_value_input_12;
    packet->sue_number_of_inputs = sue_number_of_inputs;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_MIN_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_CRC);
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F20 UNPACKING


/**
 * @brief Get field sue_number_of_inputs from serial_udb_extra_f20 message
 *
 * @return SUE Number of Input Channels
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f20_get_sue_number_of_inputs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field sue_trim_value_input_1 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 1
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field sue_trim_value_input_2 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 2
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field sue_trim_value_input_3 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 3
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field sue_trim_value_input_4 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 4
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field sue_trim_value_input_5 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 5
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field sue_trim_value_input_6 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 6
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field sue_trim_value_input_7 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 7
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field sue_trim_value_input_8 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 8
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field sue_trim_value_input_9 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 9
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_9(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field sue_trim_value_input_10 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 10
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field sue_trim_value_input_11 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 11
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_11(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field sue_trim_value_input_12 from serial_udb_extra_f20 message
 *
 * @return SUE UDB PWM Trim Value on Input 12
 */
static inline int16_t mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_12(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Decode a serial_udb_extra_f20 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f20 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f20_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f20_t* serial_udb_extra_f20)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    serial_udb_extra_f20->sue_trim_value_input_1 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_1(msg);
    serial_udb_extra_f20->sue_trim_value_input_2 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_2(msg);
    serial_udb_extra_f20->sue_trim_value_input_3 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_3(msg);
    serial_udb_extra_f20->sue_trim_value_input_4 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_4(msg);
    serial_udb_extra_f20->sue_trim_value_input_5 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_5(msg);
    serial_udb_extra_f20->sue_trim_value_input_6 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_6(msg);
    serial_udb_extra_f20->sue_trim_value_input_7 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_7(msg);
    serial_udb_extra_f20->sue_trim_value_input_8 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_8(msg);
    serial_udb_extra_f20->sue_trim_value_input_9 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_9(msg);
    serial_udb_extra_f20->sue_trim_value_input_10 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_10(msg);
    serial_udb_extra_f20->sue_trim_value_input_11 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_11(msg);
    serial_udb_extra_f20->sue_trim_value_input_12 = mavlink_msg_serial_udb_extra_f20_get_sue_trim_value_input_12(msg);
    serial_udb_extra_f20->sue_number_of_inputs = mavlink_msg_serial_udb_extra_f20_get_sue_number_of_inputs(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN? msg->len : MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN;
        memset(serial_udb_extra_f20, 0, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F20_LEN);
    memcpy(serial_udb_extra_f20, _MAV_PAYLOAD(msg), len);
#endif
}
