#pragma once
// MESSAGE STATUS_GPS PACKING

#define MAVLINK_MSG_ID_STATUS_GPS 194

MAVPACKED(
typedef struct __mavlink_status_gps_t {
 float magVar; /*< Magnetic variation, degrees */
 uint16_t csFails; /*< Number of times checksum has failed*/
 uint8_t gpsQuality; /*< The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a*/
 uint8_t msgsType; /*<  Indicates if GN, GL or GP messages are being received*/
 uint8_t posStatus; /*<  A = data valid, V = data invalid*/
 int8_t magDir; /*<  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course*/
 uint8_t modeInd; /*<  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid*/
}) mavlink_status_gps_t;

#define MAVLINK_MSG_ID_STATUS_GPS_LEN 11
#define MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN 11
#define MAVLINK_MSG_ID_194_LEN 11
#define MAVLINK_MSG_ID_194_MIN_LEN 11

#define MAVLINK_MSG_ID_STATUS_GPS_CRC 51
#define MAVLINK_MSG_ID_194_CRC 51



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STATUS_GPS { \
    194, \
    "STATUS_GPS", \
    7, \
    {  { "magVar", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_status_gps_t, magVar) }, \
         { "csFails", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_status_gps_t, csFails) }, \
         { "gpsQuality", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_status_gps_t, gpsQuality) }, \
         { "msgsType", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_status_gps_t, msgsType) }, \
         { "posStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_status_gps_t, posStatus) }, \
         { "magDir", NULL, MAVLINK_TYPE_INT8_T, 0, 9, offsetof(mavlink_status_gps_t, magDir) }, \
         { "modeInd", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_status_gps_t, modeInd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STATUS_GPS { \
    "STATUS_GPS", \
    7, \
    {  { "magVar", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_status_gps_t, magVar) }, \
         { "csFails", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_status_gps_t, csFails) }, \
         { "gpsQuality", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_status_gps_t, gpsQuality) }, \
         { "msgsType", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_status_gps_t, msgsType) }, \
         { "posStatus", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_status_gps_t, posStatus) }, \
         { "magDir", NULL, MAVLINK_TYPE_INT8_T, 0, 9, offsetof(mavlink_status_gps_t, magDir) }, \
         { "modeInd", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_status_gps_t, modeInd) }, \
         } \
}
#endif

/**
 * @brief Pack a status_gps message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param csFails Number of times checksum has failed
 * @param gpsQuality The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
 * @param msgsType  Indicates if GN, GL or GP messages are being received
 * @param posStatus  A = data valid, V = data invalid
 * @param magVar Magnetic variation, degrees 
 * @param magDir  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
 * @param modeInd  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_gps_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t csFails, uint8_t gpsQuality, uint8_t msgsType, uint8_t posStatus, float magVar, int8_t magDir, uint8_t modeInd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_GPS_LEN];
    _mav_put_float(buf, 0, magVar);
    _mav_put_uint16_t(buf, 4, csFails);
    _mav_put_uint8_t(buf, 6, gpsQuality);
    _mav_put_uint8_t(buf, 7, msgsType);
    _mav_put_uint8_t(buf, 8, posStatus);
    _mav_put_int8_t(buf, 9, magDir);
    _mav_put_uint8_t(buf, 10, modeInd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUS_GPS_LEN);
#else
    mavlink_status_gps_t packet;
    packet.magVar = magVar;
    packet.csFails = csFails;
    packet.gpsQuality = gpsQuality;
    packet.msgsType = msgsType;
    packet.posStatus = posStatus;
    packet.magDir = magDir;
    packet.modeInd = modeInd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUS_GPS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUS_GPS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
}

/**
 * @brief Pack a status_gps message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param csFails Number of times checksum has failed
 * @param gpsQuality The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
 * @param msgsType  Indicates if GN, GL or GP messages are being received
 * @param posStatus  A = data valid, V = data invalid
 * @param magVar Magnetic variation, degrees 
 * @param magDir  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
 * @param modeInd  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_gps_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t csFails,uint8_t gpsQuality,uint8_t msgsType,uint8_t posStatus,float magVar,int8_t magDir,uint8_t modeInd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_GPS_LEN];
    _mav_put_float(buf, 0, magVar);
    _mav_put_uint16_t(buf, 4, csFails);
    _mav_put_uint8_t(buf, 6, gpsQuality);
    _mav_put_uint8_t(buf, 7, msgsType);
    _mav_put_uint8_t(buf, 8, posStatus);
    _mav_put_int8_t(buf, 9, magDir);
    _mav_put_uint8_t(buf, 10, modeInd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STATUS_GPS_LEN);
#else
    mavlink_status_gps_t packet;
    packet.magVar = magVar;
    packet.csFails = csFails;
    packet.gpsQuality = gpsQuality;
    packet.msgsType = msgsType;
    packet.posStatus = posStatus;
    packet.magDir = magDir;
    packet.modeInd = modeInd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STATUS_GPS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STATUS_GPS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
}

/**
 * @brief Encode a status_gps struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param status_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_status_gps_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_status_gps_t* status_gps)
{
    return mavlink_msg_status_gps_pack(system_id, component_id, msg, status_gps->csFails, status_gps->gpsQuality, status_gps->msgsType, status_gps->posStatus, status_gps->magVar, status_gps->magDir, status_gps->modeInd);
}

/**
 * @brief Encode a status_gps struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status_gps C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_status_gps_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_status_gps_t* status_gps)
{
    return mavlink_msg_status_gps_pack_chan(system_id, component_id, chan, msg, status_gps->csFails, status_gps->gpsQuality, status_gps->msgsType, status_gps->posStatus, status_gps->magVar, status_gps->magDir, status_gps->modeInd);
}

/**
 * @brief Send a status_gps message
 * @param chan MAVLink channel to send the message
 *
 * @param csFails Number of times checksum has failed
 * @param gpsQuality The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
 * @param msgsType  Indicates if GN, GL or GP messages are being received
 * @param posStatus  A = data valid, V = data invalid
 * @param magVar Magnetic variation, degrees 
 * @param magDir  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
 * @param modeInd  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_status_gps_send(mavlink_channel_t chan, uint16_t csFails, uint8_t gpsQuality, uint8_t msgsType, uint8_t posStatus, float magVar, int8_t magDir, uint8_t modeInd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STATUS_GPS_LEN];
    _mav_put_float(buf, 0, magVar);
    _mav_put_uint16_t(buf, 4, csFails);
    _mav_put_uint8_t(buf, 6, gpsQuality);
    _mav_put_uint8_t(buf, 7, msgsType);
    _mav_put_uint8_t(buf, 8, posStatus);
    _mav_put_int8_t(buf, 9, magDir);
    _mav_put_uint8_t(buf, 10, modeInd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_GPS, buf, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
#else
    mavlink_status_gps_t packet;
    packet.magVar = magVar;
    packet.csFails = csFails;
    packet.gpsQuality = gpsQuality;
    packet.msgsType = msgsType;
    packet.posStatus = posStatus;
    packet.magDir = magDir;
    packet.modeInd = modeInd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_GPS, (const char *)&packet, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
#endif
}

/**
 * @brief Send a status_gps message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_status_gps_send_struct(mavlink_channel_t chan, const mavlink_status_gps_t* status_gps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_status_gps_send(chan, status_gps->csFails, status_gps->gpsQuality, status_gps->msgsType, status_gps->posStatus, status_gps->magVar, status_gps->magDir, status_gps->modeInd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_GPS, (const char *)status_gps, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
#endif
}

#if MAVLINK_MSG_ID_STATUS_GPS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_status_gps_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t csFails, uint8_t gpsQuality, uint8_t msgsType, uint8_t posStatus, float magVar, int8_t magDir, uint8_t modeInd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, magVar);
    _mav_put_uint16_t(buf, 4, csFails);
    _mav_put_uint8_t(buf, 6, gpsQuality);
    _mav_put_uint8_t(buf, 7, msgsType);
    _mav_put_uint8_t(buf, 8, posStatus);
    _mav_put_int8_t(buf, 9, magDir);
    _mav_put_uint8_t(buf, 10, modeInd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_GPS, buf, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
#else
    mavlink_status_gps_t *packet = (mavlink_status_gps_t *)msgbuf;
    packet->magVar = magVar;
    packet->csFails = csFails;
    packet->gpsQuality = gpsQuality;
    packet->msgsType = msgsType;
    packet->posStatus = posStatus;
    packet->magDir = magDir;
    packet->modeInd = modeInd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_GPS, (const char *)packet, MAVLINK_MSG_ID_STATUS_GPS_MIN_LEN, MAVLINK_MSG_ID_STATUS_GPS_LEN, MAVLINK_MSG_ID_STATUS_GPS_CRC);
#endif
}
#endif

#endif

// MESSAGE STATUS_GPS UNPACKING


/**
 * @brief Get field csFails from status_gps message
 *
 * @return Number of times checksum has failed
 */
static inline uint16_t mavlink_msg_status_gps_get_csFails(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field gpsQuality from status_gps message
 *
 * @return The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
 */
static inline uint8_t mavlink_msg_status_gps_get_gpsQuality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field msgsType from status_gps message
 *
 * @return  Indicates if GN, GL or GP messages are being received
 */
static inline uint8_t mavlink_msg_status_gps_get_msgsType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field posStatus from status_gps message
 *
 * @return  A = data valid, V = data invalid
 */
static inline uint8_t mavlink_msg_status_gps_get_posStatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field magVar from status_gps message
 *
 * @return Magnetic variation, degrees 
 */
static inline float mavlink_msg_status_gps_get_magVar(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field magDir from status_gps message
 *
 * @return  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
 */
static inline int8_t mavlink_msg_status_gps_get_magDir(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  9);
}

/**
 * @brief Get field modeInd from status_gps message
 *
 * @return  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
 */
static inline uint8_t mavlink_msg_status_gps_get_modeInd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Decode a status_gps message into a struct
 *
 * @param msg The message to decode
 * @param status_gps C-struct to decode the message contents into
 */
static inline void mavlink_msg_status_gps_decode(const mavlink_message_t* msg, mavlink_status_gps_t* status_gps)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    status_gps->magVar = mavlink_msg_status_gps_get_magVar(msg);
    status_gps->csFails = mavlink_msg_status_gps_get_csFails(msg);
    status_gps->gpsQuality = mavlink_msg_status_gps_get_gpsQuality(msg);
    status_gps->msgsType = mavlink_msg_status_gps_get_msgsType(msg);
    status_gps->posStatus = mavlink_msg_status_gps_get_posStatus(msg);
    status_gps->magDir = mavlink_msg_status_gps_get_magDir(msg);
    status_gps->modeInd = mavlink_msg_status_gps_get_modeInd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STATUS_GPS_LEN? msg->len : MAVLINK_MSG_ID_STATUS_GPS_LEN;
        memset(status_gps, 0, MAVLINK_MSG_ID_STATUS_GPS_LEN);
    memcpy(status_gps, _MAV_PAYLOAD(msg), len);
#endif
}
