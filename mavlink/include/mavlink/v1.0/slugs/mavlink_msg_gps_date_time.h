#pragma once
// MESSAGE GPS_DATE_TIME PACKING

#define MAVLINK_MSG_ID_GPS_DATE_TIME 179

MAVPACKED(
typedef struct __mavlink_gps_date_time_t {
 uint8_t year; /*< Year reported by Gps */
 uint8_t month; /*< Month reported by Gps */
 uint8_t day; /*< Day reported by Gps */
 uint8_t hour; /*< Hour reported by Gps */
 uint8_t min; /*< Min reported by Gps */
 uint8_t sec; /*< Sec reported by Gps  */
 uint8_t clockStat; /*< Clock Status. See table 47 page 211 OEMStar Manual  */
 uint8_t visSat; /*< Visible satellites reported by Gps  */
 uint8_t useSat; /*< Used satellites in Solution  */
 uint8_t GppGl; /*< GPS+GLONASS satellites in Solution  */
 uint8_t sigUsedMask; /*< GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)*/
 uint8_t percentUsed; /*< Percent used GPS*/
}) mavlink_gps_date_time_t;

#define MAVLINK_MSG_ID_GPS_DATE_TIME_LEN 12
#define MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN 12
#define MAVLINK_MSG_ID_179_LEN 12
#define MAVLINK_MSG_ID_179_MIN_LEN 12

#define MAVLINK_MSG_ID_GPS_DATE_TIME_CRC 132
#define MAVLINK_MSG_ID_179_CRC 132



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_DATE_TIME { \
    179, \
    "GPS_DATE_TIME", \
    12, \
    {  { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_date_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_date_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gps_date_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gps_date_time_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gps_date_time_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gps_date_time_t, sec) }, \
         { "clockStat", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gps_date_time_t, clockStat) }, \
         { "visSat", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_gps_date_time_t, visSat) }, \
         { "useSat", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gps_date_time_t, useSat) }, \
         { "GppGl", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gps_date_time_t, GppGl) }, \
         { "sigUsedMask", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gps_date_time_t, sigUsedMask) }, \
         { "percentUsed", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gps_date_time_t, percentUsed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_DATE_TIME { \
    "GPS_DATE_TIME", \
    12, \
    {  { "year", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_date_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gps_date_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gps_date_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gps_date_time_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_gps_date_time_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_gps_date_time_t, sec) }, \
         { "clockStat", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_gps_date_time_t, clockStat) }, \
         { "visSat", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_gps_date_time_t, visSat) }, \
         { "useSat", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gps_date_time_t, useSat) }, \
         { "GppGl", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gps_date_time_t, GppGl) }, \
         { "sigUsedMask", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gps_date_time_t, sigUsedMask) }, \
         { "percentUsed", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gps_date_time_t, percentUsed) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_date_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param clockStat Clock Status. See table 47 page 211 OEMStar Manual  
 * @param visSat Visible satellites reported by Gps  
 * @param useSat Used satellites in Solution  
 * @param GppGl GPS+GLONASS satellites in Solution  
 * @param sigUsedMask GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
 * @param percentUsed Percent used GPS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_date_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t clockStat, uint8_t visSat, uint8_t useSat, uint8_t GppGl, uint8_t sigUsedMask, uint8_t percentUsed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATE_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, min);
    _mav_put_uint8_t(buf, 5, sec);
    _mav_put_uint8_t(buf, 6, clockStat);
    _mav_put_uint8_t(buf, 7, visSat);
    _mav_put_uint8_t(buf, 8, useSat);
    _mav_put_uint8_t(buf, 9, GppGl);
    _mav_put_uint8_t(buf, 10, sigUsedMask);
    _mav_put_uint8_t(buf, 11, percentUsed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN);
#else
    mavlink_gps_date_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;
    packet.clockStat = clockStat;
    packet.visSat = visSat;
    packet.useSat = useSat;
    packet.GppGl = GppGl;
    packet.sigUsedMask = sigUsedMask;
    packet.percentUsed = percentUsed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_DATE_TIME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
}

/**
 * @brief Pack a gps_date_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param clockStat Clock Status. See table 47 page 211 OEMStar Manual  
 * @param visSat Visible satellites reported by Gps  
 * @param useSat Used satellites in Solution  
 * @param GppGl GPS+GLONASS satellites in Solution  
 * @param sigUsedMask GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
 * @param percentUsed Percent used GPS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_date_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec,uint8_t clockStat,uint8_t visSat,uint8_t useSat,uint8_t GppGl,uint8_t sigUsedMask,uint8_t percentUsed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATE_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, min);
    _mav_put_uint8_t(buf, 5, sec);
    _mav_put_uint8_t(buf, 6, clockStat);
    _mav_put_uint8_t(buf, 7, visSat);
    _mav_put_uint8_t(buf, 8, useSat);
    _mav_put_uint8_t(buf, 9, GppGl);
    _mav_put_uint8_t(buf, 10, sigUsedMask);
    _mav_put_uint8_t(buf, 11, percentUsed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN);
#else
    mavlink_gps_date_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;
    packet.clockStat = clockStat;
    packet.visSat = visSat;
    packet.useSat = useSat;
    packet.GppGl = GppGl;
    packet.sigUsedMask = sigUsedMask;
    packet.percentUsed = percentUsed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_DATE_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
}

/**
 * @brief Encode a gps_date_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_date_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_date_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_date_time_t* gps_date_time)
{
    return mavlink_msg_gps_date_time_pack(system_id, component_id, msg, gps_date_time->year, gps_date_time->month, gps_date_time->day, gps_date_time->hour, gps_date_time->min, gps_date_time->sec, gps_date_time->clockStat, gps_date_time->visSat, gps_date_time->useSat, gps_date_time->GppGl, gps_date_time->sigUsedMask, gps_date_time->percentUsed);
}

/**
 * @brief Encode a gps_date_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_date_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_date_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_date_time_t* gps_date_time)
{
    return mavlink_msg_gps_date_time_pack_chan(system_id, component_id, chan, msg, gps_date_time->year, gps_date_time->month, gps_date_time->day, gps_date_time->hour, gps_date_time->min, gps_date_time->sec, gps_date_time->clockStat, gps_date_time->visSat, gps_date_time->useSat, gps_date_time->GppGl, gps_date_time->sigUsedMask, gps_date_time->percentUsed);
}

/**
 * @brief Send a gps_date_time message
 * @param chan MAVLink channel to send the message
 *
 * @param year Year reported by Gps 
 * @param month Month reported by Gps 
 * @param day Day reported by Gps 
 * @param hour Hour reported by Gps 
 * @param min Min reported by Gps 
 * @param sec Sec reported by Gps  
 * @param clockStat Clock Status. See table 47 page 211 OEMStar Manual  
 * @param visSat Visible satellites reported by Gps  
 * @param useSat Used satellites in Solution  
 * @param GppGl GPS+GLONASS satellites in Solution  
 * @param sigUsedMask GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
 * @param percentUsed Percent used GPS
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_date_time_send(mavlink_channel_t chan, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t clockStat, uint8_t visSat, uint8_t useSat, uint8_t GppGl, uint8_t sigUsedMask, uint8_t percentUsed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATE_TIME_LEN];
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, min);
    _mav_put_uint8_t(buf, 5, sec);
    _mav_put_uint8_t(buf, 6, clockStat);
    _mav_put_uint8_t(buf, 7, visSat);
    _mav_put_uint8_t(buf, 8, useSat);
    _mav_put_uint8_t(buf, 9, GppGl);
    _mav_put_uint8_t(buf, 10, sigUsedMask);
    _mav_put_uint8_t(buf, 11, percentUsed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, buf, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
#else
    mavlink_gps_date_time_t packet;
    packet.year = year;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;
    packet.clockStat = clockStat;
    packet.visSat = visSat;
    packet.useSat = useSat;
    packet.GppGl = GppGl;
    packet.sigUsedMask = sigUsedMask;
    packet.percentUsed = percentUsed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, (const char *)&packet, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
#endif
}

/**
 * @brief Send a gps_date_time message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_date_time_send_struct(mavlink_channel_t chan, const mavlink_gps_date_time_t* gps_date_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_date_time_send(chan, gps_date_time->year, gps_date_time->month, gps_date_time->day, gps_date_time->hour, gps_date_time->min, gps_date_time->sec, gps_date_time->clockStat, gps_date_time->visSat, gps_date_time->useSat, gps_date_time->GppGl, gps_date_time->sigUsedMask, gps_date_time->percentUsed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, (const char *)gps_date_time, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_DATE_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_date_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t clockStat, uint8_t visSat, uint8_t useSat, uint8_t GppGl, uint8_t sigUsedMask, uint8_t percentUsed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, year);
    _mav_put_uint8_t(buf, 1, month);
    _mav_put_uint8_t(buf, 2, day);
    _mav_put_uint8_t(buf, 3, hour);
    _mav_put_uint8_t(buf, 4, min);
    _mav_put_uint8_t(buf, 5, sec);
    _mav_put_uint8_t(buf, 6, clockStat);
    _mav_put_uint8_t(buf, 7, visSat);
    _mav_put_uint8_t(buf, 8, useSat);
    _mav_put_uint8_t(buf, 9, GppGl);
    _mav_put_uint8_t(buf, 10, sigUsedMask);
    _mav_put_uint8_t(buf, 11, percentUsed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, buf, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
#else
    mavlink_gps_date_time_t *packet = (mavlink_gps_date_time_t *)msgbuf;
    packet->year = year;
    packet->month = month;
    packet->day = day;
    packet->hour = hour;
    packet->min = min;
    packet->sec = sec;
    packet->clockStat = clockStat;
    packet->visSat = visSat;
    packet->useSat = useSat;
    packet->GppGl = GppGl;
    packet->sigUsedMask = sigUsedMask;
    packet->percentUsed = percentUsed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATE_TIME, (const char *)packet, MAVLINK_MSG_ID_GPS_DATE_TIME_MIN_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN, MAVLINK_MSG_ID_GPS_DATE_TIME_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_DATE_TIME UNPACKING


/**
 * @brief Get field year from gps_date_time message
 *
 * @return Year reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_year(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field month from gps_date_time message
 *
 * @return Month reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_month(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field day from gps_date_time message
 *
 * @return Day reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_day(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field hour from gps_date_time message
 *
 * @return Hour reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_hour(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field min from gps_date_time message
 *
 * @return Min reported by Gps 
 */
static inline uint8_t mavlink_msg_gps_date_time_get_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sec from gps_date_time message
 *
 * @return Sec reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_sec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field clockStat from gps_date_time message
 *
 * @return Clock Status. See table 47 page 211 OEMStar Manual  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_clockStat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field visSat from gps_date_time message
 *
 * @return Visible satellites reported by Gps  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_visSat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field useSat from gps_date_time message
 *
 * @return Used satellites in Solution  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_useSat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field GppGl from gps_date_time message
 *
 * @return GPS+GLONASS satellites in Solution  
 */
static inline uint8_t mavlink_msg_gps_date_time_get_GppGl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field sigUsedMask from gps_date_time message
 *
 * @return GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
 */
static inline uint8_t mavlink_msg_gps_date_time_get_sigUsedMask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field percentUsed from gps_date_time message
 *
 * @return Percent used GPS
 */
static inline uint8_t mavlink_msg_gps_date_time_get_percentUsed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Decode a gps_date_time message into a struct
 *
 * @param msg The message to decode
 * @param gps_date_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_date_time_decode(const mavlink_message_t* msg, mavlink_gps_date_time_t* gps_date_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_date_time->year = mavlink_msg_gps_date_time_get_year(msg);
    gps_date_time->month = mavlink_msg_gps_date_time_get_month(msg);
    gps_date_time->day = mavlink_msg_gps_date_time_get_day(msg);
    gps_date_time->hour = mavlink_msg_gps_date_time_get_hour(msg);
    gps_date_time->min = mavlink_msg_gps_date_time_get_min(msg);
    gps_date_time->sec = mavlink_msg_gps_date_time_get_sec(msg);
    gps_date_time->clockStat = mavlink_msg_gps_date_time_get_clockStat(msg);
    gps_date_time->visSat = mavlink_msg_gps_date_time_get_visSat(msg);
    gps_date_time->useSat = mavlink_msg_gps_date_time_get_useSat(msg);
    gps_date_time->GppGl = mavlink_msg_gps_date_time_get_GppGl(msg);
    gps_date_time->sigUsedMask = mavlink_msg_gps_date_time_get_sigUsedMask(msg);
    gps_date_time->percentUsed = mavlink_msg_gps_date_time_get_percentUsed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_DATE_TIME_LEN? msg->len : MAVLINK_MSG_ID_GPS_DATE_TIME_LEN;
        memset(gps_date_time, 0, MAVLINK_MSG_ID_GPS_DATE_TIME_LEN);
    memcpy(gps_date_time, _MAV_PAYLOAD(msg), len);
#endif
}
