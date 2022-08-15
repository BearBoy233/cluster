#pragma once
// MESSAGE SET_HOME_POS_GPS_INT PACKING

#define MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT 1122


typedef struct __mavlink_set_home_pos_gps_int_t {
 int32_t lat; /*<  degE7, Latitude, expressed.*/
 int32_t lon; /*<  degE7, Longitude, expressed.*/
 int32_t alt; /*<  mm, Altitude.*/
} mavlink_set_home_pos_gps_int_t;

#define MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN 12
#define MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN 12
#define MAVLINK_MSG_ID_1122_LEN 12
#define MAVLINK_MSG_ID_1122_MIN_LEN 12

#define MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC 79
#define MAVLINK_MSG_ID_1122_CRC 79



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_GPS_INT { \
    1122, \
    "SET_HOME_POS_GPS_INT", \
    3, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_home_pos_gps_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_home_pos_gps_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_home_pos_gps_int_t, alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_GPS_INT { \
    "SET_HOME_POS_GPS_INT", \
    3, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_home_pos_gps_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_home_pos_gps_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_home_pos_gps_int_t, alt) }, \
         } \
}
#endif

/**
 * @brief Pack a set_home_pos_gps_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_gps_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN);
#else
    mavlink_set_home_pos_gps_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
}

/**
 * @brief Pack a set_home_pos_gps_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_gps_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN);
#else
    mavlink_set_home_pos_gps_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
}

/**
 * @brief Encode a set_home_pos_gps_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_gps_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_gps_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_home_pos_gps_int_t* set_home_pos_gps_int)
{
    return mavlink_msg_set_home_pos_gps_int_pack(system_id, component_id, msg, set_home_pos_gps_int->lat, set_home_pos_gps_int->lon, set_home_pos_gps_int->alt);
}

/**
 * @brief Encode a set_home_pos_gps_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_gps_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_gps_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_home_pos_gps_int_t* set_home_pos_gps_int)
{
    return mavlink_msg_set_home_pos_gps_int_pack_chan(system_id, component_id, chan, msg, set_home_pos_gps_int->lat, set_home_pos_gps_int->lon, set_home_pos_gps_int->alt);
}

/**
 * @brief Send a set_home_pos_gps_int message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_home_pos_gps_int_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT, buf, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
#else
    mavlink_set_home_pos_gps_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT, (const char *)&packet, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
#endif
}

/**
 * @brief Send a set_home_pos_gps_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_home_pos_gps_int_send_struct(mavlink_channel_t chan, const mavlink_set_home_pos_gps_int_t* set_home_pos_gps_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_home_pos_gps_int_send(chan, set_home_pos_gps_int->lat, set_home_pos_gps_int->lon, set_home_pos_gps_int->alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT, (const char *)set_home_pos_gps_int, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_home_pos_gps_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT, buf, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
#else
    mavlink_set_home_pos_gps_int_t *packet = (mavlink_set_home_pos_gps_int_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT, (const char *)packet, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_HOME_POS_GPS_INT UNPACKING


/**
 * @brief Get field lat from set_home_pos_gps_int message
 *
 * @return  degE7, Latitude, expressed.
 */
static inline int32_t mavlink_msg_set_home_pos_gps_int_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from set_home_pos_gps_int message
 *
 * @return  degE7, Longitude, expressed.
 */
static inline int32_t mavlink_msg_set_home_pos_gps_int_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from set_home_pos_gps_int message
 *
 * @return  mm, Altitude.
 */
static inline int32_t mavlink_msg_set_home_pos_gps_int_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a set_home_pos_gps_int message into a struct
 *
 * @param msg The message to decode
 * @param set_home_pos_gps_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_home_pos_gps_int_decode(const mavlink_message_t* msg, mavlink_set_home_pos_gps_int_t* set_home_pos_gps_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_home_pos_gps_int->lat = mavlink_msg_set_home_pos_gps_int_get_lat(msg);
    set_home_pos_gps_int->lon = mavlink_msg_set_home_pos_gps_int_get_lon(msg);
    set_home_pos_gps_int->alt = mavlink_msg_set_home_pos_gps_int_get_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN? msg->len : MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN;
        memset(set_home_pos_gps_int, 0, MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT_LEN);
    memcpy(set_home_pos_gps_int, _MAV_PAYLOAD(msg), len);
#endif
}
