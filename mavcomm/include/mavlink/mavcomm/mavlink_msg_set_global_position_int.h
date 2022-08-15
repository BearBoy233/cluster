#pragma once
// MESSAGE SET_GLOBAL_POSITION_INT PACKING

#define MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT 1102


typedef struct __mavlink_set_global_position_int_t {
 int32_t lat; /*<  degE7, Latitude, expressed.*/
 int32_t lon; /*<  degE7, Longitude, expressed.*/
 int32_t alt; /*<  mm, Altitude.*/
 uint16_t hdg_yaw; /*<  cdeg, Vehicle heading(yaw angle), 0.0..359.99 degrees.*/
 uint8_t flag; /*<  参数标志*/
} mavlink_set_global_position_int_t;

#define MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN 15
#define MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN 15
#define MAVLINK_MSG_ID_1102_LEN 15
#define MAVLINK_MSG_ID_1102_MIN_LEN 15

#define MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC 131
#define MAVLINK_MSG_ID_1102_CRC 131



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_GLOBAL_POSITION_INT { \
    1102, \
    "SET_GLOBAL_POSITION_INT", \
    5, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_global_position_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_global_position_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_global_position_int_t, alt) }, \
         { "hdg_yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_set_global_position_int_t, hdg_yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_set_global_position_int_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_GLOBAL_POSITION_INT { \
    "SET_GLOBAL_POSITION_INT", \
    5, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_set_global_position_int_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_set_global_position_int_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_set_global_position_int_t, alt) }, \
         { "hdg_yaw", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_set_global_position_int_t, hdg_yaw) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_set_global_position_int_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a set_global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 * @param hdg_yaw  cdeg, Vehicle heading(yaw angle), 0.0..359.99 degrees.
 * @param flag  参数标志
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_global_position_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, int32_t alt, uint16_t hdg_yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint16_t(buf, 12, hdg_yaw);
    _mav_put_uint8_t(buf, 14, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN);
#else
    mavlink_set_global_position_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdg_yaw = hdg_yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
}

/**
 * @brief Pack a set_global_position_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 * @param hdg_yaw  cdeg, Vehicle heading(yaw angle), 0.0..359.99 degrees.
 * @param flag  参数标志
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_global_position_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,int32_t alt,uint16_t hdg_yaw,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint16_t(buf, 12, hdg_yaw);
    _mav_put_uint8_t(buf, 14, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN);
#else
    mavlink_set_global_position_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdg_yaw = hdg_yaw;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
}

/**
 * @brief Encode a set_global_position_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_global_position_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_global_position_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_global_position_int_t* set_global_position_int)
{
    return mavlink_msg_set_global_position_int_pack(system_id, component_id, msg, set_global_position_int->lat, set_global_position_int->lon, set_global_position_int->alt, set_global_position_int->hdg_yaw, set_global_position_int->flag);
}

/**
 * @brief Encode a set_global_position_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_global_position_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_global_position_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_global_position_int_t* set_global_position_int)
{
    return mavlink_msg_set_global_position_int_pack_chan(system_id, component_id, chan, msg, set_global_position_int->lat, set_global_position_int->lon, set_global_position_int->alt, set_global_position_int->hdg_yaw, set_global_position_int->flag);
}

/**
 * @brief Send a set_global_position_int message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  degE7, Latitude, expressed.
 * @param lon  degE7, Longitude, expressed.
 * @param alt  mm, Altitude.
 * @param hdg_yaw  cdeg, Vehicle heading(yaw angle), 0.0..359.99 degrees.
 * @param flag  参数标志
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_global_position_int_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, uint16_t hdg_yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN];
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint16_t(buf, 12, hdg_yaw);
    _mav_put_uint8_t(buf, 14, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
#else
    mavlink_set_global_position_int_t packet;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.hdg_yaw = hdg_yaw;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT, (const char *)&packet, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
#endif
}

/**
 * @brief Send a set_global_position_int message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_global_position_int_send_struct(mavlink_channel_t chan, const mavlink_set_global_position_int_t* set_global_position_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_global_position_int_send(chan, set_global_position_int->lat, set_global_position_int->lon, set_global_position_int->alt, set_global_position_int->hdg_yaw, set_global_position_int->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT, (const char *)set_global_position_int, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_global_position_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt, uint16_t hdg_yaw, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, lat);
    _mav_put_int32_t(buf, 4, lon);
    _mav_put_int32_t(buf, 8, alt);
    _mav_put_uint16_t(buf, 12, hdg_yaw);
    _mav_put_uint8_t(buf, 14, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT, buf, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
#else
    mavlink_set_global_position_int_t *packet = (mavlink_set_global_position_int_t *)msgbuf;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->hdg_yaw = hdg_yaw;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT, (const char *)packet, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_MIN_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_GLOBAL_POSITION_INT UNPACKING


/**
 * @brief Get field lat from set_global_position_int message
 *
 * @return  degE7, Latitude, expressed.
 */
static inline int32_t mavlink_msg_set_global_position_int_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field lon from set_global_position_int message
 *
 * @return  degE7, Longitude, expressed.
 */
static inline int32_t mavlink_msg_set_global_position_int_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field alt from set_global_position_int message
 *
 * @return  mm, Altitude.
 */
static inline int32_t mavlink_msg_set_global_position_int_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field hdg_yaw from set_global_position_int message
 *
 * @return  cdeg, Vehicle heading(yaw angle), 0.0..359.99 degrees.
 */
static inline uint16_t mavlink_msg_set_global_position_int_get_hdg_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field flag from set_global_position_int message
 *
 * @return  参数标志
 */
static inline uint8_t mavlink_msg_set_global_position_int_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Decode a set_global_position_int message into a struct
 *
 * @param msg The message to decode
 * @param set_global_position_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_global_position_int_decode(const mavlink_message_t* msg, mavlink_set_global_position_int_t* set_global_position_int)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_global_position_int->lat = mavlink_msg_set_global_position_int_get_lat(msg);
    set_global_position_int->lon = mavlink_msg_set_global_position_int_get_lon(msg);
    set_global_position_int->alt = mavlink_msg_set_global_position_int_get_alt(msg);
    set_global_position_int->hdg_yaw = mavlink_msg_set_global_position_int_get_hdg_yaw(msg);
    set_global_position_int->flag = mavlink_msg_set_global_position_int_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN? msg->len : MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN;
        memset(set_global_position_int, 0, MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT_LEN);
    memcpy(set_global_position_int, _MAV_PAYLOAD(msg), len);
#endif
}
