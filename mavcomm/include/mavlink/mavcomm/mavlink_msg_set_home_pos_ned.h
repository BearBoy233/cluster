#pragma once
// MESSAGE SET_HOME_POS_NED PACKING

#define MAVLINK_MSG_ID_SET_HOME_POS_NED 1121


typedef struct __mavlink_set_home_pos_ned_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
} mavlink_set_home_pos_ned_t;

#define MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN 12
#define MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN 12
#define MAVLINK_MSG_ID_1121_LEN 12
#define MAVLINK_MSG_ID_1121_MIN_LEN 12

#define MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC 113
#define MAVLINK_MSG_ID_1121_CRC 113



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_NED { \
    1121, \
    "SET_HOME_POS_NED", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_home_pos_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_home_pos_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_home_pos_ned_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_NED { \
    "SET_HOME_POS_NED", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_home_pos_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_home_pos_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_home_pos_ned_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a set_home_pos_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN);
#else
    mavlink_set_home_pos_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
}

/**
 * @brief Pack a set_home_pos_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN);
#else
    mavlink_set_home_pos_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
}

/**
 * @brief Encode a set_home_pos_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_home_pos_ned_t* set_home_pos_ned)
{
    return mavlink_msg_set_home_pos_ned_pack(system_id, component_id, msg, set_home_pos_ned->x, set_home_pos_ned->y, set_home_pos_ned->z);
}

/**
 * @brief Encode a set_home_pos_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_home_pos_ned_t* set_home_pos_ned)
{
    return mavlink_msg_set_home_pos_ned_pack_chan(system_id, component_id, chan, msg, set_home_pos_ned->x, set_home_pos_ned->y, set_home_pos_ned->z);
}

/**
 * @brief Send a set_home_pos_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_home_pos_ned_send(mavlink_channel_t chan, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_NED, buf, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
#else
    mavlink_set_home_pos_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_NED, (const char *)&packet, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
#endif
}

/**
 * @brief Send a set_home_pos_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_home_pos_ned_send_struct(mavlink_channel_t chan, const mavlink_set_home_pos_ned_t* set_home_pos_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_home_pos_ned_send(chan, set_home_pos_ned->x, set_home_pos_ned->y, set_home_pos_ned->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_NED, (const char *)set_home_pos_ned, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_home_pos_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_NED, buf, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
#else
    mavlink_set_home_pos_ned_t *packet = (mavlink_set_home_pos_ned_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_NED, (const char *)packet, MAVLINK_MSG_ID_SET_HOME_POS_NED_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN, MAVLINK_MSG_ID_SET_HOME_POS_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_HOME_POS_NED UNPACKING


/**
 * @brief Get field x from set_home_pos_ned message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_set_home_pos_ned_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from set_home_pos_ned message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_set_home_pos_ned_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from set_home_pos_ned message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_set_home_pos_ned_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a set_home_pos_ned message into a struct
 *
 * @param msg The message to decode
 * @param set_home_pos_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_home_pos_ned_decode(const mavlink_message_t* msg, mavlink_set_home_pos_ned_t* set_home_pos_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_home_pos_ned->x = mavlink_msg_set_home_pos_ned_get_x(msg);
    set_home_pos_ned->y = mavlink_msg_set_home_pos_ned_get_y(msg);
    set_home_pos_ned->z = mavlink_msg_set_home_pos_ned_get_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN? msg->len : MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN;
        memset(set_home_pos_ned, 0, MAVLINK_MSG_ID_SET_HOME_POS_NED_LEN);
    memcpy(set_home_pos_ned, _MAV_PAYLOAD(msg), len);
#endif
}
