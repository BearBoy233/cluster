#pragma once
// MESSAGE SET_HOME_POS_ENU PACKING

#define MAVLINK_MSG_ID_SET_HOME_POS_ENU 1121


typedef struct __mavlink_set_home_pos_enu_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
} mavlink_set_home_pos_enu_t;

#define MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN 12
#define MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN 12
#define MAVLINK_MSG_ID_1121_LEN 12
#define MAVLINK_MSG_ID_1121_MIN_LEN 12

#define MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC 228
#define MAVLINK_MSG_ID_1121_CRC 228



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_ENU { \
    1121, \
    "SET_HOME_POS_ENU", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_home_pos_enu_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_home_pos_enu_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_home_pos_enu_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_HOME_POS_ENU { \
    "SET_HOME_POS_ENU", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_home_pos_enu_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_set_home_pos_enu_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_set_home_pos_enu_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a set_home_pos_enu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_enu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN);
#else
    mavlink_set_home_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_ENU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
}

/**
 * @brief Pack a set_home_pos_enu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_home_pos_enu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN);
#else
    mavlink_set_home_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_HOME_POS_ENU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
}

/**
 * @brief Encode a set_home_pos_enu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_enu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_enu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_home_pos_enu_t* set_home_pos_enu)
{
    return mavlink_msg_set_home_pos_enu_pack(system_id, component_id, msg, set_home_pos_enu->x, set_home_pos_enu->y, set_home_pos_enu->z);
}

/**
 * @brief Encode a set_home_pos_enu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_home_pos_enu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_home_pos_enu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_home_pos_enu_t* set_home_pos_enu)
{
    return mavlink_msg_set_home_pos_enu_pack_chan(system_id, component_id, chan, msg, set_home_pos_enu->x, set_home_pos_enu->y, set_home_pos_enu->z);
}

/**
 * @brief Send a set_home_pos_enu message
 * @param chan MAVLink channel to send the message
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_home_pos_enu_send(mavlink_channel_t chan, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU, buf, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
#else
    mavlink_set_home_pos_enu_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU, (const char *)&packet, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
#endif
}

/**
 * @brief Send a set_home_pos_enu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_home_pos_enu_send_struct(mavlink_channel_t chan, const mavlink_set_home_pos_enu_t* set_home_pos_enu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_home_pos_enu_send(chan, set_home_pos_enu->x, set_home_pos_enu->y, set_home_pos_enu->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU, (const char *)set_home_pos_enu, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_home_pos_enu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU, buf, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
#else
    mavlink_set_home_pos_enu_t *packet = (mavlink_set_home_pos_enu_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_HOME_POS_ENU, (const char *)packet, MAVLINK_MSG_ID_SET_HOME_POS_ENU_MIN_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN, MAVLINK_MSG_ID_SET_HOME_POS_ENU_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_HOME_POS_ENU UNPACKING


/**
 * @brief Get field x from set_home_pos_enu message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_set_home_pos_enu_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from set_home_pos_enu message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_set_home_pos_enu_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from set_home_pos_enu message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_set_home_pos_enu_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a set_home_pos_enu message into a struct
 *
 * @param msg The message to decode
 * @param set_home_pos_enu C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_home_pos_enu_decode(const mavlink_message_t* msg, mavlink_set_home_pos_enu_t* set_home_pos_enu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_home_pos_enu->x = mavlink_msg_set_home_pos_enu_get_x(msg);
    set_home_pos_enu->y = mavlink_msg_set_home_pos_enu_get_y(msg);
    set_home_pos_enu->z = mavlink_msg_set_home_pos_enu_get_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN? msg->len : MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN;
        memset(set_home_pos_enu, 0, MAVLINK_MSG_ID_SET_HOME_POS_ENU_LEN);
    memcpy(set_home_pos_enu, _MAV_PAYLOAD(msg), len);
#endif
}
