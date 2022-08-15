#pragma once
// MESSAGE LOCAL_POSITION_NED PACKING

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED 1001


typedef struct __mavlink_local_position_ned_t {
 float x; /*<  x(m).*/
 float y; /*<  y(m).*/
 float z; /*<  z(m).*/
 float yaw; /*<  yaw(rad).*/
} mavlink_local_position_ned_t;

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN 16
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN 16
#define MAVLINK_MSG_ID_1001_LEN 16
#define MAVLINK_MSG_ID_1001_MIN_LEN 16

#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC 13
#define MAVLINK_MSG_ID_1001_CRC 13



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED { \
    1001, \
    "LOCAL_POSITION_NED", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_local_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_ned_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_ned_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED { \
    "LOCAL_POSITION_NED", \
    4, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_local_position_ned_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_local_position_ned_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_local_position_ned_t, z) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_local_position_ned_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a local_position_ned message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mavlink_local_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Pack a local_position_ned message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_local_position_ned_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#else
    mavlink_local_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LOCAL_POSITION_NED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
}

/**
 * @brief Encode a local_position_ned struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack(system_id, component_id, msg, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->yaw);
}

/**
 * @brief Encode a local_position_ned struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param local_position_ned C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_local_position_ned_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_local_position_ned_t* local_position_ned)
{
    return mavlink_msg_local_position_ned_pack_chan(system_id, component_id, chan, msg, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->yaw);
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 *
 * @param x  x(m).
 * @param y  y(m).
 * @param z  z(m).
 * @param yaw  yaw(rad).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_local_position_ned_send(mavlink_channel_t chan, float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mavlink_local_position_ned_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)&packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

/**
 * @brief Send a local_position_ned message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_local_position_ned_send_struct(mavlink_channel_t chan, const mavlink_local_position_ned_t* local_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_local_position_ned_send(chan, local_position_ned->x, local_position_ned->y, local_position_ned->z, local_position_ned->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)local_position_ned, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}

#if MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_local_position_ned_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);
    _mav_put_float(buf, 12, yaw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, buf, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#else
    mavlink_local_position_ned_t *packet = (mavlink_local_position_ned_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->yaw = yaw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LOCAL_POSITION_NED, (const char *)packet, MAVLINK_MSG_ID_LOCAL_POSITION_NED_MIN_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN, MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC);
#endif
}
#endif

#endif

// MESSAGE LOCAL_POSITION_NED UNPACKING


/**
 * @brief Get field x from local_position_ned message
 *
 * @return  x(m).
 */
static inline float mavlink_msg_local_position_ned_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from local_position_ned message
 *
 * @return  y(m).
 */
static inline float mavlink_msg_local_position_ned_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from local_position_ned message
 *
 * @return  z(m).
 */
static inline float mavlink_msg_local_position_ned_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from local_position_ned message
 *
 * @return  yaw(rad).
 */
static inline float mavlink_msg_local_position_ned_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a local_position_ned message into a struct
 *
 * @param msg The message to decode
 * @param local_position_ned C-struct to decode the message contents into
 */
static inline void mavlink_msg_local_position_ned_decode(const mavlink_message_t* msg, mavlink_local_position_ned_t* local_position_ned)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    local_position_ned->x = mavlink_msg_local_position_ned_get_x(msg);
    local_position_ned->y = mavlink_msg_local_position_ned_get_y(msg);
    local_position_ned->z = mavlink_msg_local_position_ned_get_z(msg);
    local_position_ned->yaw = mavlink_msg_local_position_ned_get_yaw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN? msg->len : MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN;
        memset(local_position_ned, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN);
    memcpy(local_position_ned, _MAV_PAYLOAD(msg), len);
#endif
}
