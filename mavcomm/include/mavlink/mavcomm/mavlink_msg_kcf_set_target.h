#pragma once
// MESSAGE kcf_set_target PACKING

#define MAVLINK_MSG_ID_kcf_set_target 702


typedef struct __mavlink_kcf_set_target_t {
 uint16_t x; /*<  图像点 x.*/
 uint16_t y; /*<  图像点 y.*/
 uint16_t width; /*<  width.*/
 uint16_t height; /*<  height.*/
 uint8_t state; /*<  1-on 0-off*/
 uint8_t target_no; /*<  kcf_tracker 目标的编号*/
} mavlink_kcf_set_target_t;

#define MAVLINK_MSG_ID_kcf_set_target_LEN 10
#define MAVLINK_MSG_ID_kcf_set_target_MIN_LEN 10
#define MAVLINK_MSG_ID_702_LEN 10
#define MAVLINK_MSG_ID_702_MIN_LEN 10

#define MAVLINK_MSG_ID_kcf_set_target_CRC 105
#define MAVLINK_MSG_ID_702_CRC 105



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_kcf_set_target { \
    702, \
    "kcf_set_target", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_kcf_set_target_t, state) }, \
         { "target_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_kcf_set_target_t, target_no) }, \
         { "x", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_kcf_set_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_kcf_set_target_t, y) }, \
         { "width", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_kcf_set_target_t, width) }, \
         { "height", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_kcf_set_target_t, height) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_kcf_set_target { \
    "kcf_set_target", \
    6, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_kcf_set_target_t, state) }, \
         { "target_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_kcf_set_target_t, target_no) }, \
         { "x", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_kcf_set_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_kcf_set_target_t, y) }, \
         { "width", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_kcf_set_target_t, width) }, \
         { "height", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_kcf_set_target_t, height) }, \
         } \
}
#endif

/**
 * @brief Pack a kcf_set_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  1-on 0-off
 * @param target_no  kcf_tracker 目标的编号
 * @param x  图像点 x.
 * @param y  图像点 y.
 * @param width  width.
 * @param height  height.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kcf_set_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, uint8_t target_no, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_set_target_LEN];
    _mav_put_uint16_t(buf, 0, x);
    _mav_put_uint16_t(buf, 2, y);
    _mav_put_uint16_t(buf, 4, width);
    _mav_put_uint16_t(buf, 6, height);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_kcf_set_target_LEN);
#else
    mavlink_kcf_set_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.width = width;
    packet.height = height;
    packet.state = state;
    packet.target_no = target_no;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_kcf_set_target_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_kcf_set_target;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
}

/**
 * @brief Pack a kcf_set_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  1-on 0-off
 * @param target_no  kcf_tracker 目标的编号
 * @param x  图像点 x.
 * @param y  图像点 y.
 * @param width  width.
 * @param height  height.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kcf_set_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,uint8_t target_no,uint16_t x,uint16_t y,uint16_t width,uint16_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_set_target_LEN];
    _mav_put_uint16_t(buf, 0, x);
    _mav_put_uint16_t(buf, 2, y);
    _mav_put_uint16_t(buf, 4, width);
    _mav_put_uint16_t(buf, 6, height);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_kcf_set_target_LEN);
#else
    mavlink_kcf_set_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.width = width;
    packet.height = height;
    packet.state = state;
    packet.target_no = target_no;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_kcf_set_target_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_kcf_set_target;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
}

/**
 * @brief Encode a kcf_set_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kcf_set_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kcf_set_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kcf_set_target_t* kcf_set_target)
{
    return mavlink_msg_kcf_set_target_pack(system_id, component_id, msg, kcf_set_target->state, kcf_set_target->target_no, kcf_set_target->x, kcf_set_target->y, kcf_set_target->width, kcf_set_target->height);
}

/**
 * @brief Encode a kcf_set_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kcf_set_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kcf_set_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kcf_set_target_t* kcf_set_target)
{
    return mavlink_msg_kcf_set_target_pack_chan(system_id, component_id, chan, msg, kcf_set_target->state, kcf_set_target->target_no, kcf_set_target->x, kcf_set_target->y, kcf_set_target->width, kcf_set_target->height);
}

/**
 * @brief Send a kcf_set_target message
 * @param chan MAVLink channel to send the message
 *
 * @param state  1-on 0-off
 * @param target_no  kcf_tracker 目标的编号
 * @param x  图像点 x.
 * @param y  图像点 y.
 * @param width  width.
 * @param height  height.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kcf_set_target_send(mavlink_channel_t chan, uint8_t state, uint8_t target_no, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_set_target_LEN];
    _mav_put_uint16_t(buf, 0, x);
    _mav_put_uint16_t(buf, 2, y);
    _mav_put_uint16_t(buf, 4, width);
    _mav_put_uint16_t(buf, 6, height);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_set_target, buf, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
#else
    mavlink_kcf_set_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.width = width;
    packet.height = height;
    packet.state = state;
    packet.target_no = target_no;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_set_target, (const char *)&packet, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
#endif
}

/**
 * @brief Send a kcf_set_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kcf_set_target_send_struct(mavlink_channel_t chan, const mavlink_kcf_set_target_t* kcf_set_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kcf_set_target_send(chan, kcf_set_target->state, kcf_set_target->target_no, kcf_set_target->x, kcf_set_target->y, kcf_set_target->width, kcf_set_target->height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_set_target, (const char *)kcf_set_target, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
#endif
}

#if MAVLINK_MSG_ID_kcf_set_target_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kcf_set_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, uint8_t target_no, uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, x);
    _mav_put_uint16_t(buf, 2, y);
    _mav_put_uint16_t(buf, 4, width);
    _mav_put_uint16_t(buf, 6, height);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_set_target, buf, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
#else
    mavlink_kcf_set_target_t *packet = (mavlink_kcf_set_target_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->width = width;
    packet->height = height;
    packet->state = state;
    packet->target_no = target_no;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_set_target, (const char *)packet, MAVLINK_MSG_ID_kcf_set_target_MIN_LEN, MAVLINK_MSG_ID_kcf_set_target_LEN, MAVLINK_MSG_ID_kcf_set_target_CRC);
#endif
}
#endif

#endif

// MESSAGE kcf_set_target UNPACKING


/**
 * @brief Get field state from kcf_set_target message
 *
 * @return  1-on 0-off
 */
static inline uint8_t mavlink_msg_kcf_set_target_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field target_no from kcf_set_target message
 *
 * @return  kcf_tracker 目标的编号
 */
static inline uint8_t mavlink_msg_kcf_set_target_get_target_no(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field x from kcf_set_target message
 *
 * @return  图像点 x.
 */
static inline uint16_t mavlink_msg_kcf_set_target_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field y from kcf_set_target message
 *
 * @return  图像点 y.
 */
static inline uint16_t mavlink_msg_kcf_set_target_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field width from kcf_set_target message
 *
 * @return  width.
 */
static inline uint16_t mavlink_msg_kcf_set_target_get_width(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field height from kcf_set_target message
 *
 * @return  height.
 */
static inline uint16_t mavlink_msg_kcf_set_target_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Decode a kcf_set_target message into a struct
 *
 * @param msg The message to decode
 * @param kcf_set_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_kcf_set_target_decode(const mavlink_message_t* msg, mavlink_kcf_set_target_t* kcf_set_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kcf_set_target->x = mavlink_msg_kcf_set_target_get_x(msg);
    kcf_set_target->y = mavlink_msg_kcf_set_target_get_y(msg);
    kcf_set_target->width = mavlink_msg_kcf_set_target_get_width(msg);
    kcf_set_target->height = mavlink_msg_kcf_set_target_get_height(msg);
    kcf_set_target->state = mavlink_msg_kcf_set_target_get_state(msg);
    kcf_set_target->target_no = mavlink_msg_kcf_set_target_get_target_no(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_kcf_set_target_LEN? msg->len : MAVLINK_MSG_ID_kcf_set_target_LEN;
        memset(kcf_set_target, 0, MAVLINK_MSG_ID_kcf_set_target_LEN);
    memcpy(kcf_set_target, _MAV_PAYLOAD(msg), len);
#endif
}
