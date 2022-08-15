#pragma once
// MESSAGE kcf_target_pos PACKING

#define MAVLINK_MSG_ID_kcf_target_pos 701


typedef struct __mavlink_kcf_target_pos_t {
 float x; /*<  x (m).*/
 float y; /*<  y (m).*/
 uint8_t state; /*<  当前追踪状态，0失败，1追踪中，2追踪效果差，3已定位发送给其他无人机，4地面站发送框选，等待无人机响应*/
 uint8_t target_no; /*<  kcf_tracker 目标的编号*/
 uint8_t flag; /*<  1-本机仅定位,(compid=追踪无人机编号)，2-本机持续追踪,(compid=地面站)*/
} mavlink_kcf_target_pos_t;

#define MAVLINK_MSG_ID_kcf_target_pos_LEN 11
#define MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN 11
#define MAVLINK_MSG_ID_701_LEN 11
#define MAVLINK_MSG_ID_701_MIN_LEN 11

#define MAVLINK_MSG_ID_kcf_target_pos_CRC 16
#define MAVLINK_MSG_ID_701_CRC 16



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_kcf_target_pos { \
    701, \
    "kcf_target_pos", \
    5, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_kcf_target_pos_t, state) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kcf_target_pos_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kcf_target_pos_t, y) }, \
         { "target_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_kcf_target_pos_t, target_no) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_kcf_target_pos_t, flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_kcf_target_pos { \
    "kcf_target_pos", \
    5, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_kcf_target_pos_t, state) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_kcf_target_pos_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_kcf_target_pos_t, y) }, \
         { "target_no", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_kcf_target_pos_t, target_no) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_kcf_target_pos_t, flag) }, \
         } \
}
#endif

/**
 * @brief Pack a kcf_target_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  当前追踪状态，0失败，1追踪中，2追踪效果差，3已定位发送给其他无人机，4地面站发送框选，等待无人机响应
 * @param x  x (m).
 * @param y  y (m).
 * @param target_no  kcf_tracker 目标的编号
 * @param flag  1-本机仅定位,(compid=追踪无人机编号)，2-本机持续追踪,(compid=地面站)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kcf_target_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, float x, float y, uint8_t target_no, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_target_pos_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);
    _mav_put_uint8_t(buf, 10, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_kcf_target_pos_LEN);
#else
    mavlink_kcf_target_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.state = state;
    packet.target_no = target_no;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_kcf_target_pos_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_kcf_target_pos;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
}

/**
 * @brief Pack a kcf_target_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  当前追踪状态，0失败，1追踪中，2追踪效果差，3已定位发送给其他无人机，4地面站发送框选，等待无人机响应
 * @param x  x (m).
 * @param y  y (m).
 * @param target_no  kcf_tracker 目标的编号
 * @param flag  1-本机仅定位,(compid=追踪无人机编号)，2-本机持续追踪,(compid=地面站)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_kcf_target_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,float x,float y,uint8_t target_no,uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_target_pos_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);
    _mav_put_uint8_t(buf, 10, flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_kcf_target_pos_LEN);
#else
    mavlink_kcf_target_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.state = state;
    packet.target_no = target_no;
    packet.flag = flag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_kcf_target_pos_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_kcf_target_pos;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
}

/**
 * @brief Encode a kcf_target_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param kcf_target_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kcf_target_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_kcf_target_pos_t* kcf_target_pos)
{
    return mavlink_msg_kcf_target_pos_pack(system_id, component_id, msg, kcf_target_pos->state, kcf_target_pos->x, kcf_target_pos->y, kcf_target_pos->target_no, kcf_target_pos->flag);
}

/**
 * @brief Encode a kcf_target_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param kcf_target_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_kcf_target_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_kcf_target_pos_t* kcf_target_pos)
{
    return mavlink_msg_kcf_target_pos_pack_chan(system_id, component_id, chan, msg, kcf_target_pos->state, kcf_target_pos->x, kcf_target_pos->y, kcf_target_pos->target_no, kcf_target_pos->flag);
}

/**
 * @brief Send a kcf_target_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param state  当前追踪状态，0失败，1追踪中，2追踪效果差，3已定位发送给其他无人机，4地面站发送框选，等待无人机响应
 * @param x  x (m).
 * @param y  y (m).
 * @param target_no  kcf_tracker 目标的编号
 * @param flag  1-本机仅定位,(compid=追踪无人机编号)，2-本机持续追踪,(compid=地面站)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_kcf_target_pos_send(mavlink_channel_t chan, uint8_t state, float x, float y, uint8_t target_no, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_kcf_target_pos_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);
    _mav_put_uint8_t(buf, 10, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_target_pos, buf, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
#else
    mavlink_kcf_target_pos_t packet;
    packet.x = x;
    packet.y = y;
    packet.state = state;
    packet.target_no = target_no;
    packet.flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_target_pos, (const char *)&packet, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
#endif
}

/**
 * @brief Send a kcf_target_pos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_kcf_target_pos_send_struct(mavlink_channel_t chan, const mavlink_kcf_target_pos_t* kcf_target_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_kcf_target_pos_send(chan, kcf_target_pos->state, kcf_target_pos->x, kcf_target_pos->y, kcf_target_pos->target_no, kcf_target_pos->flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_target_pos, (const char *)kcf_target_pos, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
#endif
}

#if MAVLINK_MSG_ID_kcf_target_pos_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_kcf_target_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, float x, float y, uint8_t target_no, uint8_t flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_uint8_t(buf, 8, state);
    _mav_put_uint8_t(buf, 9, target_no);
    _mav_put_uint8_t(buf, 10, flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_target_pos, buf, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
#else
    mavlink_kcf_target_pos_t *packet = (mavlink_kcf_target_pos_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->state = state;
    packet->target_no = target_no;
    packet->flag = flag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_kcf_target_pos, (const char *)packet, MAVLINK_MSG_ID_kcf_target_pos_MIN_LEN, MAVLINK_MSG_ID_kcf_target_pos_LEN, MAVLINK_MSG_ID_kcf_target_pos_CRC);
#endif
}
#endif

#endif

// MESSAGE kcf_target_pos UNPACKING


/**
 * @brief Get field state from kcf_target_pos message
 *
 * @return  当前追踪状态，0失败，1追踪中，2追踪效果差，3已定位发送给其他无人机，4地面站发送框选，等待无人机响应
 */
static inline uint8_t mavlink_msg_kcf_target_pos_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field x from kcf_target_pos message
 *
 * @return  x (m).
 */
static inline float mavlink_msg_kcf_target_pos_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from kcf_target_pos message
 *
 * @return  y (m).
 */
static inline float mavlink_msg_kcf_target_pos_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field target_no from kcf_target_pos message
 *
 * @return  kcf_tracker 目标的编号
 */
static inline uint8_t mavlink_msg_kcf_target_pos_get_target_no(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field flag from kcf_target_pos message
 *
 * @return  1-本机仅定位,(compid=追踪无人机编号)，2-本机持续追踪,(compid=地面站)
 */
static inline uint8_t mavlink_msg_kcf_target_pos_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Decode a kcf_target_pos message into a struct
 *
 * @param msg The message to decode
 * @param kcf_target_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_kcf_target_pos_decode(const mavlink_message_t* msg, mavlink_kcf_target_pos_t* kcf_target_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    kcf_target_pos->x = mavlink_msg_kcf_target_pos_get_x(msg);
    kcf_target_pos->y = mavlink_msg_kcf_target_pos_get_y(msg);
    kcf_target_pos->state = mavlink_msg_kcf_target_pos_get_state(msg);
    kcf_target_pos->target_no = mavlink_msg_kcf_target_pos_get_target_no(msg);
    kcf_target_pos->flag = mavlink_msg_kcf_target_pos_get_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_kcf_target_pos_LEN? msg->len : MAVLINK_MSG_ID_kcf_target_pos_LEN;
        memset(kcf_target_pos, 0, MAVLINK_MSG_ID_kcf_target_pos_LEN);
    memcpy(kcf_target_pos, _MAV_PAYLOAD(msg), len);
#endif
}
