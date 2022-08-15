#pragma once
// MESSAGE mission_info_back PACKING

#define MAVLINK_MSG_ID_mission_info_back 801


typedef struct __mavlink_mission_info_back_t {
 uint8_t flag; /*<  flag.*/
 uint8_t mission_num; /*<  mission_num.*/
 uint8_t param1; /*<  param1.*/
 uint8_t param2; /*<  param2.*/
} mavlink_mission_info_back_t;

#define MAVLINK_MSG_ID_mission_info_back_LEN 4
#define MAVLINK_MSG_ID_mission_info_back_MIN_LEN 4
#define MAVLINK_MSG_ID_801_LEN 4
#define MAVLINK_MSG_ID_801_MIN_LEN 4

#define MAVLINK_MSG_ID_mission_info_back_CRC 9
#define MAVLINK_MSG_ID_801_CRC 9



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_mission_info_back { \
    801, \
    "mission_info_back", \
    4, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_info_back_t, flag) }, \
         { "mission_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_info_back_t, mission_num) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_info_back_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_info_back_t, param2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_mission_info_back { \
    "mission_info_back", \
    4, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_info_back_t, flag) }, \
         { "mission_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_info_back_t, mission_num) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_info_back_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_info_back_t, param2) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_info_back message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param flag  flag.
 * @param mission_num  mission_num.
 * @param param1  param1.
 * @param param2  param2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_info_back_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_back_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_mission_info_back_LEN);
#else
    mavlink_mission_info_back_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_mission_info_back_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_mission_info_back;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
}

/**
 * @brief Pack a mission_info_back message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param flag  flag.
 * @param mission_num  mission_num.
 * @param param1  param1.
 * @param param2  param2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_info_back_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t flag,uint8_t mission_num,uint8_t param1,uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_back_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_mission_info_back_LEN);
#else
    mavlink_mission_info_back_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_mission_info_back_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_mission_info_back;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
}

/**
 * @brief Encode a mission_info_back struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_info_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_info_back_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_info_back_t* mission_info_back)
{
    return mavlink_msg_mission_info_back_pack(system_id, component_id, msg, mission_info_back->flag, mission_info_back->mission_num, mission_info_back->param1, mission_info_back->param2);
}

/**
 * @brief Encode a mission_info_back struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_info_back C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_info_back_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_info_back_t* mission_info_back)
{
    return mavlink_msg_mission_info_back_pack_chan(system_id, component_id, chan, msg, mission_info_back->flag, mission_info_back->mission_num, mission_info_back->param1, mission_info_back->param2);
}

/**
 * @brief Send a mission_info_back message
 * @param chan MAVLink channel to send the message
 *
 * @param flag  flag.
 * @param mission_num  mission_num.
 * @param param1  param1.
 * @param param2  param2.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_info_back_send(mavlink_channel_t chan, uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_back_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info_back, buf, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
#else
    mavlink_mission_info_back_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info_back, (const char *)&packet, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
#endif
}

/**
 * @brief Send a mission_info_back message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_info_back_send_struct(mavlink_channel_t chan, const mavlink_mission_info_back_t* mission_info_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_info_back_send(chan, mission_info_back->flag, mission_info_back->mission_num, mission_info_back->param1, mission_info_back->param2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info_back, (const char *)mission_info_back, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
#endif
}

#if MAVLINK_MSG_ID_mission_info_back_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_info_back_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info_back, buf, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
#else
    mavlink_mission_info_back_t *packet = (mavlink_mission_info_back_t *)msgbuf;
    packet->flag = flag;
    packet->mission_num = mission_num;
    packet->param1 = param1;
    packet->param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info_back, (const char *)packet, MAVLINK_MSG_ID_mission_info_back_MIN_LEN, MAVLINK_MSG_ID_mission_info_back_LEN, MAVLINK_MSG_ID_mission_info_back_CRC);
#endif
}
#endif

#endif

// MESSAGE mission_info_back UNPACKING


/**
 * @brief Get field flag from mission_info_back message
 *
 * @return  flag.
 */
static inline uint8_t mavlink_msg_mission_info_back_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_num from mission_info_back message
 *
 * @return  mission_num.
 */
static inline uint8_t mavlink_msg_mission_info_back_get_mission_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param1 from mission_info_back message
 *
 * @return  param1.
 */
static inline uint8_t mavlink_msg_mission_info_back_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field param2 from mission_info_back message
 *
 * @return  param2.
 */
static inline uint8_t mavlink_msg_mission_info_back_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mission_info_back message into a struct
 *
 * @param msg The message to decode
 * @param mission_info_back C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_info_back_decode(const mavlink_message_t* msg, mavlink_mission_info_back_t* mission_info_back)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_info_back->flag = mavlink_msg_mission_info_back_get_flag(msg);
    mission_info_back->mission_num = mavlink_msg_mission_info_back_get_mission_num(msg);
    mission_info_back->param1 = mavlink_msg_mission_info_back_get_param1(msg);
    mission_info_back->param2 = mavlink_msg_mission_info_back_get_param2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_mission_info_back_LEN? msg->len : MAVLINK_MSG_ID_mission_info_back_LEN;
        memset(mission_info_back, 0, MAVLINK_MSG_ID_mission_info_back_LEN);
    memcpy(mission_info_back, _MAV_PAYLOAD(msg), len);
#endif
}
