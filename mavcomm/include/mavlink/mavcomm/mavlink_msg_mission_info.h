#pragma once
// MESSAGE mission_info PACKING

#define MAVLINK_MSG_ID_mission_info 800


typedef struct __mavlink_mission_info_t {
 uint8_t flag; /*<  flag.*/
 uint8_t mission_num; /*<  mission_num.*/
 uint8_t param1; /*<  param1.*/
 uint8_t param2; /*<  param2.*/
} mavlink_mission_info_t;

#define MAVLINK_MSG_ID_mission_info_LEN 4
#define MAVLINK_MSG_ID_mission_info_MIN_LEN 4
#define MAVLINK_MSG_ID_800_LEN 4
#define MAVLINK_MSG_ID_800_MIN_LEN 4

#define MAVLINK_MSG_ID_mission_info_CRC 98
#define MAVLINK_MSG_ID_800_CRC 98



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_mission_info { \
    800, \
    "mission_info", \
    4, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_info_t, flag) }, \
         { "mission_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_info_t, mission_num) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_info_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_info_t, param2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_mission_info { \
    "mission_info", \
    4, \
    {  { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mission_info_t, flag) }, \
         { "mission_num", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mission_info_t, mission_num) }, \
         { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mission_info_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_mission_info_t, param2) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_info message
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
static inline uint16_t mavlink_msg_mission_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_mission_info_LEN);
#else
    mavlink_mission_info_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_mission_info_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_mission_info;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
}

/**
 * @brief Pack a mission_info message on a channel
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
static inline uint16_t mavlink_msg_mission_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t flag,uint8_t mission_num,uint8_t param1,uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_mission_info_LEN);
#else
    mavlink_mission_info_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_mission_info_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_mission_info;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
}

/**
 * @brief Encode a mission_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_info_t* mission_info)
{
    return mavlink_msg_mission_info_pack(system_id, component_id, msg, mission_info->flag, mission_info->mission_num, mission_info->param1, mission_info->param2);
}

/**
 * @brief Encode a mission_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_info_t* mission_info)
{
    return mavlink_msg_mission_info_pack_chan(system_id, component_id, chan, msg, mission_info->flag, mission_info->mission_num, mission_info->param1, mission_info->param2);
}

/**
 * @brief Send a mission_info message
 * @param chan MAVLink channel to send the message
 *
 * @param flag  flag.
 * @param mission_num  mission_num.
 * @param param1  param1.
 * @param param2  param2.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_info_send(mavlink_channel_t chan, uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_mission_info_LEN];
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info, buf, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
#else
    mavlink_mission_info_t packet;
    packet.flag = flag;
    packet.mission_num = mission_num;
    packet.param1 = param1;
    packet.param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info, (const char *)&packet, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
#endif
}

/**
 * @brief Send a mission_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_info_send_struct(mavlink_channel_t chan, const mavlink_mission_info_t* mission_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_info_send(chan, mission_info->flag, mission_info->mission_num, mission_info->param1, mission_info->param2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info, (const char *)mission_info, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
#endif
}

#if MAVLINK_MSG_ID_mission_info_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t flag, uint8_t mission_num, uint8_t param1, uint8_t param2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, flag);
    _mav_put_uint8_t(buf, 1, mission_num);
    _mav_put_uint8_t(buf, 2, param1);
    _mav_put_uint8_t(buf, 3, param2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info, buf, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
#else
    mavlink_mission_info_t *packet = (mavlink_mission_info_t *)msgbuf;
    packet->flag = flag;
    packet->mission_num = mission_num;
    packet->param1 = param1;
    packet->param2 = param2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_mission_info, (const char *)packet, MAVLINK_MSG_ID_mission_info_MIN_LEN, MAVLINK_MSG_ID_mission_info_LEN, MAVLINK_MSG_ID_mission_info_CRC);
#endif
}
#endif

#endif

// MESSAGE mission_info UNPACKING


/**
 * @brief Get field flag from mission_info message
 *
 * @return  flag.
 */
static inline uint8_t mavlink_msg_mission_info_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field mission_num from mission_info message
 *
 * @return  mission_num.
 */
static inline uint8_t mavlink_msg_mission_info_get_mission_num(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param1 from mission_info message
 *
 * @return  param1.
 */
static inline uint8_t mavlink_msg_mission_info_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field param2 from mission_info message
 *
 * @return  param2.
 */
static inline uint8_t mavlink_msg_mission_info_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a mission_info message into a struct
 *
 * @param msg The message to decode
 * @param mission_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_info_decode(const mavlink_message_t* msg, mavlink_mission_info_t* mission_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_info->flag = mavlink_msg_mission_info_get_flag(msg);
    mission_info->mission_num = mavlink_msg_mission_info_get_mission_num(msg);
    mission_info->param1 = mavlink_msg_mission_info_get_param1(msg);
    mission_info->param2 = mavlink_msg_mission_info_get_param2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_mission_info_LEN? msg->len : MAVLINK_MSG_ID_mission_info_LEN;
        memset(mission_info, 0, MAVLINK_MSG_ID_mission_info_LEN);
    memcpy(mission_info, _MAV_PAYLOAD(msg), len);
#endif
}
