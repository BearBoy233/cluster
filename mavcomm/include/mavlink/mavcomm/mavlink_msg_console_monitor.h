#pragma once
// MESSAGE CONSOLE_Monitor PACKING

#define MAVLINK_MSG_ID_CONSOLE_Monitor 41


typedef struct __mavlink_console_monitor_t {
 uint8_t param1; /*<  参数1.*/
 uint8_t param2; /*<  参数2.*/
 uint8_t param3; /*<  参数3.*/
 uint8_t param4; /*<  参数4.*/
} mavlink_console_monitor_t;

#define MAVLINK_MSG_ID_CONSOLE_Monitor_LEN 4
#define MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN 4
#define MAVLINK_MSG_ID_41_LEN 4
#define MAVLINK_MSG_ID_41_MIN_LEN 4

#define MAVLINK_MSG_ID_CONSOLE_Monitor_CRC 165
#define MAVLINK_MSG_ID_41_CRC 165



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONSOLE_Monitor { \
    41, \
    "CONSOLE_Monitor", \
    4, \
    {  { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_console_monitor_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_console_monitor_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_console_monitor_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_console_monitor_t, param4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONSOLE_Monitor { \
    "CONSOLE_Monitor", \
    4, \
    {  { "param1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_console_monitor_t, param1) }, \
         { "param2", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_console_monitor_t, param2) }, \
         { "param3", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_console_monitor_t, param3) }, \
         { "param4", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_console_monitor_t, param4) }, \
         } \
}
#endif

/**
 * @brief Pack a console_monitor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param1  参数1.
 * @param param2  参数2.
 * @param param3  参数3.
 * @param param4  参数4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_console_monitor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_Monitor_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);
    _mav_put_uint8_t(buf, 2, param3);
    _mav_put_uint8_t(buf, 3, param4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN);
#else
    mavlink_console_monitor_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONSOLE_Monitor;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
}

/**
 * @brief Pack a console_monitor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param1  参数1.
 * @param param2  参数2.
 * @param param3  参数3.
 * @param param4  参数4.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_console_monitor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t param1,uint8_t param2,uint8_t param3,uint8_t param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_Monitor_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);
    _mav_put_uint8_t(buf, 2, param3);
    _mav_put_uint8_t(buf, 3, param4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN);
#else
    mavlink_console_monitor_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONSOLE_Monitor;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
}

/**
 * @brief Encode a console_monitor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param console_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_console_monitor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_console_monitor_t* console_monitor)
{
    return mavlink_msg_console_monitor_pack(system_id, component_id, msg, console_monitor->param1, console_monitor->param2, console_monitor->param3, console_monitor->param4);
}

/**
 * @brief Encode a console_monitor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param console_monitor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_console_monitor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_console_monitor_t* console_monitor)
{
    return mavlink_msg_console_monitor_pack_chan(system_id, component_id, chan, msg, console_monitor->param1, console_monitor->param2, console_monitor->param3, console_monitor->param4);
}

/**
 * @brief Send a console_monitor message
 * @param chan MAVLink channel to send the message
 *
 * @param param1  参数1.
 * @param param2  参数2.
 * @param param3  参数3.
 * @param param4  参数4.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_console_monitor_send(mavlink_channel_t chan, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_Monitor_LEN];
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);
    _mav_put_uint8_t(buf, 2, param3);
    _mav_put_uint8_t(buf, 3, param4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE_Monitor, buf, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
#else
    mavlink_console_monitor_t packet;
    packet.param1 = param1;
    packet.param2 = param2;
    packet.param3 = param3;
    packet.param4 = param4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE_Monitor, (const char *)&packet, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
#endif
}

/**
 * @brief Send a console_monitor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_console_monitor_send_struct(mavlink_channel_t chan, const mavlink_console_monitor_t* console_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_console_monitor_send(chan, console_monitor->param1, console_monitor->param2, console_monitor->param3, console_monitor->param4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE_Monitor, (const char *)console_monitor, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONSOLE_Monitor_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_console_monitor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, param1);
    _mav_put_uint8_t(buf, 1, param2);
    _mav_put_uint8_t(buf, 2, param3);
    _mav_put_uint8_t(buf, 3, param4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE_Monitor, buf, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
#else
    mavlink_console_monitor_t *packet = (mavlink_console_monitor_t *)msgbuf;
    packet->param1 = param1;
    packet->param2 = param2;
    packet->param3 = param3;
    packet->param4 = param4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE_Monitor, (const char *)packet, MAVLINK_MSG_ID_CONSOLE_Monitor_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN, MAVLINK_MSG_ID_CONSOLE_Monitor_CRC);
#endif
}
#endif

#endif

// MESSAGE CONSOLE_Monitor UNPACKING


/**
 * @brief Get field param1 from console_monitor message
 *
 * @return  参数1.
 */
static inline uint8_t mavlink_msg_console_monitor_get_param1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param2 from console_monitor message
 *
 * @return  参数2.
 */
static inline uint8_t mavlink_msg_console_monitor_get_param2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param3 from console_monitor message
 *
 * @return  参数3.
 */
static inline uint8_t mavlink_msg_console_monitor_get_param3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field param4 from console_monitor message
 *
 * @return  参数4.
 */
static inline uint8_t mavlink_msg_console_monitor_get_param4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a console_monitor message into a struct
 *
 * @param msg The message to decode
 * @param console_monitor C-struct to decode the message contents into
 */
static inline void mavlink_msg_console_monitor_decode(const mavlink_message_t* msg, mavlink_console_monitor_t* console_monitor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    console_monitor->param1 = mavlink_msg_console_monitor_get_param1(msg);
    console_monitor->param2 = mavlink_msg_console_monitor_get_param2(msg);
    console_monitor->param3 = mavlink_msg_console_monitor_get_param3(msg);
    console_monitor->param4 = mavlink_msg_console_monitor_get_param4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONSOLE_Monitor_LEN? msg->len : MAVLINK_MSG_ID_CONSOLE_Monitor_LEN;
        memset(console_monitor, 0, MAVLINK_MSG_ID_CONSOLE_Monitor_LEN);
    memcpy(console_monitor, _MAV_PAYLOAD(msg), len);
#endif
}
