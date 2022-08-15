#pragma once
// MESSAGE CONSOLE PACKING

#define MAVLINK_MSG_ID_CONSOLE 40


typedef struct __mavlink_console_t {
 uint8_t command; /*<  脚本ID.*/
 uint8_t flag; /*<  1启动launch,3关闭node.*/
 uint8_t type1; /*<  参数1.*/
 uint8_t type2; /*<  参数2.*/
} mavlink_console_t;

#define MAVLINK_MSG_ID_CONSOLE_LEN 4
#define MAVLINK_MSG_ID_CONSOLE_MIN_LEN 4
#define MAVLINK_MSG_ID_40_LEN 4
#define MAVLINK_MSG_ID_40_MIN_LEN 4

#define MAVLINK_MSG_ID_CONSOLE_CRC 93
#define MAVLINK_MSG_ID_40_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONSOLE { \
    40, \
    "CONSOLE", \
    4, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_console_t, command) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_console_t, flag) }, \
         { "type1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_console_t, type1) }, \
         { "type2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_console_t, type2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONSOLE { \
    "CONSOLE", \
    4, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_console_t, command) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_console_t, flag) }, \
         { "type1", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_console_t, type1) }, \
         { "type2", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_console_t, type2) }, \
         } \
}
#endif

/**
 * @brief Pack a console message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command  脚本ID.
 * @param flag  1启动launch,3关闭node.
 * @param type1  参数1.
 * @param type2  参数2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_console_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t command, uint8_t flag, uint8_t type1, uint8_t type2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, flag);
    _mav_put_uint8_t(buf, 2, type1);
    _mav_put_uint8_t(buf, 3, type2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONSOLE_LEN);
#else
    mavlink_console_t packet;
    packet.command = command;
    packet.flag = flag;
    packet.type1 = type1;
    packet.type2 = type2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONSOLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONSOLE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
}

/**
 * @brief Pack a console message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command  脚本ID.
 * @param flag  1启动launch,3关闭node.
 * @param type1  参数1.
 * @param type2  参数2.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_console_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t command,uint8_t flag,uint8_t type1,uint8_t type2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, flag);
    _mav_put_uint8_t(buf, 2, type1);
    _mav_put_uint8_t(buf, 3, type2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONSOLE_LEN);
#else
    mavlink_console_t packet;
    packet.command = command;
    packet.flag = flag;
    packet.type1 = type1;
    packet.type2 = type2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONSOLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONSOLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
}

/**
 * @brief Encode a console struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param console C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_console_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_console_t* console)
{
    return mavlink_msg_console_pack(system_id, component_id, msg, console->command, console->flag, console->type1, console->type2);
}

/**
 * @brief Encode a console struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param console C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_console_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_console_t* console)
{
    return mavlink_msg_console_pack_chan(system_id, component_id, chan, msg, console->command, console->flag, console->type1, console->type2);
}

/**
 * @brief Send a console message
 * @param chan MAVLink channel to send the message
 *
 * @param command  脚本ID.
 * @param flag  1启动launch,3关闭node.
 * @param type1  参数1.
 * @param type2  参数2.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_console_send(mavlink_channel_t chan, uint8_t command, uint8_t flag, uint8_t type1, uint8_t type2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONSOLE_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, flag);
    _mav_put_uint8_t(buf, 2, type1);
    _mav_put_uint8_t(buf, 3, type2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE, buf, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
#else
    mavlink_console_t packet;
    packet.command = command;
    packet.flag = flag;
    packet.type1 = type1;
    packet.type2 = type2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE, (const char *)&packet, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
#endif
}

/**
 * @brief Send a console message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_console_send_struct(mavlink_channel_t chan, const mavlink_console_t* console)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_console_send(chan, console->command, console->flag, console->type1, console->type2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE, (const char *)console, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONSOLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_console_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, uint8_t flag, uint8_t type1, uint8_t type2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, flag);
    _mav_put_uint8_t(buf, 2, type1);
    _mav_put_uint8_t(buf, 3, type2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE, buf, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
#else
    mavlink_console_t *packet = (mavlink_console_t *)msgbuf;
    packet->command = command;
    packet->flag = flag;
    packet->type1 = type1;
    packet->type2 = type2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONSOLE, (const char *)packet, MAVLINK_MSG_ID_CONSOLE_MIN_LEN, MAVLINK_MSG_ID_CONSOLE_LEN, MAVLINK_MSG_ID_CONSOLE_CRC);
#endif
}
#endif

#endif

// MESSAGE CONSOLE UNPACKING


/**
 * @brief Get field command from console message
 *
 * @return  脚本ID.
 */
static inline uint8_t mavlink_msg_console_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field flag from console message
 *
 * @return  1启动launch,3关闭node.
 */
static inline uint8_t mavlink_msg_console_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field type1 from console message
 *
 * @return  参数1.
 */
static inline uint8_t mavlink_msg_console_get_type1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field type2 from console message
 *
 * @return  参数2.
 */
static inline uint8_t mavlink_msg_console_get_type2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a console message into a struct
 *
 * @param msg The message to decode
 * @param console C-struct to decode the message contents into
 */
static inline void mavlink_msg_console_decode(const mavlink_message_t* msg, mavlink_console_t* console)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    console->command = mavlink_msg_console_get_command(msg);
    console->flag = mavlink_msg_console_get_flag(msg);
    console->type1 = mavlink_msg_console_get_type1(msg);
    console->type2 = mavlink_msg_console_get_type2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONSOLE_LEN? msg->len : MAVLINK_MSG_ID_CONSOLE_LEN;
        memset(console, 0, MAVLINK_MSG_ID_CONSOLE_LEN);
    memcpy(console, _MAV_PAYLOAD(msg), len);
#endif
}
