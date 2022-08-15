#pragma once
// MESSAGE GET_HOME_POSITION PACKING

#define MAVLINK_MSG_ID_GET_HOME_POSITION 1111


typedef struct __mavlink_get_home_position_t {
 uint8_t param; /*<  ENU, Loc*/
} mavlink_get_home_position_t;

#define MAVLINK_MSG_ID_GET_HOME_POSITION_LEN 1
#define MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN 1
#define MAVLINK_MSG_ID_1111_LEN 1
#define MAVLINK_MSG_ID_1111_MIN_LEN 1

#define MAVLINK_MSG_ID_GET_HOME_POSITION_CRC 203
#define MAVLINK_MSG_ID_1111_CRC 203



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GET_HOME_POSITION { \
    1111, \
    "GET_HOME_POSITION", \
    1, \
    {  { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_get_home_position_t, param) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GET_HOME_POSITION { \
    "GET_HOME_POSITION", \
    1, \
    {  { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_get_home_position_t, param) }, \
         } \
}
#endif

/**
 * @brief Pack a get_home_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param  ENU, Loc
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_home_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_HOME_POSITION_LEN];
    _mav_put_uint8_t(buf, 0, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN);
#else
    mavlink_get_home_position_t packet;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_HOME_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
}

/**
 * @brief Pack a get_home_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param  ENU, Loc
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_get_home_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_HOME_POSITION_LEN];
    _mav_put_uint8_t(buf, 0, param);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN);
#else
    mavlink_get_home_position_t packet;
    packet.param = param;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GET_HOME_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
}

/**
 * @brief Encode a get_home_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param get_home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_home_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_get_home_position_t* get_home_position)
{
    return mavlink_msg_get_home_position_pack(system_id, component_id, msg, get_home_position->param);
}

/**
 * @brief Encode a get_home_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param get_home_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_get_home_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_get_home_position_t* get_home_position)
{
    return mavlink_msg_get_home_position_pack_chan(system_id, component_id, chan, msg, get_home_position->param);
}

/**
 * @brief Send a get_home_position message
 * @param chan MAVLink channel to send the message
 *
 * @param param  ENU, Loc
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_get_home_position_send(mavlink_channel_t chan, uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GET_HOME_POSITION_LEN];
    _mav_put_uint8_t(buf, 0, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_HOME_POSITION, buf, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
#else
    mavlink_get_home_position_t packet;
    packet.param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_HOME_POSITION, (const char *)&packet, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
#endif
}

/**
 * @brief Send a get_home_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_get_home_position_send_struct(mavlink_channel_t chan, const mavlink_get_home_position_t* get_home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_get_home_position_send(chan, get_home_position->param);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_HOME_POSITION, (const char *)get_home_position, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_GET_HOME_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_get_home_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t param)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, param);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_HOME_POSITION, buf, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
#else
    mavlink_get_home_position_t *packet = (mavlink_get_home_position_t *)msgbuf;
    packet->param = param;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GET_HOME_POSITION, (const char *)packet, MAVLINK_MSG_ID_GET_HOME_POSITION_MIN_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN, MAVLINK_MSG_ID_GET_HOME_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE GET_HOME_POSITION UNPACKING


/**
 * @brief Get field param from get_home_position message
 *
 * @return  ENU, Loc
 */
static inline uint8_t mavlink_msg_get_home_position_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a get_home_position message into a struct
 *
 * @param msg The message to decode
 * @param get_home_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_get_home_position_decode(const mavlink_message_t* msg, mavlink_get_home_position_t* get_home_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    get_home_position->param = mavlink_msg_get_home_position_get_param(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GET_HOME_POSITION_LEN? msg->len : MAVLINK_MSG_ID_GET_HOME_POSITION_LEN;
        memset(get_home_position, 0, MAVLINK_MSG_ID_GET_HOME_POSITION_LEN);
    memcpy(get_home_position, _MAV_PAYLOAD(msg), len);
#endif
}
