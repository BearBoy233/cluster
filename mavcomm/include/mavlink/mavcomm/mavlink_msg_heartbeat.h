#pragma once
// MESSAGE HEARTBEAT PACKING

#define MAVLINK_MSG_ID_HEARTBEAT 0


typedef struct __mavlink_heartbeat_t {
 float battery_cell_voltage; /*<  电池单节电压 V.*/
 float battery_percentage; /*<  电池百分比 %.*/
 uint8_t px4_mode; /*<  rostopic /mavros/state/#mode .*/
 uint8_t px4_sys_state; /*<  rostopic /mavros/state/#system_state .*/
 uint8_t px4_state; /*<  rostopic /mavros/state (including connected, armed, manual_input, etc.).*/
 uint8_t ctrl_state; /*<  px4_ctrl 状态机.*/
 uint8_t gps_fix_type; /*<  rostopic /mavros/gpsstatus/gps1/raw/#gps_fix_type .*/
 uint8_t gps_satellites_visible; /*<  rostopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible .*/
} mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 14
#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN 14
#define MAVLINK_MSG_ID_0_LEN 14
#define MAVLINK_MSG_ID_0_MIN_LEN 14

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 209
#define MAVLINK_MSG_ID_0_CRC 209



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    0, \
    "HEARTBEAT", \
    8, \
    {  { "px4_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, px4_mode) }, \
         { "px4_sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heartbeat_t, px4_sys_state) }, \
         { "px4_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, px4_state) }, \
         { "ctrl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, ctrl_state) }, \
         { "battery_cell_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_heartbeat_t, battery_cell_voltage) }, \
         { "battery_percentage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_heartbeat_t, battery_percentage) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, gps_fix_type) }, \
         { "gps_satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heartbeat_t, gps_satellites_visible) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HEARTBEAT { \
    "HEARTBEAT", \
    8, \
    {  { "px4_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_heartbeat_t, px4_mode) }, \
         { "px4_sys_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_heartbeat_t, px4_sys_state) }, \
         { "px4_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_heartbeat_t, px4_state) }, \
         { "ctrl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_heartbeat_t, ctrl_state) }, \
         { "battery_cell_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_heartbeat_t, battery_cell_voltage) }, \
         { "battery_percentage", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_heartbeat_t, battery_percentage) }, \
         { "gps_fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_heartbeat_t, gps_fix_type) }, \
         { "gps_satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_heartbeat_t, gps_satellites_visible) }, \
         } \
}
#endif

/**
 * @brief Pack a heartbeat message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param px4_mode  rostopic /mavros/state/#mode .
 * @param px4_sys_state  rostopic /mavros/state/#system_state .
 * @param px4_state  rostopic /mavros/state (including connected, armed, manual_input, etc.).
 * @param ctrl_state  px4_ctrl 状态机.
 * @param battery_cell_voltage  电池单节电压 V.
 * @param battery_percentage  电池百分比 %.
 * @param gps_fix_type  rostopic /mavros/gpsstatus/gps1/raw/#gps_fix_type .
 * @param gps_satellites_visible  rostopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t px4_mode, uint8_t px4_sys_state, uint8_t px4_state, uint8_t ctrl_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_mode);
    _mav_put_uint8_t(buf, 9, px4_sys_state);
    _mav_put_uint8_t(buf, 10, px4_state);
    _mav_put_uint8_t(buf, 11, ctrl_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.px4_state = px4_state;
    packet.ctrl_state = ctrl_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Pack a heartbeat message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_mode  rostopic /mavros/state/#mode .
 * @param px4_sys_state  rostopic /mavros/state/#system_state .
 * @param px4_state  rostopic /mavros/state (including connected, armed, manual_input, etc.).
 * @param ctrl_state  px4_ctrl 状态机.
 * @param battery_cell_voltage  电池单节电压 V.
 * @param battery_percentage  电池百分比 %.
 * @param gps_fix_type  rostopic /mavros/gpsstatus/gps1/raw/#gps_fix_type .
 * @param gps_satellites_visible  rostopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible .
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_heartbeat_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t px4_mode,uint8_t px4_sys_state,uint8_t px4_state,uint8_t ctrl_state,float battery_cell_voltage,float battery_percentage,uint8_t gps_fix_type,uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_mode);
    _mav_put_uint8_t(buf, 9, px4_sys_state);
    _mav_put_uint8_t(buf, 10, px4_state);
    _mav_put_uint8_t(buf, 11, ctrl_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#else
    mavlink_heartbeat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.px4_state = px4_state;
    packet.ctrl_state = ctrl_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
}

/**
 * @brief Encode a heartbeat struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack(system_id, component_id, msg, heartbeat->px4_mode, heartbeat->px4_sys_state, heartbeat->px4_state, heartbeat->ctrl_state, heartbeat->battery_cell_voltage, heartbeat->battery_percentage, heartbeat->gps_fix_type, heartbeat->gps_satellites_visible);
}

/**
 * @brief Encode a heartbeat struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param heartbeat C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_heartbeat_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_heartbeat_t* heartbeat)
{
    return mavlink_msg_heartbeat_pack_chan(system_id, component_id, chan, msg, heartbeat->px4_mode, heartbeat->px4_sys_state, heartbeat->px4_state, heartbeat->ctrl_state, heartbeat->battery_cell_voltage, heartbeat->battery_percentage, heartbeat->gps_fix_type, heartbeat->gps_satellites_visible);
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 *
 * @param px4_mode  rostopic /mavros/state/#mode .
 * @param px4_sys_state  rostopic /mavros/state/#system_state .
 * @param px4_state  rostopic /mavros/state (including connected, armed, manual_input, etc.).
 * @param ctrl_state  px4_ctrl 状态机.
 * @param battery_cell_voltage  电池单节电压 V.
 * @param battery_percentage  电池百分比 %.
 * @param gps_fix_type  rostopic /mavros/gpsstatus/gps1/raw/#gps_fix_type .
 * @param gps_satellites_visible  rostopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible .
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_heartbeat_send(mavlink_channel_t chan, uint8_t px4_mode, uint8_t px4_sys_state, uint8_t px4_state, uint8_t ctrl_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HEARTBEAT_LEN];
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_mode);
    _mav_put_uint8_t(buf, 9, px4_sys_state);
    _mav_put_uint8_t(buf, 10, px4_state);
    _mav_put_uint8_t(buf, 11, ctrl_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t packet;
    packet.battery_cell_voltage = battery_cell_voltage;
    packet.battery_percentage = battery_percentage;
    packet.px4_mode = px4_mode;
    packet.px4_sys_state = px4_sys_state;
    packet.px4_state = px4_state;
    packet.ctrl_state = ctrl_state;
    packet.gps_fix_type = gps_fix_type;
    packet.gps_satellites_visible = gps_satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)&packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

/**
 * @brief Send a heartbeat message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_heartbeat_send_struct(mavlink_channel_t chan, const mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_heartbeat_send(chan, heartbeat->px4_mode, heartbeat->px4_sys_state, heartbeat->px4_state, heartbeat->ctrl_state, heartbeat->battery_cell_voltage, heartbeat->battery_percentage, heartbeat->gps_fix_type, heartbeat->gps_satellites_visible);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)heartbeat, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}

#if MAVLINK_MSG_ID_HEARTBEAT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_heartbeat_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t px4_mode, uint8_t px4_sys_state, uint8_t px4_state, uint8_t ctrl_state, float battery_cell_voltage, float battery_percentage, uint8_t gps_fix_type, uint8_t gps_satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, battery_cell_voltage);
    _mav_put_float(buf, 4, battery_percentage);
    _mav_put_uint8_t(buf, 8, px4_mode);
    _mav_put_uint8_t(buf, 9, px4_sys_state);
    _mav_put_uint8_t(buf, 10, px4_state);
    _mav_put_uint8_t(buf, 11, ctrl_state);
    _mav_put_uint8_t(buf, 12, gps_fix_type);
    _mav_put_uint8_t(buf, 13, gps_satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, buf, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#else
    mavlink_heartbeat_t *packet = (mavlink_heartbeat_t *)msgbuf;
    packet->battery_cell_voltage = battery_cell_voltage;
    packet->battery_percentage = battery_percentage;
    packet->px4_mode = px4_mode;
    packet->px4_sys_state = px4_sys_state;
    packet->px4_state = px4_state;
    packet->ctrl_state = ctrl_state;
    packet->gps_fix_type = gps_fix_type;
    packet->gps_satellites_visible = gps_satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HEARTBEAT, (const char *)packet, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
#endif
}
#endif

#endif

// MESSAGE HEARTBEAT UNPACKING


/**
 * @brief Get field px4_mode from heartbeat message
 *
 * @return  rostopic /mavros/state/#mode .
 */
static inline uint8_t mavlink_msg_heartbeat_get_px4_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field px4_sys_state from heartbeat message
 *
 * @return  rostopic /mavros/state/#system_state .
 */
static inline uint8_t mavlink_msg_heartbeat_get_px4_sys_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field px4_state from heartbeat message
 *
 * @return  rostopic /mavros/state (including connected, armed, manual_input, etc.).
 */
static inline uint8_t mavlink_msg_heartbeat_get_px4_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field ctrl_state from heartbeat message
 *
 * @return  px4_ctrl 状态机.
 */
static inline uint8_t mavlink_msg_heartbeat_get_ctrl_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field battery_cell_voltage from heartbeat message
 *
 * @return  电池单节电压 V.
 */
static inline float mavlink_msg_heartbeat_get_battery_cell_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field battery_percentage from heartbeat message
 *
 * @return  电池百分比 %.
 */
static inline float mavlink_msg_heartbeat_get_battery_percentage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field gps_fix_type from heartbeat message
 *
 * @return  rostopic /mavros/gpsstatus/gps1/raw/#gps_fix_type .
 */
static inline uint8_t mavlink_msg_heartbeat_get_gps_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field gps_satellites_visible from heartbeat message
 *
 * @return  rostopic /mavros/gpsstatus/gps1/raw/#gps_satellites_visible .
 */
static inline uint8_t mavlink_msg_heartbeat_get_gps_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a heartbeat message into a struct
 *
 * @param msg The message to decode
 * @param heartbeat C-struct to decode the message contents into
 */
static inline void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    heartbeat->battery_cell_voltage = mavlink_msg_heartbeat_get_battery_cell_voltage(msg);
    heartbeat->battery_percentage = mavlink_msg_heartbeat_get_battery_percentage(msg);
    heartbeat->px4_mode = mavlink_msg_heartbeat_get_px4_mode(msg);
    heartbeat->px4_sys_state = mavlink_msg_heartbeat_get_px4_sys_state(msg);
    heartbeat->px4_state = mavlink_msg_heartbeat_get_px4_state(msg);
    heartbeat->ctrl_state = mavlink_msg_heartbeat_get_ctrl_state(msg);
    heartbeat->gps_fix_type = mavlink_msg_heartbeat_get_gps_fix_type(msg);
    heartbeat->gps_satellites_visible = mavlink_msg_heartbeat_get_gps_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HEARTBEAT_LEN? msg->len : MAVLINK_MSG_ID_HEARTBEAT_LEN;
        memset(heartbeat, 0, MAVLINK_MSG_ID_HEARTBEAT_LEN);
    memcpy(heartbeat, _MAV_PAYLOAD(msg), len);
#endif
}
