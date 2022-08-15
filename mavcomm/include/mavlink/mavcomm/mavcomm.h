/** @file
 *  @brief MAVLink comm protocol generated from mavcomm.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_MAVCOMM_H
#define MAVLINK_MAVCOMM_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_MAVCOMM.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 209, 14, 14, 0, 0, 0}, {2, 222, 2, 2, 0, 0, 0}, {40, 93, 4, 4, 0, 0, 0}, {41, 165, 4, 4, 0, 0, 0}, {701, 16, 11, 11, 0, 0, 0}, {702, 105, 10, 10, 0, 0, 0}, {800, 98, 4, 4, 0, 0, 0}, {801, 12, 4, 4, 0, 0, 0}, {802, 74, 23, 23, 0, 0, 0}, {1001, 148, 16, 16, 0, 0, 0}, {1002, 170, 14, 14, 0, 0, 0}, {1101, 232, 17, 17, 0, 0, 0}, {1102, 131, 15, 15, 0, 0, 0}, {1111, 203, 1, 1, 0, 0, 0}, {1112, 0, 16, 16, 0, 0, 0}, {1113, 6, 14, 14, 0, 0, 0}, {1121, 228, 12, 12, 0, 0, 0}, {1122, 79, 12, 12, 0, 0, 0}, {17000, 103, 179, 179, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_MAVCOMM

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_console.h"
#include "./mavlink_msg_console_monitor.h"
#include "./mavlink_msg_changestate.h"
#include "./mavlink_msg_local_position_enu.h"
#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_set_local_position_enu.h"
#include "./mavlink_msg_set_global_position_int.h"
#include "./mavlink_msg_get_home_position.h"
#include "./mavlink_msg_back_home_pos_enu.h"
#include "./mavlink_msg_back_home_pos_gps_int.h"
#include "./mavlink_msg_set_home_pos_enu.h"
#include "./mavlink_msg_set_home_pos_gps_int.h"
#include "./mavlink_msg_kcf_target_pos.h"
#include "./mavlink_msg_kcf_set_target.h"
#include "./mavlink_msg_mission_info.h"
#include "./mavlink_msg_mission_back_info.h"
#include "./mavlink_msg_mission_set.h"
#include "./mavlink_msg_test_types.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_CHANGESTATE, MAVLINK_MESSAGE_INFO_CONSOLE, MAVLINK_MESSAGE_INFO_CONSOLE_Monitor, MAVLINK_MESSAGE_INFO_kcf_target_pos, MAVLINK_MESSAGE_INFO_kcf_set_target, MAVLINK_MESSAGE_INFO_mission_info, MAVLINK_MESSAGE_INFO_mission_back_info, MAVLINK_MESSAGE_INFO_mission_set, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_ENU, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_SET_LOCAL_POSITION_ENU, MAVLINK_MESSAGE_INFO_SET_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_GET_HOME_POSITION, MAVLINK_MESSAGE_INFO_BACK_HOME_POS_ENU, MAVLINK_MESSAGE_INFO_BACK_HOME_POS_GPS_INT, MAVLINK_MESSAGE_INFO_SET_HOME_POS_ENU, MAVLINK_MESSAGE_INFO_SET_HOME_POS_GPS_INT, MAVLINK_MESSAGE_INFO_TEST_TYPES}
# define MAVLINK_MESSAGE_NAMES {{ "BACK_HOME_POS_ENU", 1112 }, { "BACK_HOME_POS_GPS_INT", 1113 }, { "CHANGESTATE", 2 }, { "CONSOLE", 40 }, { "CONSOLE_Monitor", 41 }, { "GET_HOME_POSITION", 1111 }, { "GLOBAL_POSITION_INT", 1002 }, { "HEARTBEAT", 0 }, { "LOCAL_POSITION_ENU", 1001 }, { "SET_GLOBAL_POSITION_INT", 1102 }, { "SET_HOME_POS_ENU", 1121 }, { "SET_HOME_POS_GPS_INT", 1122 }, { "SET_LOCAL_POSITION_ENU", 1101 }, { "TEST_TYPES", 17000 }, { "kcf_set_target", 702 }, { "kcf_target_pos", 701 }, { "mission_back_info", 801 }, { "mission_info", 800 }, { "mission_set", 802 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_MAVCOMM_H
