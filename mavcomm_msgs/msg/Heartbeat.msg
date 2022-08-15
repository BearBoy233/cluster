
std_msgs/Header header
uint8 sysid
uint8 compid

# px4_mode
uint8 MANUAL = 1
uint8 ACRO = 2
uint8 ALTCTL = 3
uint8 POSCTL = 4
uint8 OFFBOARD = 5
uint8 STABILIZED = 6
uint8 RATTITUDE = 7
uint8 AUTO_MISSION = 8
uint8 AUTO_LOITER = 9
uint8 AUTO_RTL = 10
uint8 AUTO_LAND = 11
uint8 AUTO_RTGS = 12
uint8 AUTO_READY = 13
uint8 AUTO_TAKEOFF = 14

# px4_state

# ctrl_state


uint8 px4_mode
uint8 px4_state
uint8 px4_sys_state
uint8 ctrl_state
float32 battery_cell_voltage
float32 battery_percentage
uint8 gps_fix_type
uint8 gps_satellites_visible

