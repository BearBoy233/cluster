// ADD sim support
// Function: 自定义 Mavlink 消息 MAVCOMM 与 ROS 话题之间转换 
// supported P900 - ATS153=1 - in [p2m] OR [Mesh] mode 
// TODO Add UDP & TCP support 
// TODO Add group support 
// TODO Refer MAVROS 

#define ID_GCS 100 
#define ID_ALL 99 

#include <ros/ros.h> 
#include <serial/serial.h> 
#include <signal.h> 

#include <mavlink/mavcomm/mavlink.h>	// Mavlink Customize

// ROS msg 
// adding 
// #include <mavcomm_msgs/XXXX.h> 
#include <mavcomm_msgs/Heartbeat.h>
#include <mavcomm_msgs/Console.h>
#include <mavcomm_msgs/Console_monitor.h>
#include <mavcomm_msgs/ChangeState.h>
#include <mavcomm_msgs/local_pos_enu.h>
#include <mavcomm_msgs/global_pos_int.h>
#include <mavcomm_msgs/get_param.h>
#include <mavcomm_msgs/kcf_set_target.h>
#include <mavcomm_msgs/kcf_target_pos.h>

#include <mavcomm_msgs/mission_info.h>
#include <mavcomm_msgs/mission_back_info.h>
#include <mavcomm_msgs/mission_set.h>

// only for sim
#include <mavcomm_msgs/Mavlink.h>
// for sim flag
int flag_sim_1s_2m = 0;	// 仿真测试 标志

class uavComm {

public :

    uavComm( 
    const ros::NodeHandle &nh_pub_, 
    const ros::NodeHandle &nh_sub_
    );

    ~uavComm();

    // ROS part
    ros::NodeHandle nh_pub;
    ros::NodeHandle nh_sub;

    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;
    // mavros (pub/sub)
    ros::NodeHandle mavros_nh;
    // desired
	ros::NodeHandle desired_nh;
    // gen
	ros::NodeHandle gen_nh;

};

//-------------------------------------------------------------------------------
// Test Print flag
int Flag_1ShowRn;	// 标志位 1将读取的 Rn 显示在屏幕上

// system param
int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
int companion_id;	// compid 接收端编号	->发送时  接收端编号	/ 接收时 接收端编号
int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

ros::Rate *loop_rate;

void mavcomm_run(int flag);

//-------------------------------------------------------------------------------
// 串口相关
serial::Serial SerialPoint;		//创建一个serial类
std::string serial_port;		//串口 端口名
int serial_baudrate;			//串口 波特率

// 串口收发的数组
uint8_t Rbuffer_rx[4096];	    // 串口接收Raw 字节 
size_t  Rn;					    // 串口接收Raw 字节数
size_t last_Rn = 0;				// 用于 读取不完全的时候

uint8_t Wbuffer_tx[4096];	    // 串口发送Raw 字节

// 多通道	来自多个数传
int Flag_ats153;		// 对于 P900 数传 开启地址模式 ATS153=1
uint8_t current_chan;	
uint8_t mac_data[80];	
int mac_data_num = 0;	// 数传地址
bool flag_p900_type3;			
size_t  p900_type3_remain_rn;

// 串口数据读取 mavlink处理
void handle_recieve_msg_ats153_type3();		// 不对的数据直接抛弃了
void handle_recieve_msg();

int serial_set_open();	// Open serial port
int read_buffer(mavlink_message_t &message, uint8_t cp);
int read_buffer_chan(mavlink_message_t &message, uint8_t cp, uint8_t chan);
void send_buffer(mavlink_message_t* mmsg);

//-------------------------------------------------------------------------------
// help func
uint64_t get_time_usec();	// Time 获取系统时间
void quit_handler(int sig);	// QuitSignalHandler  Called when you press Ctrl-C

//-----------------------------------------------------------------------------------------
//			ROS 发布 话题 pub()
//-----------------------------------------------------------------------------------------
void publish_mavcomm_recieve(mavlink_message_t mmsg);

// heartbeat
ros::Publisher 			pub_heartbeat;
mavlink_heartbeat_t 	mt_pub_heartbeat;
mavcomm_msgs::Heartbeat msg_pub_heartbeat;
//  console
ros::Publisher 			pub_console;
mavlink_console_t 	    mt_pub_console;
mavcomm_msgs::Console 	msg_pub_console;
//  Console_monitor
ros::Publisher 			pub_console_monitor;
mavlink_console_monitor_t 	    mt_pub_console_monitor;
mavcomm_msgs::Console_monitor 	msg_pub_console_monitor;
//  changestate
ros::Publisher 				pub_changestate;
mavlink_changestate_t 	    mt_pub_changestate;
mavcomm_msgs::ChangeState 	msg_pub_changestate;

// 1001 LOCAL_POSITION_ENU 无 flag
ros::Publisher 					pub_local_pos_enu;
mavlink_local_position_enu_t 	mt_pub_local_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_pub_local_pos_enu;
// 1101 SET_LOCAL_POSITION_ENU
ros::Publisher 						pub_set_local_pos_enu;
mavlink_set_local_position_enu_t 	mt_pub_set_local_pos_enu;
mavcomm_msgs::local_pos_enu 		msg_pub_set_local_pos_enu;
// 1112 BACK_HOME_POS_ENU 无 flag
ros::Publisher 					pub_back_home_pos_enu;
mavlink_back_home_pos_enu_t 	mt_pub_back_home_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_pub_back_home_pos_enu;
// 1121 SET_HOME_POS_ENU 无 flag yaw
ros::Publisher 					pub_set_home_pos_enu;
mavlink_set_home_pos_enu_t 		mt_pub_set_home_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_pub_set_home_pos_enu;

// 1002 GLOBAL_POSITION_INT 无 flag
ros::Publisher 					pub_global_position_int;
mavlink_global_position_int_t 	mt_pub_global_position_int;
mavcomm_msgs::global_pos_int 	msg_pub_global_position_int;
// 1102 SET_GLOBAL_POSITION_INT
ros::Publisher 						pub_set_global_position_int;
mavlink_set_global_position_int_t 	mt_pub_set_global_position_int;
mavcomm_msgs::global_pos_int 		msg_pub_set_global_position_int;
// 1113 BACK_HOME_POS_GPS_INT 无 flag
ros::Publisher 					pub_back_home_gps_pos_int;
mavlink_back_home_pos_gps_int_t mt_pub_back_home_gps_pos_int;
mavcomm_msgs::global_pos_int 	msg_pub_back_home_gps_pos_int;
// 1122 SET_HOME_POS_GPS_INT 无 flag hdg_yaw
ros::Publisher 					pub_set_home_pos_gps_int;
mavlink_set_home_pos_gps_int_t 	mt_pub_set_home_pos_gps_int;
mavcomm_msgs::global_pos_int 	msg_pub_set_home_pos_gps_int;

// 1111 GET_HOME_POSITION
ros::Publisher 					pub_get_home_position;
mavlink_get_home_position_t 	mt_pub_get_home_position;
mavcomm_msgs::get_param 		msg_pub_get_home_position;

// 702 kcf_set_target
ros::Publisher 					pub_kcf_set_target;
mavlink_kcf_set_target_t 		mt_pub_kcf_set_target;
mavcomm_msgs::kcf_set_target 	msg_pub_kcf_set_target;

// 701 kcf_target_pos
ros::Publisher 					pub_kcf_target_pos;
mavlink_kcf_target_pos_t 		mt_pub_kcf_target_pos;
mavcomm_msgs::kcf_target_pos 	msg_pub_kcf_target_pos;

// mission part
// mission_info
ros::Publisher 					pub_mission_info;
mavlink_mission_info_t 		mt_pub_mission_info;
mavcomm_msgs::mission_info 	msg_pub_mission_info;

// mission_back_info
ros::Publisher 					pub_mission_back_info;
mavlink_mission_back_info_t 		mt_pub_mission_back_info;
mavcomm_msgs::mission_back_info 	msg_pub_mission_back_info;	

// mission_set
ros::Publisher 					pub_mission_set;
mavlink_mission_set_t 		mt_pub_mission_set;
mavcomm_msgs::mission_set 	msg_pub_mission_set;	

//-----------------------------------------------------------------------------------------
//			ROS 订阅 话题 sub()
//-----------------------------------------------------------------------------------------

// heartbeat
ros::Subscriber 		sub_heartbeat;
mavlink_heartbeat_t 	mt_sub_heartbeat;
mavcomm_msgs::Heartbeat msg_sub_heartbeat; 
// console
ros::Subscriber 		sub_console;
mavlink_console_t 		mt_sub_console;
mavcomm_msgs::Console 	msg_sub_console; 
// console_monitor
ros::Subscriber 		sub_console_monitor;
mavlink_console_monitor_t 		mt_sub_console_monitor;
mavcomm_msgs::Console_monitor 	msg_sub_console_monitor; 
// changestate
ros::Subscriber 			sub_changestate;
mavlink_changestate_t 		mt_sub_changestate;
mavcomm_msgs::ChangeState 	msg_sub_changestate; 

// 1001 LOCAL_POSITION_ENU 无 flag
ros::Subscriber 				sub_local_pos_enu;
mavlink_local_position_enu_t 	mt_sub_local_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_sub_local_pos_enu;
// 1101 SET_LOCAL_POSITION_ENU
ros::Subscriber 					sub_set_local_pos_enu;
mavlink_set_local_position_enu_t 	mt_sub_set_local_pos_enu;
mavcomm_msgs::local_pos_enu 		msg_sub_set_local_pos_enu;
// 1112 BACK_HOME_POS_ENU 无 flag
ros::Subscriber 				sub_back_home_pos_enu;
mavlink_back_home_pos_enu_t 	mt_sub_back_home_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_sub_back_home_pos_enu;
// 1121 SET_HOME_POS_ENU 无 flag yaw
ros::Subscriber 				sub_set_home_pos_enu;
mavlink_set_home_pos_enu_t 		mt_sub_set_home_pos_enu;
mavcomm_msgs::local_pos_enu 	msg_sub_set_home_pos_enu;

// 1002 GLOBAL_POSITION_INT 无 flag
ros::Subscriber 				sub_global_position_int;
mavlink_global_position_int_t 	mt_sub_global_position_int;
mavcomm_msgs::global_pos_int 	msg_sub_global_position_int;
// 1102 SET_GLOBAL_POSITION_INT
ros::Subscriber 					sub_set_global_position_int;
mavlink_set_global_position_int_t 	mt_sub_set_global_position_int;
mavcomm_msgs::global_pos_int 		msg_sub_set_global_position_int;
// 1113 BACK_HOME_POS_GPS_INT 无 flag
ros::Subscriber 				sub_back_home_gps_pos_int;
mavlink_back_home_pos_gps_int_t mt_sub_back_home_gps_pos_int;
mavcomm_msgs::global_pos_int 	msg_sub_back_home_gps_pos_int;
// 1122 SET_HOME_POS_GPS_INT 无 flag hdg_yaw
ros::Subscriber 				sub_set_home_pos_gps_int;
mavlink_set_home_pos_gps_int_t 	mt_sub_set_home_pos_gps_int;
mavcomm_msgs::global_pos_int 	msg_sub_set_home_pos_gps_int;

// 1111 GET_HOME_POSITION
ros::Subscriber 				sub_get_home_position;
mavlink_get_home_position_t 	mt_sub_get_home_position;
mavcomm_msgs::get_param 		msg_sub_get_home_position;

// 702 kcf_set_target
ros::Subscriber 				sub_kcf_set_target;
mavlink_kcf_set_target_t 		mt_sub_kcf_set_target;
mavcomm_msgs::kcf_set_target 	msg_sub_kcf_set_target;

// 701 kcf_target_pos
ros::Subscriber 				sub_kcf_target_pos;
mavlink_kcf_target_pos_t 		mt_sub_kcf_target_pos;
mavcomm_msgs::kcf_target_pos 	msg_sub_kcf_target_pos;

// mission part
// mission_info
ros::Subscriber 				sub_mission_info;
mavlink_mission_info_t 		mt_sub_mission_info;
mavcomm_msgs::mission_info 	msg_sub_mission_info;

// mission_back_info
ros::Subscriber 				sub_mission_back_info;
mavlink_mission_back_info_t 		mt_sub_mission_back_info;
mavcomm_msgs::mission_back_info 	msg_sub_mission_back_info;

// mission_set
ros::Subscriber 				sub_mission_set;
mavlink_mission_set_t 		mt_sub_mission_set;
mavcomm_msgs::mission_set 	msg_sub_mission_set;