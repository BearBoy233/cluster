// Function: 自定义Mavlink消息 MAVCOMM 与 ROS 话题之间转换 
// supported P900 - ATS153=1 - in [p2m] OR [Mesh] mode
// TODO Add UDP & TCP supported
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

//-------------------------------------------------------------------------------
// Test Print flag
int Flag_1ShowRn;	// 标志位 1将读取的 Rn 显示在屏幕上

// system param
int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
int companion_id;	// compid 接收端编号	->发送时  接收端编号	/ 接收时 接收端编号
int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

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
//			ROS pub()
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
//			ROS sub()
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

//-----------------------------------------------------------------------------------------
//			ros_cb() 回调函数
//-----------------------------------------------------------------------------------------

void cb_sub_heartbeat(const mavcomm_msgs::Heartbeat::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_heartbeat.battery_cell_voltage = rmsg->battery_cell_voltage;
	mt_sub_heartbeat.battery_percentage = rmsg->battery_percentage;
	mt_sub_heartbeat.ctrl_state = rmsg->ctrl_state;
	mt_sub_heartbeat.px4_sys_state = rmsg->px4_sys_state;
	mt_sub_heartbeat.px4_mode = rmsg->px4_mode;
	mt_sub_heartbeat.px4_state = rmsg->px4_state;
	mt_sub_heartbeat.gps_fix_type = rmsg->gps_fix_type;
	mt_sub_heartbeat.gps_satellites_visible = rmsg->gps_satellites_visible;

	mavlink_msg_heartbeat_encode( my_id, rmsg->compid, &mmsg, &mt_sub_heartbeat);
    send_buffer( &mmsg );
}

void cb_sub_console(const mavcomm_msgs::Console::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_console.command = rmsg->command;
	mt_sub_console.flag = rmsg->flag;
	mt_sub_console.type1 = rmsg->type1;
	mt_sub_console.type2 = rmsg->type2;

	mavlink_msg_console_encode( my_id, rmsg->compid, &mmsg, &mt_sub_console);
    send_buffer( &mmsg );
}

void cb_sub_console_monitor(const mavcomm_msgs::Console_monitor::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_console_monitor.param1 = rmsg->param1;
	mt_sub_console_monitor.param2 = rmsg->param2;
	mt_sub_console_monitor.param3 = rmsg->param3;
	mt_sub_console_monitor.param4 = rmsg->param4;

	mavlink_msg_console_monitor_encode( my_id, rmsg->compid, &mmsg, &mt_sub_console_monitor);
    send_buffer( &mmsg );
}

void cb_sub_changestate(const mavcomm_msgs::ChangeState::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_changestate.node_state = rmsg->node_state;
	mt_sub_changestate.param = rmsg->param;

	mavlink_msg_changestate_encode( my_id, rmsg->compid, &mmsg, &mt_sub_changestate);
	send_buffer( &mmsg );
}

void cb_sub_local_pos_enu(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_local_pos_enu.x = rmsg->x;
	mt_sub_local_pos_enu.y = rmsg->y;
	mt_sub_local_pos_enu.z = rmsg->z;
	mt_sub_local_pos_enu.yaw = rmsg->yaw;

	mavlink_msg_local_position_enu_encode( my_id, rmsg->compid, &mmsg, &mt_sub_local_pos_enu);
	send_buffer( &mmsg );
}

void cb_sub_set_local_pos_enu(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_set_local_pos_enu.x = rmsg->x;
	mt_sub_set_local_pos_enu.y = rmsg->y;
	mt_sub_set_local_pos_enu.z = rmsg->z;
	mt_sub_set_local_pos_enu.yaw = rmsg->yaw;
	mt_sub_set_local_pos_enu.flag = rmsg->flag;

	mavlink_msg_set_local_position_enu_encode( my_id, rmsg->compid, &mmsg, &mt_sub_set_local_pos_enu);
	send_buffer( &mmsg );
}

void cb_sub_back_home_pos_enu(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_back_home_pos_enu.x = rmsg->x;
	mt_sub_back_home_pos_enu.y = rmsg->y;
	mt_sub_back_home_pos_enu.z = rmsg->z;
	mt_sub_back_home_pos_enu.yaw = rmsg->yaw;

	mavlink_msg_back_home_pos_enu_encode( my_id, rmsg->compid, &mmsg, &mt_sub_back_home_pos_enu);
	send_buffer( &mmsg );
}

void cb_sub_set_home_pos_enu(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_set_home_pos_enu.x = rmsg->x;
	mt_sub_set_home_pos_enu.y = rmsg->y;
	mt_sub_set_home_pos_enu.z = rmsg->z;

	mavlink_msg_set_home_pos_enu_encode( my_id, rmsg->compid, &mmsg, &mt_sub_set_home_pos_enu);
	send_buffer( &mmsg );
}
	
void cb_sub_global_position_int(const mavcomm_msgs::global_pos_int::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_global_position_int.lat = rmsg->lat;
	mt_sub_global_position_int.lon = rmsg->lon;
	mt_sub_global_position_int.alt = rmsg->alt;
	mt_sub_global_position_int.hdg_yaw = rmsg->hdg_yaw;

	mavlink_msg_global_position_int_encode( my_id, rmsg->compid, &mmsg, &mt_sub_global_position_int);
	send_buffer( &mmsg );
}

void cb_sub_set_global_position_int(const mavcomm_msgs::global_pos_int::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_set_global_position_int.lat = rmsg->lat;
	mt_sub_set_global_position_int.lon = rmsg->lon;
	mt_sub_set_global_position_int.alt = rmsg->alt;
	mt_sub_set_global_position_int.hdg_yaw = rmsg->hdg_yaw;
	mt_sub_set_global_position_int.flag = rmsg->flag;

	mavlink_msg_set_global_position_int_encode( my_id, rmsg->compid, &mmsg, &mt_sub_set_global_position_int);
	send_buffer( &mmsg );
}

void cb_sub_back_home_gps_pos_int(const mavcomm_msgs::global_pos_int::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_back_home_gps_pos_int.lat = rmsg->lat;
	mt_sub_back_home_gps_pos_int.lon = rmsg->lon;
	mt_sub_back_home_gps_pos_int.alt = rmsg->alt;
	mt_sub_back_home_gps_pos_int.hdg_yaw = rmsg->hdg_yaw;

	mavlink_msg_back_home_pos_gps_int_encode( my_id, rmsg->compid, &mmsg, &mt_sub_back_home_gps_pos_int);
	send_buffer( &mmsg );
}

void cb_sub_set_home_pos_gps_int(const mavcomm_msgs::global_pos_int::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_set_home_pos_gps_int.lat = rmsg->lat;
	mt_sub_set_home_pos_gps_int.lon = rmsg->lon;
	mt_sub_set_home_pos_gps_int.alt = rmsg->alt;

	mavlink_msg_set_home_pos_gps_int_encode( my_id, rmsg->compid, &mmsg, &mt_sub_set_home_pos_gps_int);
	send_buffer( &mmsg );
}

void cb_sub_get_home_position(const mavcomm_msgs::get_param::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_get_home_position.param = rmsg->param;

	mavlink_msg_get_home_position_encode( my_id, rmsg->compid, &mmsg, &mt_sub_get_home_position);
	send_buffer( &mmsg );
}

void cb_sub_kcf_set_target(const mavcomm_msgs::kcf_set_target::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_kcf_set_target.state = rmsg->state;
	mt_sub_kcf_set_target.target_no = rmsg->target_no;
	mt_sub_kcf_set_target.x = rmsg->x;
	mt_sub_kcf_set_target.y = rmsg->y;
	mt_sub_kcf_set_target.width = rmsg->width;
	mt_sub_kcf_set_target.height = rmsg->height;

	mavlink_msg_kcf_set_target_encode( my_id, rmsg->compid, &mmsg, &mt_sub_kcf_set_target);
	send_buffer( &mmsg );
}

void cb_sub_kcf_target_pos(const mavcomm_msgs::kcf_target_pos::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_kcf_target_pos.state = rmsg->state;
	mt_sub_kcf_target_pos.target_no = rmsg->target_no;
	mt_sub_kcf_target_pos.x = rmsg->x;
	mt_sub_kcf_target_pos.y = rmsg->y;
	mt_sub_kcf_target_pos.flag = rmsg->flag;

	mavlink_msg_kcf_target_pos_encode( my_id, rmsg->compid, &mmsg, &mt_sub_kcf_target_pos);
	send_buffer( &mmsg );
}

void cb_sub_mission_info(const mavcomm_msgs::mission_info::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_mission_info.flag =  rmsg->flag;
	mt_sub_mission_info.mission_num =  rmsg->mission_num;
	mt_sub_mission_info.param1 =  rmsg->param1;
	mt_sub_mission_info.param2 =  rmsg->param2;

	mavlink_msg_mission_info_encode( my_id, rmsg->compid, &mmsg, &mt_sub_mission_info);
	send_buffer( &mmsg );
}

void cb_sub_mission_back_info(const mavcomm_msgs::mission_back_info::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_mission_back_info.flag =  rmsg->flag;
	mt_sub_mission_back_info.mission_num =  rmsg->mission_num;
	mt_sub_mission_back_info.param1 =  rmsg->param1;
	mt_sub_mission_back_info.param2 =  rmsg->param2;

	mavlink_msg_mission_back_info_encode( my_id, rmsg->compid, &mmsg, &mt_sub_mission_back_info);
	send_buffer( &mmsg );
}

void cb_sub_mission_set(const mavcomm_msgs::mission_set::ConstPtr& rmsg)
{	
    mavlink_message_t		mmsg;

	mt_sub_mission_set.flag =  rmsg->flag;
	mt_sub_mission_set.mission_no =  rmsg->mission_no;
	mt_sub_mission_set.mission_task =  rmsg->mission_task;

	mt_sub_mission_set.uav_no =  rmsg->uav_no;
	mt_sub_mission_set.x =  rmsg->x;
	mt_sub_mission_set.y =  rmsg->y;
	mt_sub_mission_set.z =  rmsg->z;
	mt_sub_mission_set.yaw =  rmsg->yaw;

	mt_sub_mission_set.param1 =  rmsg->param1;
	mt_sub_mission_set.param2 =  rmsg->param2;
	mt_sub_mission_set.param3 =  rmsg->param3;

	mavlink_msg_mission_set_encode( my_id, rmsg->compid, &mmsg, &mt_sub_mission_set);
	send_buffer( &mmsg );
}

//-----------------------------------------------------------------------------------------
//			Main()
//-----------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavcomm");

	ros::NodeHandle nh("~");
    ros::NodeHandle nh_pub("mavcomm/receive"); // /mavcomm/receive");
    ros::NodeHandle nh_sub("mavcomm/send"); // /mavcomm/send");

    // ROS pub
	// Adding e,g.
	// pub_XXXX = nh_pub.advertise<mavcomm_msgs::XXXX>("XXXX", 5);
	// heartbeat
	pub_heartbeat = nh_pub.advertise<mavcomm_msgs::Heartbeat>("heartbeat", 10);
	// console
	pub_console = nh_pub.advertise<mavcomm_msgs::Console>("console", 5);
	// console_monitor
	pub_console_monitor = nh_pub.advertise<mavcomm_msgs::Console_monitor>("console_monitor", 5);
	// changestate
	pub_changestate = nh_pub.advertise<mavcomm_msgs::ChangeState>("changestate", 5);
	// local
	pub_local_pos_enu = nh_pub.advertise<mavcomm_msgs::local_pos_enu>("loc_pos_enu", 5);
	pub_set_local_pos_enu = nh_pub.advertise<mavcomm_msgs::local_pos_enu>("set_loc_pos_enu", 5);
	pub_back_home_pos_enu = nh_pub.advertise<mavcomm_msgs::local_pos_enu>("back_home_pos_enu", 5);
	pub_set_home_pos_enu = nh_pub.advertise<mavcomm_msgs::local_pos_enu>("set_home_pos_enu", 5);
	// gps
	pub_global_position_int = nh_pub.advertise<mavcomm_msgs::global_pos_int>("gps_pos", 5);
	pub_set_global_position_int = nh_pub.advertise<mavcomm_msgs::global_pos_int>("set_gps_pos", 5);
	pub_back_home_gps_pos_int = nh_pub.advertise<mavcomm_msgs::global_pos_int>("back_home_gps_pos", 5);
	pub_set_home_pos_gps_int = nh_pub.advertise<mavcomm_msgs::global_pos_int>("set_home_gps_pos", 5);
	// get param
	pub_get_home_position = nh_pub.advertise<mavcomm_msgs::get_param>("get_home_pos", 5);
	// get kcf_set_target
	pub_kcf_set_target = nh_pub.advertise<mavcomm_msgs::kcf_set_target>("kcf_set_target", 5);
	// get kcf_target_pos
	pub_kcf_target_pos = nh_pub.advertise<mavcomm_msgs::kcf_target_pos>("kcf_target_pos", 5);

	// get mission_info
	pub_mission_info = nh_pub.advertise<mavcomm_msgs::mission_info>("mission_info", 5);
	// get mission_back_info
	pub_mission_back_info = nh_pub.advertise<mavcomm_msgs::mission_back_info>("mission_back_info", 5);
	// get mission_set
	pub_mission_set = nh_pub.advertise<mavcomm_msgs::mission_set>("mission_set", 5);

    // ROS sub
    // adding 需要通过 串口发布的话题
	// heartbeat
	sub_heartbeat = nh_sub.subscribe<mavcomm_msgs::Heartbeat>("heartbeat", 1, cb_sub_heartbeat);
	// console
	sub_console = nh_sub.subscribe<mavcomm_msgs::Console>("console", 10, cb_sub_console);
	// console_monitor
	sub_console_monitor = nh_sub.subscribe<mavcomm_msgs::Console_monitor>("console_monitor", 1, cb_sub_console_monitor);
	// changestate
	sub_changestate = nh_sub.subscribe<mavcomm_msgs::ChangeState>("changestate", 10, cb_sub_changestate);
	// local_pos_enu.msg
	sub_local_pos_enu = nh_sub.subscribe<mavcomm_msgs::local_pos_enu>("loc_pos_enu", 1, cb_sub_local_pos_enu);
	sub_set_local_pos_enu = nh_sub.subscribe<mavcomm_msgs::local_pos_enu>("set_loc_pos_enu", 5, cb_sub_set_local_pos_enu);
	sub_back_home_pos_enu = nh_sub.subscribe<mavcomm_msgs::local_pos_enu>("back_home_pos_enu", 1, cb_sub_back_home_pos_enu);
	sub_set_home_pos_enu = nh_sub.subscribe<mavcomm_msgs::local_pos_enu>("set_home_pos_enu", 5, cb_sub_set_home_pos_enu);
	// gps
	sub_global_position_int = nh_sub.subscribe<mavcomm_msgs::global_pos_int>("gps_pos", 1, cb_sub_global_position_int);
	sub_set_global_position_int = nh_sub.subscribe<mavcomm_msgs::global_pos_int>("set_gps_pos", 5, cb_sub_set_global_position_int);
	sub_back_home_gps_pos_int = nh_sub.subscribe<mavcomm_msgs::global_pos_int>("back_home_gps_pos", 1, cb_sub_back_home_gps_pos_int);
	sub_set_home_pos_gps_int = nh_sub.subscribe<mavcomm_msgs::global_pos_int>("set_home_gps_pos", 5, cb_sub_set_home_pos_gps_int);
	// get param
	sub_get_home_position = nh_sub.subscribe<mavcomm_msgs::get_param>("get_home_pos", 1, cb_sub_get_home_position);
	// kcf_set_target
	sub_kcf_set_target = nh_sub.subscribe<mavcomm_msgs::kcf_set_target>("kcf_set_target", 1, cb_sub_kcf_set_target);
	// kcf_target_pos
	sub_kcf_target_pos = nh_sub.subscribe<mavcomm_msgs::kcf_target_pos>("kcf_target_pos", 1, cb_sub_kcf_target_pos);

	// mission_info
	sub_mission_info = nh_sub.subscribe<mavcomm_msgs::mission_info>("mission_info", 1, cb_sub_mission_info);
	// mission_back_info
	sub_mission_back_info = nh_sub.subscribe<mavcomm_msgs::mission_back_info>("mission_back_info", 1, cb_sub_mission_back_info);
	// mission_set
	sub_mission_set = nh_sub.subscribe<mavcomm_msgs::mission_set>("mission_set", 1, cb_sub_mission_set);

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
	nh.param<int>("serial_baudrate", serial_baudrate, 57600);
	nh.param<int>("my_id", my_id, 100);		// 本机编号 	[ 100-地面站 ] 	[99-所有无人机]		
	nh.param<int>("Flag_1ShowRn", Flag_1ShowRn, 0);  // 1将接收到的字节打印到屏幕上 2同时打印 Byte
	nh.param<int>("Flag_ats153", Flag_ats153, 0);  // 0-没有前端 1-有10byte前端 
	// 		   01 00     2d  00    00 f0 49 00 ac e3 + data
	//10 byte 包字节数   RSSI 0x00  MAC
    std::cout << "mavcomm/serial_port = " << serial_port << std::endl;
    std::cout << "mavcomm/serial_baudrate = " << serial_baudrate << std::endl;
	std::cout << "mavcomm/my_id = " << my_id << std::endl;
	std::cout << "mavcomm/Flag_1ShowRn = " << Flag_1ShowRn << std::endl;
    std::cout << "mavcomm/Flag_ats153 = " << Flag_ats153 << std::endl;

	ros::Rate loop_rate(50);

	//串口serial设置&开启
	serial_set_open();

    // Init 初值
	last_Rn = 0;    		// 之前未处理啊完的数据
	mac_data_num = 0;		// 已知mac地址的数传模块数量
	flag_p900_type3 = 0;			
	p900_type3_remain_rn = 0;
    
    // 等待
	ros::Duration ( 0.5 );

	while ( ros::ok() )
	{	
		//读取  串口数据
		Rn = SerialPoint.available();	//获取缓冲区内的字节数
		// 如果有待读取字节数
		if (Rn != 0) {
			Rn = SerialPoint.read(Rbuffer_rx+last_Rn, Rn); //读出数据
			Rn = Rn + last_Rn;
			last_Rn = 0;
			// sys_trans_data.Recieve_KB = sys_trans_data.Recieve_KB + ((double)Rn)/1024;

			if (Flag_1ShowRn) {
				std::cout <<"Receive " << std::dec << (int)Rn <<" byte."<< std::endl;
				if (Flag_1ShowRn == 2) {
					for (int i = 0; i < Rn; i++) { std::cout << std::hex << (Rbuffer_rx[i] & 0xff) << " ";}
					std::cout << std::endl;
				}
			}

			if (Flag_ats153) // Mesh/多对一模式 有MAC地址帧头 适配 P900模块
            {
				// TODO 帧头带有 RSSI 信息， 地面站可以尝试显示
				handle_recieve_msg_ats153_type3();		// 1+2的改进，长度不对，下一次读取继续解析
			}	
			else
			{	// 普通 无mac地址 一对一 / 一对多(一次读取的Mavlink信息包连续 也可)
                handle_recieve_msg();
			}
				
		}// end 如果有接收到 if (Rn != 0)

		// --------------------------
        //  此处处理需要通过串口发送出去的数据
        //  QQQ
		//  WRITE & Translate message to buffer
        ros::spinOnce();
		loop_rate.sleep();
	}

	//关闭串口s
    SerialPoint.close();

	return 0;
}

void handle_recieve_msg_ats153_type3()
{
    bool success;           // mavlink 信息正确标志
    bool flag_Rn_finish;    // flag 是否处理完当前数组
    size_t cur_rn;          // 当前处理 Rn 数

    // 处理 Rn 帧头 
    // 格式		01 00     2d  00    00 f0 49 00 ac e3 + data
    // 10 byte 包字节数   RSSI 0x00  MAC               + data

    flag_Rn_finish = 1; // flag 是否处理完当前数组
    // Rn 			    // 数组总长
    // last_Rn 		    // 剩余未处理数组[不完整...]
    cur_rn = 0;	        // 当前处理 Rn 数

    while (flag_Rn_finish) {
	    //
	    if ( Rn - cur_rn < 11) 
        {
		    // ERROR 数据长度不对   // 不应该出现的情况 ！
		    ROS_ERROR_STREAM(" ERROR - Hardware-P900 - ATS153=1 - Read_Rn < 10 !!! ");
		    last_Rn = Rn - cur_rn;
		    memcpy(Rbuffer_rx, Rbuffer_rx+cur_rn, last_Rn);
		    flag_Rn_finish = 0;
	    } else 
		{	
            // 解析帧头
			int temp_Rn, temp_RSSI;
			uint8_t temp_mac[6];

			if ( (Rbuffer_rx[3+cur_rn] == 0x00) && 
			     ( (Rbuffer_rx[0+cur_rn] == 0x00) || ( (Rbuffer_rx[0+cur_rn] == 0x01)&&(Rbuffer_rx[1+cur_rn] == 0x00) ) )  )
 			{ 	// 帧头格式符合
				temp_Rn = ((int)Rbuffer_rx[0+cur_rn]) * ((int)0xFF) + ((int)Rbuffer_rx[1+cur_rn]);	//数据包长
				temp_RSSI = (int)Rbuffer_rx[2+cur_rn];	// RSSI

				temp_mac[0] = Rbuffer_rx[9+cur_rn];
				temp_mac[1] = Rbuffer_rx[8+cur_rn];
				temp_mac[2] = Rbuffer_rx[7+cur_rn];
				temp_mac[3] = Rbuffer_rx[6+cur_rn];
				temp_mac[4] = Rbuffer_rx[5+cur_rn];
				temp_mac[5] = Rbuffer_rx[4+cur_rn];

				// 判断来源于哪一个 mac
				if ( mac_data_num == 0 ) {
					mac_data_num = mac_data_num + 1;
					for (int jj=0; jj<6; jj++)
					{ mac_data[jj] = temp_mac[jj];}
					current_chan = mac_data_num;
				} else {
					bool flag_end = 1;	// 判断是否是 已有 mac
					int jj = 0;			// current_chan 计数 jj
					while (flag_end){
						if ( (temp_mac[0] == mac_data[jj*6+0]) && 
					     	 (temp_mac[1] == mac_data[jj*6+1]) && 
						 	 (temp_mac[2] == mac_data[jj*6+2]) && 
						 	 (temp_mac[3] == mac_data[jj*6+3]) && 
						 	 (temp_mac[4] == mac_data[jj*6+4]) && 
						 	 (temp_mac[5] == mac_data[jj*6+5]))
						{	// 之前老的 mac, 记录下 chan
							current_chan = jj+1;
							flag_end = 0;
						}                                                    
						jj = jj+1;
						if ( jj==mac_data_num ) {
							// 还没有找到 来自新的 mac
							for (int jjj=0; jjj<6; jjj++) { mac_data[jjj+mac_data_num*6] = temp_mac[jjj];}
							mac_data_num = mac_data_num + 1;
							current_chan = mac_data_num;
							flag_end = 0;
						}
					}
				}

				// mavlink 数据包 长度判断
				if ( Rn - cur_rn > temp_Rn + 9 ) {
					//   字节解析Message
					for (int i = 10+cur_rn; i < temp_Rn+10+cur_rn; i++) {	
						mavlink_message_t mmsg;		//解包后得到的消息包
						// const mavlink_message_t *mmsg;

						success = read_buffer_chan(mmsg, Rbuffer_rx[i] & 0xff, current_chan);
						if (success) { 
							if (Flag_1ShowRn) {
					            std::cout << "Receive a complete Mavlink message. No = "  << mmsg.msgid << std::endl;	
							}

							//  消息时所有无人机接收 / 消息时本机接收的 / 本机是地面站
							if ( (mmsg.compid == ID_ALL) || (mmsg.compid == my_id) ) // || (my_id == ID_GCS) 
							{	
                            // 此处解析消息类型 并 pub
		                    //  QQQ
                            publish_mavcomm_recieve(mmsg);  //读取并转化为rostopic pub出去
		                    // mavlink_pub_cb(&mmsg);	
							}
						}
					}
					cur_rn = cur_rn + 10 + temp_Rn;
					
					if (cur_rn == Rn)
					{	//本数组搜索完了
						flag_Rn_finish = 0;
						last_Rn = 0;
					}
				} else {
					// Rn 的字符串长度 小于待读取的串口数据长度
					// 不正常 !!!
					ROS_ERROR_STREAM(" ERROR - Hardware-P900 - ATS153=1 - _Package_Rn < Data[00][01] !!! ");
					last_Rn = Rn - cur_rn;
					memcpy(Rbuffer_rx, Rbuffer_rx+cur_rn, last_Rn);
					flag_Rn_finish = 0;
				}

			} else 
			{   // 帧头不正常
				// 直接放弃
                if (Flag_1ShowRn == 2) {
			        std::cout << "ATS153=1 mode, Not in 0x00 0x?? 0x00 Type" << std::endl;
                    std::cout << "Date byte: ";// 打印到屏幕上
			            for (int i = cur_rn; i < cur_rn + 10; i++) {
				            std::cout << std::hex << (Rbuffer_rx[i] & 0xff) << " ";
			            }	
			    }

				flag_Rn_finish = 0;
				last_Rn = 0;
			}

		} // end if ( Rn - cur_rn < 11) {

	} // end while (flag_Rn_finish) {
}


void handle_recieve_msg()
{
    bool success;   //mavlink 信息正确标志

	// 无附加帧头的情况， 直接给 mavlink 解析去
	//   字节解析Message
	for (int i = 0; i < Rn; i++) {	
		mavlink_message_t mmsg;		//解包后得到的消息包
		// const mavlink_message_t *mmsg;
		success = read_buffer(mmsg, Rbuffer_rx[i] & 0xff);
		if (success) { 
			if (Flag_1ShowRn) {
			    std::cout << "Receive a complete Mavlink message. No = "  << mmsg.msgid << std::endl;	
			}

			//  消息时所有无人机接收 / 消息时本机接收的 / 本机是地面站
			if ( (mmsg.compid == ID_ALL) || (mmsg.compid == my_id) ) // || (my_id == ID_GCS) 
			{						
		        // 此处解析消息类型 并 pub
		        //  QQQ
                publish_mavcomm_recieve(mmsg);  //读取并转化为rostopic pub出去
		        // mavlink_pub_cb(&mmsg);	//读取并转化为rostopic pub出去
		    }
		}
	}
							
	if (!success) {	
		//std::cout<<"Not a complete"  << endl;
	}
}



//-----------------------------------------------------------------------------------------
//			串口 Serial —— 开启 & 读取 & 发送
//-----------------------------------------------------------------------------------------

int serial_set_open()	//串口serial设置&开启
{	
	serial::Timeout to = serial::Timeout::simpleTimeout(20);	//创建timeout ms为单位
	SerialPoint.setPort(serial_port);					 		//设置要打开的串口名称	//.c_str()
	SerialPoint.setBaudrate(serial_baudrate);					//设置串口通信的波特率
	SerialPoint.setTimeout(to);								 	//串口设置timeout

	try {   SerialPoint.open();}
	catch (serial::IOException &e) {
		ROS_ERROR_STREAM("Unable to open port : " + serial_port);
		// return -1;
	}
	//判断串口是否打开成功
	if (SerialPoint.isOpen()) {	ROS_INFO_STREAM( serial_port + " is Opeenu.");}
	else {
		ROS_INFO_STREAM( serial_port + " Open Failed!.");
		// return -1;
	}
}

void send_buffer(mavlink_message_t* mmsg)
{
	bool Flag_1Sendtime = false;	// 测试串口发送时间
	uint64_t temp_time1, temp_time2;

	if (Flag_1Sendtime) {	
		temp_time1 = get_time_usec();// 测试串口发送时间
	}
	
	unsigned len;
	len  = mavlink_msg_to_send_buffer((uint8_t*)Wbuffer_tx, mmsg);

	// Write buffer to serial port
	SerialPoint.write(Wbuffer_tx, len);

	// sys_trans_data.send_KB = sys_trans_data.send_KB + ((double)len)/1024;
	
	if (Flag_1ShowRn) {
		std::cout << "Send " << std::dec << (int)len << ", msgid = " << (int)mmsg->msgid << std::endl;
		if (Flag_1ShowRn == 2) {
			std::cout << "Send byte: ";// 打印到屏幕上
			for (int i = 0; i < len; i++) {
				std::cout << std::hex << (Wbuffer_tx[i] & 0xff) << " ";
			}
		}
		std::cout << std::endl;
	}

	if (Flag_1Sendtime) {
		temp_time2 = get_time_usec();// 测试串口发送时间
		std::cout << "Use Time In SerialSend = " << (double)(temp_time2-temp_time1)/1000.0 << " ms" << std::endl;
	}

}

//按字节从串口得到的数据进行校验
int read_buffer(mavlink_message_t &message, uint8_t cp)
{
	mavlink_status_t status;
	uint8_t msgReceived = false;

	//  PARSE MESSAGE
	msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);  //  MAVLINK_COMM_1 处可更改为通道数

	return msgReceived;
}

//	按字节从串口得到的数据进行校验	- 根据mac地址分配
int read_buffer_chan(mavlink_message_t &message, uint8_t cp, uint8_t chan)
{
	mavlink_status_t status;
	uint8_t msgReceived = false;

	//  PARSE MESSAGE
	msgReceived = mavlink_parse_char(chan, cp, &message, &status);  //  MAVLINK_COMM_1 处可更改为通道数

	return msgReceived;
}

//-----------------------------------------------------------------------------------------
//			工具类
//-----------------------------------------------------------------------------------------
uint64_t get_time_usec()	// Time 获取系统时间
{	
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	SerialPoint.close();

	exit(0);// end program here
}



//-----------------------------------------------------------------------------------------
//			pub 
//-----------------------------------------------------------------------------------------
void publish_mavcomm_recieve(mavlink_message_t mmsg)
{
    switch (mmsg.msgid) {

		case MAVLINK_MSG_ID_HEARTBEAT:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_heartbeat_decode(&mmsg, &(mt_pub_heartbeat));
			
			msg_pub_heartbeat.header.stamp = ros::Time::now();
			msg_pub_heartbeat.sysid = mmsg.sysid;
			msg_pub_heartbeat.compid = mmsg.compid;

			msg_pub_heartbeat.battery_cell_voltage = mt_pub_heartbeat.battery_cell_voltage;
			msg_pub_heartbeat.battery_percentage = mt_pub_heartbeat.battery_percentage;
			msg_pub_heartbeat.ctrl_state = mt_pub_heartbeat.ctrl_state;
			msg_pub_heartbeat.px4_mode = mt_pub_heartbeat.px4_mode;
			msg_pub_heartbeat.px4_state = mt_pub_heartbeat.px4_state;
			msg_pub_heartbeat.px4_sys_state = mt_pub_heartbeat.px4_sys_state;

			pub_heartbeat.publish(msg_pub_heartbeat);

			// rmsg_pub_heartbeat.
		break;

		case MAVLINK_MSG_ID_CONSOLE:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_console_decode(&mmsg, &(mt_pub_console));
			
			msg_pub_console.header.stamp = ros::Time::now();
			msg_pub_console.sysid = mmsg.sysid;
			msg_pub_console.compid = mmsg.compid;

			msg_pub_console.command = mt_pub_console.command;
			msg_pub_console.flag = mt_pub_console.flag;
			msg_pub_console.type1 = mt_pub_console.type1;
			msg_pub_console.type2 = mt_pub_console.type2;

			pub_console.publish(msg_pub_console);

			// rmsg_pub_console.
		break;

		case MAVLINK_MSG_ID_CONSOLE_Monitor:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_console_monitor_decode(&mmsg, &(mt_pub_console_monitor));
			
			msg_pub_console_monitor.header.stamp = ros::Time::now();
			msg_pub_console_monitor.sysid = mmsg.sysid;
			msg_pub_console_monitor.compid = mmsg.compid;

			msg_pub_console_monitor.param1 = mt_pub_console_monitor.param1;
			msg_pub_console_monitor.param2 = mt_pub_console_monitor.param2;
			msg_pub_console_monitor.param3 = mt_pub_console_monitor.param3;
			msg_pub_console_monitor.param4 = mt_pub_console_monitor.param4;

			pub_console_monitor.publish(msg_pub_console_monitor);

			// rmsg_pub_console_monitor.
		break;

		case MAVLINK_MSG_ID_CHANGESTATE:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_changestate_decode(&mmsg, &(mt_pub_changestate));
			
			msg_pub_changestate.header.stamp = ros::Time::now();
			msg_pub_changestate.sysid = mmsg.sysid;
			msg_pub_changestate.compid = mmsg.compid;

			msg_pub_changestate.node_state = mt_pub_changestate.node_state;
			msg_pub_changestate.param = mt_pub_changestate.param;

			pub_changestate.publish(msg_pub_changestate);

			// rmsg_pub_changestate.
		break;

		//  loc pos

		case MAVLINK_MSG_ID_LOCAL_POSITION_ENU:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_local_position_enu_decode(&mmsg, &(mt_pub_local_pos_enu));
			
			msg_pub_local_pos_enu.header.stamp = ros::Time::now();
			msg_pub_local_pos_enu.sysid = mmsg.sysid;
			msg_pub_local_pos_enu.compid = mmsg.compid;

			msg_pub_local_pos_enu.x = mt_pub_local_pos_enu.x;
			msg_pub_local_pos_enu.y = mt_pub_local_pos_enu.y;
			msg_pub_local_pos_enu.z = mt_pub_local_pos_enu.z;
			msg_pub_local_pos_enu.yaw = mt_pub_local_pos_enu.yaw;
			msg_pub_local_pos_enu.flag = 0;

			pub_local_pos_enu.publish(msg_pub_local_pos_enu);

			// rmsg_local_pos_enu.
		break;

		case MAVLINK_MSG_ID_SET_LOCAL_POSITION_ENU:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_set_local_position_enu_decode(&mmsg, &(mt_pub_set_local_pos_enu));
			
			msg_pub_set_local_pos_enu.header.stamp = ros::Time::now();
			msg_pub_set_local_pos_enu.sysid = mmsg.sysid;
			msg_pub_set_local_pos_enu.compid = mmsg.compid;

			msg_pub_set_local_pos_enu.x = mt_pub_set_local_pos_enu.x;
			msg_pub_set_local_pos_enu.y = mt_pub_set_local_pos_enu.y;
			msg_pub_set_local_pos_enu.z = mt_pub_set_local_pos_enu.z;
			msg_pub_set_local_pos_enu.yaw = mt_pub_set_local_pos_enu.yaw;
			msg_pub_set_local_pos_enu.flag = mt_pub_set_local_pos_enu.flag;

			pub_set_local_pos_enu.publish(msg_pub_set_local_pos_enu);

			// rmsg_set_local_position_enu.
		break;

		case MAVLINK_MSG_ID_BACK_HOME_POS_ENU:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_back_home_pos_enu_decode(&mmsg, &(mt_pub_back_home_pos_enu));
			
			msg_pub_back_home_pos_enu.header.stamp = ros::Time::now();
			msg_pub_back_home_pos_enu.sysid = mmsg.sysid;
			msg_pub_back_home_pos_enu.compid = mmsg.compid;

			msg_pub_back_home_pos_enu.x = mt_pub_back_home_pos_enu.x;
			msg_pub_back_home_pos_enu.y = mt_pub_back_home_pos_enu.y;
			msg_pub_back_home_pos_enu.z = mt_pub_back_home_pos_enu.z;
			msg_pub_back_home_pos_enu.yaw = mt_pub_back_home_pos_enu.yaw;
			msg_pub_back_home_pos_enu.flag = 0;

			pub_back_home_pos_enu.publish(msg_pub_back_home_pos_enu);

			// rmsg_pub_back_home_pos_enu.
		break;

		case MAVLINK_MSG_ID_SET_HOME_POS_ENU:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_set_home_pos_enu_decode(&mmsg, &(mt_pub_set_home_pos_enu));
			
			msg_pub_set_home_pos_enu.header.stamp = ros::Time::now();
			msg_pub_set_home_pos_enu.sysid = mmsg.sysid;
			msg_pub_set_home_pos_enu.compid = mmsg.compid;

			msg_pub_set_home_pos_enu.x = mt_pub_set_home_pos_enu.x;
			msg_pub_set_home_pos_enu.y = mt_pub_set_home_pos_enu.y;
			msg_pub_set_home_pos_enu.z = mt_pub_set_home_pos_enu.z;
			msg_pub_set_home_pos_enu.yaw = 0;
			msg_pub_set_home_pos_enu.flag = 0;

			pub_set_home_pos_enu.publish(msg_pub_set_home_pos_enu);

			// rmsg_pub_set_home_pos_enu.
		break;

		// adding gps 4
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_global_position_int_decode(&mmsg, &(mt_pub_global_position_int));
			
			msg_pub_global_position_int.header.stamp = ros::Time::now();
			msg_pub_global_position_int.sysid = mmsg.sysid;
			msg_pub_global_position_int.compid = mmsg.compid;

			msg_pub_global_position_int.alt = mt_pub_global_position_int.alt;
			msg_pub_global_position_int.lat = mt_pub_global_position_int.lat;
			msg_pub_global_position_int.lon = mt_pub_global_position_int.lon;
			msg_pub_global_position_int.hdg_yaw = mt_pub_global_position_int.hdg_yaw;
			msg_pub_global_position_int.flag = 0;

			pub_global_position_int.publish(msg_pub_global_position_int);

			// #1002 GLOBAL_POSITION_INT 无 flag
		break;

		case MAVLINK_MSG_ID_SET_GLOBAL_POSITION_INT:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_set_global_position_int_decode(&mmsg, &(mt_pub_set_global_position_int));
			
			msg_pub_set_global_position_int.header.stamp = ros::Time::now();
			msg_pub_set_global_position_int.sysid = mmsg.sysid;
			msg_pub_set_global_position_int.compid = mmsg.compid;

			msg_pub_set_global_position_int.alt = mt_pub_set_global_position_int.alt;
			msg_pub_set_global_position_int.lat = mt_pub_set_global_position_int.lat;
			msg_pub_set_global_position_int.lon = mt_pub_set_global_position_int.lon;
			msg_pub_set_global_position_int.hdg_yaw = mt_pub_set_global_position_int.hdg_yaw;
			msg_pub_set_global_position_int.flag = mt_pub_set_global_position_int.flag;

			pub_set_global_position_int.publish(msg_pub_set_global_position_int);

			// #1102 SET_GLOBAL_POSITION_INT
		break;

		case MAVLINK_MSG_ID_BACK_HOME_POS_GPS_INT:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_back_home_pos_gps_int_decode(&mmsg, &(mt_pub_back_home_gps_pos_int));
			
			msg_pub_back_home_gps_pos_int.header.stamp = ros::Time::now();
			msg_pub_back_home_gps_pos_int.sysid = mmsg.sysid;
			msg_pub_back_home_gps_pos_int.compid = mmsg.compid;

			msg_pub_back_home_gps_pos_int.alt = mt_pub_back_home_gps_pos_int.alt;
			msg_pub_back_home_gps_pos_int.lat = mt_pub_back_home_gps_pos_int.lat;
			msg_pub_back_home_gps_pos_int.lon = mt_pub_back_home_gps_pos_int.lon;
			msg_pub_back_home_gps_pos_int.hdg_yaw = mt_pub_back_home_gps_pos_int.hdg_yaw;
			msg_pub_back_home_gps_pos_int.flag = 0;

			pub_back_home_gps_pos_int.publish(msg_pub_back_home_gps_pos_int);

			// #1113 BACK_HOME_POS_GPS_INT 无 flag
		break;

		case MAVLINK_MSG_ID_SET_HOME_POS_GPS_INT:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_set_home_pos_gps_int_decode(&mmsg, &(mt_pub_set_home_pos_gps_int));
			
			msg_pub_set_home_pos_gps_int.header.stamp = ros::Time::now();
			msg_pub_set_home_pos_gps_int.sysid = mmsg.sysid;
			msg_pub_set_home_pos_gps_int.compid = mmsg.compid;

			msg_pub_set_home_pos_gps_int.alt = mt_pub_set_home_pos_gps_int.alt;
			msg_pub_set_home_pos_gps_int.lat = mt_pub_set_home_pos_gps_int.lat;
			msg_pub_set_home_pos_gps_int.lon = mt_pub_set_home_pos_gps_int.lon;
			msg_pub_set_home_pos_gps_int.hdg_yaw = 0;
			msg_pub_set_home_pos_gps_int.flag = 0;

			pub_set_home_pos_gps_int.publish(msg_pub_set_home_pos_gps_int);

			// #1113 BACK_HOME_POS_GPS_INT 无 flag
		break;

		// get param
		case MAVLINK_MSG_ID_GET_HOME_POSITION:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_get_home_position_decode(&mmsg, &(mt_pub_get_home_position));
			
			msg_pub_get_home_position.header.stamp = ros::Time::now();
			msg_pub_get_home_position.sysid = mmsg.sysid;
			msg_pub_get_home_position.compid = mmsg.compid;

			msg_pub_get_home_position.param = mt_pub_get_home_position.param;

			pub_get_home_position.publish(msg_pub_get_home_position);
		break;

		// kcf_set_target
		case MAVLINK_MSG_ID_kcf_set_target:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_kcf_set_target_decode(&mmsg, &(mt_pub_kcf_set_target));
			
			msg_pub_kcf_set_target.header.stamp = ros::Time::now();
			msg_pub_kcf_set_target.sysid = mmsg.sysid;
			msg_pub_kcf_set_target.compid = mmsg.compid;

			msg_pub_kcf_set_target.state = mt_pub_kcf_set_target.state;
			msg_pub_kcf_set_target.target_no = mt_pub_kcf_set_target.target_no;
			msg_pub_kcf_set_target.x = mt_pub_kcf_set_target.x;
			msg_pub_kcf_set_target.y = mt_pub_kcf_set_target.y;
			msg_pub_kcf_set_target.width = mt_pub_kcf_set_target.width;
			msg_pub_kcf_set_target.height = mt_pub_kcf_set_target.height;

			pub_kcf_set_target.publish(msg_pub_kcf_set_target);
		break;

		// kcf_target_pos
		case MAVLINK_MSG_ID_kcf_target_pos:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_kcf_target_pos_decode(&mmsg, &(mt_pub_kcf_target_pos));
			
			msg_pub_kcf_target_pos.header.stamp = ros::Time::now();
			msg_pub_kcf_target_pos.sysid = mmsg.sysid;
			msg_pub_kcf_target_pos.compid = mmsg.compid;

			msg_pub_kcf_target_pos.state = mt_pub_kcf_target_pos.state;
			msg_pub_kcf_target_pos.target_no = mt_pub_kcf_target_pos.target_no;
			msg_pub_kcf_target_pos.x = mt_pub_kcf_target_pos.x;
			msg_pub_kcf_target_pos.y = mt_pub_kcf_target_pos.y;
			msg_pub_kcf_target_pos.flag = mt_pub_kcf_target_pos.flag;

			pub_kcf_target_pos.publish(msg_pub_kcf_target_pos);
		break;

		// mission_info
		case MAVLINK_MSG_ID_mission_info:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_mission_info_decode(&mmsg, &(mt_pub_mission_info));
			
			msg_pub_mission_info.header.stamp = ros::Time::now();
			msg_pub_mission_info.sysid = mmsg.sysid;
			msg_pub_mission_info.compid = mmsg.compid;

			msg_pub_mission_info.flag = mt_pub_mission_info.flag;
			msg_pub_mission_info.mission_num = mt_pub_mission_info.mission_num;
			msg_pub_mission_info.param1 = mt_pub_mission_info.param1;
			msg_pub_mission_info.param2 = mt_pub_mission_info.param2;

			pub_mission_info.publish(msg_pub_mission_info);
		break;

		// mission_back_info
		case MAVLINK_MSG_ID_mission_back_info:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_mission_back_info_decode(&mmsg, &(mt_pub_mission_back_info));
			
			msg_pub_mission_back_info.header.stamp = ros::Time::now();
			msg_pub_mission_back_info.sysid = mmsg.sysid;
			msg_pub_mission_back_info.compid = mmsg.compid;

			msg_pub_mission_back_info.flag = mt_pub_mission_back_info.flag;
			msg_pub_mission_back_info.mission_num = mt_pub_mission_back_info.mission_num;
			msg_pub_mission_back_info.param1 = mt_pub_mission_back_info.param1;
			msg_pub_mission_back_info.param2 = mt_pub_mission_back_info.param2;

			pub_mission_back_info.publish(msg_pub_mission_back_info);
		break;

		// mission_set
		case MAVLINK_MSG_ID_mission_set:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( " Mavlink Msg " << mmsg.msgid );
			}

			mavlink_msg_mission_set_decode(&mmsg, &(mt_pub_mission_set));
			
			msg_pub_mission_set.header.stamp = ros::Time::now();
			msg_pub_mission_set.sysid = mmsg.sysid;
			msg_pub_mission_set.compid = mmsg.compid;

			msg_pub_mission_set.flag = mt_pub_mission_set.flag;
			msg_pub_mission_set.mission_no = mt_pub_mission_set.mission_no;
			msg_pub_mission_set.mission_task = mt_pub_mission_set.mission_task;
			msg_pub_mission_set.param1 = mt_pub_mission_set.param1;
			msg_pub_mission_set.param2 = mt_pub_mission_set.param2;
			msg_pub_mission_set.param3 = mt_pub_mission_set.param3;
			msg_pub_mission_set.uav_no = mt_pub_mission_set.uav_no;
			msg_pub_mission_set.x = mt_pub_mission_set.x;
			msg_pub_mission_set.y = mt_pub_mission_set.y;
			msg_pub_mission_set.z = mt_pub_mission_set.z;
			msg_pub_mission_set.yaw = mt_pub_mission_set.yaw;

			pub_mission_set.publish(msg_pub_mission_set);
		break;

		default:

			#if use_mavlink_gen_forTest

			#else
				if (Flag_1ShowRn) {
					ROS_INFO_STREAM( "UNKnow Mavlink Msgid " << mmsg.msgid );
					// ROS_ERROR_STREAM( "Unknow mavlink Msgid " << mmsg.msgid );
				}
			#endif
		
		break;
		}

}