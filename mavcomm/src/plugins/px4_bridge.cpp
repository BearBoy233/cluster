// Function 订阅 MAVROS话题 转为 RosTopic 消息 发送 
// TODO 发布频率修改？
// px4 /mavros/(enu)local_pos 信息 初始化

#define ID_GCS 100
#define ID_ALL 99 

#include <ros/ros.h>
#include <signal.h>
#include <bitset>
#include <string.h>

// ROS msg
// sub 
#include <mavros_msgs/State.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/VFR_HUD.h>

// pub
// #include <mavcomm_msgs/XXXX.h>
#include <mavcomm_msgs/Heartbeat.h>
#include <mavcomm_msgs/local_pos_enu.h>
#include <mavcomm_msgs/global_pos_int.h>

int Flag_1ShowRn;	// 标志位 1将读取的 Rn 显示在屏幕上
int Flag_1Mavros;	// 标志位 1将读取的 mavros 显示在屏幕上

int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
int companion_id;	// compid 接收端编号	->发送时  接收端编号	/ 接收时 接收端编号
int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

uint64_t get_time_usec();	// Time 获取系统时间
void quit_handler(int sig);	// QuitSignalHandler  Called when you press Ctrl-C

// sub mavros topic
// /mavros/state
ros::Subscriber 			sub_px4_state;
mavros_msgs::State			msg_px4_state; 
// /mavros/battery
ros::Subscriber 			sub_px4_battery;
sensor_msgs::BatteryState	msg_px4_battery; 
// /mavros/gps/
ros::Subscriber 			sub_px4_gps_raw;
mavros_msgs::GPSRAW			msg_px4_gps_raw_state;
uint8_t data_gps_satellites_visible, data_gps_fix_type;
// /mavros/local_position/pose
ros::Subscriber 			sub_px4_loc_pos;
geometry_msgs::PoseStamped	msg_px4_loc_pos;
float  						loc_pos_euler[3];
float 						loc_pos_yaw;
// /mavros/global_position/global
ros::Subscriber 			sub_px4_global_pos;
sensor_msgs::NavSatFix		msg_px4_global_pos;
// /mavros/vfr_hud
ros::Subscriber 			sub_px4_VFR_HUD;
mavros_msgs::VFR_HUD		msg_px4_VFR_HUD;
bool						flag_gps_fix = false;


// 
ros::Subscriber 			sub_uav_state_machine; 
uint8_t 					node_state = 0;

// pub mavcomm topic
// heatrbeat
uint8_t return_px4_mode();	// /mavros/state/#mode 由str转为 Int 
void send_heartbeat();
void send_loc_pos();
void send_global_pos();

void quaternion_2_euler(float quat[4], float angle[3]);


ros::Publisher 			pub_heartbeat;
mavcomm_msgs::Heartbeat msg_heartbeat; 
ros::Publisher 			pub_loc_pos;
mavcomm_msgs::local_pos_enu msg_pub_loc_pos; 
ros::Publisher 			pub_global_pos;
mavcomm_msgs::global_pos_int msg_pub_global_pos; 


// static const char * px4_mode_str[] 
// mavros/state/#mode
static const char * px4_mode_str[] = {"NAN", "MANUAL", "ACRO", "ALTCTL", "POSCTL", 
		"OFFBOARD", "STABILIZED", "RATTITUDE", "AUTO.MISSION", "AUTO.LOITER", 
		"AUTO.RTL", "AUTO.LAND", "AUTO.RTGS", "AUTO.READY", "AUTO.TAKEOFF"};

// mavros/state/#system_state
enum class MAV_STATE: uint8_t {
    UNINIT=0, /* Uninitialized system, state is unknown. | */
    BOOT=1, /* System is booting up. | */
    CALIBRATING=2, /* System is calibrating and not flight-ready. | */
    STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
    ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
    CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
    EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
    POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
    FLIGHT_TERMINATION=8, /* System is terminating itself. | */
};


//-----------------------------------------------------------------------------------------
//			ros_cb() 回调函数
//-----------------------------------------------------------------------------------------

// /mavros/state
void cb_sub_px4_state(const mavros_msgs::State::ConstPtr& rmsg)
{	
	msg_px4_state = *rmsg;
}

// /mavros/battery
void cb_sub_px4_battery(const sensor_msgs::BatteryState::ConstPtr& rmsg)
{	
	msg_px4_battery = *rmsg;
}

// /mavros/gpsstatus/gps1/raw
void cb_sub_px4_gps_raw_state(const mavros_msgs::GPSRAW::ConstPtr& rmsg)
{	
	msg_px4_gps_raw_state = *rmsg;
	data_gps_satellites_visible = msg_px4_gps_raw_state.satellites_visible;
	data_gps_fix_type = msg_px4_gps_raw_state.fix_type;

	if ( flag_gps_fix==false && data_gps_fix_type > 2)
	{	flag_gps_fix = true;}
}

// /mavcomm/uav_state_machine
void cb_sub_uav_state_machine(const std_msgs::UInt8::ConstPtr& rmsg)
{	
	node_state = rmsg->data;
}

// /mavros/local_position/pose
void cb_sub_px4_loc_pos(const geometry_msgs::PoseStamped::ConstPtr& rmsg)
{	
	msg_px4_loc_pos = *rmsg;

	// msg_px4_loc_pos.pose.orientation.w;

	float temp_quad[4];
	temp_quad[0] = msg_px4_loc_pos.pose.orientation.w;
	temp_quad[1] = msg_px4_loc_pos.pose.orientation.x;
	temp_quad[2] = msg_px4_loc_pos.pose.orientation.y;
	temp_quad[3] = msg_px4_loc_pos.pose.orientation.z;
	quaternion_2_euler(temp_quad, loc_pos_euler);
	loc_pos_yaw = loc_pos_euler[2];

}

// /mavros/local_position/pose
void cb_sub_px4_global_pos(const sensor_msgs::NavSatFix::ConstPtr& rmsg)
{	
	msg_px4_global_pos = *rmsg;
	// msg_px4_global_pos.status == sensor_msgs::NavSatFix::
}

// /mavros/vfr_hud
void cb_sub_px4_VFR_HUD(const mavros_msgs::VFR_HUD::ConstPtr& rmsg)
{	
	msg_px4_VFR_HUD = *rmsg;
}

//-----------------------------------------------------------------------------------------
//			Main()
//-----------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "px4_bridge");

	ros::NodeHandle nh("~"); // 用于发布订阅绝对话题 + roslaunch param get
	ros::NodeHandle nh_mavros("mavros"); // 用于发布订阅绝对话题 + roslaunch param get
	ros::NodeHandle nh_mavcomm("mavcomm");

	// sub
	sub_px4_state = nh_mavros.subscribe<mavros_msgs::State>("state", 1, cb_sub_px4_state);
	sub_px4_gps_raw = nh_mavros.subscribe<mavros_msgs::GPSRAW>("gpsstatus/gps1/raw", 1, cb_sub_px4_gps_raw_state);
	sub_px4_battery = nh_mavros.subscribe<sensor_msgs::BatteryState>("battery", 1, cb_sub_px4_battery);

	sub_px4_loc_pos = nh_mavros.subscribe<geometry_msgs::PoseStamped>("local_position/pose", 1, cb_sub_px4_loc_pos);
	sub_px4_global_pos = nh_mavros.subscribe<sensor_msgs::NavSatFix>("global_position/global", 1, cb_sub_px4_global_pos);
	sub_px4_VFR_HUD = nh_mavros.subscribe<mavros_msgs::VFR_HUD>("vfr_hud", 1, cb_sub_px4_VFR_HUD);


	sub_uav_state_machine = nh.subscribe<std_msgs::UInt8>("uav_state_machine", 1, cb_sub_uav_state_machine);
	// pub
	pub_heartbeat = nh_mavcomm.advertise<mavcomm_msgs::Heartbeat>("send/heartbeat", 1);
	pub_loc_pos = nh_mavcomm.advertise<mavcomm_msgs::local_pos_enu>("send/loc_pos_enu", 1);
	pub_global_pos = nh_mavcomm.advertise<mavcomm_msgs::global_pos_int>("send/gps_pos", 1);


	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	nh.param<int>("my_id", my_id, 100);
	nh.param<int>("Flag_1ShowRn", Flag_1ShowRn, 0);  // 1将接收到的字节打印到屏幕上
	nh.param<int>("Flag_1Mavros", Flag_1Mavros, 1);  // 1将接收到的 mavros 输出到屏幕上

	std::cout << "px4_bridge/my_id = " << my_id << std::endl;
	std::cout << "px4_bridge/Flag_1ShowRn = " << Flag_1ShowRn << std::endl;
	std::cout << "px4_bridge/Flag_1Mavros = " << Flag_1Mavros << std::endl;

	data_gps_satellites_visible = 0;
	data_gps_fix_type = 0;

	ros::Rate loop_rate(5);

	int count_n = 0;

	ros::Duration ( 0.5 );

	while (	ros::ok() ) {	
		ros::spinOnce();

		if (count_n == 0)
		{
			send_heartbeat();
		}

		if ( count_n == 1 )
		{
			if (flag_gps_fix)
			{
				send_global_pos();
			}
		}	

		// TODO  广播 本机位置 频率
		send_loc_pos();

		if (count_n == 9) 
		{	count_n = 0;
		}
		else 
		{	count_n++;
		}
		
		loop_rate.sleep();
	}

	return 0;
}

void send_heartbeat()// msg_heartbeat 控制发布频率
{
	// 重置
	msg_heartbeat.px4_state = 0;

	msg_heartbeat.header.stamp = ros::Time::now();
	msg_heartbeat.sysid = my_id;
	msg_heartbeat.compid = ID_GCS; 		//	发送给 地面站

	if (msg_px4_battery.cell_voltage.size() == 0) {
		// 啥也没有
		msg_heartbeat.battery_percentage = -1.0;
		msg_heartbeat.battery_cell_voltage = -1.0;
	} else {
		msg_heartbeat.px4_state |= (1<<3);	//bit 3 battery_on

		msg_heartbeat.battery_percentage = msg_px4_battery.percentage;
		msg_heartbeat.battery_cell_voltage = msg_px4_battery.cell_voltage[0];
	}

	// msg_heartbeat.px4_mode
	// /mavros/state/#mode (srt => int)
	msg_heartbeat.px4_mode = return_px4_mode();

	// px4_sys_state:
    // /mavros/state/#system_state
	msg_heartbeat.px4_sys_state = msg_px4_state.system_status;

	// msg_heartbeat.px4_state
	// /mavros/state[7-4] 
	// [7 connected |6 armed |5 guided |4 manual_input 
	// /mavros/battery/#cell_voltage.size()
	// |3 battery_on 
	// |2 XX |1 XX |0 XX ]
	if (msg_px4_state.connected)  msg_heartbeat.px4_state |= (1<<7);	//bit 7 connected
	if (msg_px4_state.armed)  msg_heartbeat.px4_state |= (1<<6);	//bit 6 connected
	if (msg_px4_state.guided)  msg_heartbeat.px4_state |= (1<<5);	//bit 5 connected
	if (msg_px4_state.manual_input)  msg_heartbeat.px4_state |= (1<<4);	//bit 4 connected
	// bit 3 battery_on 	|in above
	// if (XXXX)  msg_heartbeat.px4_state |= (1<<4);	//bit 2-0 				|To Be Defienu Later
		
	// msg_heartbeat.ctrl_state
	// ctrl_state
	msg_heartbeat.ctrl_state = node_state;

	if (Flag_1Mavros) {
		std::cout << "battery_percentage = " << msg_heartbeat.battery_percentage << std::endl;
		std::cout << "battery_cell_voltage = " << msg_heartbeat.battery_cell_voltage << std::endl;
		std::cout << "px4_mode = 0x" << std::hex << ((int)(msg_heartbeat.px4_mode) & 0xFF) << std::endl;				
		std::cout << "px4_state = 0b" << std::bitset<8>(msg_heartbeat.px4_state) << std::endl;
		std::cout << std::endl;
	}

	msg_heartbeat.gps_satellites_visible = data_gps_satellites_visible;
	msg_heartbeat.gps_fix_type = data_gps_fix_type;

	pub_heartbeat.publish(msg_heartbeat);
}


void send_loc_pos()// msg_loc_pos 控制发布频率
{
	msg_pub_loc_pos.header.stamp = ros::Time::now();
	msg_pub_loc_pos.sysid = my_id;
	msg_pub_loc_pos.compid = ID_ALL; 	//ID_GCS; 		//	发送给 地面站

	msg_pub_loc_pos.flag = 0;	//	#1001 LOCAL_POSITION_ENU 无 flag
	msg_pub_loc_pos.x = (float) msg_px4_loc_pos.pose.position.x;
	msg_pub_loc_pos.y = (float) msg_px4_loc_pos.pose.position.y;
	msg_pub_loc_pos.z = (float) msg_px4_loc_pos.pose.position.z;
	msg_pub_loc_pos.yaw = loc_pos_yaw;

	pub_loc_pos.publish(msg_pub_loc_pos);
}

void send_global_pos()// msg_loc_pos 控制发布频率
{
	msg_pub_global_pos.header.stamp = ros::Time::now();
	msg_pub_global_pos.sysid = my_id;
	msg_pub_global_pos.compid = ID_GCS; 		//	发送给 地面站

	msg_pub_global_pos.flag = 0;	//	#1001 LOCAL_POSITION_ENU 无 flag
	msg_pub_global_pos.alt = msg_px4_global_pos.altitude * 1E7;
	msg_pub_global_pos.lat = msg_px4_global_pos.latitude * 1E7;
	msg_pub_global_pos.lon = msg_px4_global_pos.longitude * 1E7;
	msg_pub_global_pos.hdg_yaw = msg_px4_VFR_HUD.heading;

	pub_global_pos.publish(msg_pub_global_pos);
}

// /mavros/state/#mode 由字符串 转换为 int 
uint8_t return_px4_mode()
{
	bool flag, flag_end;
	uint8_t num_i;
	
	if (msg_px4_state.mode.size()==0) {
		return 0;
	} else{
		flag = true;
		flag_end = true;
		num_i = 1;

		while (flag && flag_end) {
			if (px4_mode_str[num_i] == msg_px4_state.mode) {
				return num_i;
				flag = false;
			} else {
				if (num_i > 16) {
					return 0;
					flag_end = false;
				} else {
					num_i++;
				}
			}
		}
	// return num_i;
	}
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
	exit(0);// end program here
}


/* 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * q0 q1 q2 q3
 * w x y z
 *关于四元数转欧拉角
 * roll = Type(atan2(dcm(2, 1), dcm(2, 2)));
 * pitch = Type(asin(-dcm(2, 0)));
 * yaw = Type(atan2(dcm(1, 0), dcm(0, 0))); */
void quaternion_2_euler(float quat[4], float angle[3])
{
    // angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    // angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    // angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));

	// roll (x-axis rotation)
    double sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3]);
    double cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    angle[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
    if (std::abs(sinp) >= 1)
        angle[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angle[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2]);
    double cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3]);
    angle[2] = atan2(siny_cosp, cosy_cosp);

}
