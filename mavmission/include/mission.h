
// #pragma once
// #ifndef 

#include <com_define.h>

#include <signal.h>
// #include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <mavcomm_msgs/ChangeState.h>
#include <mavcomm_msgs/local_pos_enu.h>

#include <mav_mission/PositionCommand.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// 主要负责任务的 设置 
#include <task_part.h>

namespace mav_mission {

/*
// help func
uint64_t get_time_usec();
void quit_handler(int sig);
float satfunc(float data, float Max);
*/

class Mav_Mission
{
public:
	Mav_Mission();
	~Mav_Mission() {};

	void run();

	// help func

	Task_part _task_part;
	// mav_mission::Mav_Mission mission;

private:

// TBD
	ros::NodeHandle nh;			// 用于发布订阅绝对话题 + roslaunch param get
	ros::NodeHandle mavlink_nh;	// TBD

// 通用部分

	void commom_init();

	int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
	int companion_id;	// compid 接收端编号 ->发送时  接收端编号	/ 接收时 接收端编号
	int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

// 任务设置 部分 mission

// px4_uav mission
// 无人机 任务模块
 
ros::Publisher pub_ctrl_set_position;   // 不使用
ros::Publisher pub_ctrl_set_pose;       // 主要的
ros::Publisher pub_ctrl_set_vel;        // TODO

geometry_msgs::Point    msg_ctrl_set_position;
geometry_msgs::Pose     msg_ctrl_set_pose;
// msg_ctrl_set_pose.position.x/y/z
// double target_yaw = msg_ctrl_set_pose.orientation.w;    [-pi,pi]
geometry_msgs::TwistStamped msg_ctrl_set_vel;

ros::Publisher  pub_CurrentMissionState; // 告知 Mission State
std_msgs::UInt8 msg_mission_state;

void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
geometry_msgs::Pose currentPose;
ros::Subscriber     currentPose_sub;

void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
geometry_msgs::TwistStamped currentVelocity; 
ros::Subscriber     currentVelocity_sub;    

void PoseControl();
void formation_pidVelocityControl();
void track_pidVelocityControl();

double delta_pos[3];    // x y z
double delta_yaw;       // yaw
double delta_yaw_add;   // * PID_yaw_i

// Mission 
// targetPose
geometry_msgs::Pose Mission_pose_current;   // 当前目标位置

	bool gcs_quiet_mode;
	ros::Time last_message_received_from_gcs;
	ros::Duration conn_timeout;

	// ros::Publisher mavlink_pub;
	// ros::Subscriber mavlink_sub;

    /*
	//! fcu link -> ros
	void mavlink_pub_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);
	//! ros -> fcu link
	void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg);

	//! message router
	void plugin_route_cb(const mavlink::mavlink_message_t *mmsg, const mavconn::Framing framing);
    */

// 任务设置 部分 mission


//-------------------------------------------------
// formation part
//-------------------------------------------------

void formation_init();

void ot_loc_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);

void set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);

//################################################################------编队飞行模块
// 订阅其他无人机的位置 (已经减掉 队形偏差量了)
ros::Subscriber ot_loc_pos_enu_sub;
mavcomm_msgs::local_pos_enu msg_ot_local_pos_enu;
double ot_pos_x[NNN];
double ot_pos_y[NNN];
double ot_pos_z[NNN];
double ot_pos_yaw[NNN];
// TODO 待拓展 当前只能编一组队
int flag_ot_num[NNN];       // ==1 已知的邻居无人机位置 ； /TODO 参与编队的其他无人机 Num;
int ot_num_sum;         // 编组内无人机个数
int ot_this_num;        // 本机编组 TODO
// 本机的编队偏差 实际发送 Loc 时, 偏差需要减掉 (px4_bridge 中完成)
// 当前仅考虑 室内定位系统(ENU) 的情况 (TODO GPS/ VINS...)
// 本机 编队偏差
float ot_offset_x = 0.0;
float ot_offset_y = 0.0;
float ot_offset_z = 0.0;      // 暂时不用
float ot_offset_yaw = 0.0;    // 暂时不用

// mavcomm 初始设置无人机编队 队形
ros::Subscriber set_local_pos_enu_sub;
mavcomm_msgs::local_pos_enu msg_local_pos_enu;

// 无人机队形 设置回应
ros::Publisher set_local_pos_enu_pub;         // 告知地面站 无人机编队误差设置




//-------------------------------------------------
// mission part
//-------------------------------------------------

// mission part
void mission_init();

void mission_set_cb(const mavcomm_msgs::mission_set::ConstPtr& msg);

void mission_info_cb(const mavcomm_msgs::mission_info::ConstPtr& msg);



ros::Publisher mission_back_info_pub;
ros::Subscriber mission_info_sub;
ros::Subscriber mission_set_sub;

int flag_mission_set = 0;           // 任务设置状态  // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误

int flag_mission_start = 0;         // 任务执行状态    0-未开始 1-任务运行 2-暂停
int flag_mission_sync = 0;          // 任务同步标志    1-顺序执行 2-同步执行
int flag_mission_pause_task = 0;    // 任务暂停       1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落

mavcomm_msgs::mission_info msg_mission_info;
mavcomm_msgs::mission_back_info msg_mission_back_info;
mavcomm_msgs::mission_set msg_mission_set;




// ----无人机避障模块 (fast_planner)
// fast
void fast_init();
ros::Subscriber fast_sub;
void fast_sub_cb(const mav_mission::PositionCommand::ConstPtr &msg);



//-------------------------------------------------
// tracker part
//-------------------------------------------------
public:

	void tracker_init();
	// void tracker_load_param();

private:
	// ROS get param
	double init_takeoff_z;  //初始起飞高度

	// 幅值限制
	float maxVelocity_x;
	float maxVelocity_y;
	float maxVelocity_z;
	float maxVelocity_yaw;

	// PID参数
	struct pid {
    	double p = 0.0;
    	double i = 0.0;
    	double d = 0.0;
	} pid_x, pid_y, pid_z, pid_yaw;


};



}	// namespace mav_mission




///-----------------------------------------------------------------------------------------
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

float satfunc(float data, float Max)
{
    if( abs(data)>Max )
        return (data>0)?Max:-Max;
    else
        return data;
}