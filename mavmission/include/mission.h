
// #pragma once
// #ifndef 

// TODO px4_ctrl 修改 ?

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

// 主要负责任务的 设置 
#include <task_part.h>
// 编队模块
#include <formation_part.h>


namespace mav_mission {

class Mav_Mission
{
public:
	Mav_Mission();
	~Mav_Mission() {};

	void run();

	// 任务设置模块
	Task_part _task_part;
	// 编队飞行模块
	Formation_part _formation_part;
	// mav_mission::Mav_Mission mission;

private:
    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;
	// mavros (pub/sub)
    ros::NodeHandle mavros_nh;
	// desired
	ros::NodeHandle desired_nh;

	// ############################################################
	// 通用部分
	// Init 
	void commom_init();

	// 通用常量
	int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
	int companion_id;	// compid 接收端编号 ->发送时  接收端编号	/ 接收时 接收端编号
	int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

	// to mavros/px4 控制 指令
	ros::Publisher pub_ctrl_set_position;   // 不使用
	ros::Publisher pub_ctrl_set_pose;       // 主要的
	ros::Publisher pub_ctrl_set_vel;        // TODO

	// from mavros/px4
	/* Local position from FCU. ENU坐标系(惯性系) */ 
	void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	geometry_msgs::Pose currentPose;
	ros::Subscriber     currentPose_sub;
	/* Local velocity from FCU. ENU坐标系(惯性系) */ 
	void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	geometry_msgs::TwistStamped currentVelocity; 
	ros::Subscriber     currentVelocity_sub;  

	// 
	ros::Publisher  pub_CurrentMissionState; // 告知其他节点 Mission State
	std_msgs::UInt8 msg_mission_state;

	// ############################################################
	/* mission_part 任务数据解析
	1. 解析 当前的任务模块
		=> 决定 进入何种 函数
	*/
	
	// Parses task information
	// 枚举 mis_array[MAX_NUM_MIS].msg_mission_set.mission_task
	enum ENUM_TASK_PARSES_INFOR {
		INFOR_PARSES_TASK_NAN = 0,
		INFOR_PARSES_TASK_takeoff, 		// 起飞
		INFOR_PARSES_TASK_land, 		// 降落
		INFOR_PARSES_TASK_pos_enu,		// 打点移动 enu (flag 避障)
		INFOR_PARSES_TASK_foramtion,	// 编队飞行
		INFOR_PARSES_TASK_track,		// 目标追踪
	};

	// msg_mission_set.mission_task 解析函数
	void parses_current_mission_task();
	// handle 不同的 switch case
	void mission_task_handle_takeoff();
	void mission_task_handle_land();
	void mission_task_handle_pos_enu();
	void mission_task_handle_foramtion();
	void mission_task_handle_track();

    enum ENUM_STATE_MISSION current_mission_state;
	// 进入下一个任务 信号
	bool flag_signal_next_mission_inorder; 			// 按顺序下一个
	bool flag_signal_next_mission_onlythisdrone; 	// 按顺序本机的下一个
	
	// 获取的任务信息状态
	bool flag_;


	// 任务目标
	geometry_msgs::Point    msg_ctrl_set_position;
	geometry_msgs::Pose     msg_ctrl_set_pose;
	// msg_ctrl_set_pose.position.x/y/z
	// double target_yaw = msg_ctrl_set_pose.orientation.w;    [-pi,pi]
	geometry_msgs::TwistStamped msg_ctrl_set_vel;

  

	void PoseControl();
	void formation_pidVelocityControl();
	void track_pidVelocityControl();

	double delta_pos[3];    // x y z
	double delta_yaw;       // yaw
	double delta_yaw_add;   // * PID_yaw_i


    // mavcomm 初始设置无人机编队 队形
    ros::Subscriber set_local_pos_enu_sub;
    // 回调函数     /mavcomm/receive/loc_pos_enu
    // 话题订阅         | gcs -> uav
    // 编队设置         // TBC flag=>enum
    // *(flag=1) gcs->uav 无人机 ENU航点Pos (mission.cpp) => // TODO 移到 mission.cpp 中 
    void set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);
    mavcomm_msgs::local_pos_enu msg_local_pos_enu;

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
// mission part
//-------------------------------------------------

int flag_mission_set = 0;           // 任务设置状态  // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误

int flag_mission_start = 0;         // 任务执行状态    0-未开始 1-任务运行 2-暂停
int flag_mission_sync = 0;          // 任务同步标志    1-顺序执行 2-同步执行
int flag_mission_pause_task = 0;    // 任务暂停       1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落




};



}	// namespace mav_mission

//-----------------------------
//  QuitSignalHandler
//-----------------------------
// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	exit(0);// end program here
}



