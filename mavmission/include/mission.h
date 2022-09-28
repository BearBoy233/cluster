
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
// 编队模块
#include <formation_part.h>


namespace mav_mission {

class Mav_Mission
{
public:
	Mav_Mission();
	~Mav_Mission() {};

	void run();

	Task_part _task_part;
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


    // mavcomm 初始设置无人机编队 队形
    ros::Subscriber set_local_pos_enu_sub;
    // 回调函数     /mavcomm/receive/loc_pos_enu
    // 话题订阅         | gcs -> uav
    // 编队设置         // TBC flag=>enum
    // *(flag=1) gcs->uav 无人机 ENU航点Pos (mission.cpp) => // TODO 移到 mission.cpp 中 
    //  (flag=2) gcs->uav 编队误差设置      (formation.cpp)
    //  (flag=3) uav->gcs 编队误差反馈      (formation.cpp)
    void set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);
    mavcomm_msgs::local_pos_enu msg_local_pos_enu;




	// 通用部分
	// init 
	void commom_init();

	int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
	int companion_id;	// compid 接收端编号 ->发送时  接收端编号	/ 接收时 接收端编号
	int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

	// 任务设置 部分 mission

	// px4_uav mission
	// 无人机 任务模块
 
	// px4 控制
	ros::Publisher pub_ctrl_set_position;   // 不使用
	ros::Publisher pub_ctrl_set_pose;       // 主要的
	ros::Publisher pub_ctrl_set_vel;        // TODO

	// 任务目标
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



