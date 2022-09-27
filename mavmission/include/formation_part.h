// 编队飞行 模块
 
// 编队 队形设置、编队飞行

// #pragma once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavcomm_msgs/local_pos_enu.h>


namespace mav_mission {

class Formation_part{

public:
	Formation_part();
	~Formation_part() {};

private:
    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;

    // param 参数读取 
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;

    // 数值 初始化
    void task_init();

    // 编队飞行模块
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


    // 分布式 编队控制算法
    // void formation_pidVelocityControl();

    // 计算得到的最终控制
    double delta_pos[3];
    double delta_yaw;

    // TBM
    geometry_msgs::Pose Mission_pose_current;

public:
//-------------------------------------------------
//                     编队邻居位置-ot_loc_pos_enu_cb
//-------------------------------------------------
    // 订阅其他无人机的位置 (已经减掉 队形偏差量了)
    ros::Subscriber ot_loc_pos_enu_sub;
    // 话题订阅     | neibor uav -> uav
    // 编队控制     |接收 其他无人机的位置信息 编队飞行
    void ot_loc_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);
    mavcomm_msgs::local_pos_enu msg_ot_local_pos_enu;

//-------------------------------------------------
// Func        无人机编队阵型位置-set_local_pos_enu_cb
//-------------------------------------------------
    // mavcomm 初始设置无人机编队 队形
    ros::Subscriber set_local_pos_enu_sub;
    // 回调函数     /mavcomm/receive/loc_pos_enu
    // 话题订阅         | gcs -> uav
    // 编队设置         // TBC flag=>enum
    //  (flag=1) gcs->uav 无人机 ENU航点Pos (mission.cpp) => // TODO 移到 mission.cpp 中 
    // *(flag=2) gcs->uav 编队误差设置      (formation.cpp)
    //  (flag=3) uav->gcs 编队误差反馈      (formation.cpp)
    void set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);
    mavcomm_msgs::local_pos_enu msg_local_pos_enu;
    

    // 无人机队形 设置回应
    ros::Publisher set_local_pos_enu_pub;         // 告知地面站 无人机编队误差设置


};
}
