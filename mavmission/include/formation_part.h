// 编队飞行 模块
  
// 编队 队形设置、编队飞行

// #pragma once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavcomm_msgs/local_pos_enu.h>
#include <mavcomm_msgs/formation_set.h>
#include <mavcomm_msgs/formation_set_info.h>
#include <mavcomm_msgs/formation_back_info.h>

// mission_back_info.msg
enum FORMATION_MISSION {
    FORMATION_STATE_NAN = 0,

    FORMATION_STATE_SETTING,

    FORMATION_STATE_CHECKING,
    FORMATION_STATE_CHECKED,
    FORMATION_STATE_CHECK_FAIL
};

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

    // formation part
    enum FORMATION_MISSION current_formation_state; // 当前的编队模块状态

    // param 参数读取 
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;

    // 数值 初始化
    void formation_init();

    // 编队阵型设置
    int current_group;  // 当前使用的编队阵型
    int 

    // 可分组编队
    struct FORM {
        // 设置的原始编队信息
        mavcomm_msgs::formation_set msg_formation_set;
        
        // 无人机N的 编队偏置量
        double offset_x;
        double offset_y;
        double offset_z;
        double offset_yaw;

        bool flag_this_group;   // 是否是本组的

    } formation_array[NNN][FORMATION_GROUP_PPP];   // 保存编队的队形信息

    struct FORM_loc_pos_enu {

        // 无人机N 编队偏置量
        double x;
        double y;
        double z;
        double yaw;

        bool flag_update;   // 是否更新
        uint64_t last_msg_rec_time;
        uint64_t cur_msg_rec_time;

    } neighbor_loc_pos_ENU[NNN];   // 邻居无人机 位置信息

    int flag_ot_num[NNN];       // ==1 已知的邻居无人机位置 ； 
    //TODO 参与编队的其他无人机 Num;
    
    int ot_num_sum;         // 编组内无人机个数
    int ot_this_num;        // 本机编组 TODO
    // 本机的编队偏差 实际发送 Loc 时, 偏差需要减掉
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
