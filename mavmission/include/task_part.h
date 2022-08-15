// 任务设置

// #pragma once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>

#include <mavcomm_msgs/mission_info.h>
#include <mavcomm_msgs/mission_back_info.h>
#include <mavcomm_msgs/mission_set.h>

namespace mav_mission {

class Task_part{

public:
	Task_part();
	~Task_part() {};
private:
    // 数值 初始化
    void task_init();

public:
    // func ros sub
    // 任务初始设置
    void mission_init(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // 任务 输入
    void task_mission_set(const mavcomm_msgs::mission_set::ConstPtr& msg);

    // 对应上面的
    // 

    // 




private:

    ros::NodeHandle tp_nh;

    ros::Subscriber test_sub;

    void test_cb(const std_msgs::Int32::ConstPtr &msg);


	int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

    // 总的任务设置
    int mis_total;                  // 当前执行任务 总数目
    int mis_array_current;          // 当前执行任务 编号

    struct MIS {
        mavcomm_msgs::mission_set msg_mission_set;
        bool flag_set;          // 是否设置了任务

        bool flag_this_uav;     // 是否为本机的任务
        int last_uav_mis_no;    // 上一个 与本机有关的任务 编号
        int next_uav_mis_no;    // 下一个 与本机有关的任务 编号
    } mis_array[255];               // 保存的详细任务

    uint8_t mis_save_param[2];      // 当前执行任务 用于快速校验 参数
    // 注意! 只有在 保存&读取 本地 的任务文件时 使用
    
    // flag 标志位
    bool flag_incomplete_task_num = 0;     // 判断接收&读取的任务是否完整
    int incomplete_task_array_total;
    uint8_t incomplete_task_array[200];


    // Mission 状态机
    // TODO  Mission 状态机   -----------------------------------
    enum MissionState {     // ???
        UNINIT,                   // 未知状态
        IDLE,                     // 什么都不干 地面待机状态
        TAKING_OFF,               // 当前 loc 点起飞
        LANDING,                  // 当前 loc 点降落
        POS_EXECUTION,            // 位置控制
        FORMATION_FLY,            // 编队飞行 
        TRACK,                    // 目标追踪 KCF_tracker
        Avoidance,                // 避障 调用 Fast planner
        WAITING_FOR_HOME_POSE,    // 此和下面的状态 无法通过地面站切换到
        TAKEOFF_FAIL,
        LANDED,
    } mission_state, mission_state_last;  //状态机 当前状态 & 上一个状态

    
};
}
