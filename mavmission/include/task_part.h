// 集群任务的管理

// 任务的 设置、校验、保存、读取
 
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
    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;

    // param 参数读取 
		
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;

private:
    // 数值 初始化
    void task_init();

    // 总的任务设置
    int mis_total;                  // 当前执行任务 总数目
    int mis_array_current;          // 当前执行任务 编号

    struct MIS {
        mavcomm_msgs::mission_set msg_mission_set;
        bool flag_set;          // 是否设置了任务

        bool flag_this_uav;     // 是否为本机的任务
        int last_uav_mis_no;    // 上一个 与本机有关的任务 编号
        int next_uav_mis_no;    // 下一个 与本机有关的任务 编号
    } mis_array[MAX_NUM_MIS];               // 保存的详细任务

    // 注意! 只有在 保存&读取 本地的任务文件时 使用，减少数据的传输
    uint8_t mis_save_param[2];      // 当前执行任务 用于快速校验 参数

    
    // flag 标志位
    bool flag_incomplete_task_num = 0;     // 判断接收&读取的任务是否完整
    int incomplete_task_array_total;
    uint8_t incomplete_task_array[MAX_NUM_MIS];

    // mission part
    // TODO 改成 枚举类型
    int flag_mission_set = 0;           // 任务设置状态  // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误

    int flag_mission_start = 0;         // 任务执行状态    0-未开始 1-任务运行 2-暂停
    int flag_mission_sync = 0;          // 任务同步标志    1-顺序执行 2-同步执行
    int flag_mission_pause_task = 0;    // 任务暂停       1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落

    // temp_data
    mavcomm_msgs::mission_info msg_temp_mission_info;
    mavcomm_msgs::mission_back_info msg_temp_mission_back_info;


public:
//-------------------------------------------------
//                          任务设置-mission_info_cb
//-------------------------------------------------
    // sub      |/mavcomm/receive/mission_info
    ros::Subscriber mission_info_sub;
    void mission_info_cb(const mavcomm_msgs::mission_info::ConstPtr &msg);
    mavcomm_msgs::mission_info msg_mission_info;
    
    // gcs -> uav   flag==1    |任务初始化   |需要回应 
    // mavcomm_msgs::mission_info   .flag=1 
    void mission_settings_init(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // gcs -> uav   flag==2    |任务设置完成进行 校验  |需要回应 
    // mavcomm_msgs::mission_info   .flag=2 
    void mission_setting_check();

//-------------------------------------------------
//                     任务设置反馈-mission_back_info
//-------------------------------------------------
    // pub      |/mavcomm/send/mission_back_info
    ros::Publisher mission_back_info_pub;
    mavcomm_msgs::mission_back_info msg_mission_back_info;

    // uav -> gcs   任务设置 回应
    void mission_settings_back_info(mavcomm_msgs::mission_back_info msg);


//-------------------------------------------------
//                        任务设置 每一条-mission_set
//-------------------------------------------------
    // sub      |/mavcomm/receive/mission_set
    ros::Subscriber mission_set_sub;
    void mission_set_cb(const mavcomm_msgs::mission_set::ConstPtr &msg);
    mavcomm_msgs::mission_set msg_mission_set;
    // gcs -> uav   任务设置 每一条
    // 任务 输入
    void task_mission_set(const mavcomm_msgs::mission_set::ConstPtr& msg);
 

private:


    // TBC
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
