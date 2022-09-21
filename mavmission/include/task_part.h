// 集群任务的管理

// 任务的 设置、校验、保存、读取
  
// #pragma once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>

#include <mavcomm_msgs/mission_info.h>
#include <mavcomm_msgs/mission_back_info.h>
#include <mavcomm_msgs/mission_set.h>

// TBC
// mision_info.msg
enum MISSION_INFO {
    MISSION_INFO_NAN = 0,
    MISSION_INFO_SET_INIT,
    MISSION_INFO_CHECK,
    MISSION_INFO_LOAD,
    MISSION_INFO_SAVE,
    MISSION_INFO_DEL,
};

// TBC
// mission_back_info.msg
enum STATE_MISSION {
    MISSION_STATE_NAN = 0,

    MISSION_STATE_SETTING,

    MISSION_STATE_LOADING,
    MISSION_STATE_LOADED,
    MISSION_STATE_LOAD_FAIL,

    MISSION_STATE_CHECKING,
    MISSION_STATE_CHECKED,
    
    MISSION_STATE_CHECK_FAIL_LEN,
    MISSION_STATE_CHECK_FAIL_CRC,
    MISSION_STATE_CHECK_FAIL_INCOMPLETE,

    MISSION_STATE_SAVING,
    MISSION_STATE_SAVED,
    MISSION_STATE_SAVE_FAIL,

    MISSION_STATE_DELING,
    MISSION_STATE_DELED,
    MISSION_STATE_DEL_FAIL,

    MISSION_STATE_EXECUTING,
    MISSION_STATE_PAUSED,
    MISSION_STATE_FINISHED,
};

namespace mav_mission {

class Task_part{

public:
	Task_part();
	~Task_part() {};

private:
    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;

    // mission part
    enum STATE_MISSION current_mission_state; // 当前的任务状态

    // TBD
    // TODO 改成 枚举类型
    int flag_mission_set = 0;           // 任务设置状态  // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误

    int flag_mission_start = 0;         // 任务执行状态    0-未开始 1-任务运行 2-暂停
    int flag_mission_sync = 0;          // 任务同步标志    1-顺序执行 2-同步执行
    int flag_mission_pause_task = 0;    // 任务暂停       1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落

    // param 参数读取 
		
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;

    std::string file_storage_path_head;
    std::string file_storage_path;

    // pa

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
    } mis_array[MAX_NUM_MIS];   // 保存的详细任务

    // 注意! 只有在 保存&读取 本地的任务文件时 使用，减少数据的传输
    uint8_t mis_save_param[2];      // 当前执行任务 用于快速校验 参数

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
    
    // gcs -> uav   |任务初始化   |需要回应 
    // mavcomm_msgs::mission_info   .flag==MISSION_INFO_SET_INIT
    void mission_settings_init(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // gcs -> uav   |任务设置完成进行 校验  |需要回应 
    // mavcomm_msgs::mission_info   .flag=MISSION_INFO_CHECK 
    void mission_setting_check(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // gcs -> uav   |读取本地任务   |需要回应 
    // mavcomm_msgs::mission_info   .flag=MISSION_INFO_LOAD 
    void mission_setting_load(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // gcs -> uav   |读取本地任务   |需要回应 
    // mavcomm_msgs::mission_info   .flag=MISSION_INFO_SAVE 
    void mission_setting_save(const mavcomm_msgs::mission_info::ConstPtr& msg);

    // gcs -> uav   |读取本地任务   |需要回应 
    // mavcomm_msgs::mission_info   .flag=MISSION_INFO_DEL 
    void mission_setting_del(const mavcomm_msgs::mission_info::ConstPtr& msg);


//-------------------------------------------------
//                     任务设置反馈-mission_back_info
//-------------------------------------------------
    // pub      |/mavcomm/send/mission_back_info
    ros::Publisher mission_back_info_pub;
    mavcomm_msgs::mission_back_info msg_mission_back_info;

    // uav -> gcs   任务设置 回应  [由 flag 决定返回信息]
    void mission_F_settings_back_info(int flag_state, int param1 = 0);

    // uav -> gcs   任务设置 回应 [常规-msg]
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
 
   
};
}
