// 集群任务的管理
  
// 任务的 设置、校验、保存、读取
  
// TODO 任务按组来分 设置 
// uav_no(单机编号0-10 分组编号201-240 地面站100 所有无人机99)

// Future TODO - PDDL (mission.h 中)

// 当前的任务

// #pragma once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>

#include <mavcomm_msgs/mission_info.h>
#include <mavcomm_msgs/mission_back_info.h>
#include <mavcomm_msgs/mission_set.h>

using json = nlohmann::json;

// mission_info_cb (mavcomm_msgs::mission_info)
// mision_info.msg
enum ENUM_INFO_MISSION {
    MISSION_INFO_NAN = 0,
    MISSION_INFO_SET_INIT,
    MISSION_INFO_CHECK,
    MISSION_INFO_LOAD,
    MISSION_INFO_SAVE,
    MISSION_INFO_RUN    // 仅供测试用,正常直接调用
};

// mission_back_info.msg
enum ENUM_STATE_MISSION {
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

    // 以下部分只用于回应
    MISSION_STATE_SAVING,
    MISSION_STATE_SAVED,
    MISSION_STATE_SAVE_FAIL,
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

    // mission part // 当前的任务状态
    enum ENUM_STATE_MISSION current_mission_state; 


    // param 参数读取    
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;
    // storage path
    std::string file_storage_path_head;
    std::string file_storage_path;
    char file_storage_path_cstr[200];
    // strcpy(file_storage_path_cstr, file_storage_path.c_str());


    // nljson
    // int nljson_file_save(std::string path, int num=2); //保存
    int nljson_file_save(char *path, int num=2); //保存
    // int nljson_file_load(std::string path, int num=2); //读取
    int nljson_file_load(char *path, int num, uint8_t param1, uint8_t param2);//读取

    // json
    json test_json_data;
    json temp_json_data;
    void from_json_to_mis_array();
    void from_mis_array_to_json(int num);



    // ------------------------------------------
    // 等待处理
    // TBD
    // TODO 改成 枚举类型

    int flag_mission_start = 0;         // 任务执行状态    0-未开始 1-任务运行 2-暂停
    int flag_mission_sync = 0;          // 任务同步标志    1-顺序执行 2-同步执行
    int flag_mission_pause_task = 0;    // 任务暂停       1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落
    // end
    // ------------------------------------------


public:

    // ------------------------------------------
    // 待添加 函数 mission 获取 本 cpp 程序

    // 接口函数 

    // 总任务数目 
    int get_mis_total()
    {   return mis_total;
    }

    // 获取当前任务状态 mission part (current_mission_state)
    ENUM_STATE_MISSION get_current_mission_state()
    {   return current_mission_state;
    }

    // 获得当前 mis_array_current 编号
    int get_mis_array_current()
    {   return mis_array_current;
    }

    // 获得当前 mis_array_current.last_mis_no 编号
    int get_mis_array_current_last_uav_mis_no()
    {   return mis_array[mis_array_current].last_uav_mis_no;
    }

    // 获得当前 mis_array_current.next_mis_no 编号
    int get_mis_array_current_next_uav_mis_no()
    {   return mis_array[mis_array_current].next_uav_mis_no;
    }

    // 获得 指定的 this_no.last_mis_no 编号
    int get_this_last_uav_mis_no(int this_no)
    {   return mis_array[this_no].last_uav_mis_no;
    }

    // 获得 指定的 this_no.next_mis_no 编号
    int get_this_next_uav_mis_no(int this_no)
    {   return mis_array[this_no].next_uav_mis_no;
    }

    // 获得当前的 任务
    void get_current_mission_task
        (mavcomm_msgs::mission_set *msg_mission, int *last_mis_no, int *next_mis_no);
    
    // 获取 判断 当前任务是否是本机的
    bool check_current_mission_this_drone()
    {   return mis_array[mis_array_current].flag_this_uav;      
    }

    // 进入下一个任务 
    void set_mission_task_next(int set_next_no)
    {   mis_array_current = set_next_no;
    }

    // end
    // ------------------------------------------




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
