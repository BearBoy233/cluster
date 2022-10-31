// 编队飞行 模块
  
// 编队 队形设置、编队飞行

// TODO yaw 保持为飞机当亲飞行方向

// #program once
// #ifndef 

#include <com_define.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <mavcomm_msgs/local_pos_enu.h>

#include <mavcomm_msgs/formation_info.h>
#include <mavcomm_msgs/formation_back_info.h>
#include <mavcomm_msgs/formation_set.h>

// 编队模式 切换 状态 指令
enum ENUM_INFO_FORMATION {
    FORMATION_INFO_NAN = 0,
    FORMATION_INFO_SET_INIT,
    FORMATION_INFO_CHECK,
    FORMATION_INFO_RUN      // 仅供测试用,正常直接调用
};

// 编队模式 当前此模块的状态
enum ENUM_STATE_FORMATION {
    FORMATION_STATE_NAN = 0,
    // 设置
    FORMATION_STATE_SETTING,
    // 校准
    FORMATION_STATE_CHECKING,
    FORMATION_STATE_CHECKED,
    FORMATION_STATE_CHECK_FAIL,
    // 编队执行
    FORMATION_STATE_RUN_PREPARE,
    FORMATION_STATE_RUN_FORMING,
    FORMATION_STATE_RUNNING,
    FORMATION_STATE_RUN_FAIL
};

// 编队模式 formation_ctrl_all_follower 返回的控制状态
enum ENUM_STATE_CTRL_FORMATION {
    FORMATION_CTRL_STATE_NAN = 0,
    // 
    FORMATION_CTRL_STATE_formation_prepare,
    FORMATION_CTRL_STATE_formation_forming,
    // 进入 编队 模式失败
    FORMATION_CTRL_STATE_unable_enter_formation,

    // 实际控制算法
    // 分布式 一阶编队控制算法
    FORMATION_CTRL_STATE_follower_distributed_1st_order,

};

namespace mav_mission {

class Formation_part{

public:
	Formation_part();
	~Formation_part() {};

    // 获取 某一状态值 
    // current_formation_state
    enum ENUM_STATE_FORMATION get_current_formation_state()
    {    return current_formation_state; 
    };

private:
    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;
    // mavros (pub/sub)
    ros::NodeHandle mavros_nh;

    // param 参数读取 
    // my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]
    int my_id;

    // 数值 初始化
    void formation_init();

    // 编队阵型设置
    // formation part
    enum ENUM_STATE_FORMATION current_formation_state; // 当前的编队模块状态
    
    int current_group;  // TBU 当前使用的编队阵型 
    int last_group;     // 上一个 编队阵型 

    // 获得当前编队模式的 flag
    bool t_flag_form_direct;    // flag l0
    bool t_flag_form_in_turn;   // flag l1

    // 编队阵型信息存储
    // 支持分组编队
    struct FORM {
        // 设置的原始编队信息
        // mavcomm_msgs::formation_set msg_formation_set;
        
        // 无人机N的 编队偏置量
        double offset_x;
        double offset_y;
        double offset_z;    // 暂时只考虑 [z相同]    
        double offset_yaw;  // 

        // 编队 标志位
        // 同组中的 flag是一致的 (除了 leader)
        uint8_t flag;   
        // TBE
        // h7-l0
        // 7 - leader_flag  置0 丛机
        //                  置1 领机 [一个编队 当前有且仅能有 1个leader]
        // 6 - yaw_leader   置0 yaw偏置相对ENU惯性系 
        //                  置1 yaw偏置相对leader   

        // 1 - form_in_turn 置0 直接按offset形成编队 [也不怎么推荐-无避障]
        //                  置1 一架架，依次形成编队
        // 0 - form_direct  置0 直接编队算法 [不推荐-无避障]
        //                  置1 先形成编队，之再编队算法 [看 flag 1]

        bool flag_this_group;   // 是否是本组的 - check 里赋值
        bool flag_set;          // 是否已经设置了 - init 里=0
        bool flag_leader;       // leader 标志位

    } formation_array[NNN][FORMATION_GROUP_PPP];   // 保存编队的队形信息

    // Init_formation_group_setting
    int type_formtation_offset_group[FORMATION_GROUP_PPP];  // TBD 编队 组队类型？ TODO 
    int num_formtation_offset_group[FORMATION_GROUP_PPP];   // 编组内无人机个数
    // check_formation_group_setting
    bool flag_formtation_offset_group_set[FORMATION_GROUP_PPP]; //标志位 设置完全 有更改则设置为0
    int formation_set_check_current_num;    // 当前校验的无人机编队设置的 group_id FORMATION_GROUP_PPP

    // 邻居无人机位置
    struct FORM_loc_pos_enu {
        // 无人机 N ENU 位置 
        double x;
        double y;
        double z;
        double yaw;

        bool flag_update;           // 是否更新 (和当前调用的时候比较？)
        uint64_t last_msg_rec_time;
        uint64_t cur_msg_rec_time;
    } neighbor_loc_pos_ENU[NNN];    // 邻居无人机 位置信息

    int flag_nb_connect_first[NNN]; // 首次与邻居无人机连接 (之后断开不变)
    //TODO 参与编队的其他无人机 Num;
    
    // PID参数
    struct pid {
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;

        double err = 0.0;
        double err_last = 0.0;
        double err_add = 0.0;
    };

    // TBC 总控制 （保证正常切换到编队模式 & 选择控制算法）
    // keep_z & keep_h 不变
    int formation_ctrl_all_follower(int switch_group_id, float keep_z, float keep_h);
    
    // 分布式 编队控制算法 type1
    // leader 自己飞
    // 从机 一阶 分布式编队控制算法 xy
    void formation_ctrl_first_order_PID_xy
        (float keep_d_z, float keep_d_yaw, geometry_msgs::TwistStamped *cal_ctrl_set_vel);

    // TODO load !!!
    //幅值限制
    float maxVelocity_1storder_x;
    float maxVelocity_1storder_y;
    float maxVelocity_1storder_z;
    float maxVelocity_1storder_yaw;
    pid pid_1storder_x;
    pid pid_1storder_y;
    pid pid_1storder_z;
    pid pid_1storder_yaw;
    
    // 计算得到的最终控制
    double delta_enu_x;
    double delta_enu_y;
    double delta_enu_z;
    double delta_enu_yaw;
    double delta_enu_yaw_add;

public:

//-------------------------------------------------
//                     编队邻居位置-ot_loc_pos_enu_cb
//-------------------------------------------------
    // 订阅其他无人机的位置 (需要 减掉 编队队形偏差量)
    ros::Subscriber ot_loc_pos_enu_sub;
    // 话题订阅     | neibor uav -> uav
    // 编队控制     |接收 其他无人机的位置信息 编队飞行
    void ot_loc_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);
    mavcomm_msgs::local_pos_enu msg_ot_loc_pos_enu;

//-------------------------------------------------
//                    无人机编队阵型位置-formation_set
//-------------------------------------------------
    // mavcomm 设置无人机编队 队形
    ros::Subscriber formation_set_sub;
    // 话题订阅     | neibor uav -> uav
    // 编队控制     |接收 其他无人机的位置信息 编队飞行
    void formation_set_cb(const mavcomm_msgs::formation_set::ConstPtr &msg);
    mavcomm_msgs::formation_set msg_formation_set;

//-------------------------------------------------
//                           编队设置-formation_info
//-------------------------------------------------
    // mavcomm 设置无人机编队 队形
    ros::Subscriber formation_info_sub;
    // 话题订阅     | neibor uav -> uav
    // 编队控制     |接收 其他无人机的位置信息 编队飞行
    void formation_info_cb(const mavcomm_msgs::formation_info::ConstPtr &msg);
    mavcomm_msgs::formation_info msg_formation_info;

    // 某组编队 设置 初始化
    void Init_formation_group_setting
        (int group_id, int group_num, int group_type, uint8_t receive_id);

    // 某组编队 设置 校准
    void check_formation_group_setting
        (int group_id, int setbit_mm, uint8_t setbit_1, uint8_t setbit_2, uint8_t receive_id);

//-------------------------------------------------
//                   编队设置反馈-formation_back_info
//-------------------------------------------------
    // 无人机队形 设置回应
    ros::Publisher formation_back_info_pub;         // 告知地面站 无人机编队误差设置
    mavcomm_msgs::formation_back_info msg_formation_back_info;
    
    // uav -> gcs/uav-leader   编队设置 回应  [常规-msg]
    void pubfunc_formation_back_info(mavcomm_msgs::formation_back_info msg);

    // uav -> gcs/uav-leader   任务设置 回应  [由 flag 决定返回信息]
    void pubfunc_F_formation_back_info
        (int flag_state, int id_group, uint8_t receive_id, 
        int param1=0, int param2=0, int param3=0);

//-------------------------------------------------
//                               px4 mavros 消息订阅 
//-------------------------------------------------
    // px4 mavros 消息订阅 enu 位置 mavros/local_position/pose
    ros::Subscriber this_uav_px4_local_position_pose_sub;
    void this_uav_px4_local_position_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    geometry_msgs::PoseStamped currentPose;
    // geometry_msgs::PoseStamped msg_PoseStamped;

    // px4 mavros 消息订阅 vel 速度 mavros/local_position/velocity_local
    ros::Subscriber this_uav_px4_local_position_velocity_local_sub;
    void this_uav_px4_local_position_velocity_local_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
    geometry_msgs::TwistStamped currentVelocity;

    // px4 mavros 消息订阅 vel 速度 mavros/local_position/velocity_local
    ros::Publisher this_uav_px4_setVelocity_pub;    // 设置速度


};
}
