
#include <formation_part.h>
 
using namespace mav_mission;

//-------------------------------------------------
// Init 初始化
//-------------------------------------------------
Formation_part::Formation_part():
    tp_nh("~"),             // param    /uav_mission/xxx
    mavcomm_nh("mavcomm"),   // mavcomm  /mavcomm/XXX   pub&sub
    mavros_nh("mavros")
{
    // 数值 初始化
    formation_init();
    
    // 节点模式
    current_formation_state = FORMATION_STATE_NAN;

    // load param
    tp_nh.param<int>("my_id", my_id, 100);
    std::cout << "formation uav_mission/my_id = " << my_id << std::endl;
    // load pid_1storder_x

    tp_nh.param<double>("formation/pid_1storder/Kp_x", pid_1storder_x.p, 1.0);
    tp_nh.param<double>("formation/pid_1storder/Ki_x", pid_1storder_x.i, 0.0);
    tp_nh.param<double>("formation/pid_1storder/Kd_x", pid_1storder_x.d, 0.5);
    
    tp_nh.param<double>("formation/pid_1storder/Kp_y", pid_1storder_y.p, 1.0);
    tp_nh.param<double>("formation/pid_1storder/Ki_y", pid_1storder_y.i, 0.0);
    tp_nh.param<double>("formation/pid_1storder/Kd_y", pid_1storder_y.d, 0.5);

    tp_nh.param<double>("formation/pid_1storder/Kp_z", pid_1storder_z.p, 1.0);
    tp_nh.param<double>("formation/pid_1storder/Ki_z", pid_1storder_z.i, 0.0);
    tp_nh.param<double>("formation/pid_1storder/Kd_z", pid_1storder_z.d, 0.5);

    tp_nh.param<float>("formation/pid_1storder/maxVelocity_x", maxVelocity_1storder_x, 0.6);
    tp_nh.param<float>("formation/pid_1storder/maxVelocity_y", maxVelocity_1storder_y, 0.6);
    tp_nh.param<float>("formation/pid_1storder/maxVelocity_z", maxVelocity_1storder_z, 0.6);
    tp_nh.param<float>("formation/pid_1storder/maxVelocity_yaw", maxVelocity_1storder_yaw, 1.0);

    // std::cout << "pid_1storder_x.d = " << pid_1storder_x.d << std::endl;




    // 话题订阅     | neibor uav -> uav
    // 编队控制     |接收 其他无人机的位置信息 编队飞行
    ot_loc_pos_enu_sub = mavcomm_nh.subscribe<mavcomm_msgs::local_pos_enu>
        ("receive/loc_pos_enu", 10, &Formation_part::ot_loc_pos_enu_cb, this );

    // 话题订阅     | gcs/leader -> uav 编队设置
    formation_set_sub = mavcomm_nh.subscribe<mavcomm_msgs::formation_set>
        ("receive/formation_set", 10, &Formation_part::formation_set_cb, this );

    // 话题订阅     | gcs/leader -> uav 编队设置
    formation_info_sub = mavcomm_nh.subscribe<mavcomm_msgs::formation_info>
        ("receive/formation_info", 10, &Formation_part::formation_info_cb, this );
    
    // 话题发布     | uav -> gcs/leader 编队设置反馈设置
    formation_back_info_pub = mavcomm_nh.advertise<mavcomm_msgs::formation_back_info>
        ("send/formation_back_info", 1);    // 告知 gcs/uav-leader 无人机编队误差设置

    // px4 mavros 消息订阅
    // 话题订阅 本机位置信息
    this_uav_px4_local_position_pose_sub = mavros_nh.subscribe<geometry_msgs::PoseStamped>
        ("local_position/pose", 1, &Formation_part::this_uav_px4_local_position_pose_cb, this);
    // 话题订阅 本机速度信息
    this_uav_px4_local_position_velocity_local_sub = mavros_nh.subscribe<geometry_msgs::TwistStamped>
        ("local_position/velocity_local", 1, &Formation_part::this_uav_px4_local_position_velocity_local_cb, this);

    // px4 mavros 消息发布
    // 控制指令
    /* Local frame setpoint position. ENU坐标系(惯性系) */ 
    this_uav_px4_setVelocity_pub = mavros_nh.advertise<geometry_msgs::TwistStamped>("setpoint_velocity/cmd_vel",5);
}

// 数值 初始化
void Formation_part::formation_init()
{
    // TBC
    current_group = -1;
    last_group = -1;

    // 编队偏差
    for (int i=0; i<NNN; i++)
    { 
        // 邻居无人机位置相关
        neighbor_loc_pos_ENU[i].flag_update = 0;
        neighbor_loc_pos_ENU[i].last_msg_rec_time = 0;
        neighbor_loc_pos_ENU[i].cur_msg_rec_time = 0;
        flag_nb_connect_first[i] = 0;   //首次与邻居无人机连接 (之后断开不变)

        // 编队阵型相关
        for (int j=0; j<FORMATION_GROUP_PPP; j++)
        {
            formation_array[i][j].flag_this_group = 0;
            formation_array[i][j].flag_set = 0;
            formation_array[i][j].flag_leader = 0;
        }
    }

    for (int i=0;i<FORMATION_GROUP_PPP;i++)
    { 
        flag_formtation_offset_group_set[i] = 0;
        num_formtation_offset_group[i] = 0;
        type_formtation_offset_group[i] = 0;
    }
    // 设置开始 Init 初始化归零
    formation_set_check_current_num = -1;

}

//-------------------------------------------------
//                                  px4 mavros 订阅
//-------------------------------------------------
// 话题订阅     | mavros/local_pose/pose
void Formation_part::this_uav_px4_local_position_pose_cb
    (const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    currentPose = *msg;
    // std::cout << "receieve px4 loc pos = " << currentPose.pose.position.x << std::endl;
}

// 话题订阅     | mavros/local_pose/velocity_local
void Formation_part::this_uav_px4_local_position_velocity_local_cb
    (const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    currentVelocity = *msg;
    // std::cout << "receieve px4 loc vel = " << currentVelocity.pose.position.x << std::endl;
}

//-------------------------------------------------
// Func              邻居无人机位置-ot_loc_pos_enu_cb
//-------------------------------------------------

// 回调函数     /mavcomm/receive/loc_pos_enu
// 话题订阅     | neibor uav -> uav
// 编队控制     |接收 其他无人机的位置信息 编队飞行
void Formation_part::ot_loc_pos_enu_cb
    (const mavcomm_msgs::local_pos_enu::ConstPtr &msg)
{
    msg_ot_loc_pos_enu = *msg;
    int num = (int) msg_ot_loc_pos_enu.sysid;   // 发送端无人机编号
    flag_nb_connect_first[num] = 1;             // 首次与邻居无人机连接 (之后断开不变)

    // 时间更新
    neighbor_loc_pos_ENU[num].last_msg_rec_time = neighbor_loc_pos_ENU[num].cur_msg_rec_time;
    neighbor_loc_pos_ENU[num].cur_msg_rec_time = get_time_usec();
    // 位置更新
    neighbor_loc_pos_ENU[num].x = (double) msg_ot_loc_pos_enu.x; 
    neighbor_loc_pos_ENU[num].y = (double) msg_ot_loc_pos_enu.y;
    neighbor_loc_pos_ENU[num].z = (double) msg_ot_loc_pos_enu.z;
    neighbor_loc_pos_ENU[num].yaw = (double) msg_ot_loc_pos_enu.yaw;
}


//-------------------------------------------------
//                    无人机编队阵型位置-formation_set
//-------------------------------------------------

// 话题订阅     | gcs/leader -> uav 编队误差设置
void Formation_part::formation_set_cb(const mavcomm_msgs::formation_set::ConstPtr &msg)
{
    msg_formation_set = *msg;

    // 1-在 设置模式 下，才能进行 编队误差设置
    // 2-在 校验不完全的状态 下，才能进行 编队误差设置
    if ( (current_formation_state==FORMATION_STATE_SETTING) ||
         (current_formation_state==FORMATION_STATE_CHECK_FAIL) )
    {
        int group_id = (int)msg_formation_set.id_group;
        int drone_id = (int)msg_formation_set.id;
        // group 设置完全标志位 = 0
        flag_formtation_offset_group_set[ group_id ] = 0;
        // 对应 id 的 队形设置

        formation_array[drone_id][group_id].offset_x = (double) msg_formation_set.x;
        formation_array[drone_id][group_id].offset_y = (double) msg_formation_set.y; 
        formation_array[drone_id][group_id].offset_z = (double) msg_formation_set.z; 
        formation_array[drone_id][group_id].offset_yaw = (double) msg_formation_set.yaw;  
        formation_array[drone_id][group_id].flag = msg_formation_set.flag;

        formation_array[drone_id][group_id].flag_set = 1;

        if ( msg_formation_set.flag && 0b1000000 )
        {
            formation_array[drone_id][group_id].flag_leader = 1;
        } else {
            formation_array[drone_id][group_id].flag_leader = 0;
        }

        // ROS_INFO_STREAM( " Set ot_offset_xy [" << this_uav_offset_x << ", " << this_uav_offset_y << 
        // ", " << this_uav_offset_z << "," << this_uav_offset_yaw / PI_3 * 180 << "]");
    }

}

//-------------------------------------------------
//                           编队设置-formation_info
//-------------------------------------------------

// 话题订阅     | gcs/leader -> uav 编队设置
void Formation_part::formation_info_cb(const mavcomm_msgs::formation_info::ConstPtr &msg)
{
    msg_formation_info = *msg;

    switch (msg_formation_info.flag)
    {
        case FORMATION_INFO_SET_INIT: // 任务设置 初始化

            // 组队设置
            // 
            // param1 = group_num
            // param2 = group_type
            // param3 = TBD
            Init_formation_group_setting(
                msg_formation_info.id_group, 
                msg_formation_info.param1, 
                msg_formation_info.param2, 
                msg_formation_info.sysid);

        break;

        case FORMATION_INFO_CHECK: // 组队信息校验

            // group_id 是否是当前校验的 formation_set_check_current_num 
            if (formation_set_check_current_num != msg_formation_info.id_group)
            {   // 不是
                formation_set_check_current_num = msg_formation_info.id_group;
                current_formation_state = FORMATION_STATE_CHECKING;
                // TBDL 校准Init 相关参数重置
                // 会需要在这里添加 (>16 需要完成 setbit_mm 支持) 
            } 

            if (current_formation_state != FORMATION_STATE_CHECKED)
            {
                current_formation_state = FORMATION_STATE_CHECKING;

                // 组队信息校验
                // 
                // param1 = n
                // param2 = setbit [h7-d0]  +n*16   //设置位 1 已设置
                // param3 = setbit [h7-d0]+8+n*16   //设置位 1 已设置
                check_formation_group_setting(
                    msg_formation_info.id_group, 
                    msg_formation_info.param1, 
                    msg_formation_info.param2,
                    msg_formation_info.param3,  
                    msg_formation_info.sysid);

            }

        break;

        case FORMATION_INFO_RUN: 
            // TODO 切换并执行编队
            // 仅供测试用，正常在 mission 里调用&解析 task 后 

        break;

        default:
            // return;
        break;
    }
}

// 编队某组队形设置
void Formation_part::Init_formation_group_setting
    (int group_id, int group_num, int group_type, uint8_t receive_id)
{
    current_formation_state = FORMATION_STATE_SETTING;

    // 设置开始 Init 初始化归零
    for (int i=0; i<NNN; i++)
    {
        formation_array[i][group_id].flag_set = 0;
    }
    flag_formtation_offset_group_set[group_id] = 0;
    formation_set_check_current_num = -1;

    type_formtation_offset_group[group_id] = group_type;
    num_formtation_offset_group[group_id] = group_num;

    // 响应
    pubfunc_F_formation_back_info(FORMATION_STATE_SETTING, group_id, receive_id);
}

// 编队某组队形设置 校验
void Formation_part::check_formation_group_setting
    (int group_id, int setbit_mm, uint8_t setbit_1, uint8_t setbit_2, uint8_t receive_id)
{
    // 组队信息校验
    // 
    // setbit_mm    param1 = n
    // setbit_1     param2 = setbit [h7-d0]  +n*16   //设置位 1 已设置
    // setbit_2     param3 = setbit [h7-d0]+8+n*16   //设置位 1 已设置
    uint8_t check_setbit_1, check_setbit_2;

    // 计算对应 setbit_mm 的设置结果 setbit_1
    check_setbit_1 = 0;
    for (int i=0; i<8; i++)
    {
        int j;
        
        j = i + setbit_mm * 16;
        
        if ( j >= NNN )
        {
            break;
        }

        if ( formation_array[j][group_id].flag_set )
        {
            check_setbit_1 |= (1<<i);
            formation_array[j][group_id].flag_this_group = 1;
        } else {
            formation_array[j][group_id].flag_this_group = 0;
        }
    }

    // 计算对应 setbit_mm 的设置结果 setbit_2
    check_setbit_2 = 0;
    for (int i=0; i<8; i++)
    {
        int j;
        
        j = i + setbit_mm * 16 + 8;
        
        if ( j >= NNN )
        {
            break;
        }

        if ( formation_array[j][group_id].flag_set )
        {
            check_setbit_2 |= (1<<i);
            formation_array[j][group_id].flag_this_group = 1;
        } else {
            formation_array[j][group_id].flag_this_group = 0;
        }
    }

    // 校验 setbit_1 setbit_2
    // 当前最大无人机理论上限数量 == 16 (NNN setbit_mm=0)
    // >16 需要完成 setbit_mm 支持 TODO 
    if ( setbit_1 == check_setbit_1 && setbit_2 == check_setbit_2 )
    {   
        // 本组 编队设置无误
        current_formation_state = FORMATION_STATE_CHECKED;
        flag_formtation_offset_group_set[group_id] = 1;

        // 响应 TBC
        pubfunc_F_formation_back_info(
            FORMATION_STATE_CHECKED, 
            group_id, 
            receive_id,
            0,  // setbit_mm,
            check_setbit_1,
            check_setbit_2
            );

    } else {
        // 无人机组数 与设置的不符合
        current_formation_state = FORMATION_STATE_CHECK_FAIL;
        flag_formtation_offset_group_set[group_id] = 0;

        // 响应 TBC
        pubfunc_F_formation_back_info(
            FORMATION_STATE_CHECK_FAIL, 
            group_id, 
            receive_id,
            0,  // setbit_mm,
            check_setbit_1,
            check_setbit_2
            );
    }

}


//-------------------------------------------------
//                   编队设置反馈-formation_back_info
//-------------------------------------------------
// uav -> gcs/uav-leader   任务设置 回应  [由 flag 决定返回信息]
void Formation_part::pubfunc_F_formation_back_info
    (int flag_state, int id_group, uint8_t receive_id, int param1, int param2, int param3)
{
    switch ( flag_state )
    {
    
    //--------------------------------------
    // 设置模式
    case FORMATION_STATE_SETTING:   // 设置模式
        msg_formation_back_info.flag = FORMATION_STATE_SETTING;
        msg_formation_back_info.id_group = id_group;
        msg_formation_back_info.param1 = id_group;
        msg_formation_back_info.param2 = id_group;
        msg_formation_back_info.param3 = id_group;
    break;
    
    //--------------------------------------
    // 校验 失败
    case FORMATION_STATE_CHECK_FAIL:
        msg_formation_back_info.flag = FORMATION_STATE_CHECK_FAIL;
        msg_formation_back_info.id_group = id_group;
        msg_formation_back_info.param1 = param1;
        msg_formation_back_info.param2 = param2;
        msg_formation_back_info.param3 = param3;
    break;

    //-------------------------------------- 
    // 校验 完成
    case FORMATION_STATE_CHECKED:   // 设置模式
        msg_formation_back_info.flag = FORMATION_STATE_CHECKED;
        msg_formation_back_info.id_group = id_group;
        msg_formation_back_info.param1 = param1;
        msg_formation_back_info.param2 = param2;
        msg_formation_back_info.param3 = param3;
    break;
    
    default:
        return;
    break;
    }

    // common ID
    msg_formation_back_info.sysid = my_id;
    msg_formation_back_info.compid = receive_id;
    // header
    msg_formation_back_info.header.seq++;
    msg_formation_back_info.header.stamp = ros::Time::now(); 
    // publish
    formation_back_info_pub.publish(msg_formation_back_info);
    
}


// uav -> gcs/uav-leader   编队设置 回应  [常规-msg]
void Formation_part::pubfunc_formation_back_info
    (mavcomm_msgs::formation_back_info msg)
{
    // 回应信号
    msg_formation_back_info.flag = msg.flag;
    msg_formation_back_info.id_group = msg.id_group;
    msg_formation_back_info.param1 = msg.param1;
    msg_formation_back_info.param2 = msg.param2;
    msg_formation_back_info.param3 = msg.param3;
    // 发送&接收 ID 
    msg_formation_back_info.sysid = msg.sysid;
    msg_formation_back_info.compid = msg.compid;

    // 
    msg_formation_back_info.header.seq++;
    msg_formation_back_info.header.stamp = ros::Time::now(); 
    formation_back_info_pub.publish(msg_formation_back_info);
}


//-------------------------------------------------
//                                      编队控制算法
//-------------------------------------------------
// follower
// TBC 总控制 （保证正常切换到编队模式 & 选择控制算法）
// switch_group_id 需要切换到的 group_id
// return -1 返回 bug

// keep_z & keep_h 不变
int Formation_part::formation_ctrl_all_follower
    (int switch_group_id, float keep_z, float keep_h)
{
    // 判断进入何种编队模式 
    int state_formation_ctrl_all = FORMATION_CTRL_STATE_NAN;
    // mavros/cmd_vel 指令
    geometry_msgs::TwistStamped d_cal_ctrl_set_vel;

    // 1 判断 switch_group_id 是否不一样了 (有的话需要切换)
    // 2 判断 当前所处的 formation 模式 （不是运行中的话也需要切换）
    
    // 当前 1编队group_id没变 
    if ( switch_group_id == current_group)
    {
        //-------------------------------------------------------
        // 实际控制
        // 当前 2 已经处于分布式编队控制状态了
        if ( current_formation_state == FORMATION_STATE_RUNNING )
        {            
            formation_ctrl_first_order_PID_xy(keep_z, keep_h, &d_cal_ctrl_set_vel);
            // 输出
            this_uav_px4_setVelocity_pub.publish( d_cal_ctrl_set_vel );

            // 分布式 一阶编队控制算法
            state_formation_ctrl_all = FORMATION_CTRL_STATE_follower_distributed_1st_order;
            return state_formation_ctrl_all;
        }

        // ---------------------------------------------------------
        // 当前 2 正要形成编队
        if ( current_formation_state == FORMATION_STATE_RUN_FORMING )
        {

            if ( t_flag_form_direct == false )
            {
                // 1 - form_in_turn 置0 直接按offset形成编队 [也不怎么推荐-无避障]
                // 根据 leader 的位置 飞到那里去
                

                
            } 
            else // 1 - form_in_turn 置1 一架架，依次形成编队
            {
                // 等待外部输入 
                // 原地停止

                // 向上 折线 ...

            }

        }

        // ---------------------------------------------------------
        // 当前 2 正要形成编队 抉择需要切换到的状态
        if ( current_formation_state == FORMATION_STATE_RUN_PREPARE )
        {
            // 判断 当前 的 任务 模式 ??? 怎么形成编队
            // 如果没有控制的话 需要保持悬停不动

            // 获得当前的 flag
            t_flag_form_direct = formation_array[my_id][current_group].flag && 0b00000001;
            t_flag_form_in_turn = formation_array[my_id][current_group].flag && 0b00000010;
            std::cout << "my_id = " << my_id;
            std::cout << ", t_flag_form_direct" << t_flag_form_direct;
            std::cout << ", t_flag_form_in_turn" << t_flag_form_in_turn << std::endl;

            if ( t_flag_form_direct == false )
            {   
                // 0 - form_direct  置0 直接编队算法 [不推荐-无避障]
                formation_ctrl_first_order_PID_xy(keep_z, keep_h, &d_cal_ctrl_set_vel);
                // 输出
                this_uav_px4_setVelocity_pub.publish( d_cal_ctrl_set_vel );
                
                // 切换模式 => 直接编队算法
                state_formation_ctrl_all = FORMATION_STATE_RUNNING;
                return state_formation_ctrl_all;

            }   
            else // 0 - form_direct  置1 先形成编队，之再编队算法 [看 flag 1]
            {
                // 初值 赋值

                // 切换模式 => 正要形成编队
                state_formation_ctrl_all = FORMATION_STATE_RUN_FORMING;
                return state_formation_ctrl_all;
            }

        }

        // 当前 2 无法进入编队
        if ( current_formation_state == FORMATION_STATE_RUN_FAIL )
        { 
            // 不应该有这个模式 !!!
        } 

    }
    else // end if ( switch_group_id == current_group)
    {   
        // 当前 1编队group_id 改变了
        // 判断能否切入 FORMATION_STATE_RUN_FORMING 模式
        if ( current_formation_state == FORMATION_STATE_CHECKED || 
             current_formation_state == FORMATION_STATE_RUN_PREPARE || 
             current_formation_state == FORMATION_STATE_RUN_FORMING || 
             current_formation_state == FORMATION_STATE_RUNNING
            )
        {
            // 切状态
            last_group = current_group;
            current_group = switch_group_id;

            // 进入编队形成模式
            current_formation_state = FORMATION_STATE_RUN_PREPARE;

            // 编队形成状态
            state_formation_ctrl_all = FORMATION_CTRL_STATE_formation_prepare;
            return state_formation_ctrl_all;
        }
        else
        {
            // 正常不应该的
            // 进入状态不对 有问题 !!!
            // 进入 编队 模式失败
            // 自行 访问 current_formation_state
            state_formation_ctrl_all = FORMATION_CTRL_STATE_unable_enter_formation;
            return state_formation_ctrl_all;
        }
    }
}


// 一阶 分布式编队控制算法 xy
// 直接给速度指令?
// 保持 keep_d_z keep_d_yaw
void Formation_part::formation_ctrl_first_order_PID_xy
    (float keep_d_z, float keep_d_yaw, 
    geometry_msgs::TwistStamped *cal_ctrl_set_vel)    
    // keep_d_yaw = Mission_pose_current.orientation.w;
{
    // Init
    delta_enu_x = 0;
    delta_enu_y = 0;
    delta_enu_z = 0;
    delta_enu_yaw = 0;

    // 无人机计数
    float num_count;
    num_count = 0.0;    

    for (int i = 0; i<NNN; i++)
    { 
        // TBC 时间判断
        // 当前只考虑到接受到历史数据了
        // 如果长时间没有更新，需要丢弃
        // 以及考虑通信延迟 !!!

        if ( flag_nb_connect_first[i]==1 )
        {
            num_count = num_count + 1;
            // x
            delta_enu_x = delta_enu_x + neighbor_loc_pos_ENU[i].x 
                        - formation_array[i][current_group].offset_x;
            // y
            delta_enu_y = delta_enu_y + neighbor_loc_pos_ENU[i].y 
                        - formation_array[i][current_group].offset_y;

            // std::cout << "i=" << i << ", num_count=" << num_count;
            // std::cout << "delta_enu_x=" << delta_enu_x << ", delta_enu_y=" 
            //           << delta_enu_y << std::endl;
        }
    }
    
    // u_i = - sum[ z_i -z_j - (delta_i - delta_j) ]
    if (num_count == 0.0)
    {   // num_count == 0 怎么办 
        // 保持不动
        delta_enu_x = 0.0;     // Mission_pose_current.position.x - currentPose.position.x;
        delta_enu_y = 0.0;     // Mission_pose_current.position.y - currentPose.position.y;
    }
    else
    {
        delta_enu_x = delta_enu_x/num_count - currentPose.pose.position.x 
                    + formation_array[my_id][current_group].offset_x; 
        delta_enu_y = delta_enu_y/num_count - currentPose.pose.position.y 
                    + formation_array[my_id][current_group].offset_y;
    }

    // keep z
    delta_enu_z = keep_d_z - currentPose.pose.position.z;

    // keep yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(currentPose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    delta_enu_yaw = keep_d_yaw - yaw;
    if ( abs(delta_enu_yaw) > PI_3 ) 
    {
        if (delta_enu_yaw > 0)
            delta_enu_yaw = delta_enu_yaw - 2 * PI_3;  
        else
            delta_enu_yaw = delta_enu_yaw + 2 * PI_3;          
    }
    
    geometry_msgs::TwistStamped msg_ctrl_set_vel;

    msg_ctrl_set_vel.twist.linear.x = pid_1storder_x.p * delta_enu_x + pid_1storder_x.d * (0 - currentVelocity.twist.linear.x);
    msg_ctrl_set_vel.twist.linear.y = pid_1storder_y.p * delta_enu_y + pid_1storder_y.d * (0 - currentVelocity.twist.linear.y);
    msg_ctrl_set_vel.twist.linear.z = pid_1storder_z.p * delta_enu_z + pid_1storder_z.d * (0 - currentVelocity.twist.linear.z);
    msg_ctrl_set_vel.twist.angular.z = pid_1storder_yaw.p * delta_enu_yaw + pid_1storder_yaw.i * delta_enu_yaw_add + pid_1storder_yaw.d * (0 - currentVelocity.twist.angular.z);
    delta_enu_yaw_add = delta_enu_yaw_add + delta_enu_yaw;  
    //限制幅值
    msg_ctrl_set_vel.twist.linear.x = satfunc(msg_ctrl_set_vel.twist.linear.x , maxVelocity_1storder_x);
    msg_ctrl_set_vel.twist.linear.y = satfunc(msg_ctrl_set_vel.twist.linear.y , maxVelocity_1storder_y);
    msg_ctrl_set_vel.twist.linear.z = satfunc(msg_ctrl_set_vel.twist.linear.z , maxVelocity_1storder_z);
    msg_ctrl_set_vel.twist.angular.z = satfunc(msg_ctrl_set_vel.twist.angular.z , maxVelocity_1storder_yaw);

    *cal_ctrl_set_vel = msg_ctrl_set_vel;

}

