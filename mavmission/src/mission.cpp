// TODO
//  1. 分组执行任务 
//      => 需要 在 mavcomm 中增加 group 的 判断 & 设置
//      部分可参考 formation 中 分组编队的设置

#include <mission.h>

using namespace mav_mission;

//-------------------------------------------------
//  Init 
//-------------------------------------------------
Mav_Mission::Mav_Mission() :
    tp_nh("~"),             // param    /uav_mission/xxx
    mavcomm_nh("mavcomm"),  // mavcomm  /mavcomm/XXX   pub&sub
    mavros_nh("mavros"),    // mavros   /mavros/XXX
    desired_nh("desired")   // desired  /desired/XXX 
{
	// load param
    tp_nh.param<int>("my_id", my_id, 100);
	std::cout << "uav_mission/my_id = " << my_id << std::endl;

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

    // Init
    commom_init();

    // pub & sub

    // 话题订阅         | gcs -> uav
    // 编队设置         // TBC flag=>enum
    // *(flag=1) gcs->uav 无人机 ENU航点Pos (mission.cpp) 
    set_local_pos_enu_sub = mavcomm_nh.subscribe<mavcomm_msgs::local_pos_enu>
        ("receive/set_loc_pos_enu", 10, &Mav_Mission::set_local_pos_enu_cb, this );
    
    // TBD
    // mission_state = UNINIT;
    // mission_state_last = UNINIT;

	// Init tracker
	// tracker_load_param();
}

void Mav_Mission::run()
{
	ros::Rate loop_rate(20);

    // init param
    while (	ros::ok() ) 
    {	
        // 获取当前任务状态信息
        current_mission_state = _task_part.get_current_mission_state();

        if ( current_mission_state == MISSION_STATE_CHECKED)
        {   
            // 进入 具体执行状态  暂停状态 还是其他的 !!!!
            if ( 1 )
            {
                // task_part 任务具体执行中
                parses_current_mission_task();
            }

        }
        else
        {
            // task_part 任务输入 校验
            // 应该 都由 task_part 中 ROS_sub 订阅完成了 
            // 待确认！

        }

        ros::spinOnce();
		loop_rate.sleep();
	} 
	// return 0;
}


//-------------------------------------------------
// commom part
//-------------------------------------------------
void Mav_Mission::commom_init()
{
    // Pub   common ctrl 通用控制指令
    // 不使用 预留
    pub_ctrl_set_position = desired_nh.advertise<geometry_msgs::Point>("setTarget_position", 1);
    // 主要的 pos fast-planner
    pub_ctrl_set_pose = desired_nh.advertise<geometry_msgs::Pose>("setTarget_pose", 1);
    // TODO 编队
    pub_ctrl_set_vel = desired_nh.advertise<geometry_msgs::TwistStamped>("setVel", 1);

    // TODO 发布无人机状态 待其他程序调用
    pub_CurrentMissionState = mavcomm_nh.advertise<std_msgs::UInt8>("mission_state", 1);

    // mavros state sub
    // 订阅 mavros 状态 位置信息(判断任务进入下一阶段!)
    /* Local position from FCU. ENU坐标系(惯性系) */ 
	currentPose_sub = mavros_nh.subscribe<geometry_msgs::PoseStamped>("local_position/pose", 10, &Mav_Mission::currentPose_cb, this ); 
    /* Local velocity from FCU. ENU坐标系(惯性系) */ 
	currentVelocity_sub = mavros_nh.subscribe<geometry_msgs::TwistStamped>("local_position/velocity_local", 10, &Mav_Mission::currentVelocity_cb, this );

    // 订阅 地面站 指令 mission_exec 开始 暂停 继续 从N开始 结束降落 降落 结束悬停
	// flag h7-l0 h7=1整体顺序执行 h6=1整体同步执行(各管各的)
	mission_exec_sub = mavcomm_nh.subscribe<mavcomm_msgs::mission_exec>("receive/mission_exec", 10, &Mav_Mission::mission_exec_cb, this );



}



//-----------------------------------------------------------------------------------------
//   Sub   px4_mavros 
//-----------------------------------------------------------------------------------------
// 无人机位置 local
void Mav_Mission::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose = msg->pose;
}

// 无人机当前速度 local
void Mav_Mission::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    currentVelocity.twist = msg->twist;
}

//-----------------------------------------------------------------------------------------
//   Sub   mavcomm_msgs 
//-----------------------------------------------------------------------------------------
// 订阅 地面站 指令 mission_exec 1从N开始(默认0) 2暂停 3继续 4紧急降落 
// TODO flag h7-l0 h7=1整体顺序执行 h6=1整体同步执行(各管各的)
void Mav_Mission::mission_exec_cb(const mavcomm_msgs::mission_exec::ConstPtr& msg)
{
    msg_mission_exec = *msg;

    // 各 case 需要 判断， 当前任务状态 是否符合要求 可以切换 !!!
    // TODO - error back 反馈 !!!

    switch (msg_mission_exec.instruct_status)
    {
    case 1: // 从N开始/切换到N (默认0)
        
        if (current_mission_state == MISSION_STATE_CHECKED)
        {   // 按指令 设置任务的no 
            // (no和当前飞机的编号不相同时, 向上追溯)

            // 设置系统当前任务编号
            int temp_current_no, temp_no, temp_last_no, temp_next_no;
            
            temp_no = (int) msg_mission_exec.mission_no;
            temp_last_no = _task_part.get_this_last_uav_mis_no(temp_no);
            temp_next_no = _task_part.get_this_next_uav_mis_no(temp_no);
            temp_current_no = _task_part.get_mis_array_current();


            // mission 当前整体按顺序执行 
            // TODO 按无人机 的个体 顺序 执行
            // 设置temp_no
            _task_part.set_mission_task_next(temp_no);
            mission_exec_status_current = STATUS_MISSION_EXEC_run;
    
        /*
            if (temp_no==0)
            {   
                // 从头开始 空的
                // 设置temp_no
                _task_part.set_mission_task_next(temp_no);
                mission_exec_status_current = STATUS_MISSION_EXEC_run;
            } 
            else 
            {
                mission_exec_status_current = STATUS_MISSION_EXEC_run;
            }
        */

        } 

        // 反馈 pub back
        
        break;

    case 2: // 暂停
        if (mission_exec_status_current == STATUS_MISSION_EXEC_run)
        {
            mission_exec_status_current = STATUS_MISSION_EXEC_pause;
        }

        break;
    
    case 3: // 继续已暂停的任务
        if (mission_exec_status_current == STATUS_MISSION_EXEC_pause)
        {
            mission_exec_status_current = STATUS_MISSION_EXEC_run;
        }

        break;

    case 4: // 降落
        if ( (mission_exec_status_current == STATUS_MISSION_EXEC_run)
            || (mission_exec_status_current == STATUS_MISSION_EXEC_pause)
            || (mission_exec_status_current == STATUS_MISSION_EXEC_prepare_move)            
            )
        {
            mission_exec_status_current = STATUS_MISSION_EXEC_emeland;
        }

        break;

    default:

        break;
    }

}



//-----------------------------------------------------------------------------------------
// px4 直接控制 px4 PID control ()
//-----------------------------------------------------------------------------------------
// 

void Mav_Mission::PoseControl()
{   
    msg_ctrl_set_pose = Mission_pose_current;

    pub_ctrl_set_pose.publish(msg_ctrl_set_pose);
    // pub_ctrl_set_vel.publish(msg_ctrl_set_vel);
}

void Mav_Mission::track_pidVelocityControl()
{   

}


//-------------------------------------------------
// Func          设置飞机单点目标-set_local_pos_enu_cb
//-------------------------------------------------
// TBC
// 回调函数     /mavcomm/receive/loc_pos_enu
// 话题订阅         | gcs -> uav
// 编队设置         // TBC flag=>enum
// *(flag=1) gcs->uav 无人机 ENU航点Pos (mission.cpp)
void Mav_Mission::set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg)
{
    // TBC mission 状态位置切换

    msg_local_pos_enu = *msg;

    if (msg_local_pos_enu.flag == 1)
    {   
        //  设置飞机目标位置
        Mission_pose_current.position.x = (double) msg_local_pos_enu.x; 
        Mission_pose_current.position.y = (double) msg_local_pos_enu.y;
        Mission_pose_current.position.z = (double) msg_local_pos_enu.z;
        Mission_pose_current.orientation.w = (double) msg_local_pos_enu.yaw;

        ROS_INFO_STREAM( " Set target Pos to [" << Mission_pose_current.position.x << ", " << 
        Mission_pose_current.position.y << ", " << Mission_pose_current.position.z << ", " << 
        Mission_pose_current.orientation.w / PI_3 * 180.0 << "]");
    }

}


/*
// 地面站 => 无人机 任务切换 
// 任务开始 & 暂停 & 继续 & 结束 & 从XX开始
// mission_info 需要 改成其他 类型
void Mav_Mission::mission_info_cb(const mavcomm_msgs::mission_info::ConstPtr& msg)
{
    msg_mission_info = *msg;

    //  msg_mission_info.flag
    //  msg_mission_info.mission_num;
    //  msg_mission_info.param1;
    //  msg_mission_info.param2;

    // 不区分 system_id  companion_id
    switch (msg_mission_info.flag)
    {
        case 4: // 任务开始
            // mis_total = (int) msg_mission_info.mission_num;
            // 判断任务总数是否
            if ( flag_mission_set==3 )    // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
            {  
                // 标志位设置
                mis_array_current = 0;  // 当前任务编号 从0开始
                flag_mission_set = 3;   // 0未设置 1设置中 2未校准 3校正完      
                flag_mission_start = 1; // 0-未开始 1-任务运行 2-暂停

                if ( msg_mission_info.param1 == 1)
                {   // 顺序执行
                    flag_mission_sync = 1;
                } else if ( msg_mission_info.param1 == 2)
                {
                    flag_mission_sync = 2;
                }
            } 
        break;

        case 5: // 跳到 #N任务 并开始执行  [无回应]
            // mis_total = (int) msg_mission_info.mission_num;
            // 判断任务总数是否
            if ( flag_mission_set==3 )    // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
            {  
                // 标志位设置
                mis_array_current = (int)msg_mission_info.mission_num;  // 当前任务编号 从0开始
                flag_mission_set = 3;   // 0未设置 1设置中 2未校准 3校正完      
                flag_mission_start = 1; // 0-未开始 1-任务运行 2-暂停

                if ( msg_mission_info.param1 == 1)
                {   // 顺序执行
                    flag_mission_sync = 1;
                } else if ( msg_mission_info.param1 == 2)
                {
                    flag_mission_sync = 2;
                }
            } 
        break;

        case 6: // 跳到 #N任务 并开始执行  [无回应]
            // mis_total = (int) msg_mission_info.mission_num;
            // 判断任务总数是否
            if ( flag_mission_set==3 )    // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
            {  
                // 标志位设置
                mis_array_current = (int)msg_mission_info.mission_num;  // 当前任务编号 从0开始
                flag_mission_set = 3;   // 0未设置 1设置中 2未校准 3校正完      
                flag_mission_start = 2; // 0-未开始 1-任务运行 2-暂停悬停 

                flag_mission_pause_task = (int) msg_mission_info.param1; // 1暂停&悬停 /2暂停&原地降落 /3暂停&起飞位置降落
            } 
        break;

        default:

        break;
    }
}

*/



//-------------------------------------------------
// Func    任务解析和执行-parses_current_mission_task
//-------------------------------------------------
// msg_mission_set.mission_task 解析函数
void Mav_Mission::parses_current_mission_task()
{

    switch ( current_mission_state )
    {
    case INFOR_PARSES_TASK_takeoff:
        /* 起飞 */
        mission_task_handle_takeoff();
    break;

    case INFOR_PARSES_TASK_land:
        /* 降落 */
        mission_task_handle_land();
    break;

    case INFOR_PARSES_TASK_pos_enu:
        /* 打点移动 enu */
        mission_task_handle_pos_enu();
    break;

    case INFOR_PARSES_TASK_foramtion:
        /* 编队飞行 */
        mission_task_handle_foramtion();
    break;


    case INFOR_PARSES_TASK_track:
        /* 目标追踪 */
        mission_task_handle_track();
    break;
    
    default:

        break;
    }
        // 

		// ros::spinOnce();
        // TODO 广播 本机任务 状态信息
        // formation_pidVelocityControl(); 
        // 编队控制

        /*
            case FORMATION_FLY:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = FORMATION_FLY " );
                }
                // 速度PID控制
                formation_pidVelocityControl();
                // 位置控制
                // positionControl();
            break;

            case TRACK:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = TRACK " );
                }
                // 速度PID控制
                track_pidVelocityControl();
                // 位置控制
                // positionControl();
            break;
        */

        // 判断 mission 有没有完成
        // 实现的话 + 1 

}


//-------------------------------------------------
// ENUM_TASK_PARSES_INFOR = INFOR_PARSES_TASK_takeoff
// mission_task_handle_takeoff
//-------------------------------------------------
void Mav_Mission::mission_task_handle_takeoff()
{


}


void Mav_Mission::mission_task_handle_land()
{


}

void Mav_Mission::mission_task_handle_pos_enu()
{


}


void Mav_Mission::mission_task_handle_foramtion()
{


}

void Mav_Mission::mission_task_handle_track()
{


}