#include <mission.h>

using namespace mav_mission;

//-------------------------------------------------
// Init
//-------------------------------------------------
Mav_Mission::Mav_Mission() :
	nh("~"),				// 用于发布订阅绝对话题 + roslaunch param get
	mavlink_nh("~")	// allow to namespace it
{
	// load param
    nh.param<int>("my_id", my_id, 100);
	std::cout << "uav_mission/my_id = " << my_id << std::endl;

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

    // Init
    commom_init();

    // TBD
    // mission_state = UNINIT;
    // mission_state_last = UNINIT;

	// Init tracker
	// tracker_load_param();
}

void Mav_Mission::run()
{
	ros::Rate loop_rate(20);

    int temp = ID_GCS;

    /*
	spinner.start();
	ros::waitForShutdown();

	ROS_INFO("Stopping mavros...");
	spinner.stop();
    */
   	
    while (	ros::ok() ) 
    {	
		// ros::spinOnce();

        // TODO 广播 本机任务 状态信息

        if ( flag_mission_set == 1 || flag_mission_set == 2  ) // 0未设置 1设置中 2未校准 3校正完 4校准错误
        {   //等待地面站 发过来的 mission

            //     
        }

        if ( flag_mission_start )
        {
            // 任务流程
            // 判断模式

            // 判断无人机模式 [! ]

            // if ()

/*
int mis_total;                  // 当前任务 总数目
int mis_array_current;          // 当前任务
uint8_t mis_save_param[3];       // 无人机存储 参数

// 任务数组
struct MIS {
    mavcomm_msgs::mission_set msg_mission_set;
    bool flag_set;          // 是否设置了
    bool flag_this_uav;     // 是否为本机的任务
    int last_uav_mis_no;    // 上一个 与本机有关的任务 编号
    int next_uav_mis_no;    // 下一个 与本机有关的任务 编号
} mis_array[200];
*/

        PoseControl();  // 位置控制 + fast_planner

        // formation_pidVelocityControl(); // 编队控制

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

    // common
    // 不使用 预留
    pub_ctrl_set_position = nh.advertise<geometry_msgs::Point>("/desired/setTarget_position", 1);
    // 主要的 pos fast-planner
    pub_ctrl_set_pose = nh.advertise<geometry_msgs::Pose>("/desired/setTarget_pose", 1);
    // TODO 编队
    pub_ctrl_set_vel = nh.advertise<geometry_msgs::TwistStamped>("/desired/setVel", 1);

    // TODO 发布无人机状态 待其他程序调用
    pub_CurrentMissionState = mavlink_nh.advertise<std_msgs::UInt8>("/mavcomm/mission_state", 1);

    // mavros state
    /* Local position from FCU. ENU坐标系(惯性系) */ 
    
	currentPose_sub = mavlink_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &Mav_Mission::currentPose_cb, this ); 
    
	//### currentVelocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, currentVelocity_cb );

	// 无人机 ENU 航点Pos(flag=1) 编队误差设计(flag=2) 
    //### set_local_pos_enu_sub = nh.subscribe<mavcomm_msgs::local_pos_enu>("/mavcomm/receive/set_loc_pos_enu", 10, set_local_pos_enu_cb );

}



//-----------------------------------------------------------------------------------------
//	px4 PID control ()
//-----------------------------------------------------------------------------------------

// 回调函数

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



void Mav_Mission::PoseControl()
{   
    msg_ctrl_set_pose = Mission_pose_current;

    pub_ctrl_set_pose.publish(msg_ctrl_set_pose);
    // pub_ctrl_set_vel.publish(msg_ctrl_set_vel);
}

void Mav_Mission::track_pidVelocityControl()
{   

}






/*
// 地面站 => 无人机
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
        case 1: // 任务设置 初始化

            Task_part::mission_settings_init(msg)

            // 回应信号
            // mission_back_info
            // 告知 无人机 本机已经在 等待接收状态了
            msg_mission_back_info.sysid = my_id;
            msg_mission_back_info.compid = ID_GCS;

            msg_mission_back_info.flag = 1;
            msg_mission_back_info.mission_num = mis_total;
            msg_mission_back_info.param1 = mis_save_param[0];
            msg_mission_back_info.param2 = mis_save_param[1];

            mission_back_info_pub.publish(msg_mission_back_info);

        break;

        case 2: // 任务设置完 校验
            

        break;

        case 3: // 读取&保存任务
            bool results; // 文件读取&保存结果 正确1 错误0
            // TODO
            if (msg_mission_info.param1 == 1)
            {   // 读取
                // 读取 文件名  XXX + msg_mission_info.param2. XXX
                // func []
                // results = 读取结果

                // 回应信号
                // mission_back_info

                if ( results )
                {   // 读取正确

                    // 标志位设置
                    flag_mission_set = 2;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                    flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                    msg_mission_back_info.sysid = my_id;
                    msg_mission_back_info.compid = ID_GCS;

                    msg_mission_back_info.flag = 3;
                    msg_mission_back_info.mission_num = mis_total;
                    msg_mission_back_info.param1 = mis_save_param[0]; //
                    msg_mission_back_info.param2 = mis_save_param[1]; //
                    mission_back_info_pub.publish(msg_mission_back_info);
                }
                else
                {
                    // 标志位设置
                    flag_mission_set = 5;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                    flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                    msg_mission_back_info.sysid = my_id;
                    msg_mission_back_info.compid = ID_GCS;

                    msg_mission_back_info.flag = 4;
                    msg_mission_back_info.mission_num = 0;
                    msg_mission_back_info.param1 = 1; //
                    msg_mission_back_info.param2 = 5; //
                    mission_back_info_pub.publish(msg_mission_back_info);
                }

            } else if (msg_mission_info.param1 == 2)
            {
                // 保存 文件名  XXX + msg_mission_info.param2. XXX
                // func []
                // results = 读取结果

                // 回应信号
                // mission_back_info

                if ( results )
                {   // 保存正确

                    // 标志位设置 (该是啥还是啥)
                    // flag_mission_set = 2;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                    // flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                    msg_mission_back_info.sysid = my_id;
                    msg_mission_back_info.compid = ID_GCS;

                    msg_mission_back_info.flag = 3;
                    msg_mission_back_info.mission_num = mis_total;
                    msg_mission_back_info.param1 = mis_save_param[0]; //
                    msg_mission_back_info.param2 = mis_save_param[1]; //
                    mission_back_info_pub.publish(msg_mission_back_info);
                }
                else
                {
                    // 保存错误 (一般不会出现)
                    flag_mission_set = 5;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                    flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                    msg_mission_back_info.sysid = my_id;
                    msg_mission_back_info.compid = ID_GCS;

                    msg_mission_back_info.flag = 4;
                    msg_mission_back_info.mission_num = 0;
                    msg_mission_back_info.param1 = 2; // 1读 2写
                    msg_mission_back_info.param2 = 5; //
                    mission_back_info_pub.publish(msg_mission_back_info);
                }

            }
        break;

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