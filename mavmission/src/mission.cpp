#include <mission.h>

using namespace mav_mission;

//-------------------------------------------------
// Init
//-------------------------------------------------
Mav_Mission::Mav_Mission() :
	nh("~"),				// 用于发布订阅绝对话题 + roslaunch param get
	mavlink_nh("mavlink")	// allow to namespace it
{
	// load param
    nh.param<int>("my_id", my_id, 100);
	std::cout << "uav_mission/my_id = " << my_id << std::endl;

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

    // Init
    // TBD
    // mission_state = UNINIT;
    // mission_state_last = UNINIT;

    // 编队偏差
    for (int i=1;i<NNN;i++)
    { 
        flag_ot_num[i] = 0;   //以后归    flag
    }

    mission_init();

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

        formation_pidVelocityControl(); // 编队控制

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
    pub_CurrentMissionState = nh.advertise<std_msgs::UInt8>("/mavcomm/mission_state", 1);

    // mavros state
    /* Local position from FCU. ENU坐标系(惯性系) */ 
    
	currentPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Mav_Mission::currentPose_cb, this ); 
    
	//### currentVelocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, currentVelocity_cb );

	// 无人机 ENU 航点Pos(flag=1) 编队误差设计(flag=2) 
    //### set_local_pos_enu_sub = nh.subscribe<mavcomm_msgs::local_pos_enu>("/mavcomm/receive/set_loc_pos_enu", 10, set_local_pos_enu_cb );



}




//-------------------------------------------------
// tracker part
//-------------------------------------------------

void Mav_Mission::tracker_init()
{
    //-------------------------------------------------
    // 追踪 Track
    // TODO
    // 1. 来自其他无人机 的 目标位置信息

    // 2. 来自本机的 目标位置信息

	// tracker_load_param();
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

//-------------------------------------------------
// fast part
//-------------------------------------------------
void fast_init()
{
	// --------------------------------------------------------------------------------------------------------------
    // 避障飞行 (TODO)
    // 发送给 fast_plan 的消息

    // 接收 fast_plan 的信息
    // fast plan
    //### fast_sub = nh.subscribe<mav_mission::PositionCommand>("/planning/pos_cmd", 1, fast_sub_cb );

}


// 回调函数
// 避障函数 fast 
void Mav_Mission::fast_sub_cb(const mav_mission::PositionCommand::ConstPtr &msg)
{
    mav_mission::PositionCommand cmd;
    cmd = *msg;
    
    Mission_pose_current.position.x = cmd.position.x;
    Mission_pose_current.position.y = cmd.position.y;
    Mission_pose_current.position.z = cmd.position.z;
    Mission_pose_current.orientation.w = cmd.yaw;
}


// fast_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 1, &uavControl::fast_sub_cb,this);





//-------------------------------------------------
// formation part
//-------------------------------------------------

void Mav_Mission::formation_init()
{
    // --------------------------------------------------------------------------------------------------------------
    // 编队控制
    // 接收 其他无人机的位置信息 编队飞行
    //### ot_loc_pos_enu_sub = nh.subscribe<mavcomm_msgs::local_pos_enu>("/mavcomm/receive/loc_pos_enu", 10, ot_loc_pos_enu_cb );
    // 无人机 ENU 航点Pos(flag=1) 编队误差设计(flag=2) 
    //### set_local_pos_enu_sub = nh.subscribe<mavcomm_msgs::local_pos_enu>("/mavcomm/receive/set_loc_pos_enu", 10, set_local_pos_enu_cb );
    // 无人机编队偏差反馈 (flag=3)
    set_local_pos_enu_pub = nh.advertise<mavcomm_msgs::local_pos_enu>("/mavcomm/send/set_loc_pos_enu", 1); // 告知地面站 无人机编队误差设置

}

// 回调函数
// 编队飞行
// /mavcomm/receive/loc_pos_enu
void Mav_Mission::ot_loc_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg)
{
    int num;

    msg_ot_local_pos_enu = *msg;

    num = (int) msg_ot_local_pos_enu.sysid;   // 发送端无人机编号
    flag_ot_num[num] = 1;                     // 以后归    flag
    ot_pos_x[num] = (double) msg_ot_local_pos_enu.x; 
    ot_pos_y[num] = (double) msg_ot_local_pos_enu.y;
    ot_pos_z[num] = (double) msg_ot_local_pos_enu.z;
    ot_pos_yaw[num] = (double) msg_ot_local_pos_enu.yaw;
}

void Mav_Mission::set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg)
{
    msg_local_pos_enu = *msg;

    if (msg_local_pos_enu.flag == 1)
    {   //  设置飞机目标位置
        Mission_pose_current.position.x = (double) msg_local_pos_enu.x; 
        Mission_pose_current.position.y = (double) msg_local_pos_enu.y;
        Mission_pose_current.position.z = (double) msg_local_pos_enu.z;
        Mission_pose_current.orientation.w = (double) msg_local_pos_enu.yaw;

        ROS_INFO_STREAM( " Set target Pos to [" << Mission_pose_current.position.x << ", " << 
        Mission_pose_current.position.y << ", " << Mission_pose_current.position.z << ", " << 
        Mission_pose_current.orientation.w / PI_3 * 180.0 << "]");
    }
    else if (msg_local_pos_enu.flag == 2)
    {   //  设置飞机编队偏差
        ot_offset_x = msg_local_pos_enu.x; 
        ot_offset_y = msg_local_pos_enu.y; 
        ot_offset_z = msg_local_pos_enu.z; 
        ot_offset_yaw = msg_local_pos_enu.yaw; 

        ROS_INFO_STREAM( " Set ot_offset_xy [" << ot_offset_x << ", " << ot_offset_y << 
        ", " << ot_offset_z << "," << ot_offset_yaw / PI_3 * 180 << "]");

        msg_local_pos_enu.header.stamp = ros::Time::now();
        msg_local_pos_enu.flag = msg_local_pos_enu.sysid;
        msg_local_pos_enu.sysid = msg_local_pos_enu.compid;
        msg_local_pos_enu.compid = msg_local_pos_enu.flag;
        msg_local_pos_enu.flag = 3;

        // flag 置3 反馈给地面站 确认已设置
        set_local_pos_enu_pub.publish(msg_local_pos_enu);
    }
}

// 分布式 编队控制算法
void Mav_Mission::formation_pidVelocityControl()
{
    // 直接给速度指令

    // Init
    for (int i=0; i<3; i++)
    {    delta_pos[i] = 0;
    }
    delta_yaw = 0;

    float num_count;
    num_count = 0.0;

    for (int i = 1; i<NNN; i++)
    { 
        if (flag_ot_num[i]==1 )
        {
            num_count = num_count + 1;
            delta_pos[0] = delta_pos[0] + ot_pos_x[i];
            delta_pos[1] = delta_pos[1] + ot_pos_y[i];
            // std::cout << "i=" << i << ", num_count=" << num_count;
            // std::cout << "ot_pos_x[i]=" << ot_pos_x[i] << ", ot_pos_y[i]=" << ot_pos_y[i] << std::endl;
        }
    }
    
    // u_i = - sum[ z_i -z_j - (delta_i - delta_j) ]
    if (num_count == 0.0)
    {   // num_count == 0 怎么办 
        // 保持不动
        delta_pos[0] = 0.0;     // Mission_pose_current.position.x - currentPose.position.x;
        delta_pos[1] = 0.0;     // Mission_pose_current.position.y - currentPose.position.y;
    }
    else
    {
        delta_pos[0] = delta_pos[0]/num_count - currentPose.position.x + ot_offset_x; 
        delta_pos[1] = delta_pos[1]/num_count - currentPose.position.y + ot_offset_y; 
    }

    delta_pos[2] = Mission_pose_current.position.z - currentPose.position.z;
    
    msg_ctrl_set_vel.twist.linear.x = pid_x.p * delta_pos[0] + pid_x.d * (0 - currentVelocity.twist.linear.x);
    msg_ctrl_set_vel.twist.linear.y = pid_y.p * delta_pos[1] + pid_y.d * (0 - currentVelocity.twist.linear.y);
    msg_ctrl_set_vel.twist.linear.z = pid_z.p * delta_pos[2] + pid_z.d * (0 - currentVelocity.twist.linear.z);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(currentPose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    double target_yaw = Mission_pose_current.orientation.w;
    delta_yaw = target_yaw - yaw;

    if ( abs(delta_yaw) > PI_3 ) 
    {
        if(delta_yaw > 0)
            delta_yaw = delta_yaw - 2 * PI_3;  
        else
            delta_yaw = delta_yaw + 2 * PI_3;          
    }
    
    msg_ctrl_set_vel.twist.angular.z = pid_yaw.p * delta_yaw + pid_yaw.i * delta_yaw_add + pid_yaw.d * (0 - currentVelocity.twist.angular.z);
    delta_yaw_add = delta_yaw_add + delta_yaw;  
    //限制幅值
    msg_ctrl_set_vel.twist.linear.x = satfunc(msg_ctrl_set_vel.twist.linear.x , maxVelocity_x);
    msg_ctrl_set_vel.twist.linear.y = satfunc(msg_ctrl_set_vel.twist.linear.y , maxVelocity_y);
    msg_ctrl_set_vel.twist.linear.z = satfunc(msg_ctrl_set_vel.twist.linear.z , maxVelocity_z);
    msg_ctrl_set_vel.twist.angular.z = satfunc(msg_ctrl_set_vel.twist.angular.z , maxVelocity_yaw);

    pub_ctrl_set_vel.publish(msg_ctrl_set_vel);

}



//-------------------------------------------------
// mission part
//-------------------------------------------------

void Mav_Mission::mission_init()
{
	// mavcomm mission 模块
    // TODO 逻辑梳理 + 优化
    // 订阅 任务指令
    // 接收 任务设置...
    
	//### mission_info_sub = nh.subscribe<mavcomm_msgs::mission_info>("/mavcomm/receive/mission_info", 10, mission_info_cb );
    // 发送 任务设置反馈
    mission_back_info_pub = nh.advertise<mavcomm_msgs::mission_back_info>
        ("/mavcomm/send/mission_back_info", 1);
    
    // 接收 获取具体的每一条任务...
    mission_info_sub = nh.subscribe<mavcomm_msgs::mission_set>
        ("/mavcomm/receive/mission_set", 10, &Mav_Mission::mission_set_cb, this );
}

// 回调函数     mission

// 无人机任务 mission
// mission 考虑整体系统 的 联系性
// 任务开始前 所有无人机设置统一的整体任务

// 无人机 任务 设置
void Mav_Mission::mission_set_cb(const mavcomm_msgs::mission_set::ConstPtr& msg)
{
    _task_part.task_mission_set(msg);
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
            mis_total = (int) msg_mission_info.mission_num;
            
            

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
            if ( (mis_total == (int) msg_mission_info.mission_num ) &&
                 (mis_save_param[0] == msg_mission_info.param1) &&
                 (mis_save_param[1] == msg_mission_info.param2) )
            {   // 任务总数 & param参数 对

                // 遍历所有的 mis_array[i].flag_set == 1;
                // 接收的任务总数是否完整
                flag_incomplete_task_num = 0;
                incomplete_task_array_total = 0;

                for (int i=0; i<mis_total; i++)
                {   
                    // 寻找未设置的任务
                    if ( mis_array[i].flag_set == 0 )
                    {
                        incomplete_task_array[incomplete_task_array_total] = i;
                        flag_incomplete_task_num = 1;
                        incomplete_task_array_total++;
                    }
                }
                // 遍历完
                if ( flag_incomplete_task_num )
                {
                    // 如果任务数据不完整
                    // 进入 设置模式 ( 固定频率回传无人机的代码 )
                    // 

                    break;
                }

                // 设置 last_uav_mis_no & next_uav_mis_no

                for (int i=0; i<mis_total; i++)
                {   
                    // 寻找本机的任务
                    if ( mis_array[i].flag_this_uav == 1 )
                    {


                    }
                }
                // mis_array[i].last_uav_mis_no = 0;
                // mis_array[i].next_uav_mis_no = 0;

                // 标志位设置
                flag_mission_set = 3;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                // 回应信号
                // mission_back_info
                // 正确
                msg_mission_back_info.sysid = my_id;
                msg_mission_back_info.compid = ID_GCS;

                msg_mission_back_info.flag = 2;
                msg_mission_back_info.mission_num = 3;
                msg_mission_back_info.param1 = 3;
                msg_mission_back_info.param2 = 3;
                mission_back_info_pub.publish(msg_mission_back_info);

            } else
            {
                // 标志位设置
                flag_mission_set = 4;       // 0未设置 1设置中 2未校准 3校正完 4校准错误 5读取错误
                flag_mission_start = 0;     // 0-未开始 1-任务运行 2-暂停

                // 回应信号
                // mission_back_info
                // 错误2 任务校验不一致 无人机和地面站的任务不一致
                msg_mission_back_info.sysid = my_id;
                msg_mission_back_info.compid = ID_GCS;

                msg_mission_back_info.flag = 2;
                msg_mission_back_info.mission_num = 4;
                msg_mission_back_info.param1 = 4;
                msg_mission_back_info.param2 = 4;
                mission_back_info_pub.publish(msg_mission_back_info);
            }

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