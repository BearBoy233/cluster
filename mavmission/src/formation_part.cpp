
//-------------------------------------------------
// formation part
//-------------------------------------------------

void formation_init();

void ot_loc_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);

void set_local_pos_enu_cb(const mavcomm_msgs::local_pos_enu::ConstPtr &msg);

//################################################################------编队飞行模块
// 订阅其他无人机的位置 (已经减掉 队形偏差量了)
ros::Subscriber ot_loc_pos_enu_sub;
mavcomm_msgs::local_pos_enu msg_ot_local_pos_enu;
double ot_pos_x[NNN];
double ot_pos_y[NNN];
double ot_pos_z[NNN];
double ot_pos_yaw[NNN];
// TODO 待拓展 当前只能编一组队
int flag_ot_num[NNN];       // ==1 已知的邻居无人机位置 ； /TODO 参与编队的其他无人机 Num;
int ot_num_sum;         // 编组内无人机个数
int ot_this_num;        // 本机编组 TODO
// 本机的编队偏差 实际发送 Loc 时, 偏差需要减掉 (px4_bridge 中完成)
// 当前仅考虑 室内定位系统(ENU) 的情况 (TODO GPS/ VINS...)
// 本机 编队偏差
float ot_offset_x = 0.0;
float ot_offset_y = 0.0;
float ot_offset_z = 0.0;      // 暂时不用
float ot_offset_yaw = 0.0;    // 暂时不用

// mavcomm 初始设置无人机编队 队形
ros::Subscriber set_local_pos_enu_sub;
mavcomm_msgs::local_pos_enu msg_local_pos_enu;

// 无人机队形 设置回应
ros::Publisher set_local_pos_enu_pub;         // 告知地面站 无人机编队误差设置



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


    // 编队偏差
    for (int i=1;i<NNN;i++)
    { 
        flag_ot_num[i] = 0;   //以后归    flag
    }
    
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