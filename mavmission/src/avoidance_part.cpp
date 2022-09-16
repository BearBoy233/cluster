


// ----无人机避障模块 (fast_planner)
// fast
void fast_init();
ros::Subscriber fast_sub;
void fast_sub_cb(const mav_mission::PositionCommand::ConstPtr &msg);


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


