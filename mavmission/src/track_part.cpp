

void Mav_Mission::tracker_load_param()
{
	// load pid param
    nh.param<double>("Kp_x", pid_x.p, 2.0);
    nh.param<double>("Ki_x", pid_x.i, 0.0);
    nh.param<double>("Kd_x", pid_x.d, 1.5);

    nh.param<double>("Kp_y", pid_y.p, 2.0);
    nh.param<double>("Ki_y", pid_y.i, 0.0);
    nh.param<double>("Kd_y", pid_y.d, 1.5);

    nh.param<double>("Kp_z", pid_z.p, 2.0);
    nh.param<double>("Ki_z", pid_z.i, 0.0);
    nh.param<double>("Kd_z", pid_z.d, 1.2);

    nh.param<double>("Kp_yaw", pid_yaw.p, 2.0);
    nh.param<double>("Ki_yaw", pid_yaw.i, 0.0);    
    nh.param<double>("Kd_yaw", pid_yaw.d, 1.2);

    nh.param<float>("maxVelocity_x", maxVelocity_x, 0.6);
    nh.param<float>("maxVelocity_y", maxVelocity_y, 0.6);
    nh.param<float>("maxVelocity_z", maxVelocity_z, 0.6);
    nh.param<float>("maxVelocity_yaw", maxVelocity_yaw, 1.4);

    nh.param<double>("init_takeoff_z", init_takeoff_z, 0.5);    // 无人机的 起飞高度 

    ROS_INFO_STREAM( "param /uav_ctrl/x_Kp = " << pid_x.p );
    ROS_INFO_STREAM( "param /uav_ctrl/x_Ki = " << pid_x.i );
    ROS_INFO_STREAM( "param /uav_ctrl/x_Kd = " << pid_x.d );

    ROS_INFO_STREAM( "param /uav_ctrl/y_Kp = " << pid_y.p );
    ROS_INFO_STREAM( "param /uav_ctrl/y_Ki = " << pid_y.i );
    ROS_INFO_STREAM( "param /uav_ctrl/y_Kd = " << pid_y.d );

    ROS_INFO_STREAM( "param /uav_ctrl/z_Kp = " << pid_z.p );
    ROS_INFO_STREAM( "param /uav_ctrl/z_Ki = " << pid_z.i );
    ROS_INFO_STREAM( "param /uav_ctrl/z_Kd = " << pid_z.d );

    ROS_INFO_STREAM( "param /uav_ctrl/yaw_Kp = " << pid_yaw.p );
    ROS_INFO_STREAM( "param /uav_ctrl/yaw_Ki = " << pid_yaw.i );
    ROS_INFO_STREAM( "param /uav_ctrl/yaw_Kd = " << pid_yaw.d );

    ROS_INFO_STREAM( "param /uav_ctrl/maxVelocity_x = " << maxVelocity_x);
    ROS_INFO_STREAM( "param /uav_ctrl/maxVelocity_y = " << maxVelocity_y );
    ROS_INFO_STREAM( "param /uav_ctrl/maxVelocity_z = " << maxVelocity_z );
    ROS_INFO_STREAM( "param /uav_ctrl/maxVelocity_yaw = " << maxVelocity_yaw );

    ROS_INFO_STREAM( "param /uav_ctrl/init_takeoff_z = " << init_takeoff_z );
}
