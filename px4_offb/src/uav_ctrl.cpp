// px4_uav ctrl
// 无人机 位置控制 和 状态切换

// targetPose.position.x/y/z
// double target_yaw = targetPose.orientation.w;    [-pi,pi]

// TODO
// gps support (放在其他模块，转换到 主体的 LOC ENU系 下)    
// 机体系控制

#define NNN 11

#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <mavcomm_msgs/ChangeState.h>

// #include <px4_offb/PositionCommand.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>


class uavControl {

public:
    // base param
    uavControl( 
    const ros::NodeHandle &nh_, 
    const ros::NodeHandle &nh_private_
    );

    ~uavControl();

    double pi = 3.1415926;

    // main
    void start();
    // Set func
    void waitConnect();
    bool arm();
    bool disarm();
    bool offboard();
    bool autoland();
    bool takeoff();
    // help func
    float satfunc(float data, float Max);

    // ROS part
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    // 用于 订阅 发布 param读取
    ros::NodeHandle tp_nh;
    // mavcomm (pub/sub)
    ros::NodeHandle mavcomm_nh;
    // mavros (pub/sub)
    ros::NodeHandle mavros_nh;
    // desired
	ros::NodeHandle desired_nh;
    // gen
	ros::NodeHandle gen_nh;

    ros::Rate *rate;

    bool Info_flag;

    // From mavros data ------------------------------------------------------
    // 回调函数
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    ros::Subscriber state_sub;
    ros::Subscriber currentPose_sub;
    ros::Subscriber currentVelocity_sub;
    
    ros::ServiceClient arming_client;       // 解锁
    ros::ServiceClient set_mode_client;     // 设置模式

    geometry_msgs::Pose currentPose;
    geometry_msgs::TwistStamped currentVelocity;  
    mavros_msgs::State current_state;       /*px4当前状态*/
    mavros_msgs::CommandBool arm_cmd; 
    mavros_msgs::SetMode offb_set_mode;     /*设置px4模式*/

    geometry_msgs::Pose homePose;
    geometry_msgs::Pose landPose;
    // mavros control
    bool received_homePose_flag = false;        // 设置 起飞点 local home 

    // ctrl 状态机  ------------------------------------------------------
    // mavcomm_changeState
    ros::Subscriber ChangeState_sub;
    void ChangeState_cb(const mavcomm_msgs::ChangeState::ConstPtr& msg);
    mavcomm_msgs::ChangeState msg_changeState;

    ros::Publisher mavCurrentState_pub;         // 告知其他 程序 无人机状态机
    std_msgs::UInt8 msg_state_machine;

    // To mavros ------------------------------------------------------
    // PID 位置环 期望位置指令
    void targetPosition_cb(const geometry_msgs::Point::ConstPtr& msg);
    void targetPose_cb(const geometry_msgs::Pose::ConstPtr& msg);

    ros::Subscriber targetPosition_sub;
    ros::Subscriber targetPose_sub;
    geometry_msgs::Pose targetPose;            // 无人机 位置控制 期望的位置
    
    // 速度环 期望速度指令 由其他环节直接计算出 期望速度
    void targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    ros::Subscriber targetVel_sub;
    geometry_msgs::TwistStamped targetVel;    // 无人机 速度控制 期望的速度

    // To mavros
    ros::Publisher setPosition_pub;             // 设置位置
    geometry_msgs::PoseStamped setPosition;     // 位置控制指令

    ros::Publisher setVelocity_pub;             // 设置速度
    geometry_msgs::TwistStamped setVelocity;    // 速度控制指令

    //  Pos PID cal 

    // control
    void pidVelocityControl();
    void positionControl();

    double init_takeoff_z;  //初始起飞高度

    //幅值限制
    float maxVelocity_x;
    float maxVelocity_y;
    float maxVelocity_z;
    float maxVelocity_yaw;

    // PID参数
    struct pid {
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;

        double err = 0.0;
        double err_last = 0.0;
        double err_add = 0.0;
    };
    pid pid_x, pid_y, pid_z, pid_yaw;

    //  uav 状态机
    //  无人机状态 changestate /uav_ctrl.cpp#enum FlightState
    //  需要同步修改 px4_ctrl.cpp & console_widget.hpp/cpp & state_single_widget.hpp
    enum FlightState {
    UNINIT,                   // 未知状态
    IDLE,                     // 什么都不干 地面待机状态
    TAKING_OFF,               // 当前 loc 点起飞
    LANDING,                  // 当前 loc 点降落
    POS_EXECUTION,            // 位置控制
    Vel_EXECUTION,           // 速度控制
    WAITING_FOR_HOME_POSE,    // 此和下面的状态 无法通过地面站切换到
    TAKEOFF_FAIL,
    LANDED,
    } node_state, node_state_last;  //状态机 当前状态 & 上一个状态

};


/* 构造函数 初始化参数 */
uavControl::uavControl( 
    const ros::NodeHandle &nh_, 
    const ros::NodeHandle &nh_private_ )
:nh(nh_),
 nh_private(nh_private_),
    tp_nh("~"),             // param    /uav_mission/xxx
    mavcomm_nh("mavcomm"),  // mavcomm  /mavcomm/XXX   pub&sub
    mavros_nh("mavros"),    // mavros   /mavros/XXX
    desired_nh("desired"),  // desired  /desired/XXX
    gen_nh("gen")           // gen      /gen/XXX 
{
    rate = new ros::Rate( 30.0 );

    // load pid param
    tp_nh.param<double>("Kp_x", pid_x.p, 2.0);
    tp_nh.param<double>("Ki_x", pid_x.i, 0.0);
    tp_nh.param<double>("Kd_x", pid_x.d, 1.5);

    tp_nh.param<double>("Kp_y", pid_y.p, 2.0);
    tp_nh.param<double>("Ki_y", pid_y.i, 0.0);
    tp_nh.param<double>("Kd_y", pid_y.d, 1.5);

    tp_nh.param<double>("Kp_z", pid_z.p, 2.0);
    tp_nh.param<double>("Ki_z", pid_z.i, 0.0);
    tp_nh.param<double>("Kd_z", pid_z.d, 1.2);

    tp_nh.param<double>("Kp_yaw", pid_yaw.p, 2.0);
    tp_nh.param<double>("Ki_yaw", pid_yaw.i, 0.0);    
    tp_nh.param<double>("Kd_yaw", pid_yaw.d, 1.2);

    tp_nh.param<float>("maxVelocity_x", maxVelocity_x, 0.6);
    tp_nh.param<float>("maxVelocity_y", maxVelocity_y, 0.6);
    tp_nh.param<float>("maxVelocity_z", maxVelocity_z, 0.6);
    tp_nh.param<float>("maxVelocity_yaw", maxVelocity_yaw, 1.4);

    tp_nh.param<double>("init_takeoff_z", init_takeoff_z, 0.5);    // 无人机的 起飞高度 

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

    // init 函数初值
    setVelocity.twist.linear.x = 0;
    setVelocity.twist.linear.y = 0;
    setVelocity.twist.linear.z = 0;
    setVelocity.twist.angular.x = 0;
    setVelocity.twist.angular.y = 0;
    setVelocity.twist.angular.z = 0;

    targetVel.twist.linear.x = 0;
    targetVel.twist.linear.y = 0;
    targetVel.twist.linear.z = 0;
    targetVel.twist.angular.x = 0;
    targetVel.twist.angular.y = 0;
    targetVel.twist.angular.z = 0;

    Info_flag = 1;

    received_homePose_flag = false;
    offb_set_mode.request.custom_mode = "OFFBOARD"; 

    // ROS sub pub
    // mavros uav state & control
    state_sub = mavros_nh.subscribe<mavros_msgs::State>("state", 10, &uavControl::state_cb,this);
    /*Change Arming status. */
    arming_client = mavros_nh.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
    /*Set FCU operation mode*/
    set_mode_client = mavros_nh.serviceClient<mavros_msgs::SetMode>("set_mode");

    // mavros 位置 速度 state
    /*Local position from FCU. ENU坐标系(惯性系)*/ 
    currentPose_sub = mavros_nh.subscribe<geometry_msgs::PoseStamped>("local_position/pose", 10,&uavControl::currentPose_cb,this); 
    currentVelocity_sub = mavros_nh.subscribe<geometry_msgs::TwistStamped>("local_position/velocity_local", 10, &uavControl::currentVelocity_cb,this);

    // 控制指令
    /*Local frame setpoint position. ENU坐标系(惯性系)*/ 
    setVelocity_pub = mavros_nh.advertise<geometry_msgs::TwistStamped>("setpoint_velocity/cmd_vel",5); 
    setPosition_pub = mavros_nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local",5);

    
    /*订阅位置 设置消息*/
    // 无人机位置控制 [期望的位置]
    targetPosition_sub = desired_nh.subscribe<geometry_msgs::Point>("setTarget_position", 5, &uavControl::targetPosition_cb,this);
    targetPose_sub = desired_nh.subscribe<geometry_msgs::Pose>("setTarget_pose", 5, &uavControl::targetPose_cb,this);
    targetVel_sub = desired_nh.subscribe<geometry_msgs::TwistStamped>("setVel", 5, &uavControl::targetVel_cb,this);

    // mavcomm_changestate - 通过 widget_console 直接切换飞机状态 
    ChangeState_sub = mavcomm_nh.subscribe<mavcomm_msgs::ChangeState>("receive/changestate", 10, &uavControl::ChangeState_cb,this);

    // 发布无人机状态 待其他程序调用
    mavCurrentState_pub = gen_nh.advertise<std_msgs::UInt8>("ctrl/uav_state_machine", 1);

}

uavControl::~uavControl()
{
    delete rate;
}

//-----------------------------------------------------------------------------------------
//			回调函数
//-----------------------------------------------------------------------------------------

void uavControl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 无人机位置 local => 初始点设置为 HomePose
void uavControl::currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose = msg->pose;

    // 初始点设置为 homePose
    if(!received_homePose_flag) 
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(currentPose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

        homePose = msg->pose;
        homePose.orientation.w = yaw;

        std::cout << "receive home pose : " << homePose << std::endl;
        
        received_homePose_flag = true;
    }
}

// 无人机当前速度 local
void uavControl::currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{    
    currentVelocity.twist = msg->twist;
}

/*处理收到的位置设置消息*/
void uavControl::targetPosition_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    targetPose.position = *msg;
}

void uavControl::targetPose_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    targetPose = *msg;
}

void uavControl::targetVel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    targetVel = *msg;
}

// 切换状态
void uavControl::ChangeState_cb(const mavcomm_msgs::ChangeState::ConstPtr &msg)
{
    msg_changeState = *msg;
    node_state = static_cast<FlightState>(msg_changeState.node_state);

    ROS_INFO_STREAM( " Change Node_state to " << node_state );
}

//-----------------------------------------------------------------------------------------
//			无人机 状态机
//-----------------------------------------------------------------------------------------
void uavControl::start()
{
    ros::Time last_time = ros::Time::now();

    node_state = WAITING_FOR_HOME_POSE;
    node_state_last = WAITING_FOR_HOME_POSE;

    tf::Quaternion quat;
    double roll, pitch, yaw;

    while(ros::ok()) {

        switch (node_state) {

            case WAITING_FOR_HOME_POSE:
                if (node_state_last != node_state) {
                    node_state_last = node_state;
                    Info_flag = 1;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = WAITING_FOR_HOME_POSE " );
                }

                if(!received_homePose_flag) {
                    // if(ros::Time::now() - last_time > ros::Duration(1.5))
                    if (Info_flag)
                    {
                        last_time = ros::Time::now();
                        ROS_INFO_STREAM( " Waiting for Home Pos Set ..." );
                        Info_flag = 0;
                    }
                } else {
                    node_state = IDLE;
                    ROS_INFO_STREAM( " Home Pos already set " );
                }

            break;

            case TAKING_OFF:
                if (!received_homePose_flag) {
                    node_state = WAITING_FOR_HOME_POSE;
                    break;
                }

                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = TAKING_OFF " );

                    homePose = currentPose;
                    targetPose = homePose;
                    targetPose.position.z = homePose.position.z + init_takeoff_z;
                    tf::quaternionMsgToTF(homePose.orientation, quat);
                    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
                    targetPose.orientation.w = yaw;
                    // received_homePose_flag = false;
                }

                if( takeoff() )
                    node_state = POS_EXECUTION;
                else
                    node_state = TAKEOFF_FAIL;
            break;

            case TAKEOFF_FAIL:
                if (node_state_last != node_state) {
                    node_state_last = node_state;
                    Info_flag = 1;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = TAKEOFF_FAIL " );
                }

                // if (ros::Time::now() - last_time > ros::Duration(1.5)) 
                if (Info_flag)
                {
                    last_time = ros::Time::now();
                    ROS_INFO(" Offboard enabled! Vehicle armed failed! ");
                    Info_flag = 0;
                }
            break;
            
            case LANDING:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = LANDING " );

                    landPose = currentPose;
                    landPose.position.z = homePose.position.z;                  // 0.1
                    tf::quaternionMsgToTF(currentPose.orientation, quat);
                    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);               //进行转换
                    landPose.orientation.w = yaw; 

                    targetPose = landPose;
                }

                if ( autoland() )
                    node_state = LANDED;

                pidVelocityControl();
                // ros::spinOnce();
            break;

            case LANDED:
                if (node_state_last != node_state) {
                    node_state_last = node_state;
                    Info_flag = 1;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = LANDED " );
                }

                if ( disarm() ) {
                    node_state = IDLE;

                    if (Info_flag)
                    {
                        ROS_INFO(" Landed  and disarm ... "); 
                        Info_flag = 0;
                    }               
                
                }
            break;

            case IDLE:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = IDLE " );
                }
                // 

            break;

            case POS_EXECUTION:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = POS_EXECUTION " );
                }
                // 速度PID控制
                pidVelocityControl();
                // 位置控制
                // positionControl();
            break;

            case Vel_EXECUTION:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = Vel_EXECUTION " );
                }
                
                // 速度PID控制
                // 限制幅值
                setVelocity.twist.linear.x = satfunc(targetVel.twist.linear.x , maxVelocity_x);
                setVelocity.twist.linear.y = satfunc(targetVel.twist.linear.y , maxVelocity_y);
                setVelocity.twist.linear.z = satfunc(targetVel.twist.linear.z , maxVelocity_z);
                // yaw
                setVelocity.twist.angular.z = satfunc(targetVel.twist.angular.z , maxVelocity_yaw);

                setVelocity_pub.publish(setVelocity);
            break;

            default:
                if (node_state_last != node_state) {
                    node_state_last = node_state;

                    msg_state_machine.data = node_state & 0xFF;
                    mavCurrentState_pub.publish(msg_state_machine);
                    ROS_INFO_STREAM( " Current node_state = default " );
                }

            break;
        }

        ros::spinOnce();
        rate->sleep();
    }
}

void uavControl::pidVelocityControl()
{
    pid_x.err = targetPose.position.x - currentPose.position.x;
    pid_y.err = targetPose.position.y - currentPose.position.y;
    pid_z.err = targetPose.position.z - currentPose.position.z;
    
    setVelocity.twist.linear.x = pid_x.p * pid_x.err + pid_x.d * (0 - currentVelocity.twist.linear.x);
    setVelocity.twist.linear.y = pid_y.p * pid_y.err + pid_y.d * (0 - currentVelocity.twist.linear.y);
    setVelocity.twist.linear.z = pid_z.p * pid_z.err + pid_z.d * (0 - currentVelocity.twist.linear.z);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(currentPose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

    double target_yaw = targetPose.orientation.w;
    pid_yaw.err = target_yaw - yaw;

    if ( abs(pid_yaw.err) > pi ) {
        // std::cout << "pid_yaw.err  ------------------------- " << pid_yaw.err << std::endl;
        if(pid_yaw.err > 0)
            pid_yaw.err = pid_yaw.err - 2*pi;  
        else
            pid_yaw.err = pid_yaw.err + 2*pi;          
    }
    // std::cout << "pid_yaw.err  " << pid_yaw.err << "   " << yaw << "  " << target_yaw << std::endl;
    
    setVelocity.twist.angular.z = pid_yaw.p * pid_yaw.err + pid_yaw.i * pid_yaw.err_add + pid_yaw.d * (0 - currentVelocity.twist.angular.z);
    pid_yaw.err_add = pid_yaw.err_add + pid_yaw.err;  
    //限制幅值
    setVelocity.twist.linear.x = satfunc(setVelocity.twist.linear.x , maxVelocity_x);
    setVelocity.twist.linear.y = satfunc(setVelocity.twist.linear.y , maxVelocity_y);
    setVelocity.twist.linear.z = satfunc(setVelocity.twist.linear.z , maxVelocity_z);

    setVelocity.twist.angular.z = satfunc(setVelocity.twist.angular.z , maxVelocity_yaw);

    setVelocity_pub.publish(setVelocity);
    // std::cout << "out: " << setVelocity.twist.linear.x << "\t" << setVelocity.twist.linear.y << "\t" << setVelocity.twist.linear.z << "\t" << setVelocity.twist.angular.z  << std::endl;
}


void uavControl::positionControl()
{
    //转换为仿真世界参考坐标系 无人机位置参考原点在（0 -3 0）
    //只有在仿真环境中用得到，真实测试不需要    
    setPosition.pose.position.x = targetPose.position.x;
    setPosition.pose.position.y = targetPose.position.y;
    setPosition.pose.position.z = targetPose.position.z;

    // setPosition.pose.orientation = targetPose.orientation;
    // setPosition.pose.position = targetPose.position;

    setPosition_pub.publish(setPosition);    
}


//-----------------------------------------------------------------------------------------
//			px4 state check help ()
//-----------------------------------------------------------------------------------------

// 设置OFFBOARD 模式
bool uavControl::offboard()
{
    offb_set_mode.request.custom_mode = "OFFBOARD";

    setPosition.header.stamp = ros::Time::now();
    setPosition.pose.position.x = homePose.position.x;
    setPosition.pose.position.y = homePose.position.y;
    setPosition.pose.position.z = homePose.position.z + init_takeoff_z;

    int i = 20;

    ROS_INFO(" Waring!!! Changed to offboard control!!! ");

    for(i = 100; ros::ok() && i > 0; --i)
    {
        setPosition_pub.publish(setPosition);

        /* 当前不是 OFFBOARD 模式 */
        if( current_state.mode != "OFFBOARD" ) 
        {   /*设置 OFFBOARD 模式*/
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
            {   ROS_INFO(" Offboard enabled ");
            }
        } 
        else 
        {   /* 当前未解锁 */
            if( !current_state.armed ) 
            {
                if( this->arm() ) 
                {   ROS_INFO(" Vehicle Armed ");   
                }
            }
        }

        /*解锁成功提前退出*/
        if (current_state.mode == "OFFBOARD" && current_state.armed)
            return true;

        ros::spinOnce();
        rate->sleep();
    }
    return false;
}

bool uavControl::takeoff()
{
    if ( !this->offboard() ) 
    {   ROS_INFO(" Offboard enabled! Vehicle armed failed! ");
        return false;
    }
    return true;
}

bool uavControl::autoland()
{
  if (currentPose.position.z - homePose.position.z <= 0.3) 
  {
    if (current_state.mode != "AUTO.LAND") 
    {
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
        {
            ROS_INFO(" AUTO.LAND enabled ");
            return true;
        }
    }
  }
  else 
  {
    targetPose = landPose;
  }

  return false;
}

void uavControl::waitConnect()
{
    ROS_INFO("Wait for FCU connection...  Please start mavros");
    while(ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate->sleep();
    }
    ROS_INFO("FCU connection ok!");
}

//解锁
bool uavControl::arm()
{
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        return true;
    } 
    else
        return false;
}

//上锁
bool uavControl::disarm()
{
    arm_cmd.request.value = false;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        return true;
    } 
    else
        return false;
}


//-----------------------------------------------------------------------------------------
//			Main()
//-----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_ctrl");
    
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    uavControl uav(nh_,nh_private_);

    uav.waitConnect();
    uav.start();

    return 0;
}


//-----------------------------------------------------------------------------------------
//			Help Func()
//-----------------------------------------------------------------------------------------

float uavControl::satfunc(float data, float Max)
{
    if( abs(data)>Max )
        return (data>0)?Max:-Max;
    else
        return data;
}
