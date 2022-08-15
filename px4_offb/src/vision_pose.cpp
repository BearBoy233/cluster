
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>

using namespace std;

class vision_pose
{
public:
    // vision_pose();
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);

    double pi;
    struct attitude
    {
        double pitch;
        double roll;
        double yaw;
    };
    attitude vrpnAttitude;
    attitude t265Attitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped vrpnPose;
    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped realsenseBridgePose;
    geometry_msgs::PoseStamped initOffsetPose;
    nav_msgs::Odometry t265_Pose;
    std_msgs::Int32 t265_error_data;
 
    bool vrpnPoseRec_flag;
    bool realsenseBridgePoseRec_flag;
    bool initOffsetPoseRec_flag;

    ros::Rate *rate;
    ros::Time start_time;
    double setupTime;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber vrpn_pose_sub;
    ros::Subscriber px4Pose_sub;
    ros::Subscriber realsense_bridge_pose_sub;
    ros::Subscriber t265_pose_sub;
    ros::Publisher vision_pose_pub;
    ros::Publisher t265_error_pub;

    bool error_flag;
    int flag_1vrpn_2vio_3both;
    std::string uavName;
    std::string odomTopic;

    void vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void t265_pose_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void start();

};
/* 构造函数 */
// vision_pose::vision_pose()
vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_)
:nh(nh_), nh_private(nh_private_)
{
    realsenseBridgePose.pose.position.x = 1;
    pi = 3.1415926;
    setupTime = 10;
    rate = new ros::Rate(40.0);

    int my_id;
    
    // 没有传进参数来
    nh_private.param<int>("my_id", my_id, 1);
    nh_private.param<int>("flag_1vrpn_2vio_3both", flag_1vrpn_2vio_3both, 3);
    nh_private.param<std::string>("vio_odomTopic", odomTopic, "/camera/odom/sample");
    //  t265    /camera/odom/sample
    //  vins     /vins_fusion/odometry

    uavName = to_string(my_id);

    std::cout << "vision_pose/px4_uav = " << my_id << std::endl;
	std::cout << "vision_pose/flag_1vrpn_2vio_3both = " << flag_1vrpn_2vio_3both << std::endl;
    std::cout << "vision_pose/vio_odomTopic = " << odomTopic << std::endl;

    vrpn_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/px4_uav" + uavName + "/pose", 1, &vision_pose::vrpn_pose_cb,this);
    // realsense_bridge_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/px4/vision_pose/pose", 1, &vision_pose::realsense_bridge_pose_cb,this);

    t265_pose_sub = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1, &vision_pose::t265_pose_cb,this);

    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10,&vision_pose::px4Pose_cb,this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    t265_error_pub = nh.advertise<std_msgs::Int32>("/VIO_error", 10);

    realsenseBridgePoseRec_flag = false;
    vrpnPoseRec_flag = false;
    initOffsetPoseRec_flag = false;

    t265Attitude.pitch = 0;
    t265Attitude.roll = 0;
    t265Attitude.yaw = 0;

    vrpnAttitude.pitch = 0;
    vrpnAttitude.roll = 0;
    vrpnAttitude.yaw = 0;

    px4Attitude.pitch = 0;
    px4Attitude.roll = 0;
    px4Attitude.yaw = 0;    

    t265_error_data.data = 0;
    error_flag = 1;

}

void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    px4Pose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

/*通过vrpn接受bebop位置消息*/
void vision_pose::vrpn_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*Motive通过 VRPN 发布的位置消息 单位是 米
    */
    vrpnPose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    vrpnAttitude.pitch = pitch * 180 / pi;
    vrpnAttitude.roll = roll * 180 / pi;
    vrpnAttitude.yaw = yaw * 180 / pi;

    vrpnPoseRec_flag = true;
}

void vision_pose::realsense_bridge_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    realsenseBridgePose.pose = msg->pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    t265Attitude.pitch = pitch * 180 / pi;
    t265Attitude.roll = roll * 180 / pi;
    t265Attitude.yaw = yaw * 180 / pi;

    //realsenseBridgePose.header = msg->header;
    realsenseBridgePoseRec_flag = true;
    // cout << "realsenseBridgePose                      :" << realsenseBridgePose.pose.position.x << "   " << realsenseBridgePose.pose.position.y << "   " << realsenseBridgePose.pose.position.z << endl;
}

void vision_pose::t265_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    realsenseBridgePose.pose = msg->pose.pose;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    t265Attitude.pitch = pitch * 180 / pi;
    t265Attitude.roll = roll * 180 / pi;
    t265Attitude.yaw = yaw * 180 / pi;

    //realsenseBridgePose.header = msg->header;
    realsenseBridgePoseRec_flag = true;
    // cout << "realsenseBridgePose                      :" << realsenseBridgePose.pose.position.x << "   " << realsenseBridgePose.pose.position.y << "   " << realsenseBridgePose.pose.position.z << endl;
}

void vision_pose::start()
{
    if (flag_1vrpn_2vio_3both == 2)
    {
        cout << "\033[K" << "\033[31m Send  VIO  Data \033[0m" << endl;
    }
    else if (flag_1vrpn_2vio_3both == 3)
    {    
        cout << "\033[K" << "\033[31m S end  VIO & Vrpn  Data \033[0m" << endl;
    }
    else 
    {
        cout << "\033[K" << "\033[31m Send  Vrpn  Data \033[0m" << endl;
    }

    while(ros::ok())
    {

        if(vrpnPoseRec_flag == false)
	    {
            cout << "\033[K" << "\033[31m vrpn error!!! \033[0m" << endl;
	    }
	    else
        { 
	        cout << "\033[K"  << "\033[32m vrpn ok \033[0m" << endl;
        }
        //if(realsenseBridgePoseRec_flag)

        if (flag_1vrpn_2vio_3both == 2)
        {
            realsenseBridgePoseRec_flag = false;

            realsenseBridgePose.header.stamp = ros::Time::now();
            vision_pose_pub.publish(realsenseBridgePose);
            
            cout << "\033[K"  << "\033[31m VIO MayBe OK, Check Carefully !! \033[0m" << endl;
        }
        else
        {
            realsenseBridgePoseRec_flag = false;

            double errx = realsenseBridgePose.pose.position.x - vrpnPose.pose.position.x;
            double erry = realsenseBridgePose.pose.position.y - vrpnPose.pose.position.y ;
            double errz = realsenseBridgePose.pose.position.z - vrpnPose.pose.position.z ;
            if(abs(errx) > 0.4 || abs(erry) > 0.4 || abs(errz) > 0.4)
            {
		        vrpnPose.header.stamp = ros::Time::now();
                vision_pose_pub.publish(vrpnPose);
                cout << "\033[K"  << "\033[31m VIO Error !!! VIO-Vrpn > 0.4 ---------------------------------------------- \033[0m" << endl;

                if (error_flag)
                    {
                        t265_error_data.data = t265_error_data.data + 1;
                        t265_error_pub.publish(t265_error_data);
                        error_flag = 0;
                    }
            }
            else
            {
                if (error_flag == 0)
                {
                    error_flag = 1;
                }

                if ( flag_1vrpn_2vio_3both == 1 )
                {
		            vrpnPose.header.stamp = ros::Time::now();
                    vision_pose_pub.publish(vrpnPose);
                }
                else    
                {
                    realsenseBridgePose.header.stamp = ros::Time::now();
                    vision_pose_pub.publish(realsenseBridgePose);
                }
                cout << "\033[K"  << "\033[32m VIO ok \033[0m" << endl;
            } 
        }
            cout << "\033[K"  << "       VIO Pose                  vrpnPose               px4Pose" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "x      " << realsenseBridgePose.pose.position.x << "\t\t" << vrpnPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "y      " << realsenseBridgePose.pose.position.y << "\t\t" << vrpnPose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "z      " << realsenseBridgePose.pose.position.z << "\t\t" << vrpnPose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "pitch  " << t265Attitude.pitch << "\t\t" << vrpnAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "roll   " << t265Attitude.roll << "\t\t" << vrpnAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
		        << "\033[K"  << "yaw    " << t265Attitude.yaw << "\t\t" << vrpnAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[10A" << endl;

        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[2J" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_pose");

    // vision_pose vision;
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");
    vision_pose vision(nh_,nh_private_);
    vision.start();

    return 0;
}
