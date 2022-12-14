#include <ros/ros.h>  
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tf.h>

#define pi 3.1415926

geometry_msgs::PoseStamped cameraPose;
geometry_msgs::PoseStamped px4Pose;
geometry_msgs::TwistStamped px4Twist;
bool odomRec_flag = false;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    tf::Quaternion q1,q2,q3;
    q1.setW( cos(pi/4) );
    q1.setX( 0);
    q1.setY(sin(pi/4));
    q1.setZ( 0);

    q2.setW( cos(pi/4) );
    q2.setX( 0);
    q2.setY( 0);
    q2.setZ(-sin(pi/4));

    cameraPose.header = msg->header;

    cameraPose.pose.position.x = msg->pose.position.x + 0;
    cameraPose.pose.position.y = msg->pose.position.y + 0;
    cameraPose.pose.position.z = msg->pose.position.z + 0;

    q3.setW(msg->pose.orientation.w);
    q3.setX(msg->pose.orientation.x);
    q3.setY(msg->pose.orientation.y);
    q3.setZ(msg->pose.orientation.z);

    q3 = q3 * q1; //用于旋转的四元数一定要乘在右边！！！！
    q3 = q3 * q2;

    cameraPose.pose.orientation.w = q3.getW();
    cameraPose.pose.orientation.x = q3.getX();
    cameraPose.pose.orientation.y = q3.getY();
    cameraPose.pose.orientation.z = q3.getZ();

    // px4Pose.pose = msg->pose;
    // px4Twist.twist = msg->twist;

    odomRec_flag = true;
}

int main(int argc, char** argv)  
{  
    ros::init(argc, argv,"odom2camerapose");
    ros::NodeHandle nh;//创建句柄
    ros::Rate loop_rate(30);

    ros::Subscriber odom_sub;
    ros::Publisher cameraPose_pub;
    ros::Publisher px4Pose_pub;
    ros::Publisher px4Twist_pub;

    // odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10, pose_cb);

    cameraPose_pub = nh.advertise<geometry_msgs::PoseStamped>("/d534i/camerapose", 1);
    // px4Pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/px4/gazebo/pose", 1);
    // px4Twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/px4/gazebo/twist", 1);
    
    while (ros::ok()) {
        if(odomRec_flag) {
            cameraPose_pub.publish(cameraPose);
            // px4Pose_pub.publish(px4Pose);
            // px4Twist_pub.publish(px4Twist);
        }    

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
