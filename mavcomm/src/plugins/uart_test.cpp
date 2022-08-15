// 测试 mavcomm 发送 频率 & 丢包 & 通信时延 等 

#define ID_GCS 100
#define ID_ALL 99 

#include <ros/ros.h>
#include <signal.h>
#include <bitset>
#include <string.h>

// sub & pub
#include <std_msgs/UInt8.h>
#include <mavcomm_msgs/Heartbeat.h>
#include <mavcomm_msgs/local_pos_enu.h>
#include <mavcomm_msgs/global_pos_int.h>

int system_id;		// sysid 发送端编号	->发送时  发送端编号 / 接收时 发送端编号
int companion_id;	// compid 接收端编号 ->发送时  接收端编号 / 接收时 接收端编号
int my_id;			// my_id 本机编号 [100-地面站] [99-所有无人机]

// help func
uint64_t get_time_usec();	// Time 获取系统时间
void quit_handler(int sig);	// QuitSignalHandler  Called when you press Ctrl-C
void quaternion_2_euler(float quat[4], float angle[3]);

// 
ros::Subscriber sub_flag;
int flag;

// 
ros::Publisher 			pub_loc_pos;
ros::Subscriber 		sub_loc_pos;
mavcomm_msgs::local_pos_enu msg_pub_loc_pos_0; 
mavcomm_msgs::local_pos_enu msg_sub_loc_pos_0; 

// 
ros::Publisher 			pub_loc_pos1;
ros::Subscriber 		sub_loc_pos1;
mavcomm_msgs::local_pos_enu msg_pub_loc_pos_1; 
mavcomm_msgs::local_pos_enu msg_sub_loc_pos_1; 

mavcomm_msgs::local_pos_enu store_msg_pub_loc_pos_0;
bool flag_rece_0;	//是否收到的标志
int rec_true_0;
int rec_total_0;

mavcomm_msgs::local_pos_enu store_msg_pub_loc_pos_1;
bool flag_rece_1;	//是否收到的标志
int rec_true_1;
int rec_total_1;

//-----------------------------------------------------------------------------------------
// 回调
//-----------------------------------------------------------------------------------------
void cd_test_flag(const std_msgs::UInt8::ConstPtr& rmsg)
{	
	flag = (int) rmsg->data;
}

void cb_sub_loc_pos(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{
	msg_sub_loc_pos_0 = *rmsg;
    rec_total_0 = rec_total_0 + 1;

	if (msg_sub_loc_pos_0.x == store_msg_pub_loc_pos_0.x)
	{	// 收到了
		flag_rece_0 = 1;
		rec_true_0 = rec_true_0 + 1;

		double time1 = msg_sub_loc_pos_0.header.stamp.toSec();
		double time = store_msg_pub_loc_pos_0.header.stamp.toSec();
		double time_now = ros::Time::now().toSec();
		
		std::cout << std::fixed << std::setprecision(0) << "[5->1] No=" << msg_sub_loc_pos_0.x;
		std::cout << ", t/total = " << rec_true_0 << "/" << rec_total_0;
		std::cout << std::fixed << std::setprecision(5) << ", time = " << time_now - time << " / " << time_now - time1 << std::endl;
	}
	else if ( msg_sub_loc_pos_0.x > store_msg_pub_loc_pos_0.x )
	{
		flag_rece_0 = 1;

		double time1 = msg_sub_loc_pos_0.header.stamp.toSec();
		double time = store_msg_pub_loc_pos_0.header.stamp.toSec();
		double time_now = ros::Time::now().toSec();
		
		std::cout << std::fixed << std::setprecision(0) << "[5  1] No=" << msg_sub_loc_pos_0.x;
		std::cout << ", t/total = " << rec_true_0 << "/" << rec_total_0;
		std::cout << std::fixed << std::setprecision(5) << ", time = " << time_now - time << " / " << time_now - time1 << std::endl;

	}


}

void cb_sub_loc_pos1(const mavcomm_msgs::local_pos_enu::ConstPtr& rmsg)
{		
	msg_sub_loc_pos_1 = *rmsg;
    rec_total_1 = rec_total_1 + 1;

	if (msg_sub_loc_pos_1.x == store_msg_pub_loc_pos_1.x)
	{	// 收到了
		flag_rece_1 = 1;
		rec_true_1 = rec_true_1 + 1;

		double time1 = msg_sub_loc_pos_1.header.stamp.toSec();
		double time = store_msg_pub_loc_pos_1.header.stamp.toSec();
		double time_now = ros::Time::now().toSec();
		
		std::cout << std::fixed << std::setprecision(0) << "[1->5] No=" << msg_sub_loc_pos_1.x;
		std::cout << ", t/total = " << rec_true_1 << "/" << rec_total_1;
		std::cout << std::fixed << std::setprecision(5) << ", time = " << time_now - time << " / " << time_now - time1 << std::endl;
	}
	else if ( msg_sub_loc_pos_1.x < store_msg_pub_loc_pos_1.x )
	{
		flag_rece_1 = 1;

		double time1 = msg_sub_loc_pos_1.header.stamp.toSec();
		double time = store_msg_pub_loc_pos_1.header.stamp.toSec();
		double time_now = ros::Time::now().toSec();
		
		std::cout << std::fixed << std::setprecision(0) << "[1  5] No=" << msg_sub_loc_pos_1.x;
		std::cout << ", t/total = " << rec_true_1 << "/" << rec_total_1;
		std::cout << std::fixed << std::setprecision(5) << ", time = " << time_now - time << " / " << time_now - time1 << std::endl;

	}
}

//-----------------------------------------------------------------------------------------
// Main()
//-----------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavcomm_uart_test");

	ros::NodeHandle nh("~");

	sub_flag = nh.subscribe<std_msgs::UInt8>("/test_mavcomm/flag", 1, cd_test_flag);

	pub_loc_pos = nh.advertise<mavcomm_msgs::local_pos_enu>("/uav0/mavcomm/send/loc_pos_enu", 5); 
	sub_loc_pos = nh.subscribe<mavcomm_msgs::local_pos_enu>("/uav0/mavcomm/receive/loc_pos_enu", 5, cb_sub_loc_pos);
	
    pub_loc_pos1 = nh.advertise<mavcomm_msgs::local_pos_enu>("/uav1/mavcomm/send/loc_pos_enu", 5); 
	sub_loc_pos1 = nh.subscribe<mavcomm_msgs::local_pos_enu>("/uav1/mavcomm/receive/loc_pos_enu", 5, cb_sub_loc_pos1);

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	ros::Rate loop_rate(10);

	// init data
	msg_pub_loc_pos_0.sysid = 1;
	msg_pub_loc_pos_0.compid = 5;
	msg_pub_loc_pos_0.x = 1.0;
	msg_pub_loc_pos_0.y = 1.0;
	msg_pub_loc_pos_0.z = 1.0;
	msg_pub_loc_pos_0.yaw = 1.0;
	msg_pub_loc_pos_0.flag = 1;

	msg_pub_loc_pos_1.sysid = 5;
	msg_pub_loc_pos_1.compid = 1;
	msg_pub_loc_pos_1.x = -1.0;
	msg_pub_loc_pos_1.y = -1.0;
	msg_pub_loc_pos_1.z = -1.0;
	msg_pub_loc_pos_1.yaw = -1.0;
	msg_pub_loc_pos_1.flag = 1;

	flag_rece_0 = 1;
	rec_true_0 = 0;
    rec_total_0 = 0;

	flag_rece_1 = 1;
	rec_true_1 = 0;
    rec_total_1 = 0;


	ros::Duration ( 0.5 );

	while (	ros::ok() ) 
	{	
		ros::spinOnce();

		if ( flag == 1 )
		{
			msg_pub_loc_pos_0.header.stamp = ros::Time::now();
			pub_loc_pos.publish(msg_pub_loc_pos_0);

			if (flag_rece_1)
			{
				store_msg_pub_loc_pos_1 = msg_pub_loc_pos_0;
				flag_rece_1 = 0;
			}

			msg_pub_loc_pos_0.x = msg_pub_loc_pos_0.x + 1;
			msg_pub_loc_pos_0.y = msg_pub_loc_pos_0.y + 1;
			msg_pub_loc_pos_0.z = msg_pub_loc_pos_0.z + 1;
			msg_pub_loc_pos_0.yaw = msg_pub_loc_pos_0.yaw + 1;
			msg_pub_loc_pos_0.flag = 1;


			msg_pub_loc_pos_1.header.stamp = ros::Time::now();
			pub_loc_pos1.publish(msg_pub_loc_pos_1);

			if (flag_rece_0)
			{
				store_msg_pub_loc_pos_0 = msg_pub_loc_pos_1;
				flag_rece_0 = 0;
			}

			msg_pub_loc_pos_1.x = msg_pub_loc_pos_1.x - 1;
			msg_pub_loc_pos_1.y = msg_pub_loc_pos_1.y - 1;
			msg_pub_loc_pos_1.z = msg_pub_loc_pos_1.z - 1;
			msg_pub_loc_pos_1.yaw = msg_pub_loc_pos_1.yaw - 1;
			msg_pub_loc_pos_1.flag = 1;

		}
		
		loop_rate.sleep();
	}

	return 0;
}



//-----------------------------------------------------------------------------------------
// 工具类
//-----------------------------------------------------------------------------------------

// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	exit(0);// end program here
}

uint64_t get_time_usec()	// Time 获取系统时间
{	
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

/* 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * q0 q1 q2 q3
 * w x y z
 *关于四元数转欧拉角
 * phi_val = Type(atan2(dcm(2, 1), dcm(2, 2)));
 * theta_val = Type(asin(-dcm(2, 0)));
 * psi_val = Type(atan2(dcm(1, 0), dcm(0, 0))); */
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}
