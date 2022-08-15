// Function: /1.启动 roslaunch /2.关闭 rosnode
// 订阅 RosTopic - mavcomm/receive/console - console.msg
// TODO
// 1. 指令读取 本地文本 避免每次增补都得编译
// cjson / js / yaml / 其他 ?

#define use_mavlink_gen_forTest 0
#define ID_GCS 100
#define ID_ALL 99 

#include <ros/ros.h>
#include <signal.h>

// ROS msg
#include <mavcomm_msgs/Console.h>
#include <mavcomm_msgs/Console_monitor.h>

int Flag_1ShowRn;	// 标志位 1将读取的 Rn 显示在屏幕上

int system_id;		// sysid 发送端编号	->发送时  发送端编号	/ 接收时 发送端编号
int companion_id;	// compid 接收端编号	->发送时  接收端编号	/ 接收时 接收端编号
int my_id;			// my_id 本机编号 	[ 100-地面站 ] 	[99-所有无人机]

uint64_t get_time_usec();	// Time 获取系统时间
void quit_handler(int sig);	// QuitSignalHandler  Called when you press Ctrl-C

ros::Subscriber 		mavlink_sub;
mavcomm_msgs::Console 	msg_Console; 

ros::Publisher					pub_console_monitor;
mavcomm_msgs::Console_monitor 	msg_console_monitor;

enum Case_console {
mavros,
uav_ctrl,
vision_pose,
usb_cam,
csi_cam,
realsense_cam,
web_video_server,
vins,
fast_planner,
yolov3,
};

// 功能模块运行状态反馈 rosnode list/kill
int ros_node_num = 9;					// 与下面数组长度有关
static const char * ros_node_str[] =	// 注意无空格
{	// 0
	"/mavros", 
	"/uav_ctrl", 
	"/vision_pose", 
	"/usb_cam", 
	// 4
	"/csi_cam_0", 
	"/camera/realsense2_camera", 
	"/web_video_server", 
	"/vins_fusion",
	// 8 
	"/fast_planner_node", 
	"/darknet_ros", 
	"/", 
	"/", 
	// 12
	"/", 
	"/", 
	"/",
	"/"
};

void func_mavros();
void func_uav_ctrl();
void func_vision_pose();
void func_usb_cam();
void func_csi_cam();
void func_realsense_cam();
void func_web_video_server();
void func_vins();
void func_fast_planner();
void func_yolov3();
//-----------------------------------------------------------------------------------------
//			ros_cb() 回调函数
//-----------------------------------------------------------------------------------------

void mavlink_sub_cb(const mavcomm_msgs::Console::ConstPtr& rmsg)
{	
	msg_Console = *rmsg;
	// msg_Console
	// flag   	/1启动launch   /3关闭node 
	// command 	/指令编号
	// type1 type2 /对应编号 的 具体类型

	switch (msg_Console.command) 
	{
		case mavros: //mavros
			func_mavros();
		break;

		case uav_ctrl: //mavros
			func_uav_ctrl();
		break;

		case vision_pose: //vision_pose
			func_vision_pose();
		break;

		case usb_cam: //usb_cam
			func_usb_cam();
		break;

		case csi_cam: //csi_cam_0
			func_csi_cam();
		break;

		case realsense_cam: //realsense_cam
			func_realsense_cam();
		break;

		case web_video_server: //web_video_server
			func_web_video_server();
		break;

		case vins:	//vins_fusion
			func_vins();
		break;

		case fast_planner:
			func_fast_planner();
		break;

		case yolov3:
			func_yolov3();
		break;

		default:
			if (Flag_1ShowRn) {
				ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command );
			}
		break;
	}

}


//-----------------------------------------------------------------------------------------
//			Main()
//-----------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavconsole");

    ros::NodeHandle nh("~"); // 用于发布订阅绝对话题 + roslaunch param get
	
	// ROS mavlink bridge
	mavlink_sub = nh.subscribe<mavcomm_msgs::Console>("/mavcomm/receive/console", 10, mavlink_sub_cb);
	pub_console_monitor = nh.advertise<mavcomm_msgs::Console_monitor>("/mavcomm/send/console_monitor", 5);

	//Responds to early exits signaled with Ctrl-C. 
	signal(SIGINT, quit_handler);

	nh.param<int>("my_id", my_id, 100);
	nh.param<int>("Flag_1ShowRn", Flag_1ShowRn, 0);  // 1将接收到的字节打印到屏幕上

	std::cout << "mavconsole/my_id = " << my_id<<std::endl;
	std::cout << "mavconsole/Flag_1ShowRn = " << Flag_1ShowRn<<std::endl;

	ros::Rate loop_rate(10);

	ros::Duration ( 0.5 );

	// ros::spin();
	int count_n = 0;

	while (	ros::ok() ) {	
		ros::spinOnce();

		if (count_n==9) 
		{	
			// 功能模块运行状态反馈
			count_n = 0;

			ros::V_string v_nodes;
    		ros::master::getNodes(v_nodes);

			uint8_t param1, param2, param3, param4;
			param1 = 0; 
			param2 = 0;
			param3 = 0; 
			param4 = 0;
    		
			for (auto elem : v_nodes) 
			{
        		// std::cout << elem << " "; // << std::endl;
				// 将 elem 转为 对应数字
				int num_i = 0;
				int num_j = 0;
				bool flag = 1;
				while (flag) 
				{
					if ( num_i == ros_node_num || ros_node_str[num_i] == elem ) 
					{	flag = false;	} 
					else 
					{	num_i++;	}
				}

				if (num_i == ros_node_num)
				{	}
				else 
				{
					// std::cout << "no = " << num_i << " "; 
					num_j = (int) (num_i/8);
					num_i = num_i - 8*num_j;
					// std::cout << "[j,i]=" << num_j << " " << num_i ;

					switch (num_j)
					{
						case 0:
							param1 |= (1<<num_i);
							// std::cout << " " << (int) param1 << std::endl;
						break;

						case 1:
							param2 |= (1<<num_i);
							// std::cout << " " << (int) param2 << std::endl;
						break;

						case 2:
							param3 |= (1<<num_i);
							// std::cout << " " << (int) param3 << std::endl;
						break;

						case 3:
							param4 |= (1<<num_i);
							// std::cout << " " << (int) param4 << std::endl;
						break;
				
						default:
					
						break;
					}
				}
    		}

			msg_console_monitor.header.stamp = ros::Time::now();
			msg_console_monitor.sysid = my_id;
			msg_console_monitor.compid = ID_GCS;	//	发送给 地面站

			msg_console_monitor.param1 = param1;
			msg_console_monitor.param2 = param2;
			msg_console_monitor.param3 = param3;
			msg_console_monitor.param4 = param4;

			pub_console_monitor.publish(msg_console_monitor);
		}
		else
		{
			count_n++;
		}

		loop_rate.sleep();
	}

	return 0;
}

//-----------------------------------------------------------------------------------------
//			func
//-----------------------------------------------------------------------------------------

//####################################################################################################
void func_mavros()
{
	/*
	- flag == 3		rosnode kill /mavros
	- flag == 1	 
		- type1 = 
		1 ACM0	2 USB0	3 THS0	4 THS1	5 THS2 
		- type2 = 
	    1 57600	2 921600
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /mavros");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_mavros_type1[6] = {
			"NaN",	//0
			"ACM0",	//1
			"USB0",	//2
			"THS0",	//3
			"THS1",	//4
			"THS2"	//5
		};

	    std::string str_mavros_type2[3] = {
			"NaN",	//0
			"57600",	//1
			"921600",	//2
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole mavros_px4.launch";
		temp_string = temp_string + " fcu_url:=/dev/tty" + str_mavros_type1[msg_Console.type1];
		temp_string = temp_string + ":" + str_mavros_type2[msg_Console.type2];
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}


//####################################################################################################
void func_uav_ctrl()
{
	/*
		- flag == 3		rosnode kill /uav_ctrl
		- flag == 1	 
			- type1 = type2 
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /uav_ctrl");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_vspose_type1[1] = {
			"NaN",	//0
		};

	    std::string str_vspose_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole px4_ctrl.launch";
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
		
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}



//####################################################################################################
void func_vision_pose()
{
	/*
		- flag == 3		rosnode kill /vision_pose
		- flag == 1	 
			- type1 = flag_1vrpn_2vio_3both
			1 2 3
			- type2 = vio_odomTopic
		    1 /camera/odom/sample	2 /vins_fusion/odometry
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /vision_pose");
		system("rosnode kill /vrpn_client_node");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_vspose_type1[4] = {
			"NaN",	//0
			"1",	//1
			"2",	//2
			"3",	//3
		};

	    std::string str_vspose_type2[3] = {
			"NaN",	//0
			"/camera/odom/sample",	//1
			"/vins_fusion/odometry",	//2
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole px4_vision_pose.launch";
		temp_string = temp_string + " px4_uav_no:=" + std::to_string(my_id);
		temp_string = temp_string + " flag_1vrpn_2vio_3both:=" + str_vspose_type1[msg_Console.type1];
		temp_string = temp_string + " vio_odomTopic:=" + str_vspose_type2[msg_Console.type2];
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());

		if ( msg_Console.type1==1 || msg_Console.type1==3 )
		{
			temp_string = "gnome-terminal --tab -e 'bash -c \"";
			temp_string = temp_string + "roslaunch mavconsole vrpn_sample.launch";
			temp_string = temp_string + "; exec bash\"'";
			std::cout << "exec = " << temp_string << std::endl;
			system(temp_string.c_str());
		}

	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}




//####################################################################################################
void func_csi_cam()
{
	/*
		- flag == 3		rosnode kill /csi_cam_0
		- flag == 1	 
			- type1 = type2 - width height
			1 1080	2 1920	3 3264
			- type2 = vio_odomTopic
		    1 720	2 1080	3 2464
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /csi_cam_0");
		system("rosnode kill /csi_cam_0/image_proc");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_csi_cam_type1[4] = {
			"NaN",	//0
			"1080",	//1
			"1920",	//2
			"3264",	//3
		};

	    std::string str_csi_cam_type2[4] = {
			"NaN",	//0
			"720",	//1
			"1080",	//2
			"2464",	//3
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole jetson_csi_cam.launch";
		temp_string = temp_string + " width:=" + str_csi_cam_type1[msg_Console.type1];
		temp_string = temp_string + " height:=" + str_csi_cam_type2[msg_Console.type2];
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());

		// 图像畸变校正 需要和校正前的分辨率一样 
		// TODO 不同分辨率下，加载不同校正文件
		// 待验证
		std::string temp_string1;
		temp_string1 = "gnome-terminal --tab -e 'bash -c \"";
		temp_string1 = temp_string + "ROS_NAMESPACE=/csi_cam_0 rosrun image_proc image_proc";
		temp_string1 = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string1 << std::endl;
		system(temp_string1.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}

//####################################################################################################
void func_usb_cam()
{
	/*
		- flag == 3		rosnode kill /usb_cam
		- flag == 1	 
			- type1 = type2 - image_width image_height
			1 640	2 960	3 1440	4 2560
			- type2 = vio_odomTopic
		    1 480	2 720	3 1080	4 1920
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /usb_cam");
		system("rosnode kill /usb_cam/image_proc");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_usb_cam_type1[5] = {
			"NaN",	//0
			"640",	//1
			"960",	//2
			"1440",	//3
			"2560",	//4
		};

	    std::string str_usb_cam_type2[5] = {
			"NaN",	//0
			"480",	//1
			"720",	//2
			"1080",	//3
			"1920",	//4
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole usb_cam.launch";
		temp_string = temp_string + " image_width:=" + str_usb_cam_type1[msg_Console.type1];
		temp_string = temp_string + " image_height:=" + str_usb_cam_type2[msg_Console.type2];
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());

		// 图像畸变校正 需要和校正前的分辨率一样 
		// TODO 不同分辨率下，加载不同校正文件
		std::string temp_string1;
		temp_string1 = "gnome-terminal --tab -e 'bash -c \"";
		temp_string1 = temp_string + "ROS_NAMESPACE=/usb_cam rosrun image_proc image_proc";
		temp_string1 = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string1 << std::endl;
		system(temp_string1.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}

//####################################################################################################
void func_realsense_cam()
{
	/*
		- flag == 3		rosnode kill /
		- flag == 1	 
			- type1 = type2
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /camera/realsense2_camera");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_rs_cam_type1[1] = {
			"NaN",	//0
		};

	    std::string str_rs_cam_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole rs_camera.launch";
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}

//####################################################################################################
void func_web_video_server()
{
	/*
		- flag == 3		rosnode kill /
		- flag == 1	 
			- type1 = type2
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /web_video_server");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_web_video_server_type1[3] = {
			"NaN",			//0
			"192.168.50.",	//1
			"192.168.1.",	//2
		};

	    std::string str_web_video_server_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole web_video_server.launch";
		temp_string = temp_string + " address:=" + str_web_video_server_type1[msg_Console.type1] + std::to_string( 130+(int)msg_Console.compid ); //TODO
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}


//####################################################################################################
void func_vins()
{
	/*
		- flag == 3		rosnode kill /
		- flag == 1	 
			- type1 = type2
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /vins_fusion");
	} 
	else if (msg_Console.flag == 1) {
		std::string str_vins_type1[1] = {
			"NaN",			//0
		};

	    std::string str_vins_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole vins_d435i.launch";
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}

//####################################################################################################
void func_fast_planner()
{
	/*
		- flag == 3		rosnode kill /
		- flag == 1	 
			- type1 = type2
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /fast_planner_node");
		system("rosnode kill /traj_server");
		system("rosnode kill /waypoint_generator");
	}
	else if (msg_Console.flag == 1) {
		std::string str_fast_planner_type1[1] = {
			"NaN",			//0
		};

	    std::string str_fast_planner_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch plan_manage kino_replan.launch";
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());

		std::string temp_string1;
		temp_string1 = "gnome-terminal --tab -e 'bash -c \"";
		temp_string1 = temp_string + "rosrun px4_offb odom2camerapose";
		temp_string1 = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string1 << std::endl;
		system(temp_string1.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}


//####################################################################################################
void func_yolov3()
{
	/*
		- flag == 3		rosnode kill /
		- flag == 1	 
			- type1 = type2
	*/
	if (msg_Console.flag == 3) {
		system("rosnode kill /darknet_ros");
	}
	else if (msg_Console.flag == 1) {
		std::string str_yolov3_type1[4] = {
			"NaN",			//0
			"/csi_cam_0/image_raw",			//1
			"/usb_cam/image_raw",			//1
			"/camera/color/image_raw",			//1
		};

	    std::string str_yolov3_type2[1] = {
			"NaN",	//0
		};
		
		std::string temp_string;
		temp_string = "gnome-terminal --tab -e 'bash -c \"";
		temp_string = temp_string + "roslaunch mavconsole yolo_v3tiny.launch";
		temp_string = temp_string + " image:=" + str_yolov3_type1[msg_Console.type1];
		temp_string = temp_string + "; exec bash\"'";
		std::cout << "exec = " << temp_string << std::endl;
		system(temp_string.c_str());
	} 
	else 
	{	if (Flag_1ShowRn) {	ROS_INFO_STREAM( "UnKnow MavConsole command = " << msg_Console.command << ", flag = " << msg_Console.flag );}
	}
}




//-----------------------------------------------------------------------------------------
//			工具类
//-----------------------------------------------------------------------------------------
uint64_t get_time_usec()	// Time 获取系统时间
{	
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

// QuitSignalHandler  Called when you press Ctrl-C
void quit_handler(int sig)
{	
	printf("\n TERMINATING AT USER REQUEST \n \n");
	exit(0);// end program here
}
