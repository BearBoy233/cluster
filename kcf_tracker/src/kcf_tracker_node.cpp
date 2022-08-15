// TODO 
// KCF 只负责图像解析 => 目标在图像中的信息，后续的追踪策略单独出去。
// KCF 与 本地图片识别,（地面站/无人机传送一张目标图片）

#include "kcftracker.hpp"
#include <unistd.h>
#include <ros/ros.h>  
#include <image_transport/image_transport.h> // 用来在ROS系统中的话题上发布和订阅图象消息
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>     		// cv_bridge 中包含CvBridge库
#include <sensor_msgs/image_encodings.h> 	// ROS图象类型的编码函数

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <mavcomm_msgs/kcf_set_target.h>
#include <mavcomm_msgs/kcf_target_pos.h>

#include <kcf_tracker/track_target.h>
#include <kcf_tracker/set_track_mission.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// #include <>

using namespace std;
using namespace cv;

#define ID_GCS 100
#define ID_ALL 99 

int my_id;
// # my_id 	本设备编号 	/100-地面站 	/99-所有无人机
// # /system_id 发送端编号	/companion_id 接收端编号

// KCF tracker 初始化参数
bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

float response_value =0;
int trackFailCnt=0;

// img
std::string camera_topic;

cv::Mat colorImg ;
bool colorImgRec_Flag = false;
cv::Mat depthImg,depthImg_copy;
bool depthImgRec_Flag = false;

Mat videoFrame;
Rect result;

Rect2d trackerRoi_Rect2d;
bool is_tracking = false;
bool is_recTrackerRoi = false;

// calculate pos
std::string camera_info_topic;
sensor_msgs::CameraInfo msg_CameraInfo;
bool flag_cameramsg = 0;

// uav pose state
geometry_msgs::Pose currentPose;
double current_yaw;

// mavcomm widget 
mavcomm_msgs::kcf_set_target trackerResultRoi;
mavcomm_msgs::kcf_target_pos trackerResultRoi_serial;

// 用于 mavcomm 发布的计数 控制发布频率
int mavcomm_MAX_count = 0;
int mavcomm_curr_count = 0;
bool mavcomm_send_flag = 0;
int mavcomm_state1;
// Mission SET
kcf_tracker::set_track_mission msg_set_track_mission;
// 1任务开始
bool mission_flag_track_on = 0;
int mission_track_mission_type = 0;	// ## 任务类型 1定位 2本机追踪 3TBD
int mission_camera_type = 0;	// ## 相机位置 1向下 2向前TODO 3TBD

// uav mission
kcf_tracker::track_target msg_track_target;


//-----------------------------------------------------------------------------------------
//			ros_cb() 回调函数
//-----------------------------------------------------------------------------------------

// 无人机位置 local
void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose = msg->pose;
	
	// 求 yaw
	tf::Quaternion quat;
    tf::quaternionMsgToTF(currentPose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);	//进行转换

	current_yaw = yaw;
}

// track widget gcs 来的 框选目标的 区域
void trackerRoi_serial_cb(const mavcomm_msgs::kcf_set_target::ConstPtr& msg)
{
	if (msg->state == 0)
	{
		// 停止追踪
		is_tracking = false;
	}
	else if (msg->state == 1)
	{
		// 开始追踪
		// 存储 目标编号 标签
		trackerResultRoi.target_no = msg->target_no; 

		trackerResultRoi_serial.target_no = msg->target_no;
		trackerResultRoi_serial.compid = ID_ALL;		// 接收端 ID
		trackerResultRoi_serial.sysid = my_id;			// 发送端 ID

		// 地面站设定的 初始框位置
		trackerRoi_Rect2d.x = msg->x;
		trackerRoi_Rect2d.y = msg->y;
		trackerRoi_Rect2d.width = msg->width;
		trackerRoi_Rect2d.height = msg->height;
		is_recTrackerRoi = true;
	}
	// TODO mavcomm 数据回应 确保设置上了？

}

// 相机参数
void camera_info_topic_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
	msg_CameraInfo = *msg;
	flag_cameramsg = 1;
}

// opencv 图像处理
void colorImg_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        colorImg = cv_bridge::toCvCopy(msg,"bgr8")->image;
        colorImgRec_Flag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "colorImg_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}

// D4XX 深度图像信息
/*
void depthImg_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        depthImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        depthImgRec_Flag = true ;
    }
    catch (cv_bridge::Exception& e)
    {
        cout << "depthImg_cb could not convert from " << msg->encoding.c_str() << "to 'brg8'." << endl;
    }   
}
*/

// mission 设定
void set_track_mission_cb(const kcf_tracker::set_track_mission::ConstPtr& msg)
{
	msg_set_track_mission = *msg;

	mission_flag_track_on = msg_set_track_mission.flag_track_on;
	mission_track_mission_type = msg_set_track_mission.track_mission_type;	// ## 任务类型 1定位 2本机追踪 3TBD
	mission_camera_type = msg_set_track_mission.camera_type;				// ## 相机位置 1向下 2向前TODO 3TBD

	msg_track_target.track_mission_type = msg_set_track_mission.track_mission_type; // ## 任务类型 1定位 2本机追踪 3TBD
}

//-----------------------------------------------------------------------------------------
//			main() 
//-----------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{	
    ros::init(argc, argv,"kcf_tracker_node");
    ros::NodeHandle nh;		//创建句柄
	ros::NodeHandle nh_private("~");
    image_transport::ImageTransport it(nh);

	// TODO 读取参数
	double response_threshold_ = 0;
	int trackFailCnt_threshold_ = 0;

  	nh_private.param<double>("response_threshold", response_threshold_, 0.3);		// 识别判断 %
  	nh_private.param<int>("trackFailCnt_threshold", trackFailCnt_threshold_, 20);	// 失败计次
	nh_private.param<bool>("MULTISCALE", MULTISCALE, true);

	nh_private.param<std::string>("camera_topic", camera_topic, "/usb_cam/image_raw");				// 图像话题
	nh_private.param<std::string>("camera_info_topic", camera_info_topic, "/usb_cam/camera_info"); 	// 相机参数 读取

	cout << endl;
	cout << "response_threshold_     :  " <<  response_threshold_  << endl;
	cout << "trackFailCnt_threshold_ :  " <<  trackFailCnt_threshold_  << endl;
	cout << "camera_topic :  " <<  camera_topic  << endl;
	cout << "camera_info_topic :  " <<  camera_info_topic  << endl;

	cout << endl;

	// kcf 识别结果图像
	image_transport::Publisher sourceImg_pub = it.advertise("/kcfTracker/image_raw",1);
	// kcf 订阅图像话题
	image_transport::Subscriber colorImg_sub = it.subscribe(camera_topic, 1, colorImg_cb);
	// 读取校准的相机参数
	ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic, 1, camera_info_topic_cb);
	// 无人机当前位置
	ros::Subscriber currentPose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, currentPose_cb); 
   
    // 深度相机有关
	// D4XX 深度图像 (直接测距)
	// image_transport::Subscriber depthImg_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthImg_cb);
	// 深度 KCF 识别 结果
	// ros::Publisher trackerResultRoi_pb = nh.advertise<kcf_tracker::kcfTrackerRoi>("/kcfTracker/result/trackRoi", 1);
	
	// mavcomm widget
	// KCF 图像设置
	ros::Subscriber trackerRoi_sub = nh.subscribe<mavcomm_msgs::kcf_set_target>("/mavcomm/receive/kcf_set_target", 1, trackerRoi_serial_cb);
	// TODO 用于 本飞机自己追踪 / 告知相邻无人机
	// 所以可以减少发布的频率 (暂定 1Hz)
	// 广播 告知地面站不需要应答
	// 但告知 无人机 时 需要！确保指令传输到了 
	// (TODO 如果 多长时间都没能有应答 怎么办! 逻辑待补充)
	// pub 结果 send
	ros::Publisher trackerResultRoi_serial_pub = nh.advertise<mavcomm_msgs::kcf_target_pos>("/mavcomm/send/kcf_target_pos", 1);
	
	// pub kcf results
	// KCF 图像设置
	ros::Subscriber set_track_mission_sub = nh.subscribe<kcf_tracker::set_track_mission>("/mission/kcf/mission_set", 1, set_track_mission_cb);
	// 
	ros::Publisher track_target_pub = nh.advertise<kcf_tracker::track_target>("/mission/kcf/target_pos", 1);

	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	ros::Rate loop_rate(20);	//同步修改 下面的 mavcomm_MAX_count   50
	// Init
	mavcomm_MAX_count = 20; // 当前 1Hz 频率控制 与 loop_rate 有关
	mavcomm_curr_count = 0;
	mavcomm_send_flag = 0;
	mavcomm_state1 = 0;

	while (ros::ok())
	{
		mavcomm_curr_count = mavcomm_curr_count + 1;
		if ( mavcomm_curr_count==mavcomm_MAX_count )
		{
			mavcomm_send_flag = 1;
			mavcomm_curr_count = 0;
		}

		if(is_recTrackerRoi)	// 收到 widget 设定框
		{
			is_recTrackerRoi = false;
			tracker.init(trackerRoi_Rect2d, videoFrame);		// Init tracker追踪
			rectangle(videoFrame, trackerRoi_Rect2d, Scalar(0, 255, 0), 2, 8);
			is_tracking = true;

			mavcomm_curr_count = 0;	// 频率控制
			mavcomm_send_flag = 0;	// 发送标志
			mavcomm_state1 = 0;		// 不同的状态
		}
		/* D4XX 深度相机
		if(depthImgRec_Flag)
		{
			depthImgRec_Flag = false;
			depthImg.copyTo(depthImg_copy);
		}
		*/

		if (colorImgRec_Flag && mission_flag_track_on)	//设置了图像框 & mission中 任务开始
		{
			colorImgRec_Flag = false;
			colorImg.copyTo(videoFrame);
			if (is_tracking == true)
			{
				if (videoFrame.rows == 0 || videoFrame.cols == 0)
				{
					cout << " Error!  VideoFrame is error" << endl;
					
					trackerResultRoi.state = 5;	// state 定义 见 tracker widget
					trackerResultRoi_serial.state = trackerResultRoi.state;	// state 定义 见 tracker widget

					// TODO 
					// 告知 mission 返航!
					{
						msg_track_target.track_state = trackerResultRoi_serial.state; // state 定义 见 tracker widget
						msg_track_target.pos_x = 0;
						msg_track_target.pos_y = 0;
						msg_track_target.pos_z = 0;				// 地面目标 认为高度 = 0
						msg_track_target.pos_yaw = 0;			// TODO
								
						track_target_pub.publish( msg_track_target );
					}
					

					// 告知地面站 
					// 频率控制
					if ( mavcomm_state1!=1 )  // mavcomm_state1 此处为1
					{
						mavcomm_state1 = 1;
						mavcomm_send_flag = 1;
						mavcomm_curr_count = 0;
					}
					// 告知地面站
					if (mavcomm_send_flag)
					{
						trackerResultRoi_serial_pub.publish(trackerResultRoi_serial);
						mavcomm_send_flag = 0;
					}

					continue;
				}
				else
				{
					// KCF 识别结果
					result = tracker.update(videoFrame, response_value);
				
					trackerResultRoi.x = max(result.x, 0);
					trackerResultRoi.y = max(result.y, 0);
					trackerResultRoi.width = result.x + result.width - max(result.x, 0);
					trackerResultRoi.height = result.y + result.height - max(result.y, 0);
					trackerResultRoi.state = 1;		// state 定义 见 tracker widget

					// 在图像中 画出 框
					Point point1 = Point(trackerResultRoi.x + trackerResultRoi.width/4, trackerResultRoi.y + trackerResultRoi.height/4);
					Point point2 = Point(trackerResultRoi.x + trackerResultRoi.width*3/4, trackerResultRoi.y + trackerResultRoi.height*3/4);
					rectangle(videoFrame, point1, point2, Scalar(255, 0, 0), 1, LINE_4);

					// 判断丢失
					if(response_value <= response_threshold_)
					{
						trackFailCnt++;
						cout << " trackFailCnt /response_value = " << trackFailCnt << " /" << response_value << endl;
						trackerResultRoi.state = 2;	// state 定义 见 tracker widget
						
						if(trackFailCnt > trackFailCnt_threshold_)
						{
							cout << " !!! Track Fail !!!" << endl;

							trackFailCnt = 0;
							is_tracking = false;
							trackerResultRoi.state = 0;	// state 定义 见 tracker widget

							// trackerResultRoi_serial.x = 
							// trackerResultRoi_serial.y = 
							// trackerResultRoi_serial.flag = 
							trackerResultRoi_serial.state = trackerResultRoi.state;	// state 定义 见 tracker widget

							// 告知 mission
							{
								msg_track_target.track_state = trackerResultRoi_serial.state; // state 定义 见 tracker widget
								msg_track_target.pos_x = 0;
								msg_track_target.pos_y = 0;
								msg_track_target.pos_z = 0;				// 地面目标 认为高度 = 0
								msg_track_target.pos_yaw = 0;			// TODO
								
								track_target_pub.publish( msg_track_target );
							}

							// 告知地面站
							// 频率控制
							if ( mavcomm_state1!=2 )  // mavcomm_state1 此处为2
							{
								mavcomm_state1 = 2;
								mavcomm_send_flag = 1;
								mavcomm_curr_count = 0;
							}
							// 告知地面站
							if (mavcomm_send_flag)
							{
								trackerResultRoi_serial_pub.publish(trackerResultRoi_serial);
								mavcomm_send_flag = 0;
							}
							
						} else
						{
							trackerResultRoi_serial.state = trackerResultRoi.state;	// state 定义 见 tracker widget

							// 告知 mission
							{ 	
								msg_track_target.track_state = trackerResultRoi_serial.state; // state 定义 见 tracker widget
								msg_track_target.pos_x = 0;
								msg_track_target.pos_y = 0;
								msg_track_target.pos_z = 0;				// 地面目标 认为高度 = 0
								msg_track_target.pos_yaw = 0;			// TODO
								
								track_target_pub.publish( msg_track_target );
							}

							// 告知地面站
							// 频率控制
							if ( mavcomm_state1!=3 )  // mavcomm_state1 此处为3
							{
								mavcomm_state1 = 3;
								mavcomm_send_flag = 1;
								mavcomm_curr_count = 0;
							}
							// 告知地面站
							if (mavcomm_send_flag)
							{
								mavcomm_send_flag = 0;
								trackerResultRoi_serial_pub.publish(trackerResultRoi_serial);
							}
						}	
					}
					else
					{
						trackFailCnt = 0;
					
						// 解算 框选目标的绝对位置
						// 如果有相机参数 再计算
						// (TODO) 估计模型 优化？

						// 当前仅考虑 相机 竖直朝下安装
						// TODO1 机头朝前 安装 	mission_camera_type == 2
						// TODO2 任意角度安装 ???
						// TODO3 云台 ???
						if ( flag_cameramsg && (mission_camera_type==1) )	// ## 相机位置 1向下 2向前TODO 3TBD
						{
							// int 追踪框的最左边 result.x result.y / result.width result.height

							// camera_matrix K
							// angle of sight
							double aos_x, aos_y;
							aos_x = atan( ((double)msg_CameraInfo.width) / 2.0 / ((double)msg_CameraInfo.K[0]) );	// [0][0]
							aos_y = atan( ((double)msg_CameraInfo.height) / 2.0 / ((double)msg_CameraInfo.K[4]) );	// [1][1]

							double sight_angle[2];
							sight_angle[0] = ( ( (double)result.x + (double)result.width / 2.0) / ( (double)msg_CameraInfo.width /2.0) - 1) * aos_x; 
							sight_angle[1] = ( ( (double)result.y + (double)result.height / 2.0) / ( (double)msg_CameraInfo.height /2.0) - 1) * aos_y; 

							// 距离计算方式
							double depth; 
							// type1 无人机 高度 对于垂直朝下的相机，直接使用无人机 ENU 的高度
							depth = currentPose.position.z;

							/* 相机朝前
							// (TODO) type 2 根据 图像大小 已知人身高等估计 
							// 用于装在垂直超前的 飞机上 
							double cls_hs =	0.024;							//已知目标高度
							// (TODO) type 1 根据 图像大小 已知人身高等估计 
							depth = ( cls_hs * (double)msg_CameraInfo.K[4] ) / ( (double)result.height );
							*/

							// 假设目标在地面
							// 且相机正 无人机机头 ENU
							// 此处控制策略 求解出目标相对位置
							// TODO 另一种控制策略  直接根据图像相对偏差来 机体系下 
							double dx, dy; // 相对位置 偏移量
							double uav_dx, uav_dy;
							dx = - ( tan( sight_angle[1] ) * depth);	// 相机-y 无人机机头 +x ENU
							dy = - ( tan( sight_angle[0] ) * depth);	// 相机-x 无人机左 +y ENU

							uav_dx = dx * cos(current_yaw) + dy * sin(current_yaw);
							uav_dy = dx * -sin(current_yaw) + dy * cos(current_yaw);

							// 目标的位置
							trackerResultRoi_serial.x = (float) (currentPose.position.x + uav_dx);
							trackerResultRoi_serial.y = (float) (currentPose.position.y + uav_dy);
							// trackerResultRoi_serial.z = 0; // TODO
							trackerResultRoi_serial.state = trackerResultRoi.state;	// state 定义 见 tracker widget

							// int mission_track_mission_type = 0;	// ## 任务类型 1定位 2本机追踪 3TBD
							if ( mission_track_mission_type == 1 )
							{ 	
								// 1 仅定位
								// TODO 需要告知协同的无人机 (暂定 mission 中完成)
								// 告知 mission
								// 具体干啥 mission 判断 2本机追踪
								{
								msg_track_target.track_state = trackerResultRoi_serial.state; // state 定义 见 tracker widget
								msg_track_target.pos_x = trackerResultRoi_serial.x;
								msg_track_target.pos_y = trackerResultRoi_serial.y;
								msg_track_target.pos_z = 0;				// 地面目标 认为高度 = 0
								msg_track_target.pos_yaw = 0;			// TODO
								
								track_target_pub.publish( msg_track_target );
								}


								// 告知地面站
								// 频率控制
								if ( mavcomm_state1!=4 )  // mavcomm_state1 此处为4
								{
									mavcomm_state1 = 4;
									mavcomm_send_flag = 1;
									mavcomm_curr_count = 0;
								}
								// 告知地面站
								if (mavcomm_send_flag)
								{
									mavcomm_send_flag = 0;
									trackerResultRoi_serial_pub.publish(trackerResultRoi_serial);
								}
							} 
							else if ( mission_track_mission_type == 2 )
							{ 	
								// 2 本机追踪
								// 告知 mission
								// 具体干啥 mission 判断 2本机追踪
								{
								msg_track_target.track_state = trackerResultRoi_serial.state; // state 定义 见 tracker widget
								msg_track_target.pos_x = trackerResultRoi_serial.x;
								msg_track_target.pos_y = trackerResultRoi_serial.y;
								msg_track_target.pos_z = 0;				// 地面目标 认为高度 = 0
								msg_track_target.pos_yaw = 0;			// TODO
								
								track_target_pub.publish( msg_track_target );
								}

								// 告知地面站
								// 频率控制
								if ( mavcomm_state1!=5 )  // mavcomm_state1 此处为5
								{
									mavcomm_state1 = 5;
									mavcomm_send_flag = 1;
									mavcomm_curr_count = 0;
								}
								// 告知地面站
								if (mavcomm_send_flag)
								{
									mavcomm_send_flag = 0;
									trackerResultRoi_serial_pub.publish(trackerResultRoi_serial);
								}
							}

						}
					}

					// 画框
					switch (trackerResultRoi_serial.state)	
					{ // state 定义 见 tracker widget
					case 1:
						rectangle(videoFrame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 255, 0), 2, 8);
					break;

					case 2:
						rectangle(videoFrame, Point(result.x, result.y), Point(result.x + result.width, result.y + result.height), Scalar(0, 0, 255), 2, 8);
					break;	

					default:
					break;
					}
					
					/*
					// realsense_d4XX 深度相机 测距
					double distance = 0;
					int cnt = 0;
					for (int i = trackerResultRoi.y + trackerResultRoi.height/4 ; 
					    i < min(depthImg_copy.rows,trackerResultRoi.y + trackerResultRoi.height*3/4); 
						i++)
					{
						for (int j = trackerResultRoi.x + trackerResultRoi.width/4; 
						    j <  min(depthImg_copy.cols, trackerResultRoi.x + trackerResultRoi.width*3/4); 
							j++)
						{
							distance += depthImg_copy.at<u_int16_t>(i,j);
							cnt++ ;
						}
					}
					distance = 0.001 * distance / cnt;
					trackerResultRoi.distance = distance;
					trackerResultRoi.header.stamp = ros::Time::now();
					trackerResultRoi_pb.publish(trackerResultRoi);
					*/ 

					/*
					cout << "Center_x/y = " << setiosflags(ios::fixed) << setprecision(5)						
						 << trackerResultRoi.x + trackerResultRoi.width / 2.0  << " "
						 << trackerResultRoi.y + trackerResultRoi.height / 2.0  << " "																
						 << endl;
					*/
				}		
			}
			
			// pub img
			// imshow("Image", videoFrame);
			// waitKey(1);
			sensor_msgs::ImagePtr sourceImg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",videoFrame).toImageMsg();
			sourceImg_pub.publish(sourceImg);
		}

        ros::spinOnce();
        loop_rate.sleep(); 
	}

	return 0;
}




