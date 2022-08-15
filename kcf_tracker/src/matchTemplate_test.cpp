#include <unistd.h>
#include <ros/ros.h>  
#include <image_transport/image_transport.h> //用来在ROS系统中的话题上发布和订阅图象消息
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>     //cv_bridge中包含CvBridge库
#include <sensor_msgs/image_encodings.h> //ROS图象类型的编码函数
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

cv::Mat videoFrame;
cv::Mat colorImg ;
bool colorImgRec_Flag = false;

cv::Mat image_template;
cv::Mat image_matched;


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

int main(int argc, char* argv[])
{	
    ros::init(argc, argv,"matchTemplate_node");
    ros::NodeHandle nh("");//创建句柄
	// ros::NodeHandle nh_private("~");
    image_transport::ImageTransport it(nh);

	image_transport::Publisher sourceImg_pub = it.advertise("kcfTracker/image_raw",1);
	image_transport::Subscriber colorImg_sub = it.subscribe("/usb_cam/image_raw",1,colorImg_cb,image_transport::TransportHints("compressed"));

    image_template = cv::imread("/home/linux/Pictures/11.png", cv::IMREAD_GRAYSCALE);

			cv::imshow("image_template", image_template);
			cv::waitKey(1);


    cv::VideoCapture videoCap;
    cv::startWindowThread();
    videoCap.open(1);

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
        // if(videoCap.isOpened())
        // {
        //     videoCap >> videoFrame;
		// 	cv::imshow("videoFrame", videoFrame);
        //     cout << videoFrame.cols << "  " << videoFrame.rows << endl;
		// 	// cv::waitKey(1);            
        // }
		if(colorImgRec_Flag)
		{
			colorImgRec_Flag = false;
			// colorImg.copyTo(videoFrame);
			cvtColor(colorImg, videoFrame, CV_RGB2GRAY);
            cv::matchTemplate(videoFrame, image_template, image_matched, cv::TM_CCOEFF_NORMED);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            //寻找最佳匹配位置
            cv::minMaxLoc(image_matched, &minVal, &maxVal, &minLoc, &maxLoc);

            cv::circle(videoFrame,
                        cv::Point(maxLoc.x + image_template.cols/2, maxLoc.y + image_template.rows/2),
                        20, 
                        cv::Scalar(0, 0, 255), 
                        2, 
                        8, 
                        0);

			cv::imshow("videoFrame", videoFrame);
			cv::waitKey(1);
			sensor_msgs::ImagePtr sourceImg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",videoFrame).toImageMsg();
			sourceImg_pub.publish(sourceImg);
		}
        ros::spinOnce();
        loop_rate.sleep(); 
	}

	return 0;
}

