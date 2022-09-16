
// 通用宏定义
//#pragma once

#ifndef COM_DEFINE_H
#define COM_DEFINE_H

#define NNN 11

// sysid    发送端编号
// compid   接收端编号
// my_id    本机编号 	[ 100-地面站 ] 	[99-所有无人机]

#define ID_GCS 100
#define ID_ALL 99 

// mission 数组最大
#define MAX_NUM_MIS 255

// double pi = 3.1415926;
#define PI_3 3.1415926


#include <ros/ros.h>
#include <iostream>


#endif
