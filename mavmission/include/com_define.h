
// 通用宏定义
//#pragma once

#ifndef COM_DEFINE_H
#define COM_DEFINE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <optional>
// #include <cJSON.h>
#include "nlohmann/json.hpp"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// 无人机数量 
// 需要使用编队模块时 最大16 (拓展setbit_mm后可增加)
#define NNN 11

// sysid    发送端编号
// compid   接收端编号
// my_id    本机编号 	[ 100-地面站 ] 	[99-所有无人机]

#define ID_GCS 100
#define ID_ALL 99 

// mission 数组最大
#define MAX_NUM_MIS 255

// formation 设置队形编组
#define FORMATION_GROUP_PPP 10

// double pi = 3.1415926;
#define PI_3 3.1415926

// Help Func
// 系统时间
uint64_t get_time_usec();
// 上下限函数
float satfunc(float data, float Max);

#endif
