#include <iostream>
#include <time.h>
#include <ros/ros.h> 
#include <tf/transform_datatypes.h>
#include <transform.h>

using namespace std;

transform::transform()
{

}

Eigen::Vector3d transform::cameraToBody()
{
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Vector3d cameraPosition;

}
