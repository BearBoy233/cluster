#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <iostream>
#include <time.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

class transform
{
public:
    transform();

    Eigen::Vector3d cameraToBody_T;
    
    Eigen::Vector3d cameraToBody();


};

#endif
