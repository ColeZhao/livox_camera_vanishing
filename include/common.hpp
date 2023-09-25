#ifndef __COMMON_HPP__
#define __COMMON_HPP__
#include  <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

double calcPointDepth(pcl::PointXYZI pts_3d)
{
    return sqrt(pow(pts_3d.x , 2) +pow(pts_3d.y , 2) + pow(pts_3d.z , 2));
}

#endif