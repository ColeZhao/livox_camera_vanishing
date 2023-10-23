#ifndef __COMMON_HPP__
#define __COMMON_HPP__
#include  <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

using namespace std;

double calcPointDepth(pcl::PointXYZI pts_3d)
{
    return sqrt(pow(pts_3d.x , 2) +pow(pts_3d.y , 2) + pow(pts_3d.z , 2));
}

struct vanishingData
{

    vector<cv::Point2d> pts_orin;
    vector<cv::Point2d> pts_project;
};

#endif