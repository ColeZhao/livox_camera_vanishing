#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <include/lidar_projection.hpp>


using namespace std;

string image_file;
string pcd_file;

vector<double> camera_matrix;
vector<double> dist_coeffs;
Eigen::Matrix3d inner;
Eigen::Vector4d distor;//camera parameters


typedef Eigen::Matrix<double , 6 , 1> Vector6d;

int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file" , pcd_file ,"");
    nh.param<vector<double>>("camera/camera_inner" , camera_matrix , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    cv::Mat image_orin = cv::imread(image_file , cv::IMREAD_UNCHANGED);
    cv::Mat image_gray;

    if(!image_orin.data)
    {
        ROS_ERROR("Failed load image!");
        exit(1);
    }
    {
        ROS_INFO("Load image successfully!");
    }

    if(image_orin.type() == CV_8UC1)
    {
        image_gray = image_orin;
    }
    else if(image_orin.type() == CV_8UC3)
    {
        cv::cvtColor(image_orin , image_gray , cv::COLOR_BGR2GRAY);
    }
    /*代码模块测试用*/
    LidarProjection test(pcd_file);
    Vector6d test_vector;
    test_vector << 0 , 0 , 0 , 0 , 0 , 0;
    test.getProjectionImage(test_vector);
    
    return 0;
}
