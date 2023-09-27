#include <iostream>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <include/lidar_projection.hpp>



using namespace std;

typedef Eigen::Matrix<double , 6 , 1> Vector6d;

string image_file;
string pcd_file;

vector<double> camera_matrix;
vector<double> dist_coeffs;
Eigen::Matrix3d inner;
Eigen::Vector4d distor;//camera parameters

double depth_weight;
vector<double> init_transform;

int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file" , pcd_file ,"");

    nh.param<vector<double>>("camera/camera_inner" , camera_matrix , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    nh.param<double>("calib/depth_weight" , depth_weight , 0.6);
    nh.param<vector<double>>("calib/init_transform" , init_transform , vector<double>());

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
    cv::Mat test_mat;
    
    test.width = image_gray.size().width;
    test.height = image_gray.size().height;
    test.fx = camera_matrix[0];
    test.fy = camera_matrix[4];
    test.cx = camera_matrix[2];
    test.cy = camera_matrix[5];
    test.k1 = dist_coeffs[0];
    test.k2 = dist_coeffs[1];
    test.p1 = dist_coeffs[2];
    test.p2 = dist_coeffs[3];
    test.k3 = dist_coeffs[4];
    test.depth_weight = depth_weight;
    test_vector << init_transform[0] , init_transform[1] , init_transform[2] , init_transform[3] , init_transform[4] , init_transform[5];

    test_mat = test.getProjectionImage(test_vector);
    cv::imshow("projection_test" , test_mat);
    cv::waitKey(0);
    return 0;
}
