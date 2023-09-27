#include <iostream>
#include <ros/ros.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>

#include <include/lidar_projection.hpp>



using namespace std;

typedef Eigen::Matrix<double , 6 , 1> Vector6d;

string image_file;
string pcd_file;

vector<double> camera_matrix;
vector<double> dist_coeffs;
Eigen::Matrix3d inner;
Eigen::Vector4d distor;
//Camera parameters

double depth_weight;
vector<double> init_transform;

double img_lines_num , proj_lines_num;
//Calibration parameters

int sets_num = 0;
int *label_img_lines;
int *label_proj_lines;

cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE , 0.8 , 0.6 , 1.5 , 30 , 0.0 , 0.7);//TODO:这里的30度指的是梯度之间的夹角，不是直线之间的夹角，2则是直线检出的梯度大小
std::vector<cv::Vec4f> lines_lsd_img;
std::vector<cv::Vec4f> lines_lsd_proj_img;
//LSD detector

struct sort_lines_by_length
{
    inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
        return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
    }
};


int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file" , pcd_file ,"");

    nh.param<vector<double>>("camera/camera_inner" , camera_matrix , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    nh.param<double>("calib/depth_weight" , depth_weight , 0.6);
    nh.param<vector<double>>("calib/init_transform" , init_transform , vector<double>());\
    nh.param<double>("calib/img_lines_num" , img_lines_num , 250);
    nh.param<double>("calib/proj_lines_num" , proj_lines_num , 250);

    //Read the param form config file.

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


    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(image_gray, image_gray);
    cv::medianBlur(image_gray , image_gray , 3);
    lsd->detect(image_gray , lines_lsd_img );
    sort(lines_lsd_img.begin() , lines_lsd_img.end() , sort_lines_by_length());
    cout << lines_lsd_img.size() << endl;
    lines_lsd_img.resize(img_lines_num);
    lsd->drawSegments(image_gray , lines_lsd_img);
    for(int i = 0 ; i < lines_lsd_img.size() ; ++i)
    {
        cv::putText(image_gray , to_string(i) , cv::Point((lines_lsd_img[i](0) + lines_lsd_img[i](2)) / 2 , (lines_lsd_img[i](1) + lines_lsd_img[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("lsd image" , image_gray);
    cv::waitKey(5);

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
    cv::medianBlur(test_mat , test_mat , 3);
    lsd->detect(test_mat , lines_lsd_proj_img);
    sort(lines_lsd_proj_img.begin() , lines_lsd_proj_img.end() , sort_lines_by_length());
    lines_lsd_proj_img.resize(proj_lines_num);
    lsd->drawSegments(test_mat , lines_lsd_proj_img);
    for(int i = 0 ; i < lines_lsd_proj_img.size() ; ++i)
    {
        cv::putText(test_mat , to_string(i) , cv::Point((lines_lsd_proj_img[i](0) + lines_lsd_proj_img[i](2)) / 2 , (lines_lsd_proj_img[i](1) + lines_lsd_proj_img[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("projection_test" , test_mat);
    cv::waitKey(0);
    
    cout << "Please enter how many sets of lines to select." << endl;
    cin >> sets_num;
    cout << "Please select " << sets_num << " sets of lines." << endl;

    cv::waitKey(0);
    return 0;
}
