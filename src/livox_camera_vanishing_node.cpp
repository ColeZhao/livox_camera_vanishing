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
//Path of image and PCD

vector<double> camera_inner;
vector<double> dist_coeffs;
//Camera parameters

double depth_weight;
vector<double> init_transform;

double max_orig_lines_num , max_proj_lines_num;
//Calibration parameters

int calib_lines_sets_num = 0;
//Number of sets that will be selected to calibrate the extrinsic parameters

cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE , 0.8 , 0.6 , 1.5 , 30 , 0.0 , 0.7);
vector<cv::Vec4f> lines_lsd_orig;
vector<cv::Vec4f> lines_lsd_proj;
//LSD detector

vector<int> label_lines_selected_orig;
vector<int> label_lines_selected_proj;
int label_line_selected;
//The label of lines which are selected
vector<cv::Vec4f> lines_selected_orig;
vector<cv::Vec4f> lines_selected_proj;
//Lines which are selected


struct sort_lines_by_length
{
    inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
        return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
    }
};

vector<cv::Point2d> calcIntersectionPoint(vector<cv::Vec4f> lines_calib)
{   //这个函数就不考虑只选择了三条直线的情况了，三条直线的情况后面单独实现，跟在selected后边
    for(int i = 0 ; i < calib_lines_sets_num ; i++)
    {

    }
}


int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file" , pcd_file ,"");

    nh.param<vector<double>>("camera/camera_inner" , camera_inner , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    nh.param<double>("calib/depth_weight" , depth_weight , 0.6);
    nh.param<vector<double>>("calib/init_transform" , init_transform , vector<double>());\
    nh.param<double>("calib/max_orig_lines_num" , max_orig_lines_num , 250);
    nh.param<double>("calib/max_proj_lines_num" , max_proj_lines_num , 250);

    // cout << "test" << endl;

    // while(cin >> label_test)
    // {
    //     cout << label_test << endl;
    // }

    // cout << "continue" << endl;
    // cin.clear();
    // cout<<cin.rdstate()<<endl;
    // cin.ignore(10, '\n'); // cin无限制输入测试
    //Read the param form config file.
    // TODO:替换后续的输入，变成可以输入任意数量直线

    cv::Mat image_orig = cv::imread(image_file , cv::IMREAD_UNCHANGED);
    cv::Mat image_gray;

    if(!image_orig.data)
    {
        ROS_ERROR("Failed load image!");
        exit(1);
    }
    {
        ROS_INFO("Load image successfully!");
    }

    if(image_orig.type() == CV_8UC1)
    {
        image_gray = image_orig;
    }
    else if(image_orig.type() == CV_8UC3)
    {
        cv::cvtColor(image_orig , image_gray , cv::COLOR_BGR2GRAY);
    }
    //Load the image

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(image_gray, image_gray);
    cv::medianBlur(image_gray , image_gray , 3);
    lsd->detect(image_gray , lines_lsd_orig );
    sort(lines_lsd_orig.begin() , lines_lsd_orig.end() , sort_lines_by_length());
    cout << lines_lsd_orig.size() << endl;
    lines_lsd_orig.resize(max_proj_lines_num);
    cv::Mat lsd_orig_gray = image_gray.clone();
    lsd->drawSegments(lsd_orig_gray , lines_lsd_orig);
    for(int i = 0 ; i < lines_lsd_orig.size() ; ++i)
    {
        cv::putText(lsd_orig_gray , to_string(i) , cv::Point((lines_lsd_orig[i](0) + lines_lsd_orig[i](2)) / 2 , (lines_lsd_orig[i](1) + lines_lsd_orig[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("lsd origin image" , lsd_orig_gray);
    cv::waitKey(100);
    //Detect lines in image and show them

    /*代码模块测试用*/
    LidarProjection test(pcd_file);
    Vector6d test_vector;
    cv::Mat test_mat;
    
    test.width = image_gray.size().width;
    test.height = image_gray.size().height;
    test.fx = camera_inner[0];
    test.fy = camera_inner[4];
    test.cx = camera_inner[2];
    test.cy = camera_inner[5];
    test.k1 = dist_coeffs[0];
    test.k2 = dist_coeffs[1];
    test.p1 = dist_coeffs[2];
    test.p2 = dist_coeffs[3];
    test.k3 = dist_coeffs[4];
    test.depth_weight = depth_weight;
    test_vector << init_transform[0] , init_transform[1] , init_transform[2] , init_transform[3] , init_transform[4] , init_transform[5];
    test_mat = test.getProjectionImage(test_vector);
    //Load PCD file and get projection image of it

    cv::medianBlur(test_mat , test_mat , 3);
    lsd->detect(test_mat , lines_lsd_proj);
    sort(lines_lsd_proj.begin() , lines_lsd_proj.end() , sort_lines_by_length());
    lines_lsd_proj.resize(max_proj_lines_num);
    cv::Mat lsd_proj_gray = test_mat.clone();
    lsd->drawSegments(lsd_proj_gray , lines_lsd_proj);
    for(int i = 0 ; i < lines_lsd_proj.size() ; ++i)
    {
        cv::putText(lsd_proj_gray , to_string(i) , cv::Point((lines_lsd_proj[i](0) + lines_lsd_proj[i](2)) / 2 , (lines_lsd_proj[i](1) + lines_lsd_proj[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("projection_test" , lsd_proj_gray);
    cv::waitKey(100);
    //Detect lines in projection image and show them
    
    cout << "Please enter how many sets of lines to select." << endl;
    cin >> calib_lines_sets_num;
    cout << "Please select " << calib_lines_sets_num << " sets of lines in origin image." << endl;
    for(int i = 0 ; i < 4 * calib_lines_sets_num ; i++)
    {
        cin >> label_line_selected;
        label_lines_selected_orig.emplace_back(label_line_selected);
    }
    cout << "You have choosen lines in origin image whose label are :" << endl;

    for(auto it = label_lines_selected_orig.begin() ; it != label_lines_selected_orig.end() ; ++it)
    {
        auto label = *it;
        lines_selected_orig.emplace_back(lines_lsd_orig[label]);
        cout << label << " ";
    }
    cout << endl;
    
    lsd->drawSegments(image_gray , lines_selected_orig);
    for(int i = 0 ; i < lines_selected_orig.size() ; ++i)
    {
        cv::putText(image_gray , to_string(label_lines_selected_orig[i]) , cv::Point((lines_selected_orig[i](0) + lines_selected_orig[i](2)) / 2 , (lines_selected_orig[i](1) + lines_selected_orig[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("lsd image selected" , image_gray);
    cv::waitKey(100);


    cout << "Please select the same lines in projection image." << endl;
    for(int i = 0 ; i < 4 * calib_lines_sets_num ; i++)
    {
        cin >> label_line_selected;
        label_lines_selected_proj.emplace_back(label_line_selected);
    }
    cout << "You have choosen lines in projection image whose label are :" << endl;
    for(auto it = label_lines_selected_proj.begin() ; it != label_lines_selected_proj.end() ; ++it)
    {
        auto label = *it;
        lines_selected_proj.emplace_back(lines_lsd_proj[label]);
        cout << label << " ";
    }
    cout << endl;

    lsd->drawSegments(test_mat , lines_selected_proj);
    for(int i = 0 ; i < lines_selected_proj.size() ; ++i)
    {
        cv::putText(test_mat , to_string(label_lines_selected_proj[i]) , cv::Point((lines_selected_proj[i](0) + lines_selected_proj[i](2)) / 2 , (lines_selected_proj[i](1) + lines_selected_proj[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("proj image selected" , test_mat);
    //TODO:需要修改成能够选择多条直线，然后完成3条直线确定一个正方形的代码
    //Select lines that will be used in calibration manually and show them


    cv::waitKey(0);
    return 0;
}
