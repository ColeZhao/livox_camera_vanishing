#include <iostream>
#include <ros/ros.h>
#include <math.h>

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

vector<double> Extrinsic_vector;
Eigen::Matrix3d rotation_matrix;
Eigen::Vector3d transform_vector;
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
vector<cv::Point2d> intersection_pts_orig;
vector<cv::Point2d> intersection_pts_proj;

Eigen::Matrix3d R_w_c , R_w_l , R_l_c;
Eigen::Vector3d t_w_c , t_w_l , t_l_c;
struct sort_lines_by_length
{
    inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
        return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
    }
};

vector<cv::Point2d> calcIntersectionPoint(vector<cv::Vec4f> lines_calib)
{   //这个函数就不考虑只选择了三条直线的情况了，三条直线的情况后面单独实现，跟在selected后边
    vector<cv::Point2d> intersection_pts;
    cv::Point2d intersection_pt;
    double slope[4];
    double intercept[4];
    for(int i = 0 ; i < calib_lines_sets_num ; i++)
    {
        for(int j = 0 ; j < 4 ; j++)
        {   //求解直线斜率和截距
            slope[j] = (lines_calib[4 * i + j](1) - lines_calib[4 * i + j](3)) / (lines_calib[4 * i + j](0) - lines_calib[4 * i + j](2));
            intercept[j] = lines_calib[4 * i + j](1) - slope[j] * lines_calib[4 * i + j](0);
        }
        if(slope[0] != slope[1])
        {   //直线在图像上不平行
            intersection_pt.x = (intercept[1] - intercept[0]) / (slope[0] - slope[1]);
            intersection_pt.y = slope[0] * intersection_pt.x + intercept[0];
            intersection_pts.emplace_back(intersection_pt);
        }
        else
        {
            cout << "Error! These two lines are parallel." << endl;
        }
        if(slope[2] != slope[3])
        {   //直线在图像上不平行
            intersection_pt.x = (intercept[3] - intercept[2]) / (slope[2] - slope[3]);
            intersection_pt.y = slope[2] * intersection_pt.x + intercept[2];
            intersection_pts.emplace_back(intersection_pt);

        }
        else
        {
            cout << "Error! These two lines are parallel." << endl;
        }

        //然后开始求解四条平行线的交点
        intersection_pt.x = (intercept[0] - intercept[2]) / (slope[2] - slope[0]);
        intersection_pt.y = slope[2] * intersection_pt.x + intercept[2];
        intersection_pts.emplace_back(intersection_pt);//corner_a

        intersection_pt.x = (intercept[0] - intercept[3]) / (slope[3] - slope[0]);
        intersection_pt.y = slope[3] * intersection_pt.x + intercept[3];
        intersection_pts.emplace_back(intersection_pt);//corner_b
    }
    return intersection_pts;
}


Eigen::Matrix3d calcRotation(vector<double> camera_inner , vector<cv::Point2d> intersection_pts)
{
    //TODO：使用消失点坐标作叉乘时原点坐标需要被确定，理论上应该是以光新为原点才能够获得对应的旋转矩阵
    Eigen::Matrix3d R_w;
    Eigen::Matrix3d K_c;
    Eigen::Vector3d vanishing_pt1 , vanishing_pt2 , mian_pt;
    Eigen::Vector3d R_w_1 , R_w_2 , R_w_3;

    double f_average = (camera_inner[0] + camera_inner[4]) / 2;

    // vanishing_pt1 << intersection_pts[0].x - camera_inner[2] , intersection_pts[0].y - camera_inner[5] , f_average;
    // vanishing_pt2 << intersection_pts[1].x - camera_inner[2] , intersection_pts[1].y - camera_inner[5] , f_average;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上

    vanishing_pt1 << intersection_pts[0].x , intersection_pts[0].y , 1;
    vanishing_pt2 << intersection_pts[1].x , intersection_pts[1].y , 1;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上

    K_c << camera_inner[0] , camera_inner[1] , camera_inner[2] , camera_inner[3] , camera_inner[4] , camera_inner[5] , camera_inner[6] , camera_inner[7] , camera_inner[8];

    R_w_1 = (K_c.inverse() * vanishing_pt1).normalized();
    R_w_2 = (K_c.inverse() * vanishing_pt2).normalized();//目前看来两种表述方法获得的结果是一致的
    // R_w_1 = (vanishing_pt1).normalized();
    // R_w_2 = (vanishing_pt2).normalized();
    R_w_3 = (R_w_1.cross(R_w_2)).normalized();

    R_w << R_w_1 , R_w_2 , R_w_3;
    R_w = R_w.transpose().normalized();
    return R_w;
}

Eigen::Vector3d calcPose(vector<double> camera_inner , vector<cv::Point2d> intersection_pts , double length_AB)
{
    //TODO：同样如上需要对坐标系与坐标原点进行确定
    Eigen::Vector3d t_w;

    double length = 0;
    // double f_average = (camera_inner[0] + camera_inner[4]) / 2;//平均焦距
    double f_average = 5120;

    Eigen::Vector3d corner_a , corner_b , main_pt , vanishing_pt1 , vanishing_pt2;
    corner_a << intersection_pts[3].x - camera_inner[2] , intersection_pts[3].y - camera_inner[5] , f_average;
    corner_b << intersection_pts[4].x - camera_inner[2] , intersection_pts[4].y - camera_inner[5] , f_average;
    main_pt << 0 , 0 , f_average;
    vanishing_pt1 << intersection_pts[0].x - camera_inner[2] , intersection_pts[0].y - camera_inner[5] , f_average;
    vanishing_pt2 << intersection_pts[1].x - camera_inner[2] , intersection_pts[1].y - camera_inner[5] , f_average;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上


    double sinbop , sinaob;
    sinbop = main_pt.norm() * (corner_b - vanishing_pt1).norm() / (vanishing_pt1.norm() * corner_b.norm());

    length = length_AB * sinbop / sinaob;

    t_w = corner_a.normalized() * length;
    return t_w;
}

//TODO:激光部分的可能直接使用激光点要相对更准确一些？但是像素如何和三维点对应，并且如果对应的话点不在同一个平面上如何进行处理呢，若果能解决这个问题可以减小激光提取的误差
int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file" , pcd_file ,"");

    nh.param<vector<double>>("camera/camera_inner" , camera_inner , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    int getMat = nh.param<vector<double>>("calib/ExtrinsicMat" , Extrinsic_vector , vector<double>());
    nh.param<double>("calib/depth_weight" , depth_weight , 0.6);
    int getAngle = nh.param<vector<double>>("calib/init_transform" , init_transform , {0 , 0 , 0 , 0 , 0 , 0});
    nh.param<double>("calib/max_orig_lines_num" , max_orig_lines_num , 250);
    nh.param<double>("calib/max_proj_lines_num" , max_proj_lines_num , 250);

    rotation_matrix << Extrinsic_vector[0] , Extrinsic_vector[1] , Extrinsic_vector[2] , Extrinsic_vector[4] , Extrinsic_vector[5] , Extrinsic_vector[6] ,Extrinsic_vector[8] , Extrinsic_vector[9] , Extrinsic_vector[10];
    transform_vector << Extrinsic_vector[3] , Extrinsic_vector[7] , Extrinsic_vector[11];

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
    if(getAngle)
    {
        test_mat = test.getProjectionImage(test_vector);
    }
    else if(getMat)
    {
        test_mat = test.getProjectionImage(rotation_matrix , transform_vector);
    }
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
        lines_selected_orig.emplace_back(lines_lsd_orig[*it]);
        cout << *it << " ";
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
        lines_selected_proj.emplace_back(lines_lsd_proj[*it]);
        cout << *it << " ";
    }
    cout << endl;

    lsd->drawSegments(test_mat , lines_selected_proj);
    for(int i = 0 ; i < lines_selected_proj.size() ; ++i)
    {
        cv::putText(test_mat , to_string(label_lines_selected_proj[i]) , cv::Point((lines_selected_proj[i](0) + lines_selected_proj[i](2)) / 2 , (lines_selected_proj[i](1) + lines_selected_proj[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("proj image selected" , test_mat);
    //TODO:需要修改成能够选择多条直线，然后完成3条直线确定一个正方形的代码，输入选择直线的时候要求按照一组平行线连续输入
    //Select lines that will be used in calibration manually and show them

    intersection_pts_orig = calcIntersectionPoint(lines_selected_orig);
    intersection_pts_proj = calcIntersectionPoint(lines_selected_proj);
    //这里已经把消失点算出来了，然后根据消失点理论分别计算外参的旋转矩阵和位置
    cout << "The vanishing points in origin image are:" << endl;
    for(auto it = intersection_pts_orig.begin() ; it != intersection_pts_orig.end() ; it++)
    {
        cout << *it << endl;
    }
    cout << "The vanishing point in projection image are:" << endl;
    for(auto it = intersection_pts_proj.begin() ; it != intersection_pts_proj.end() ; it++)
    {
        cout << *it << endl;
    }

    R_w_c = calcRotation(camera_inner , intersection_pts_orig);
    R_w_l = calcRotation(camera_inner , intersection_pts_proj);//现在都暂定的是相机内参的标定是准确的
    R_l_c = R_w_l * (R_w_c.inverse());

    Eigen::Matrix3d result_matrix = R_l_c * rotation_matrix;//这里应该不是简单的叠加，是在原有基础上再转 0 . 1 2

    if(getAngle)
    {
        Eigen::Vector3d result_euler = result_matrix.eulerAngles(2 , 1 , 0);
        cout << "The rotation angle between lidar and camera is: " << result_euler << endl;
    }
    else if(getMat)
    {
        cout << "The rotation angle between lidar and camera is: " << result_matrix << endl;
    }

    t_w_c = calcPose(camera_inner , intersection_pts_orig , 47);
    t_w_l = calcPose(camera_inner , intersection_pts_proj , 47);
    t_l_c = t_w_l - t_w_c;
    cout << "transform vector between lidar and camera is " << transform_vector + t_l_c << endl;
    

    cv::waitKey(0);
    return 0;
}
