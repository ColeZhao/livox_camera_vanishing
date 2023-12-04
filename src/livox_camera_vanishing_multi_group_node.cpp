#include <iostream>
#include <ros/ros.h>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ceres/ceres.h>

#include <include/lidar_projection.hpp>

//TODO:尝试2D-3D对应，以计算标定直线尺寸，如果3D-2D对应实现，那么残差部分也可以进行改变
//TODO:添加残差计算权重相关内容
//TODO:添加单帧多标定物标定代码
//TODO:添加多帧多标定物标定代码
#define useAuxiliaryLines 0
#define calcResiduals 1


using namespace std;

typedef Eigen::Matrix<double , 6 , 1> Vector6d;

string image_file;
string pcd_file;
//Path of image and PCD

int frame_num;
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


cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV , 0.99 , 1.0 , 3.0 , 20 , 0.0 , 0.7);
// cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE , 0.8 , 0.6 , 1.5 , 30 , 0.0 , 0.7);
cv::Ptr<cv::LineSegmentDetector> project_lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV , 0.99 , 1.0 , 1.8 , 20 , 0.0 , 0.65);
// cv::Ptr<cv::LineSegmentDetector> project_lsd = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV , 0.99 , 1.0 , 1.8 , 30 , 0.0 , 0.65);


vector<cv::Vec4f> lines_lsd_orig;
vector<cv::Vec4f> lines_lsd_proj;
//LSD detector

int label_line_selected;
vector<int> label_lines_selected_orig;
vector<int> label_lines_selected_proj;
//The label of lines which are selected
vector<cv::Vec4f> lines_selected_orig;
vector<cv::Vec4f> lines_selected_proj;
//Lines which are selected
vector<cv::Point2d> intersection_pts_orig;
vector<cv::Point2d> intersection_pts_proj;

#if useAuxiliaryLines
//Auxiliary lines
int auxiliary_num;
vector<int> label_lines_auxiliary_orig1;
vector<int> label_lines_auxiliary_orig2;
vector<int> label_lines_auxiliary_proj1;
vector<int> label_lines_auxiliary_proj2;
//The label of lines which are selected
vector<cv::Vec4f> lines_auxiliary_orig1;
vector<cv::Vec4f> lines_auxiliary_orig2;
vector<cv::Vec4f> lines_auxiliary_proj1;
vector<cv::Vec4f> lines_auxiliary_proj2;
#endif

#if calcResiduals

vector<cv::Vec4f> lines_lsd_proj_R;
vector<int> label_lines_selected_proj_R;
vector<cv::Vec4f> lines_selected_proj_R;
vector<cv::Point2d> intersection_pts_proj_R;

vector<double> triangle_ori;
vector<double> triangle_res;
double residuals_tmp;
double residuals[3];
double res_weight_sum = 0.0;
#endif

Eigen::Matrix3d R_w_c , R_w_l , R_l_c;
Eigen::Vector3d t_c_w , t_l_w , t_w_l , t_l_c;
vector<Eigen::Quaterniond> q_container;
vector<Eigen::Vector3d> t_container;

struct sort_lines_by_length
{   //根据特征直线的长度进行排序
    inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
        return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
    }
};

vector<cv::Point2d> calcIntersectionPoint(vector<cv::Vec4f> lines_calib)
{   //计算消失点与垂直直线交点
    vector<cv::Point2d> intersection_pts;
    cv::Point2d intersection_pt;
    double slope[4];
    double intercept[4];
    for(int j = 0 ; j < 4 ; j++)
    {   //求解直线斜率和截距
        slope[j] = (lines_calib[j](1) - lines_calib[j](3)) / (lines_calib[j](0) - lines_calib[j](2));
        intercept[j] = lines_calib[j](1) - slope[j] * lines_calib[j](0);
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

    //然后开始求解四条线的交点
    intersection_pt.x = (intercept[0] - intercept[2]) / (slope[2] - slope[0]);
    intersection_pt.y = slope[2] * intersection_pt.x + intercept[2];
    intersection_pts.emplace_back(intersection_pt);//corner_a

    intersection_pt.x = (intercept[0] - intercept[3]) / (slope[3] - slope[0]);
    intersection_pt.y = slope[3] * intersection_pt.x + intercept[3];
    intersection_pts.emplace_back(intersection_pt);//corner_b

    intersection_pt.x = (intercept[1] - intercept[2]) / (slope[2] - slope[1]);
    intersection_pt.y = slope[2] * intersection_pt.x + intercept[2];
    intersection_pts.emplace_back(intersection_pt);//corner_d

    intersection_pt.x = (intercept[1] - intercept[3]) / (slope[3] - slope[1]);
    intersection_pt.y = slope[3] * intersection_pt.x + intercept[3];
    intersection_pts.emplace_back(intersection_pt);//corner_c
    return intersection_pts;
}

cv::Point2d calcVanishingPoint(vector<cv::Vec4f> lines_calib)
{   //辅助直线计算消失点
    cv::Point2d vanishing_pt;
    double *slope = (double *)malloc(lines_calib.size() * sizeof(double));
    double *intercept = (double *)malloc(lines_calib.size() * sizeof(double));
    for(int i = 0 ; i < lines_calib.size() ; i++)
    {  //求解直线斜率和截距
        slope[i] = (lines_calib[i](1) - lines_calib[i](3)) / (lines_calib[i](0) - lines_calib[i](2));
        intercept[i] = lines_calib[i](1) - slope[i] * lines_calib[i](0);
    }
    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(lines_calib.size() , 2);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(lines_calib.size());
    for(int i = 0 ; i < lines_calib.size() ; i++)
    {
        U.row(i) << -slope[i] , 1;
        b(i) = intercept[i];
        // cout << i << "th lines." << endl;
        // cout << " k :" << slope[i] << " b :" << intercept[i] << endl; 
    }
    Eigen::Vector2d cv_pt = (U.transpose() * U).inverse() * U.transpose() * b;
    vanishing_pt.x = cv_pt(0);
    vanishing_pt.y = cv_pt(1);
    free(slope);
    free(intercept);
    return vanishing_pt;
}


Eigen::Vector3d calcPose(vector<double> camera_inner , vector<cv::Point2d> intersection_pts , double length_AB)
{
    Eigen::Vector3d t_c;

    double length = 0;
    double f_average = (camera_inner[0] + camera_inner[4]) / 2;//平均焦距

    Eigen::Vector3d corner_a , corner_b , main_pt , vanishing_pt1 , vanishing_pt2;
    corner_a << intersection_pts[2].x - camera_inner[2] , intersection_pts[2].y - camera_inner[5] , f_average;
    corner_b << intersection_pts[3].x - camera_inner[2] , intersection_pts[3].y - camera_inner[5] , f_average;
    main_pt << 0 , 0 , f_average;
    vanishing_pt1 << intersection_pts[0].x - camera_inner[2] , intersection_pts[0].y - camera_inner[5] , f_average;
    vanishing_pt2 << intersection_pts[1].x - camera_inner[2] , intersection_pts[1].y - camera_inner[5] , f_average;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上
    double sinbop , sinaob;
    sinbop = main_pt.norm() * (corner_a - vanishing_pt1).norm() / (vanishing_pt1.norm() * corner_a.norm());
    sinaob = sin(acos(corner_a.dot(corner_b) / (corner_a.norm() * corner_b.norm())));
    length = length_AB * sinbop / sinaob;

    t_c = corner_a.normalized() * length;
    return t_c;
}

Eigen::Matrix3d calcRotation(vector<double> camera_inner , vector<cv::Point2d> vanishing_pts)
{
    //TODO：使用消失点坐标作叉乘时原点坐标需要被确定，理论上应该是以光新为原点才能够获得对应的旋转矩阵
    Eigen::Matrix3d R_w;
    Eigen::Matrix3d K_c;
    Eigen::Vector3d vanishing_pt1 , vanishing_pt2 , mian_pt;
    Eigen::Vector3d R_w_1 , R_w_2 , R_w_3;

    double f_average = (camera_inner[0] + camera_inner[4]) / 2;

    vanishing_pt1 << vanishing_pts[0].x - camera_inner[2] , vanishing_pts[0].y - camera_inner[5] , 1;
    vanishing_pt2 << vanishing_pts[1].x- camera_inner[2] , vanishing_pts[1].y - camera_inner[5] , 1;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上

    // K_c << camera_inner[0] , camera_inner[1] , camera_inner[2] , camera_inner[3] , camera_inner[4] , camera_inner[5] , camera_inner[6] , camera_inner[7] , camera_inner[8];
    K_c << f_average , 0 , 0 , 0 , f_average , 0 , 0 , 0 , 1;

    if(vanishing_pt1.y() < 0)
    {
        vanishing_pt1 = -vanishing_pt1;
    }
    if(vanishing_pt2.x() > 0)
    {
        vanishing_pt2 = -vanishing_pt2;
    }

    R_w_1 = (K_c.inverse() * vanishing_pt1).normalized();
    R_w_2 = (K_c.inverse() * vanishing_pt2).normalized();//目前看来两种表述方法获得的结果是一致的
    R_w_3 = (R_w_1.cross(R_w_2)).normalized();

    R_w << R_w_1 , R_w_2 , R_w_3;
    // R_w = R_w.transpose().normalized();
    Eigen::Matrix3d R_w_x = R_w.transpose();
    return R_w_x;
}

Eigen::Vector3d calcPose(vector<double> camera_inner , vector<cv::Point2d> intersection_pts , vector<cv::Point2d> vanishing_pts , double length_AB)
{
    Eigen::Vector3d t_w;

    double length = 0;
    double f_average = (camera_inner[0] + camera_inner[4]) / 2;//平均焦距

    Eigen::Vector3d corner_a , corner_b , main_pt , vanishing_pt1 , vanishing_pt2;
    corner_a << intersection_pts[0].x - camera_inner[2] , intersection_pts[0].y - camera_inner[5] , f_average;
    corner_b << intersection_pts[1].x - camera_inner[2] , intersection_pts[1].y - camera_inner[5] , f_average;
    main_pt << 0 , 0 , f_average;
    vanishing_pt1 << vanishing_pts[0].x - camera_inner[2] , vanishing_pts[0].y - camera_inner[5] , f_average;
    vanishing_pt2 << vanishing_pts[1].x - camera_inner[2] , vanishing_pts[1].y - camera_inner[5] , f_average;//光心在空间坐标系当中与主点重合，所以把相平面坐标系转化到相机坐标系下的消失点如上
    double sinbop , sinaob;
    sinbop = main_pt.norm() * (corner_b - vanishing_pt1).norm() / (vanishing_pt1.norm() * corner_b.norm());

    length = length_AB * sinbop / sinaob;

    t_w = corner_a.normalized() * length;
    return t_w;
}




//TODO:激光部分的可能直接使用激光点要相对更准确一些？但是像素如何和三维点对应，并且如果对应的话点不在同一个平面上如何进行处理呢，若果能解决这个问题可以减小激光提取的误差
int main(int argc , char **argv)
{
    /*读取config文件里面的参数*/
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<int>("frame_num" , frame_num , 1);

    nh.param<vector<double>>("camera/camera_inner" , camera_inner , vector<double>());
    nh.param<vector<double>>("camera/camera_distortion" , dist_coeffs , vector<double>());

    int getMat = nh.param<vector<double>>("calib/ExtrinsicMat" , Extrinsic_vector , vector<double>());
    nh.param<double>("calib/depth_weight" , depth_weight , 0.6);
    int getAngle = nh.param<vector<double>>("calib/init_transform" , init_transform , {0 , 0 , 0 , 0 , 0 , 0});
    nh.param<double>("calib/max_orig_lines_num" , max_orig_lines_num , 250);
    nh.param<double>("calib/max_proj_lines_num" , max_proj_lines_num , 250);

    rotation_matrix << Extrinsic_vector[0] , Extrinsic_vector[1] , Extrinsic_vector[2] , Extrinsic_vector[4] , Extrinsic_vector[5] , Extrinsic_vector[6] ,Extrinsic_vector[8] , Extrinsic_vector[9] , Extrinsic_vector[10];
    transform_vector << Extrinsic_vector[3] , Extrinsic_vector[7] , Extrinsic_vector[11];


    for(int i = 0 ; i < frame_num ; i++)
    {
        string image_str = "image_file" + to_string(i);
        string pcd_str = "pcd_file" + to_string(i);
        nh.param<string>(image_str , image_file , "");
        nh.param<string>(pcd_str , pcd_file ,"");

            /*光学图像和pcd投影图像的读取*/
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

        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if(pcl::io::loadPCDFile(pcd_file , *lidar_point_cloud) == -1)
        {
            ROS_ERROR("Failed load PCD file.");
            exit(1);
        }
        else
        {
            ROS_INFO("Load PCD file sucessfully!");
        }
        //Load the PCD file

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(40.0, cv::Size(5, 5));
        clahe->apply(image_gray, image_gray);//直方图均衡化做了图像增强

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

        LidarProjection test(lidar_point_cloud);
        Vector6d test_vector;
        cv::Mat test_mat;
        cv::Mat compared_mat;
        
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
            compared_mat = test.getComparedImage(image_gray , rotation_matrix , transform_vector);
            cv::imshow("com" , compared_mat);
        }

        cv::medianBlur(test_mat , test_mat , 3);
        cv::fastNlMeansDenoising(test_mat , test_mat);//
        project_lsd->detect(test_mat , lines_lsd_proj);
        sort(lines_lsd_proj.begin() , lines_lsd_proj.end() , sort_lines_by_length());
        lines_lsd_proj.resize(max_proj_lines_num);
        cv::Mat lsd_proj_gray = test_mat.clone();
        project_lsd->drawSegments(lsd_proj_gray , lines_lsd_proj);
        for(int i = 0 ; i < lines_lsd_proj.size() ; ++i)
        {
            cv::putText(lsd_proj_gray , to_string(i) , cv::Point((lines_lsd_proj[i](0) + lines_lsd_proj[i](2)) / 2 , (lines_lsd_proj[i](1) + lines_lsd_proj[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow("projection_test" , lsd_proj_gray);
        cv::waitKey(1000);
        //Detect lines in projection image and show them

        while(ros::ok())
        {
            int exit_flag_orig = 1;
            label_lines_selected_orig.clear();
            lines_selected_orig.clear();
            /*选取原始图像上的一组正交直线*/
            ROS_INFO("Please select 1 sets of lines in origin image.");
            for(int i = 0 ; i < 4 ; i++)
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
            intersection_pts_orig = calcIntersectionPoint(lines_selected_orig);

        #if useAuxiliaryLines
            // Calculate the intersection points and vanishing points with auxiliary lines.
            ROS_INFO("Please enter how many auxiliary lines in first direction to selected in origin image.");
            cin >> auxiliary_num;
            ROS_INFO("Please selected lines.");
            for(int i = 0 ; i < auxiliary_num ; i++)
            {
                cin >> label_line_selected;
                label_lines_auxiliary_orig1.emplace_back(label_line_selected);
            }
            lines_auxiliary_orig1.emplace_back(lines_selected_orig.at(0));
            lines_auxiliary_orig1.emplace_back(lines_selected_orig.at(1));
            for(auto it = label_lines_auxiliary_orig1.begin() ; it != label_lines_auxiliary_orig1.end() ; ++it)
            {
                lines_auxiliary_orig1.emplace_back(lines_lsd_orig[*it]);
                cout << *it << " ";
            }
            cout << endl;

            ROS_INFO("Please enter how many auxiliary lines in second direction to selected in  origin image.");
            cin >> auxiliary_num;
            ROS_INFO("Please selected lines.");
            for(int i = 0 ; i < auxiliary_num ; i++)
            {
                cin >> label_line_selected;
                label_lines_auxiliary_orig2.emplace_back(label_line_selected);
            }
            lines_auxiliary_orig2.emplace_back(lines_selected_orig.at(2));
            lines_auxiliary_orig2.emplace_back(lines_selected_orig.at(3));
            for(auto it = label_lines_auxiliary_orig2.begin() ; it != label_lines_auxiliary_orig2.end() ; ++it)
            {
                lines_auxiliary_orig2.emplace_back(lines_lsd_orig[*it]);
                cout << *it << " ";
            }
            cout << endl;

            intersection_pts_orig[0] = calcVanishingPoint(lines_auxiliary_orig1);
            intersection_pts_orig[1] = calcVanishingPoint(lines_auxiliary_orig2);
        #endif

            cout << "The intersection points in origin image are:" << endl;
            cout << intersection_pts_orig << endl;

        #if calcResiduals
            // Calculate residuals to optimize the quaternion of rotation.
            residuals_tmp = sqrt(pow((intersection_pts_orig[2] - intersection_pts_orig[3]).x , 2) + pow((intersection_pts_orig[2] - intersection_pts_orig[3]).y , 2));
            triangle_ori.emplace_back(residuals_tmp);
            residuals_tmp = sqrt(pow((intersection_pts_orig[2] - intersection_pts_orig[4]).x , 2) + pow((intersection_pts_orig[2] - intersection_pts_orig[4]).y , 2));
            triangle_ori.emplace_back(residuals_tmp);
            residuals_tmp = sqrt(pow((intersection_pts_orig[4] - intersection_pts_orig[3]).x , 2) + pow((intersection_pts_orig[4] - intersection_pts_orig[3]).y , 2));
            triangle_ori.emplace_back(residuals_tmp);
        #endif

            R_w_c = calcRotation(camera_inner , intersection_pts_orig);
            t_c_w = calcPose(camera_inner , intersection_pts_orig , 60);
            //Get the transform matrix between camera and target object.

        /************************************************************************************/
            while(ros::ok())
            {
                int exit_flag_proj = 1;
                label_lines_selected_proj.clear();
                lines_selected_proj.clear();
                ROS_INFO("Please select the same lines in projection image.");
                for(int i = 0 ; i < 4 ; i++)
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
                project_lsd->drawSegments(test_mat , lines_selected_proj);
                for(int i = 0 ; i < lines_selected_proj.size() ; ++i)
                {
                    cv::putText(test_mat , to_string(label_lines_selected_proj[i]) , cv::Point((lines_selected_proj[i](0) + lines_selected_proj[i](2)) / 2 , (lines_selected_proj[i](1) + lines_selected_proj[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
                cv::imshow("proj image selected" , test_mat);
                cv::waitKey(1000);
                //Select lines that will be used in calibration manually and show them
                intersection_pts_proj = calcIntersectionPoint(lines_selected_proj);

            #if useAuxiliaryLines
                //选择投影图像用于消失点计算的辅助直线
                ROS_INFO("Please enter how many auxiliary lines in first direction to selected in projection image.");
                cin >> auxiliary_num;
                ROS_INFO("Please selected lines.");
                for(int i = 0 ; i < auxiliary_num ; i++)
                {
                    cin >> label_line_selected;
                    label_lines_auxiliary_proj1.emplace_back(label_line_selected);
                }
                lines_auxiliary_proj1.emplace_back(lines_selected_proj.at(0));
                lines_auxiliary_proj1.emplace_back(lines_selected_proj.at(1));
                for(auto it = label_lines_auxiliary_proj1.begin() ; it != label_lines_auxiliary_proj1.end() ; ++it)
                {
                    lines_auxiliary_proj1.emplace_back(lines_lsd_proj[*it]);
                    cout << *it << " ";
                }
                cout << endl;

                ROS_INFO("Please enter how many auxiliary lines in second direction to selected in projection image.");
                cin >> auxiliary_num;.
                ROS_INFO("Please selected lines.");
                for(int i = 0 ; i < auxiliary_num ; i++)
                {
                    cin >> label_line_selected;
                    label_lines_auxiliary_proj2.emplace_back(label_line_selected);
                }
                lines_auxiliary_proj2.emplace_back(lines_selected_proj.at(2));
                lines_auxiliary_proj2.emplace_back(lines_selected_proj.at(3));
                for(auto it = label_lines_auxiliary_proj2.begin() ; it != label_lines_auxiliary_proj2.end() ; ++it)
                {
                    lines_auxiliary_proj2.emplace_back(lines_lsd_proj[*it]);
                    cout << *it << " ";
                }
                cout << endl;

                intersection_pts_proj[0] = calcVanishingPoint(lines_auxiliary_proj1);
                intersection_pts_proj[1] = calcVanishingPoint(lines_auxiliary_proj2);
            #endif

                cout << "The intersection point in projection image are:" << endl;
                cout << intersection_pts_proj << endl;

                R_w_l = calcRotation(camera_inner , intersection_pts_proj);//现在都暂定的是相机内参的标定是准确的
                R_l_c = R_w_l * (R_w_c.inverse());
                cout << " r_l_c is " << R_l_c  << endl; 

                t_l_w = calcPose(camera_inner , intersection_pts_proj , 60);
                t_l_w = R_l_c.inverse() * t_l_w;//如果是小角度的话其实可以忽略掉
                t_l_c = t_c_w - t_l_w;
                cout << " t_l_c is " << t_l_w << endl;

                Eigen::Matrix3d rotation_tmp = R_l_c * rotation_matrix;//这里应该不是简单的叠加，是在原有基础上再转 0 . 1 2
                ROS_INFO("matrix this group is \n");
                cout << rotation_tmp << endl;;
                Eigen::Quaterniond q_tmp(rotation_tmp);

                Eigen::Vector3d transpose_tmp = transform_vector + t_l_c;
                ROS_INFO("transpose this group is \n");
                cout << transpose_tmp << endl;
                //Get the transform matrix between lidar and target object. Then calculate the result in this group

        #if calcResiduals
                cv::Mat test_mat1;
                test_mat1 = test.getProjectionImage(rotation_tmp , t_l_c / 100);

                cv::medianBlur(test_mat1 , test_mat1 , 3);
                cv::fastNlMeansDenoising(test_mat1 , test_mat1);
                project_lsd->detect(test_mat1 , lines_lsd_proj_R);
                sort(lines_lsd_proj_R.begin() , lines_lsd_proj_R.end() , sort_lines_by_length());
                lines_lsd_proj_R.resize(max_proj_lines_num);
                cv::Mat lsd_proj_gray_R = test_mat1.clone();
                project_lsd->drawSegments(lsd_proj_gray_R , lines_lsd_proj_R);
                for(int i = 0 ; i < lines_lsd_proj_R.size() ; ++i)
                {
                    cv::putText(lsd_proj_gray_R , to_string(i) , cv::Point((lines_lsd_proj_R[i](0) + lines_lsd_proj_R[i](2)) / 2 , (lines_lsd_proj_R[i](1) + lines_lsd_proj_R[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
                cv::imshow("projection_residuals" , lsd_proj_gray_R);
                cv::waitKey(3000);

                label_lines_selected_proj_R.clear();
                lines_selected_proj_R.clear();
                triangle_res.clear();

                ROS_INFO("Please select the same lines in new projection image to calculate residuals.");
                for(int i = 0 ; i < 4 ; i++)
                {
                    cin >> label_line_selected;
                    label_lines_selected_proj_R.emplace_back(label_line_selected);
                }
                cout << "You have choosen lines in new projection image whose label are :" << endl;
                for(auto it = label_lines_selected_proj_R.begin() ; it != label_lines_selected_proj_R.end() ; ++it)
                {
                    lines_selected_proj_R.emplace_back(lines_lsd_proj_R[*it]);
                    cout << *it << " ";
                }
                cout << endl;

                project_lsd->drawSegments(test_mat1 , lines_selected_proj_R);
                for(int i = 0 ; i < lines_selected_proj_R.size() ; ++i)
                {
                    cv::putText(test_mat1 , to_string(label_lines_selected_proj_R[i]) , cv::Point((lines_selected_proj_R[i](0) + lines_selected_proj_R[i](2)) / 2 , (lines_selected_proj_R[i](1) + lines_selected_proj_R[i](3)) / 2 ) ,cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                }
                cv::imshow("proj image selected residuals" , test_mat1);
                cv::waitKey(1000);

                intersection_pts_proj_R = calcIntersectionPoint(lines_selected_proj_R);
                cout << intersection_pts_proj_R << endl;

                residuals_tmp = sqrt(pow((intersection_pts_proj_R[2] - intersection_pts_proj_R[3]).x , 2) + pow((intersection_pts_proj_R[2] - intersection_pts_proj_R[3]).y , 2));
                triangle_res.emplace_back(residuals_tmp);
                residuals_tmp = sqrt(pow((intersection_pts_proj_R[2] - intersection_pts_proj_R[4]).x , 2) + pow((intersection_pts_proj_R[2] - intersection_pts_proj_R[4]).y , 2));
                triangle_res.emplace_back(residuals_tmp);
                residuals_tmp = sqrt(pow((intersection_pts_proj_R[4] - intersection_pts_proj_R[3]).x , 2) + pow((intersection_pts_proj_R[4] - intersection_pts_proj_R[3]).y , 2));
                triangle_res.emplace_back(residuals_tmp);

                residuals[0] = (triangle_ori[0] / triangle_res[0] - triangle_ori[1] /  triangle_res[1]) * 50;
                residuals[1] = (triangle_ori[0] / triangle_res[0] - triangle_ori[2] /  triangle_res[2]) * 50;
                residuals[2] = (intersection_pts_orig[3] - intersection_pts_orig[4]).y / (intersection_pts_orig[3] - intersection_pts_orig[4]).x - (intersection_pts_proj_R[3] - intersection_pts_proj_R[4]).y / (intersection_pts_proj_R[3] - intersection_pts_proj_R[4]).x;
                cout << "The residuals of this group is" << residuals[0] << " "  << residuals[1] << " " << residuals[2] << endl;

                res_weight_sum = res_weight_sum + abs(1 / (residuals[0] * residuals[1] * residuals[2]));
                q_tmp.coeffs() = q_tmp.coeffs() * abs(1 / (residuals[0] * residuals[1] * residuals[2]));
                transpose_tmp = transpose_tmp * abs(1 / (residuals[0] * residuals[1] * residuals[2]));
        #endif


                q_container.emplace_back(q_tmp);
                t_container.emplace_back(transpose_tmp);

                ROS_INFO("Type any key except 0 to add group of lines in projection image to optimize result.");
                cin >> exit_flag_proj;
                if(exit_flag_proj)
                {
                    continue;
                }
                else
                {
                    break;
                }
            }

            ROS_INFO("Type any key except 0 to add group of lines in origin image to optimize result.");
            cin >> exit_flag_orig;
            if(exit_flag_orig)
            {
                continue;
            }
            else
            {
                break;
            }
        }

        ROS_INFO("Exit!");
        cv::destroyAllWindows();
    }

    Eigen::Quaterniond q_result(0.0 , 0.0 , 0.0 , 0.0);
    Eigen::Vector3d t_result(0.0 , 0.0 , 0.0);
    for(int i = 0 ; i < q_container.size() ; i++)
    {
        if(q_container[i].z() < 0)
        {
            q_result.coeffs() = q_result.coeffs() - q_container[i].coeffs(); 
        }
        else if(q_container[i].z() > 0)
        {
            q_result.coeffs() = q_result.coeffs() + q_container[i].coeffs(); 
        }
        t_result = t_result + t_container[i];
    }
#if calcResiduals
    q_result.coeffs() = q_result.coeffs() / res_weight_sum;
    t_result = t_result / res_weight_sum;
#else
    q_result.coeffs() = q_result.coeffs() / (float)q_container.size();
    t_result = t_result / (float)t_container.size();
#endif
    q_result.normalize();

    Eigen::Matrix3d result_matrix;
    result_matrix = q_result.toRotationMatrix();//TODO:做个施密特正交化
    if(getAngle)
    {
        Eigen::Vector3d result_euler = result_matrix.eulerAngles(2 , 1 , 0);
        ROS_INFO("The rotation angle between lidar and camera is: \n");
        cout <<  result_euler << endl;
    }
    else if(getMat)
    {
        ROS_INFO("The rotation angle between lidar and camera is: \n");
        cout << result_matrix << endl;
    }
    ROS_INFO("The transpose between lidar and camera is : \n");
    cout << t_result << endl;
    

    cv::waitKey(0);
    return 0;
}
