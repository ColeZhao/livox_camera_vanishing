#ifndef __LIDAR_PROJECTION_HPP__
#define __LIDAR_PROJECTION_HPP__

#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <include/common.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

typedef Eigen::Matrix<double , 6 , 1> Vector6d;

class LidarProjection
{
    public:
        LidarProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud);
        ~LidarProjection(){};

        ros::NodeHandle nh_;

        void projection(const Vector6d &extrinsic_params , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img);
        void projection(const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img);
        cv::Mat getProjectionImage(const Vector6d &extrinsic_params);
        cv::Mat getProjectionImage(const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector);
        cv::Mat fillImg(const cv::Mat &input_img);
        cv::Mat getComparedImage(const cv::Mat &input_img , const Vector6d &extrinsic_params);
        cv::Mat getComparedImage(const cv::Mat &input_img , const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector);

        std::vector<cv::Point3f> pts_3d;

        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_orig_cloud;

        double min_depth = 1.0;
        double max_depth =30.0;
        double fx , fy , cx , cy;
        double k1 , k2 , p1 , p2 , k3;
        int height , width;
        double depth_weight = 0.5;


};

LidarProjection::LidarProjection(const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud)
{   
    lidar_orig_cloud = lidar_cloud;
};

void LidarProjection::projection(const Vector6d &extrinsic_params , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img)
{ 
    int test_num = 0;
    // std::vector<cv::Point3f> pts_3d;
    std::vector<float> intensity_list;
    Eigen::AngleAxisd rotation_vector;
    double max_depth_pt = 0;
    double max_intensity_pt = 0;

    rotation_vector = Eigen::AngleAxisd(extrinsic_params[0] , Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(extrinsic_params[1] , Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(extrinsic_params[2] , Eigen::Vector3d::UnitX());

    #pragma omp parallel for num_threads(16)
    for(u_int32_t i = 0 ; i < lidar_cloud->size() ; i++)
    {
        pcl::PointXYZI point_3d = lidar_cloud->points[i];
        float depth = calcPointDepth(point_3d);
        if(depth > min_depth && depth < max_depth)
        {
            pts_3d.emplace_back(cv::Point3f(point_3d.x , point_3d.y , point_3d.z));
            intensity_list.emplace_back(lidar_cloud->points[i].intensity);
            if(depth > max_depth_pt)
            {
                max_depth_pt = depth;
            }
            if(point_3d.intensity > max_intensity_pt)
            {
                max_intensity_pt = point_3d.intensity;
            }
            test_num++;
        }
    }
    cv::Mat camera_matrix = (cv::Mat_<double>(3 , 3) << fx , 0.0 , cx , 0.0 , fy , cy , 0.0 , 0.0 , 1.0); 

    cv::Mat distortion_coeff = (cv::Mat_<double>(1 , 5) << k1 , k2 , p1 , p2 , k3);

    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector.angle() * rotation_vector.axis().transpose()[0] , rotation_vector.angle() * rotation_vector.axis().transpose()[1] , rotation_vector.angle() * rotation_vector.axis().transpose()[2]);

    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << extrinsic_params[3] , extrinsic_params[4] , extrinsic_params[5]);

    std::vector<cv::Point2f> pts_2d;

    cv::projectPoints(pts_3d , r_vec , t_vec , camera_matrix , distortion_coeff , pts_2d);

    cv::Mat image_project = cv::Mat::zeros(height , width , CV_16UC1);
    cv::Mat rgb_image_project = cv::Mat::zeros(height , width , CV_8UC3);

    #pragma omp parallel for num_threads(16)
    for(u_int64_t i = 0 ; i < pts_2d.size() ; ++i)
    {
        cv::Point2f point_2d = pts_2d[i];
        if(point_2d.x <= 0 || point_2d.x >= width || point_2d.y <= 0 || point_2d.y >= height)
        {
            continue;
        }
        else
        {
            float depth = sqrt(pow(pts_3d[i].x - extrinsic_params[3] , 2) + pow(pts_3d[i].y - extrinsic_params[4], 2) +pow(pts_3d[i].z - extrinsic_params[5], 2));
            if(image_project.at<ushort>(point_2d.y, point_2d.x) == 0 || depth < image_project.at<ushort>(point_2d.y, point_2d.x))
            {
                float intensity = intensity_list[i];
                float gray = depth_weight * depth / max_depth_pt * 65535 + (1 - depth_weight) * intensity / max_intensity_pt * 65535;
                image_project.at<ushort>(point_2d.y, point_2d.x) = gray;
                rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = depth / max_depth_pt * 255;
                rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = intensity / max_intensity_pt * 255;
            }
        }
    }
    image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);

    projection_img = image_project.clone();
    pts_3d.clear();
    intensity_list.clear();
};

void LidarProjection::projection(const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img)
{ 
    int test_num = 0;
    std::vector<float> intensity_list;
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    double max_depth_pt = 0;
    double max_intensity_pt = 0;
    for(u_int32_t i = 0 ; i < lidar_cloud->size() ; i++)
    {
        pcl::PointXYZI point_3d = lidar_cloud->points[i];
        float depth = calcPointDepth(point_3d);
        if(depth > min_depth && depth < max_depth)
        {
            pts_3d.emplace_back(cv::Point3f(point_3d.x , point_3d.y , point_3d.z));
            intensity_list.emplace_back(lidar_cloud->points[i].intensity);
            if(depth > max_depth_pt)
            {
                max_depth_pt = depth;
            }
            if(point_3d.intensity > max_intensity_pt)
            {
                max_intensity_pt = point_3d.intensity;
            }
            test_num++;
        }
    }
    cv::Mat camera_matrix = (cv::Mat_<double>(3 , 3) << fx , 0.0 , cx , 0.0 , fy , cy , 0.0 , 0.0 , 1.0); 

    cv::Mat distortion_coeff = (cv::Mat_<double>(1 , 5) << k1 , k2 , p1 , p2 , k3);

    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector.angle() * rotation_vector.axis().transpose()[0] , rotation_vector.angle() * rotation_vector.axis().transpose()[1] , rotation_vector.angle() * rotation_vector.axis().transpose()[2]);

    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << transform_vector[0] , transform_vector[1] , transform_vector[2]);

    std::vector<cv::Point2f> pts_2d;

    cv::projectPoints(pts_3d , r_vec , t_vec , camera_matrix , distortion_coeff , pts_2d);

    cv::Mat image_project = cv::Mat::zeros(height , width , CV_16UC1);
    cv::Mat rgb_image_project = cv::Mat::zeros(height , width , CV_8UC3);


    for(u_int64_t i = 0 ; i < pts_2d.size() ; ++i)
    {
        cv::Point2f point_2d = pts_2d[i];
        if(point_2d.x <= 0 || point_2d.x >= width || point_2d.y <= 0 || point_2d.y >= height)
        {
            continue;
        }
        else
        {

            float depth = sqrt(pow(pts_3d[i].x - transform_vector[0] , 2) + pow(pts_3d[i].y - transform_vector[1], 2) +pow(pts_3d[i].z - transform_vector[2], 2));
            if(image_project.at<ushort>(point_2d.y, point_2d.x) == 0 || depth < image_project.at<ushort>(point_2d.y, point_2d.x))
            {
                float intensity = intensity_list[i];
                float gray = depth_weight * depth / max_depth_pt * 65535 +  intensity / max_intensity_pt * 65535 *(1 - depth_weight);
                image_project.at<ushort>(point_2d.y, point_2d.x) = gray;
                rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[0] = depth / max_depth_pt * 255;
                rgb_image_project.at<cv::Vec3b>(point_2d.y, point_2d.x)[1] = intensity / max_intensity_pt * 255;

            }
        }
    }

    image_project.convertTo(image_project, CV_8UC1, 1 / 256.0);

    projection_img = image_project.clone();
    pts_3d.clear();
    intensity_list.clear();
};

cv::Mat LidarProjection::getProjectionImage(const Vector6d &extrinsic_params)
{
    cv::Mat projection_img;
    projection(extrinsic_params , lidar_orig_cloud , projection_img);
    return projection_img;
};

cv::Mat LidarProjection::getProjectionImage(const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector)
{
    cv::Mat projection_img;
    projection(rotation_matrix , transform_vector , lidar_orig_cloud , projection_img);
    return projection_img;
};


cv::Mat LidarProjection::fillImg(const cv::Mat &input_img)
{   
    cv::Mat output_img = input_img.clone();
    for( int y = 2 ; y < input_img.rows - 2 ; y++)
    {
        for( int x = 2 ; x < input_img.cols - 2 ; x++)
        {
            if(input_img.at<uchar>(y , x) == 0)
            {
                if(input_img.at<uchar>(y - 1) != 0)
                {
                    output_img.at<uchar>(y , x) = input_img.at<uchar>(y - 1 , x);
                }
                else if(input_img.at<uchar>(y , x - 1) != 0)
                {
                    output_img.at<uchar>(y , x) = input_img.at<uchar>(y , x - 1);
                }
            }
        }
    }
    return output_img;
}

cv::Mat LidarProjection::getComparedImage(const cv::Mat &input_img , const Eigen::Matrix3d &rotation_matrix , const Eigen::Vector3d &transform_vector)
{
    cv::Mat compared_img , src_img , lidar_img;
    lidar_img = getProjectionImage(rotation_matrix , transform_vector);
    cv::Mat map_img = cv::Mat::zeros(height, width, CV_8UC3);
    for (int x = 0; x < map_img.cols; x++) {
        for (int y = 0; y < map_img.rows; y++) {
            uint8_t r, g, b;
            float norm = lidar_img.at<uchar>(y, x) / 256.0;
            mapJet(norm, 0, 1, r, g, b);
            map_img.at<cv::Vec3b>(y, x)[0] = b;
            map_img.at<cv::Vec3b>(y, x)[1] = g;
            map_img.at<cv::Vec3b>(y, x)[2] = r;
        }
    }
    if(input_img.type() == CV_8UC3)
    {
        compared_img = 0.8 * input_img + 0.5 * map_img;
    }
    else
    {
        cv::cvtColor(input_img , src_img , cv::COLOR_GRAY2BGR);
        compared_img = 0.8 * src_img + 0.5 * map_img;
    }
    return compared_img;
}

#endif