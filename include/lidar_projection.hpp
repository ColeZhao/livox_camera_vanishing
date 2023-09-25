#ifndef __LIDAR_PROJECTION_HPP__
#define __LIDAR_PROJECTION_HPP__

#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <include/common.hpp>

using namespace std;

typedef Eigen::Matrix<double , 6 , 1> Vector6d;

class LidarProjection
{
    public:
        LidarProjection(const std::string &pcd_file);
        ~LidarProjection(){};

        ros::NodeHandle nh_;

        void projection(const Vector6d &extrinsic_params , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img);
        cv::Mat getProjectionImage(const Vector6d & extrinsic_params);

        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_orin_cloud;

        double min_depth = 1.0;
        double max_depth = 10.0;


};

LidarProjection::LidarProjection(const std::string &pcd_file)
{   
    lidar_orin_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    if(pcl::io::loadPCDFile(pcd_file , *lidar_orin_cloud) == -1)
    {
        ROS_ERROR("Failed load PCD file.");
        exit(1);
    }
    else
    {
        ROS_INFO("Load PCD file sucessfully!");
    }
    cout<<pcd_file << endl;
};

void LidarProjection::projection(const Vector6d &extrinsic_params , const pcl::PointCloud<pcl::PointXYZI>::Ptr &lidar_cloud , cv::Mat &projection_img)
{
    int test_num = 0;
    std::vector<cv::Point3d> pts_3d;
    std::vector<float> intensity_list;
    Eigen::AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(extrinsic_params[0] , Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(extrinsic_params[1] , Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(extrinsic_params[2] , Eigen::Vector3d::UnitX());

    for(u_int32_t i = 0 ; i < lidar_cloud->size() ; i++)
    {
        pcl::PointXYZI point_3d = lidar_cloud->points[i];
        float depth = calcPointDepth(point_3d);
        if(depth > min_depth && depth < max_depth)
        {
            pts_3d.emplace_back(cv::Point3f(point_3d.x , point_3d.y , point_3d.z));
            intensity_list.emplace_back(lidar_cloud->points[i].intensity);
            test_num++;
        }
    }
    cout<<"test_num:" << test_num<< endl;
    cout<<"lidar_point num:" << lidar_cloud->size() <<endl;
    pts_3d.clear();
    intensity_list.clear();
};

cv::Mat LidarProjection::getProjectionImage(const Vector6d &extrinsic_params)
{
    cv::Mat projection_img;
    projection(extrinsic_params , lidar_orin_cloud , projection_img);
    return projection_img;
};


#endif