#include <iostream>
#include <ros/ros.h>


using namespace std;

string image_file;
string pcd_file;
int main(int argc , char **argv)
{
    ros::init(argc , argv , "lidarCamClib");
    ros::NodeHandle nh;

    nh.param<string>("image_file" , image_file , "");
    nh.param<string>("pcd_file:" , pcd_file ,"");

    


    cout<<"l_c_v_c test!" << endl;
    
    return 0;
}
