
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
using namespace std;
 
//删除字符串中空格，制表符tab等无效字符
string Trim(string& str)
{
    str.erase(0, str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
}
 
void csv2pointCloud(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    cloud->points.clear();
    ifstream fin(filename); //打开文件流操作
    string line;
    while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
    {
        istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
        vector<string> fields; //声明一个字符串向量
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
        {
            fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
        }
        pcl::PointXYZI p;
 
        p.x = atof(Trim(fields[1]).c_str());
        p.y = atof(Trim(fields[2]).c_str());
        p.z = atof(Trim(fields[3]).c_str());
        p.intensity = atof(Trim(fields[4]).c_str());
        cloud->points.push_back(p);
    }
 
}
 
int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::string filename = "/home/collar/data/sample/pcds/000000.csv";
    csv2pointCloud(filename, cloud);
    cloud->width = (int)cloud->points.size();
    cloud->height = 1;
    pcl::io::savePCDFileASCII("/home/collar/data/sample/pcds/000000.pcd", *cloud);
 
    return 0;
}