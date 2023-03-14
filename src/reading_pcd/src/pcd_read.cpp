#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <cstring>
#include <sensor_msgs/PointCloud2.h>
#include <string>
using std::string;
string file_name;
using namespace std;
// bool flag_ = true;
int num = 0;
int main(int argc,char** argv)
{
	/* ROS中读取pcd点云文件并在rviz显示 */
    ros::init (argc, argv, "random_complex_scene");
    ros::NodeHandle nh;
    nh.param<string>("file_name", file_name, "/pcd/hard.pcd");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_generator/global_cloud", 1);
    pcl::PointCloud<pcl::PointXYZ> rviz_cloud;
    sensor_msgs::PointCloud2 rviz_output;
    std::cout << file_name<<std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("reading_pcd") + file_name, rviz_cloud);
    pcl::toROSMsg(rviz_cloud, rviz_output);
    rviz_output.header.frame_id = "world";
    std::cout <<"ok"<<std::endl;
    int num = 0;
    ros::Rate loop_rate(50);
    while (num < 500 ){   
        pcl_pub.publish(rviz_output);
        ros::spinOnce();
        loop_rate.sleep();
        num ++ ;
    }
    return 0;
}
