#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <string>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <Eigen/Eigen>
#include <deque>
#include <unordered_set>
#include <sstream>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "state.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>

typedef std::vector<std::pair<double, double>> CandidateSol;


std::vector<std::vector<CandidateSol>>  candidatesolutions;

using Eigen::Vector4d;
using Eigen::Vector3i;
using Eigen::Vector3d;
using std::string;
//*visual relative
static string mesh_resource;
int color_r = 0, color_g = 0, color_b = 1.0;
static double color_a = 0.8;
bool stop_ = true;
std::vector<visualization_msgs::Marker> Agentmesh;
visualization_msgs::Marker meshROS;

visualization_msgs::MarkerArray MarkerArray;//target
visualization_msgs::MarkerArray MarkerBaseArray;//stop position

geometry_msgs::PoseStamped current_pose;
double showspin;

string _frame_id;
string inputFile;

// 动态路径数据包
std::vector<std::deque<Vector3d>> PoseBag;
std::deque<Vector3d> Posebag;

std::vector<nav_msgs::Path> UAVPath;
nav_msgs::Path UAVpath;
std::vector<nav_msgs::Path> UAVTraj;
nav_msgs::Path path_tmp;
//添加无人机个数

nav_msgs::Path UGVpath;

std::vector<ros::Publisher> Spath_Pub;
ros::Publisher Spath_pub;

std::vector<ros::Publisher> Dpath_Pub;
ros::Publisher Dpath_pub;

ros::Publisher Base_Pub;

ros::Publisher MarkerBasePub;

ros::Publisher MarkerPub;

std::vector<ros::Publisher> mesh_Pub;
ros::Publisher mesh_pub;
std::vector<Vector3i> colors = { Vector3i(255,85,0), Vector3i(0,85,0), Vector3i(0,85,127), Vector3i(0,0,255), Vector3i(85,0,255) };


std::vector<std::vector<State>> agents;

int agv_count = 0;
geometry_msgs::Quaternion ToQuaternion(double yaw, double pitch = 0, double roll = 0) {// yaw (Z), pitch (Y), roll (X)
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

//*发布rviz可视化信息*//
void timerCallback(const ros::TimerEvent& e) {
    if (stop_)
        return;
    Vector3d tm_pose;
    color_r = 0, color_g = 0, color_b = 0;
    tm_pose = PoseBag[0].front();
    int num = 0;
    // std::cout << "start publishing" << std::endl;
    Base_Pub.publish(UGVpath);

    MarkerBasePub.publish(MarkerBaseArray);
    MarkerPub.publish(MarkerArray);
    UAVTraj.resize(agv_count);
    for (int i = 0; i < agv_count; i++) {
        // std::cout << "UAVPath" << i << std::endl;
        Dpath_Pub[i].publish(UAVPath[i]);
        if (PoseBag[i].empty())
            continue;
        tm_pose = PoseBag[i].front();
        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = "world";
        path_tmp.header.frame_id = _frame_id;
        path_tmp.header.stamp = ros::Time::now();
        UAVTraj[i] = path_tmp;//会造成内存浪费
        current_pose.pose.position.x = tm_pose(0);
        current_pose.pose.position.y = tm_pose(1);
        current_pose.pose.position.z = tm_pose(2);
        UAVTraj[i].poses.push_back(current_pose);
        Spath_Pub[i].publish(UAVTraj[i]);
        meshROS.header.stamp = ros::Time::now();
        meshROS.id = i;
        meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action = visualization_msgs::Marker::ADD;
        // meshROS.pose.orientation = ToQuaternion(tm_pose(2));
        meshROS.scale.x = 2.5;
        meshROS.scale.y = 2.5;
        meshROS.scale.z = 2.5;
        meshROS.color.a = color_a;
        meshROS.color.r = colors[i][0];
        meshROS.color.g = colors[i][1];
        meshROS.color.b = colors[i][2];
        meshROS.mesh_resource = mesh_resource;
        meshROS.header.frame_id = _frame_id;
        meshROS.ns = "mesh";
        Agentmesh.push_back(meshROS);
        Agentmesh[i].pose.position.x = tm_pose(0);
        Agentmesh[i].pose.position.y = tm_pose(1);
        Agentmesh[i].pose.position.z = tm_pose(2);
        mesh_Pub[i].publish(Agentmesh[i]);

        PoseBag[i].pop_front();
        // std::cout << "finished publishing " << i << std::endl;
    }

}

void yaml_read(std::string filename, std::vector<std::vector<State>>& agents) {
    std::cout << filename << std::endl;
    YAML::Node config = YAML::LoadFile(filename);

    auto cnt = config["schedule"].size();
    for (int i = 0; i < cnt; i++) {
        std::string name = "agent" + std::to_string(i);
        std::vector<State> single_solution;
        std::cout << name << std::endl;
        for (auto s : config["schedule"][name]) {
            State state;
            double scale = 5;
            if (s["z"]) {
                state = State(s["x"].as<double>() * scale, s["y"].as<double>() * scale, s["z"].as<double>() * scale);
            }
            else {
                state = State(s["x"].as<double>() * scale, s["y"].as<double>() * scale, 0.0);
            }
            // std::cout << s << std::endl;
            single_solution.emplace_back(state);
        }
        agents.emplace_back(single_solution);
    }

    // }

}

void rviz_show(std::vector<std::vector<State>>& solutions) {
    //*rviz可视化
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.stamp = ros::Time::now();
    current_pose.header.frame_id = "world";
    geometry_msgs::Pose stopPose;
    std::unordered_set<geometry_msgs::Pose, boost::hash<geometry_msgs::Pose>> stopSet;
    geometry_msgs::Pose uavPose;
    std::unordered_set<geometry_msgs::Pose, boost::hash<geometry_msgs::Pose>> uavSet;
    Vector3d tm_vec;
    int num = 0;

    for (int i = 0; i < agv_count;i++) {
        stopPose.position.x = solutions[i][0].x;
        stopPose.position.y = solutions[i][0].y;
        stopPose.position.z = solutions[i][0].z;
        stopSet.insert(stopPose);

        UAVpath.header.stamp = ros::Time::now();
        UAVpath.header.frame_id = "world";
        PoseBag.push_back(Posebag);//empty deque
        UAVPath.push_back(UAVpath);//empty path

        UGVpath.header.stamp = ros::Time::now();
        UGVpath.header.frame_id = "world";
        current_pose.pose.position.x = solutions[i][0].x;
        current_pose.pose.position.y = solutions[i][0].y;
        current_pose.pose.position.z = solutions[i][0].z;
        UGVpath.poses.push_back(current_pose);

        for (int j = 0; j < solutions[i].size();j++) {
            uavPose.position.y = solutions[i][j].y;
            uavPose.position.z = solutions[i][j].z;
            uavPose.position.x = solutions[i][j].x;
            if (stopSet.count(uavPose) == 0) {
                uavSet.insert(uavPose);
            }
            // std::cout<<"i="<<i<<", j="<<j<<std::endl;
            tm_vec(0) = solutions[i][j].x;
            tm_vec(1) = solutions[i][j].y;
            tm_vec(2) = solutions[i][j].z;
            // std::cout<<tm_vec<<std::endl;
            PoseBag[i].push_back(tm_vec);
            current_pose.pose.position.x = solutions[i][j].x;
            current_pose.pose.position.y = solutions[i][j].y;
            current_pose.pose.position.z = solutions[i][j].z;
            UAVPath[i].poses.push_back(current_pose);
        };
    };
    current_pose.pose.position.x = solutions[0][0].x;
    current_pose.pose.position.y = solutions[0][0].y;
    current_pose.pose.position.z = solutions[0][0].z;
    UGVpath.poses.push_back(current_pose);
    
    
    //path marker for ugv&uav
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "ugvpath";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 5;
    marker.scale.y = 5;
    marker.scale.z = 2;
    marker.color.a = color_a;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    int cnt = 0;
    for (auto& pose : stopSet) {
        marker.pose = pose;
        marker.id = cnt++;
        MarkerBaseArray.markers.push_back(marker);
    }
    cnt = 0;
    marker.ns = "uavpath";
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    for (auto& pose : uavSet) {
        marker.pose = pose;
        marker.id = cnt++;
        MarkerArray.markers.push_back(marker);
    }
}




int main(int argc, char* argv[]) {
    //以下为添加部分，初始化ros节点
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    //↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓以下为918日添加↓↓↓↓↓↓↓↓↓↓//

    nh.param("mesh_resource", mesh_resource, string("package://path_search/meshes/hummingbird.mesh"));
    nh.param("frame_id", _frame_id, string("world"));
    nh.param("showspin", showspin, 0.05);
    //添加无人机个数
    nh.param("inputFile", inputFile, string("output.yaml"));//ymlshuru
    //↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑以上为添加↑↑↑↑↑↑↑↑↑↑↑↑//
    ros::Timer Ttimer = nh.createTimer(ros::Duration(showspin), timerCallback);

    ros::Rate rate(10);
    bool status = ros::ok();
    yaml_read(inputFile, agents);
    agv_count = agents.size();
    std::cout << "agv_count = " << agents.size() << std::endl;
    string bstr = "/Dpath/base";
    Base_Pub = nh.advertise<nav_msgs::Path>(bstr, 10);
    string mstr = "/Marker/base";
    MarkerBasePub = nh.advertise<visualization_msgs::MarkerArray>(mstr, 10);
    mstr = "/Marker/agent";
    MarkerPub = nh.advertise<visualization_msgs::MarkerArray>(mstr, 10);
    for (int i = 0; i < agents.size(); i++) {
        string str1 = "/Spath/agent";
        string str2 = "/Dpath/agent";
        string str3 = "/Robot/agent";
        Spath_pub = nh.advertise<nav_msgs::Path>(str1 + std::to_string(i), 2);
        Spath_Pub.push_back(Spath_pub);

        Dpath_pub = nh.advertise<nav_msgs::Path>(str2 + std::to_string(i), 2);
        Dpath_Pub.push_back(Dpath_pub);

        mesh_pub = nh.advertise<visualization_msgs::Marker>(str3 + std::to_string(i), 1);
        mesh_Pub.push_back(mesh_pub);
    }

    rviz_show(agents);



    stop_ = false;
    ros::spin();


    return 0;

}
