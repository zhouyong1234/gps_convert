#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "LocalCartesian.hpp"

struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
ros::Publisher state_pub_;
nav_msgs::Path ros_path_;
bool init;
my_pose init_pose;
GeographicLib::LocalCartesian geoConverter;
double xyz[3];

std::ofstream out_file("/home/touchair/test/gnss.txt", std::ios::out);


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    //初始化
    if(!init)
    {
        geoConverter.Reset(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
        // geoConverter.Reset(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0);
        init = true;
    }

    geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude, xyz[0], xyz[1], xyz[2]);
    // geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0, xyz[0], xyz[1], xyz[2]);


    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    out_file << x << "," << y << "," << z << "\n";


    //发布轨迹
    ros_path_.header.frame_id = "path";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    ros_path_.poses.push_back(pose);

    ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );

    state_pub_.publish(ros_path_);

}
int main(int argc,char **argv)
{
    init = false;
    ros::init(argc,argv,"gps_subscriber");
    ros::NodeHandle n;
    ros::Subscriber pose_sub=n.subscribe("/gps",10,gpsCallback);
        
    state_pub_ = n.advertise<nav_msgs::Path>("/gps_path", 10);



    ros::spin();
    return 0;
}

