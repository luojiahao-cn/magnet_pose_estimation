#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

struct Vec3
{
    double x{0}, y{0}, z{0};
};

class ScanOrchestrator
{
public:
    ScanOrchestrator(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    bool startSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool stopSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    void publishWaypoints();

    ros::NodeHandle nh_, pnh_;
    ros::Publisher waypoints_pub_;
    ros::ServiceServer srv_start_, srv_stop_;
    std::string frame_id_;
    double yaw_;
    Vec3 vol_min_, vol_max_, step_;
};