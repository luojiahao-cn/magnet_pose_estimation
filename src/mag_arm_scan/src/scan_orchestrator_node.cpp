#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>

#include <mag_arm_scan/scan_orchestrator_node.hpp>

#include <algorithm>
#include <string>
#include <vector>

ScanOrchestrator::ScanOrchestrator(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
{
    // 默认参数
    frame_id_ = "world";
    yaw_ = 0.0;
    vol_min_ = {0.0, 0.0, 0.0};
    vol_max_ = {0.10, 0.10, 0.05};
    step_ = {0.02, 0.02, 0.02};
    bool autostart = false;

    // 读取参数
    std::vector<double> vmin, vmax, vstep;
    pnh_.param<std::string>("frame_id", frame_id_, frame_id_);
    pnh_.param<double>("yaw", yaw_, yaw_);
    pnh_.param<bool>("autostart", autostart, autostart);
    if (pnh_.getParam("volume_min", vmin) && vmin.size() == 3)
        vol_min_ = {vmin[0], vmin[1], vmin[2]};
    if (pnh_.getParam("volume_max", vmax) && vmax.size() == 3)
        vol_max_ = {vmax[0], vmax[1], vmax[2]};
    if (pnh_.getParam("step", vstep) && vstep.size() == 3)
        step_ = {vstep[0], vstep[1], vstep[2]};

    // 发布器（使用 latch，便于 RViz 获取最后一次航点）
    waypoints_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/mag_arm_scan/scan_waypoints", 1, /*latch=*/true);

    // 服务
    srv_start_ = pnh_.advertiseService("start", &ScanOrchestrator::startSrv, this);
    srv_stop_ = pnh_.advertiseService("stop", &ScanOrchestrator::stopSrv, this);

    ROS_INFO(
        "[mag_arm_scan] ready. frame=%s yaw=%.3f vol_min(%.3f,%.3f,%.3f) vol_max(%.3f,%.3f,%.3f) step(%.3f,%.3f,%.3f)",
        frame_id_.c_str(),
        yaw_,
        vol_min_.x,
        vol_min_.y,
        vol_min_.z,
        vol_max_.x,
        vol_max_.y,
        vol_max_.z,
        step_.x,
        step_.y,
        step_.z);

    if (autostart)
        publishWaypoints();
}

bool ScanOrchestrator::startSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
    publishWaypoints();
    return true;
}

bool ScanOrchestrator::stopSrv(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
    // 仅发布空数组作为"停止/清空"的信号（可按需更改）
    geometry_msgs::PoseArray arr;
    arr.header.stamp = ros::Time::now();
    arr.header.frame_id = frame_id_;
    waypoints_pub_.publish(arr);
    ROS_WARN("[mag_arm_scan] stop: cleared waypoints");
    return true;
}

void ScanOrchestrator::publishWaypoints()
{
    geometry_msgs::PoseArray arr;
    arr.header.stamp = ros::Time::now();
    arr.header.frame_id = frame_id_;

    // 生成 Z 分层、XY 扫描（蛇形遍历）
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    geometry_msgs::Pose pose;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    auto clamp_positive = [](double v)
    { return v > 0 ? v : 1e-9; };
    double dx = clamp_positive(step_.x), dy = clamp_positive(step_.y), dz = clamp_positive(step_.z);

    int layer = 0;
    for (double z = vol_min_.z; z <= vol_max_.z + 1e-12; z += dz, ++layer)
    {
        bool reverse = (layer % 2 == 1); // 交替蛇形
        std::vector<geometry_msgs::Pose> layer_pts;
        for (double y = vol_min_.y; y <= vol_max_.y + 1e-12; y += dy)
        {
            std::vector<geometry_msgs::Pose> row_pts;
            for (double x = vol_min_.x; x <= vol_max_.x + 1e-12; x += dx)
            {
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;
                row_pts.push_back(pose);
            }
            if (reverse)
                std::reverse(row_pts.begin(), row_pts.end());
            layer_pts.insert(layer_pts.end(), row_pts.begin(), row_pts.end());
        }
        arr.poses.insert(arr.poses.end(), layer_pts.begin(), layer_pts.end());
    }

    waypoints_pub_.publish(arr);
    ROS_INFO("[mag_arm_scan] published %zu waypoints", arr.poses.size());
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "scan_orchestrator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ScanOrchestrator node(nh, pnh);
    ros::spin();
    return 0;
}
