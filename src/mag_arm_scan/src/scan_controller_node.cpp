/*
文件: scan_controller_node.cpp
功能概述:
  - 使用 MoveIt 控制机械臂在设定体积内按网格顺序逐点运动，等待稳定后采集磁传感器多帧数据并求均值，
    将(平均磁场, 位姿)保存为 CSV。支持节点启动自动扫描或通过服务触发。

主要职责:
  - 读取参数: 坐标系 frame_id、扫描体积(volume_min/max/step)、姿态(yaw/pitch)、等待时间、话题、输出文件等
  - 生成扫描点: 在体积内按步长生成(x,y,z)网格点，姿态固定为 RPY(0, pitch, yaw)
  - 运动执行: 规划+执行至目标点，等待系统稳定(wait_time)
  - 数据采集: 订阅磁传感器话题，取最近若干帧求均值，写入 CSV
  - 接口提供: /start_scan 服务触发扫描；支持 ~autostart 自动扫描

ROS/MoveIt 接口:
    - 订阅: mag_topic (mag_sensor_node::MagSensorData)，默认 /mag_sensor/data_mT
  - 服务: /start_scan (std_srvs/Trigger)
  - 规划组: fr5v6_arm；命名位姿: ready

参数(私有命名空间 ~):
  - frame_id[string]，yaw[double]，pitch[double]，autostart[bool]
  - mag_topic[string]，wait_time[double]，output_file[string]
  - volume_min[double[3]]，volume_max[double[3]]，step[double[3]]

数据记录:
  - CSV 列: timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z
  - 说明: 当前实现取最近消息窗口内的样本求均值；可按需改为中值/去异常等鲁棒统计

注意事项:
  - 坐标一致性需外部标定(传感器/机械臂/世界坐标)
  - 等待时间与控制器/传感器稳定时间相关，需根据实际系统调优
  - 采样窗口较小(默认10帧)，可根据传感器频率/噪声调整
*/

#include <mag_arm_scan/scan_controller_node.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <iostream>

ScanControllerNode::ScanControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), move_group_("fr5v6_arm")
{
    loadParams();
    generateScanPoints();

    start_scan_srv_ = nh_.advertiseService("start_scan", &ScanControllerNode::startScan, this);
    mag_data_sub_ = nh_.subscribe(mag_topic_, 100, &ScanControllerNode::magDataCallback, this);

    ROS_INFO("ScanControllerNode initialized with %zu scan points", scan_points_.size());
}

void ScanControllerNode::loadParams()
{
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    pnh_.param<double>("yaw", yaw_, 0.0);
    pnh_.param<double>("pitch", pitch_, -M_PI);
    pnh_.param<bool>("autostart", autostart_, true);
    pnh_.param<std::string>("mag_topic", mag_topic_, "/mag_sensor/data_mT");
    pnh_.param<double>("wait_time", wait_time_, 2.0);
    pnh_.param<std::string>("output_file", output_file_, "/tmp/scan_data.csv");

    std::vector<double> default_min = {0.0, 0.0, 0.0};
    std::vector<double> default_max = {0.1, 0.1, 0.05};
    std::vector<double> default_step = {0.02, 0.02, 0.02};

    pnh_.param("volume_min", volume_min_, default_min);
    pnh_.param("volume_max", volume_max_, default_max);
    pnh_.param("step", step_, default_step);
}

void ScanControllerNode::generateScanPoints()
{
    scan_points_.clear();

    for (double x = volume_min_[0]; x <= volume_max_[0]; x += step_[0])
    {
        for (double y = volume_min_[1]; y <= volume_max_[1]; y += step_[1])
        {
            for (double z = volume_min_[2]; z <= volume_max_[2]; z += step_[2])
            {
                geometry_msgs::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;

                tf2::Quaternion q;
                q.setRPY(0, pitch_, yaw_);
                pose.orientation = tf2::toMsg(q);

                scan_points_.push_back(pose);
            }
        }
    }
}

bool ScanControllerNode::moveToPose(const geometry_msgs::Pose& pose)
{
    move_group_.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        success = (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    return success;
}

bool ScanControllerNode::moveToReadyPose()
{
    ROS_INFO("Moving to ready position using named target");

    move_group_.setNamedTarget("ready");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        success = (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    return success;
}

void ScanControllerNode::collectDataAtPoint(const geometry_msgs::Pose& pose)
{
    ROS_INFO("Moving to position: x=%.3f, y=%.3f, z=%.3f", pose.position.x, pose.position.y, pose.position.z);

    if (!moveToPose(pose))
    {
        ROS_ERROR("Failed to move to pose");
        return;
    }

    // Wait for settling
    ros::Duration(wait_time_).sleep();

    // Collect multiple data samples
    const int num_samples = 10;
    std::vector<mag_sensor_node::MagSensorData> samples;
    
    for (int i = 0; i < num_samples; ++i)
    {
        ros::spinOnce();
        if (!collected_data_.empty())
        {
            samples.push_back(collected_data_.back());
        }
        ros::Duration(0.1).sleep();
    }

    // Compute average
    if (!samples.empty())
    {
        double avg_mag_x = 0.0, avg_mag_y = 0.0, avg_mag_z = 0.0;
        for (const auto& sample : samples)
        {
            avg_mag_x += sample.mag_x;
            avg_mag_y += sample.mag_y;
            avg_mag_z += sample.mag_z;
        }
        avg_mag_x /= samples.size();
        avg_mag_y /= samples.size();
        avg_mag_z /= samples.size();

        // Create averaged data
        mag_sensor_node::MagSensorData data = samples.back();  // Use last sample as template
        data.header.stamp = ros::Time::now();
        data.mag_x = avg_mag_x;
        data.mag_y = avg_mag_y;
        data.mag_z = avg_mag_z;
        data.sensor_pose = pose;

        // Save to file
        std::ofstream file(output_file_, std::ios::app);
        if (file.is_open())
        {
            file << data.header.stamp << "," << data.mag_x << "," << data.mag_y << "," << data.mag_z << ","
                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
            file.close();
        }
    }
}

bool ScanControllerNode::startScan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    ROS_INFO("Starting scan with %zu points", scan_points_.size());

    // First move to ready position
    if (!moveToReadyPose())
    {
        ROS_ERROR("Failed to move to ready position");
        res.success = false;
        res.message = "Failed to move to ready position";
        return true;
    }

    // Clear output file
    std::ofstream file(output_file_, std::ios::trunc);
    file << "timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z" << std::endl;
    file.close();

    for (const auto& pose : scan_points_)
    {
        collectDataAtPoint(pose);
    }

    res.success = true;
    res.message = "Scan completed";
    return true;
}

void ScanControllerNode::magDataCallback(const mag_sensor_node::MagSensorData::ConstPtr& msg)
{
    collected_data_.push_back(*msg);
    // Keep only recent data
    if (collected_data_.size() > 10)
    {
        collected_data_.erase(collected_data_.begin());
    }
}

void ScanControllerNode::run()
{
    if (autostart_)
    {
        // 等待控制器启动
        ROS_INFO("Waiting for controller manager services to be available...");
        while (move_group_.getPlanningFrame().empty())
        {
            ROS_INFO("Waiting for MoveGroup to be ready...");
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        }
        ROS_INFO("MoveGroup is ready. Waiting additional time for system stabilization...");

        // 等待系统完全启动
        ros::Duration(3.0).sleep();
        
        ROS_INFO("Starting automatic scan...");
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;
        startScan(req, res);
    }

    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ScanControllerNode node(nh, pnh);
    node.run();

    return 0;
}
