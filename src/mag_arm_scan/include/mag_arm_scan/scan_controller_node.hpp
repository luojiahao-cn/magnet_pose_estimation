#ifndef SCAN_CONTROLLER_NODE_HPP
#define SCAN_CONTROLLER_NODE_HPP

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <mag_sensor_node/MagSensorData.h>
#include <vector>
#include <string>

class ScanControllerNode
{
public:
    ScanControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ScanControllerNode() = default;

    void run();

private:
    void loadParams();
    void generateScanPoints();
    bool moveToPose(const geometry_msgs::Pose& pose);
    bool moveToReadyPose();
    void collectDataAtPoint(const geometry_msgs::Pose& pose);
    bool startScan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void magDataCallback(const mag_sensor_node::MagSensorData::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer start_scan_srv_;
    ros::Subscriber mag_data_sub_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Parameters
    std::string frame_id_;
    double yaw_;
    double pitch_;
    bool autostart_;
    std::vector<double> volume_min_;
    std::vector<double> volume_max_;
    std::vector<double> step_;
    std::string mag_topic_;
    double wait_time_;
    std::string output_file_;

    // Scan points
    std::vector<geometry_msgs::Pose> scan_points_;

    // Data collection
    std::vector<mag_sensor_node::MagSensorData> collected_data_;
};

#endif // SCAN_CONTROLLER_NODE_HPP
