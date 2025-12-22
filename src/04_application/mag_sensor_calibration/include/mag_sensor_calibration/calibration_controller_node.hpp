#pragma once

#include <mag_sensor_calibration/StartCalibration.h>
#include <mag_sensor_calibration/StopCalibration.h>
#include <mag_sensor_calibration/SaveCalibrationData.h>
#include <mag_sensor_calibration/data_collector_node.hpp>
#include <mag_sensor_calibration/calibration_algorithm.hpp>
#include <mag_sensor_calibration/calibration_viz_node.hpp>
#include <mag_device_arm/SetEndEffectorPose.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <atomic>

namespace mag_sensor_calibration
{

/**
 * @brief 校正控制节点
 * 
 * 功能：
 * 1. 接收校正启动请求
 * 2. 控制机械臂进行旋转运动
 * 3. 协调数据采集
 * 4. 计算校正参数
 * 5. 保存结果
 */
class CalibrationControllerNode
{
public:
    CalibrationControllerNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~CalibrationControllerNode();

    void start();

private:
    void setupServices();
    void setLogLevel();
    void loadParameters();

    // 服务回调
    bool startCalibrationCallback(mag_sensor_calibration::StartCalibration::Request& req,
                                  mag_sensor_calibration::StartCalibration::Response& res);
    bool stopCalibrationCallback(mag_sensor_calibration::StopCalibration::Request& req,
                                 mag_sensor_calibration::StopCalibration::Response& res);
    bool saveCalibrationDataCallback(mag_sensor_calibration::SaveCalibrationData::Request& req,
                                     mag_sensor_calibration::SaveCalibrationData::Response& res);

    // 校正流程
    void calibrationLoop();
    bool moveToAngle(double angle, const geometry_msgs::Pose& start_pose,
                    const geometry_msgs::Point& rotation_center,
                    const std::string& rotation_axis);
    geometry_msgs::Pose computePoseAtAngle(const geometry_msgs::Pose& start_pose,
                                           const geometry_msgs::Point& rotation_center,
                                           const std::string& rotation_axis,
                                           double angle);

    // 校正计算
    void computeCalibrationForAllSensors();
    bool computeCalibrationForSensor(uint32_t sensor_id);

    // 数据保存
    bool saveCalibrationParams(const std::string& output_dir);
    std::string generateSessionId();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ROS服务
    ros::ServiceServer start_calibration_srv_;
    ros::ServiceServer stop_calibration_srv_;
    ros::ServiceServer save_data_srv_;
    ros::ServiceClient arm_pose_client_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 组件
    std::unique_ptr<DataCollectorNode> data_collector_;
    std::unique_ptr<CalibrationVizNode> viz_node_;
    CalibrationAlgorithm calibration_algorithm_;

    // 校正状态
    std::atomic<bool> calibration_running_;
    std::mutex calibration_mutex_;
    std::string current_session_id_;
    
    // 校正参数
    geometry_msgs::Pose start_pose_;
    geometry_msgs::Point rotation_center_;
    std::string rotation_axis_;
    std::vector<double> angle_range_;
    double angle_step_;
    double velocity_scaling_;
    double data_collection_duration_;
    std::string arm_name_;
    std::string reference_frame_;
    std::string sensor_array_frame_;
    std::string output_directory_;

    // 校正结果
    std::map<uint32_t, CalibrationParams> calibration_params_;

    // 配置参数
    std::string arm_service_name_;
    std::string data_collector_topic_;
};

} // namespace mag_sensor_calibration

