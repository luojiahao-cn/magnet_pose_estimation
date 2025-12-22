#include <mag_sensor_calibration/calibration_controller_node.hpp>

#include <mag_core_description/sensor_array_config_loader.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <thread>
#include <algorithm>

namespace mag_sensor_calibration
{

CalibrationControllerNode::CalibrationControllerNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
    , calibration_running_(false)
    , velocity_scaling_(0.2)
    , data_collection_duration_(2.0)
{
    setLogLevel();
    loadParameters();
    setupServices();
    
    // 初始化组件
    data_collector_ = std::make_unique<DataCollectorNode>(nh_, pnh_);
    viz_node_ = std::make_unique<CalibrationVizNode>(nh_, pnh_);
    
    ROS_INFO("[calibration_controller] 校正控制节点已初始化");
}

CalibrationControllerNode::~CalibrationControllerNode()
{
    if (calibration_running_)
    {
        mag_sensor_calibration::StopCalibration::Request req;
        mag_sensor_calibration::StopCalibration::Response res;
        stopCalibrationCallback(req, res);
    }
}

void CalibrationControllerNode::setLogLevel()
{
    std::string log_level_str = "INFO";
    pnh_.param("logging_level", log_level_str, log_level_str);
    std::transform(log_level_str.begin(), log_level_str.end(), log_level_str.begin(), ::toupper);
    
    ros::console::Level level = ros::console::levels::Info;
    if (log_level_str == "DEBUG") level = ros::console::levels::Debug;
    else if (log_level_str == "INFO") level = ros::console::levels::Info;
    else if (log_level_str == "WARN") level = ros::console::levels::Warn;
    else if (log_level_str == "ERROR") level = ros::console::levels::Error;
    else if (log_level_str == "FATAL") level = ros::console::levels::Fatal;
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level);
}

void CalibrationControllerNode::loadParameters()
{
    pnh_.param("arm_service_name", arm_service_name_, std::string("/mag_device_arm/set_end_effector_pose"));
    pnh_.param("data_collector_topic", data_collector_topic_, std::string("/mag_sensor/data_mT"));
    pnh_.param("reference_frame", reference_frame_, std::string("world"));
    pnh_.param("sensor_array_frame", sensor_array_frame_, std::string("sensor_array_frame"));
}

void CalibrationControllerNode::setupServices()
{
    start_calibration_srv_ = pnh_.advertiseService(
        "start_calibration", &CalibrationControllerNode::startCalibrationCallback, this);
    stop_calibration_srv_ = pnh_.advertiseService(
        "stop_calibration", &CalibrationControllerNode::stopCalibrationCallback, this);
    save_data_srv_ = pnh_.advertiseService(
        "save_calibration_data", &CalibrationControllerNode::saveCalibrationDataCallback, this);
    
    arm_pose_client_ = nh_.serviceClient<mag_device_arm::SetEndEffectorPose>(arm_service_name_);
    
    ROS_INFO("[calibration_controller] 服务已注册");
}

void CalibrationControllerNode::start()
{
    ROS_INFO("[calibration_controller] 校正控制节点已启动，等待服务调用");
    ros::spin();
}

bool CalibrationControllerNode::startCalibrationCallback(
    mag_sensor_calibration::StartCalibration::Request& req,
    mag_sensor_calibration::StartCalibration::Response& res)
{
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    
    if (calibration_running_)
    {
        res.success = false;
        res.message = "校正已在进行中，请先停止当前校正";
        return true;
    }
    
    // 验证参数
    if (req.angle_range.size() != 2 || req.angle_range[0] >= req.angle_range[1])
    {
        res.success = false;
        res.message = "角度范围无效，需要 [min_angle, max_angle] 且 min < max";
        return true;
    }
    
    if (req.angle_step <= 0)
    {
        res.success = false;
        res.message = "角度步长必须大于0";
        return true;
    }
    
    // 保存参数
    current_session_id_ = generateSessionId();
    start_pose_ = req.start_pose;
    rotation_center_ = req.rotation_center;
    rotation_axis_ = req.rotation_axis;
    angle_range_ = req.angle_range;
    angle_step_ = req.angle_step;
    velocity_scaling_ = req.velocity_scaling;
    data_collection_duration_ = req.data_collection_duration;
    arm_name_ = req.arm;
    output_directory_ = req.output_directory;
    
    // 创建输出目录
    if (!output_directory_.empty())
    {
        std::filesystem::create_directories(output_directory_);
    }
    
    // 准备CSV文件路径
    std::string csv_filepath;
    if (req.save_raw_data && !output_directory_.empty())
    {
        csv_filepath = output_directory_ + "/calibration_data_" + current_session_id_ + ".csv";
    }
    
    // 启动数据采集
    data_collector_->startCollection(
        current_session_id_,
        reference_frame_,
        sensor_array_frame_,
        rotation_axis_,
        req.save_raw_data,
        csv_filepath);
    
    // 启动校正循环（在后台线程）
    calibration_running_ = true;
    std::thread calibration_thread(&CalibrationControllerNode::calibrationLoop, this);
    calibration_thread.detach();
    
    res.success = true;
    res.message = "校正已启动";
    res.session_id = current_session_id_;
    
    ROS_INFO_STREAM("[calibration_controller] 校正已启动 - 会话ID: " << current_session_id_);
    
    return true;
}

bool CalibrationControllerNode::stopCalibrationCallback(
    mag_sensor_calibration::StopCalibration::Request& req,
    mag_sensor_calibration::StopCalibration::Response& res)
{
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    
    if (!calibration_running_)
    {
        res.success = false;
        res.message = "没有正在进行的校正";
        res.collected_samples = 0;
        return true;
    }
    
    if (req.session_id != current_session_id_)
    {
        res.success = false;
        res.message = "会话ID不匹配";
        res.collected_samples = data_collector_->getSampleCount();
        return true;
    }
    
    // 停止校正
    calibration_running_ = false;
    
    // 停止数据采集
    data_collector_->stopCollection();
    
    // 如果请求保存数据
    if (req.save_data)
    {
        computeCalibrationForAllSensors();
        saveCalibrationParams(output_directory_);
    }
    
    res.success = true;
    res.message = "校正已停止";
    res.collected_samples = data_collector_->getSampleCount();
    
    ROS_INFO_STREAM("[calibration_controller] 校正已停止 - 样本数: " << res.collected_samples);
    
    return true;
}

bool CalibrationControllerNode::saveCalibrationDataCallback(
    mag_sensor_calibration::SaveCalibrationData::Request& req,
    mag_sensor_calibration::SaveCalibrationData::Response& res)
{
    if (req.session_id != current_session_id_)
    {
        res.success = false;
        res.message = "会话ID不匹配";
        return true;
    }
    
    // 计算校正参数（如果请求）
    if (req.compute_calibration)
    {
        computeCalibrationForAllSensors();
        saveCalibrationParams(req.output_directory);
    }
    
    // 查找CSV文件
    std::string csv_filepath = req.output_directory + "/calibration_data_" + req.session_id + ".csv";
    if (std::filesystem::exists(csv_filepath))
    {
        res.csv_file_path = csv_filepath;
    }
    
    // 查找校正参数文件
    std::string calib_filepath = req.output_directory + "/calibration_params_" + req.session_id + ".yaml";
    if (std::filesystem::exists(calib_filepath))
    {
        res.calibration_file_path = calib_filepath;
    }
    
    res.success = true;
    res.message = "数据已保存";
    
    return true;
}

void CalibrationControllerNode::calibrationLoop()
{
    ROS_INFO("[calibration_controller] 开始校正循环");
    
    // 生成角度序列
    std::vector<double> angles;
    for (double angle = angle_range_[0]; angle <= angle_range_[1]; angle += angle_step_)
    {
        angles.push_back(angle);
    }
    
    ROS_INFO_STREAM("[calibration_controller] 将采集 " << angles.size() << " 个角度位置");
    
    // 遍历每个角度
    for (size_t i = 0; i < angles.size() && calibration_running_; ++i)
    {
        double angle = angles[i];
        ROS_INFO_STREAM("[calibration_controller] 移动到角度: " << angle << " rad (" 
                        << (angle * 180.0 / M_PI) << " deg) [" << (i+1) << "/" << angles.size() << "]");
        
        // 移动到目标角度
        if (!moveToAngle(angle, start_pose_, rotation_center_, rotation_axis_))
        {
            ROS_ERROR_STREAM("[calibration_controller] 移动到角度 " << angle << " 失败");
            continue;
        }
        
        // 等待机械臂稳定
        ros::Duration(0.5).sleep();
        
        // 采集数据
        ROS_INFO_STREAM("[calibration_controller] 开始采集数据，持续时间: " 
                        << data_collection_duration_ << " 秒");
        ros::Duration(data_collection_duration_).sleep();
        
        ROS_INFO_STREAM("[calibration_controller] 角度 " << angle << " 的数据采集完成");
    }
    
    if (calibration_running_)
    {
        ROS_INFO("[calibration_controller] 校正循环完成");
        // 自动计算校正参数
        computeCalibrationForAllSensors();
        saveCalibrationParams(output_directory_);
    }
    else
    {
        ROS_INFO("[calibration_controller] 校正循环被中断");
    }
}

bool CalibrationControllerNode::moveToAngle(double angle,
                                            const geometry_msgs::Pose& start_pose,
                                            const geometry_msgs::Point& rotation_center,
                                            const std::string& rotation_axis)
{
    // 计算目标位姿
    geometry_msgs::Pose target_pose = computePoseAtAngle(
        start_pose, rotation_center, rotation_axis, angle);
    
    // 调用机械臂服务
    mag_device_arm::SetEndEffectorPose srv;
    srv.request.arm = arm_name_;
    srv.request.target = target_pose;
    srv.request.velocity_scaling = velocity_scaling_;
    srv.request.acceleration_scaling = velocity_scaling_;
    srv.request.execute = true;
    
    if (!arm_pose_client_.call(srv))
    {
        ROS_ERROR("[calibration_controller] 调用机械臂服务失败");
        return false;
    }
    
    if (!srv.response.success)
    {
        ROS_ERROR_STREAM("[calibration_controller] 机械臂移动失败: " << srv.response.message);
        return false;
    }
    
    return true;
}

geometry_msgs::Pose CalibrationControllerNode::computePoseAtAngle(
    const geometry_msgs::Pose& start_pose,
    const geometry_msgs::Point& rotation_center,
    const std::string& rotation_axis,
    double angle)
{
    geometry_msgs::Pose target_pose = start_pose;
    
    // 将旋转中心转换为相对于起始位姿的偏移
    Eigen::Vector3d center_offset(
        rotation_center.x - start_pose.position.x,
        rotation_center.y - start_pose.position.y,
        rotation_center.z - start_pose.position.z);
    
    // 根据旋转轴创建旋转矩阵
    Eigen::Matrix3d rotation_matrix;
    if (rotation_axis == "x" || rotation_axis == "X")
    {
        rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    }
    else if (rotation_axis == "y" || rotation_axis == "Y")
    {
        rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    }
    else if (rotation_axis == "z" || rotation_axis == "Z")
    {
        rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    else
    {
        ROS_WARN_STREAM("[calibration_controller] 未知旋转轴: " << rotation_axis << ", 使用Z轴");
        rotation_matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    
    // 计算旋转后的偏移
    Eigen::Vector3d rotated_offset = rotation_matrix * center_offset;
    
    // 计算目标位置
    target_pose.position.x = rotation_center.x - rotated_offset.x();
    target_pose.position.y = rotation_center.y - rotated_offset.y();
    target_pose.position.z = rotation_center.z - rotated_offset.z();
    
    // 计算目标 orientation（将旋转应用到起始 orientation）
    tf2::Quaternion start_q;
    tf2::fromMsg(start_pose.orientation, start_q);
    Eigen::Quaterniond start_quat(start_q.w(), start_q.x(), start_q.y(), start_q.z());
    Eigen::Quaterniond rotation_quat(rotation_matrix);
    Eigen::Quaterniond target_quat = rotation_quat * start_quat;
    target_pose.orientation = tf2::toMsg(tf2::Quaternion(
        target_quat.x(), target_quat.y(), target_quat.z(), target_quat.w()));
    
    return target_pose;
}

void CalibrationControllerNode::computeCalibrationForAllSensors()
{
    ROS_INFO("[calibration_controller] 开始计算所有传感器的校正参数");
    
    // 获取所有采集的数据
    std::vector<DataPoint> all_data = data_collector_->getCollectedData();
    
    // 按传感器ID分组
    std::map<uint32_t, std::vector<MeasurementPoint>> sensor_measurements;
    for (const auto& point : all_data)
    {
        MeasurementPoint mpoint;
        mpoint.angle = point.angle;
        mpoint.mag = point.mag_reading;
        mpoint.position = Eigen::Vector3d(
            point.sensor_pose.position.x,
            point.sensor_pose.position.y,
            point.sensor_pose.position.z);
        sensor_measurements[point.sensor_id].push_back(mpoint);
    }
    
    // 对每个传感器进行校正
    for (const auto& kv : sensor_measurements)
    {
        uint32_t sensor_id = kv.first;
        if (computeCalibrationForSensor(sensor_id))
        {
            ROS_INFO_STREAM("[calibration_controller] 传感器 " << sensor_id << " 校正完成");
        }
        else
        {
            ROS_WARN_STREAM("[calibration_controller] 传感器 " << sensor_id << " 校正失败");
        }
    }
}

bool CalibrationControllerNode::computeCalibrationForSensor(uint32_t sensor_id)
{
    // 获取该传感器的数据
    std::vector<DataPoint> sensor_data = data_collector_->getCollectedData(sensor_id);
    
    if (sensor_data.size() < 9)
    {
        ROS_WARN_STREAM("[calibration_controller] 传感器 " << sensor_id 
                        << " 数据点不足 (" << sensor_data.size() << " < 9)");
        return false;
    }
    
    // 转换为 MeasurementPoint
    std::vector<MeasurementPoint> measurements;
    for (const auto& point : sensor_data)
    {
        MeasurementPoint mpoint;
        mpoint.angle = point.angle;
        mpoint.mag = point.mag_reading;
        mpoint.position = Eigen::Vector3d(
            point.sensor_pose.position.x,
            point.sensor_pose.position.y,
            point.sensor_pose.position.z);
        measurements.push_back(mpoint);
    }
    
    // 计算校正参数
    CalibrationParams params;
    if (!calibration_algorithm_.calibrateSensor(measurements, params))
    {
        return false;
    }
    
    // 保存校正参数
    calibration_params_[sensor_id] = params;
    
    // 更新可视化
    viz_node_->updateCalibrationResult(sensor_id, params, measurements);
    
    return true;
}

bool CalibrationControllerNode::saveCalibrationParams(const std::string& output_dir)
{
    if (output_dir.empty() || calibration_params_.empty())
    {
        return false;
    }
    
    std::string filepath = output_dir + "/calibration_params_" + current_session_id_ + ".yaml";
    
    try
    {
        YAML::Node root;
        root["session_id"] = current_session_id_;
        root["timestamp"] = std::time(nullptr);
        
        for (const auto& kv : calibration_params_)
        {
            uint32_t sensor_id = kv.first;
            const CalibrationParams& params = kv.second;
            
            YAML::Node sensor_node;
            sensor_node["sensor_id"] = sensor_id;
            sensor_node["valid"] = params.valid;
            
            // 硬磁偏移
            sensor_node["hard_iron_offset"] = std::vector<double>{
                params.hard_iron_offset.x(),
                params.hard_iron_offset.y(),
                params.hard_iron_offset.z()
            };
            
            // 软磁矩阵
            sensor_node["soft_iron_matrix"] = std::vector<std::vector<double>>{
                {params.soft_iron_matrix(0, 0), params.soft_iron_matrix(0, 1), params.soft_iron_matrix(0, 2)},
                {params.soft_iron_matrix(1, 0), params.soft_iron_matrix(1, 1), params.soft_iron_matrix(1, 2)},
                {params.soft_iron_matrix(2, 0), params.soft_iron_matrix(2, 1), params.soft_iron_matrix(2, 2)}
            };
            
            // 质量指标
            sensor_node["min_field_strength"] = params.min_field_strength;
            sensor_node["max_field_strength"] = params.max_field_strength;
            sensor_node["fit_error"] = params.fit_error;
            sensor_node["coverage"] = params.coverage;
            
            root["sensors"].push_back(sensor_node);
        }
        
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            ROS_ERROR_STREAM("[calibration_controller] 无法打开文件: " << filepath);
            return false;
        }
        
        file << root;
        ROS_INFO_STREAM("[calibration_controller] 校正参数已保存到: " << filepath);
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[calibration_controller] 保存校正参数失败: " << e.what());
        return false;
    }
}

std::string CalibrationControllerNode::generateSessionId()
{
    std::time_t now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    return ss.str();
}

} // namespace mag_sensor_calibration

