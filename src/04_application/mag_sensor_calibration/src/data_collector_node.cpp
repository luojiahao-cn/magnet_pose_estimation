#include <mag_sensor_calibration/data_collector_node.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <atomic>

namespace mag_sensor_calibration
{

DataCollectorNode::DataCollectorNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
    , collecting_(false)
    , save_to_csv_(false)
    , total_samples_(0)
{
    setLogLevel();
    loadParameters();
    setupSubscribers();
}

DataCollectorNode::~DataCollectorNode()
{
    stopCollection();
}

void DataCollectorNode::setLogLevel()
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

void DataCollectorNode::loadParameters()
{
    pnh_.param("input_topic", input_topic_, std::string("/mag_sensor/data_mT"));
    pnh_.param("use_batch_topic", use_batch_topic_, false);
    
    if (use_batch_topic_)
    {
        pnh_.param("batch_topic", input_topic_, std::string("/mag_sensor/batch"));
    }
}

void DataCollectorNode::setupSubscribers()
{
    if (use_batch_topic_)
    {
        batch_sub_ = nh_.subscribe(
            input_topic_, 100,
            &DataCollectorNode::sensorBatchCallback, this);
        ROS_INFO_STREAM("[data_collector] 订阅批量传感器数据: " << input_topic_);
    }
    else
    {
        sensor_sub_ = nh_.subscribe(
            input_topic_, 100,
            &DataCollectorNode::sensorDataCallback, this);
        ROS_INFO_STREAM("[data_collector] 订阅单个传感器数据: " << input_topic_);
    }
}

void DataCollectorNode::start()
{
    ROS_INFO("[data_collector] 数据采集节点已启动");
    ros::spin();
}

void DataCollectorNode::startCollection(const std::string& session_id,
                                       const std::string& reference_frame,
                                       const std::string& sensor_array_frame,
                                       const std::string& rotation_axis,
                                       bool save_to_csv,
                                       const std::string& csv_filepath)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (collecting_)
    {
        ROS_WARN("[data_collector] 采集已在进行中，先停止当前采集");
        stopCollection();
    }
    
    session_id_ = session_id;
    reference_frame_ = reference_frame;
    sensor_array_frame_ = sensor_array_frame;
    rotation_axis_ = rotation_axis;
    save_to_csv_ = save_to_csv;
    csv_filepath_ = csv_filepath;
    
    // 清空旧数据
    collected_data_.clear();
    total_samples_ = 0;
    
    // 打开CSV文件
    if (save_to_csv_ && !csv_filepath_.empty())
    {
        csv_file_.open(csv_filepath_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open())
        {
            ROS_ERROR_STREAM("[data_collector] 无法打开CSV文件: " << csv_filepath_);
            save_to_csv_ = false;
        }
        else
        {
            // 写入CSV头部
            csv_file_ << "timestamp,sensor_id,angle_rad,mag_x_mT,mag_y_mT,mag_z_mT,"
                      << "sensor_pos_x,sensor_pos_y,sensor_pos_z,"
                      << "sensor_quat_x,sensor_quat_y,sensor_quat_z,sensor_quat_w\n";
            csv_file_.flush();
        }
    }
    
    collecting_ = true;
    ROS_INFO_STREAM("[data_collector] 开始采集数据 - 会话ID: " << session_id_
                    << ", 参考坐标系: " << reference_frame_
                    << ", 传感器阵列: " << sensor_array_frame_
                    << ", 旋转轴: " << rotation_axis_);
}

void DataCollectorNode::stopCollection()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!collecting_)
    {
        return;
    }
    
    collecting_ = false;
    
    // 关闭CSV文件
    if (csv_file_.is_open())
    {
        csv_file_.close();
    }
    
    ROS_INFO_STREAM("[data_collector] 停止采集数据 - 会话ID: " << session_id_
                    << ", 总样本数: " << total_samples_);
}

std::vector<DataPoint> DataCollectorNode::getCollectedData(uint32_t sensor_id) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (sensor_id == 0)
    {
        // 返回所有传感器的数据
        std::vector<DataPoint> all_data;
        for (const auto& kv : collected_data_)
        {
            all_data.insert(all_data.end(), kv.second.begin(), kv.second.end());
        }
        return all_data;
    }
    else
    {
        // 返回指定传感器的数据
        auto it = collected_data_.find(sensor_id);
        if (it != collected_data_.end())
        {
            return it->second;
        }
        return std::vector<DataPoint>();
    }
}

void DataCollectorNode::clearData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    collected_data_.clear();
    total_samples_ = 0;
}

size_t DataCollectorNode::getSampleCount() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return total_samples_;
}

void DataCollectorNode::sensorDataCallback(const mag_core_msgs::MagSensorDataConstPtr& msg)
{
    if (!collecting_)
    {
        return;
    }
    
    // 计算当前角度
    double angle = computeCurrentAngle(reference_frame_, sensor_array_frame_, rotation_axis_);
    
    // 创建数据点
    DataPoint point;
    point.timestamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    point.sensor_id = msg->sensor_id;
    point.angle = angle;
    point.mag_reading = Eigen::Vector3d(msg->mag_x, msg->mag_y, msg->mag_z);
    
    // 尝试获取传感器位姿（可选）
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            reference_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        point.sensor_pose.position.x = transform.transform.translation.x;
        point.sensor_pose.position.y = transform.transform.translation.y;
        point.sensor_pose.position.z = transform.transform.translation.z;
        point.sensor_pose.orientation = transform.transform.rotation;
    }
    catch (const tf2::TransformException& ex)
    {
        // 位姿获取失败，使用默认值
    }
    
    // 保存数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        collected_data_[point.sensor_id].push_back(point);
        ++total_samples_;
    }
    
    // 保存到CSV
    if (save_to_csv_)
    {
        saveToCSV(point);
    }
}

void DataCollectorNode::sensorBatchCallback(const mag_core_msgs::MagSensorBatchConstPtr& msg)
{
    if (!collecting_)
    {
        return;
    }
    
    // 计算当前角度（所有传感器共享同一个角度）
    double angle = computeCurrentAngle(reference_frame_, sensor_array_frame_, rotation_axis_);
    
    // 处理批量数据中的每个传感器
    for (const auto& measurement : msg->measurements)
    {
        DataPoint point;
        point.timestamp = measurement.header.stamp.isZero() ? ros::Time::now() : measurement.header.stamp;
        point.sensor_id = measurement.sensor_id;
        point.angle = angle;
        point.mag_reading = Eigen::Vector3d(
            measurement.mag_x, measurement.mag_y, measurement.mag_z);
        
        // 尝试获取传感器位姿
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                reference_frame_, measurement.header.frame_id, ros::Time(0), ros::Duration(0.1));
            point.sensor_pose.position.x = transform.transform.translation.x;
            point.sensor_pose.position.y = transform.transform.translation.y;
            point.sensor_pose.position.z = transform.transform.translation.z;
            point.sensor_pose.orientation = transform.transform.rotation;
        }
        catch (const tf2::TransformException& ex)
        {
            // 位姿获取失败，使用默认值
        }
        
        // 保存数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            collected_data_[point.sensor_id].push_back(point);
            ++total_samples_;
        }
        
        // 保存到CSV
        if (save_to_csv_)
        {
            saveToCSV(point);
        }
    }
}

double DataCollectorNode::computeCurrentAngle(const std::string& reference_frame,
                                              const std::string& sensor_array_frame,
                                              const std::string& rotation_axis)
{
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            reference_frame, sensor_array_frame, ros::Time(0), ros::Duration(0.1));
        
        // 提取旋转四元数
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        
        // 转换为旋转矩阵
        tf2::Matrix3x3 rot_matrix(q);
        
        // 根据旋转轴提取角度
        double angle = 0.0;
        if (rotation_axis == "x" || rotation_axis == "X")
        {
            // 绕X轴旋转角度（从Y-Z平面提取）
            angle = std::atan2(rot_matrix[1][2], rot_matrix[2][2]);
        }
        else if (rotation_axis == "y" || rotation_axis == "Y")
        {
            // 绕Y轴旋转角度（从X-Z平面提取）
            angle = std::atan2(-rot_matrix[0][2], rot_matrix[2][2]);
        }
        else if (rotation_axis == "z" || rotation_axis == "Z")
        {
            // 绕Z轴旋转角度（从X-Y平面提取）
            angle = std::atan2(rot_matrix[0][1], rot_matrix[0][0]);
        }
        else
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "[data_collector] 未知的旋转轴: " << rotation_axis << ", 使用Z轴");
            angle = std::atan2(rot_matrix[0][1], rot_matrix[0][0]);
        }
        
        // 归一化到 [0, 2π)
        while (angle < 0) angle += 2.0 * M_PI;
        while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
        
        return angle;
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_STREAM_THROTTLE(2.0, "[data_collector] 无法获取TF变换: " << ex.what());
        return 0.0;
    }
}

void DataCollectorNode::saveToCSV(const DataPoint& point)
{
    if (!csv_file_.is_open())
    {
        return;
    }
    
    // 格式化时间戳
    std::stringstream ss;
    ss << std::fixed << std::setprecision(9)
       << point.timestamp.toSec() << ","
       << point.sensor_id << ","
       << std::setprecision(6)
       << point.angle << ","
       << point.mag_reading.x() << ","
       << point.mag_reading.y() << ","
       << point.mag_reading.z() << ","
       << point.sensor_pose.position.x << ","
       << point.sensor_pose.position.y << ","
       << point.sensor_pose.position.z << ","
       << point.sensor_pose.orientation.x << ","
       << point.sensor_pose.orientation.y << ","
       << point.sensor_pose.orientation.z << ","
       << point.sensor_pose.orientation.w << "\n";
    
    csv_file_ << ss.str();
    
    // 定期刷新（每100个样本）
    if (total_samples_ % 100 == 0)
    {
        csv_file_.flush();
    }
}

void DataCollectorNode::flushCSV()
{
    if (csv_file_.is_open())
    {
        csv_file_.flush();
    }
}

} // namespace mag_sensor_calibration

