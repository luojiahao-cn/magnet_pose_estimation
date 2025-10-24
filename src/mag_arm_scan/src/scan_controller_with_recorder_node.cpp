/*
文件: scan_controller_with_recorder_node.cpp
功能概述:
  - 合并扫描控制器和原始数据记录器：使用 MoveIt 控制机械臂在设定体积内按网格顺序逐点运动，
    等待稳定后采集磁传感器原始数据并保存到 CSV。只在运动到位时记录数据。

主要职责:
  - 读取参数: 坐标系 frame_id、扫描体积(volume_min/max/step)、姿态(yaw/pitch)、等待时间、话题、输出文件等
  - 生成扫描点: 在体积内按步长生成(x,y,z)网格点，姿态固定为 RPY(0, pitch, yaw)
  - 运动执行: 规划+执行至目标点，等待系统稳定(wait_time)
  - 数据采集: 在到位后，记录最近若干帧原始磁传感器数据到 CSV
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
  - 说明: 在到位后记录原始样本，不求平均；可按需调整采样数量

注意事项:
  - 坐标一致性需外部标定(传感器/机械臂/世界坐标)
  - 等待时间与控制器/传感器稳定时间相关，需根据实际系统调优
  - 采样窗口较小(默认10帧)，可根据传感器频率/噪声调整
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <mag_sensor_node/MagSensorData.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <sys/stat.h>

class ScanControllerWithRecorderNode {
public:
    ScanControllerWithRecorderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_)
    {
        loadParams();
        generateScanPoints();

        // 延迟初始化MoveGroupInterface，直到ROS完全启动
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("fr5v6_arm");

        // 服务置于节点命名空间下
        start_scan_srv_ = pnh_.advertiseService("start_scan", &ScanControllerWithRecorderNode::startScan, this);
        mag_data_sub_ = nh_.subscribe(mag_topic_, 100, &ScanControllerWithRecorderNode::magDataCallback, this);
        joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ScanControllerWithRecorderNode::jointStateCallback, this);
        clear_client_ = nh_.serviceClient<std_srvs::Trigger>("/field_map_aggregator/clear_map");
        at_position_pub_ = nh_.advertise<std_msgs::Bool>("/scan_at_position", 1);
        scan_complete_pub_ = nh_.advertise<std_msgs::String>("/scan_complete", 1);

        ROS_INFO("[scan_controller_with_recorder] initialized with %zu scan points", scan_points_.size());
    }

    void loadParams()
    {
        if (!pnh_.getParam("frame_id", frame_id_))
            throw std::runtime_error("缺少参数: ~frame_id");
        if (!pnh_.getParam("yaw", yaw_))
            throw std::runtime_error("缺少参数: ~yaw");
        if (!pnh_.getParam("pitch", pitch_))
            throw std::runtime_error("缺少参数: ~pitch");
        if (!pnh_.getParam("autostart", autostart_))
            throw std::runtime_error("缺少参数: ~autostart");
        if (!pnh_.getParam("mag_topic", mag_topic_))
            throw std::runtime_error("缺少参数: ~mag_topic");
        if (!pnh_.getParam("wait_time", wait_time_))
            throw std::runtime_error("缺少参数: ~wait_time");
        if (!pnh_.getParam("max_stable_wait_time", max_stable_wait_time_))
            throw std::runtime_error("缺少参数: ~max_stable_wait_time");
        if (!pnh_.getParam("output_file", output_file_))
            throw std::runtime_error("缺少参数: ~output_file");
        if (!pnh_.getParam("output_base_dir", output_base_dir_))
            throw std::runtime_error("缺少参数: ~output_base_dir");

        // 解析output_base_dir中的$(find ...)语法
        output_base_dir_ = resolveRelativePath(output_base_dir_);

        // 创建时间戳文件夹
        createTimestampDirectory();

        if (!pnh_.getParam("volume_min", volume_min_) || volume_min_.size() != 3)
            throw std::runtime_error("缺少或非法参数: ~volume_min[3]");
        if (!pnh_.getParam("volume_max", volume_max_) || volume_max_.size() != 3)
            throw std::runtime_error("缺少或非法参数: ~volume_max[3]");
        if (!pnh_.getParam("step", step_) || step_.size() != 3)
            throw std::runtime_error("缺少或非法参数: ~step[3]");
    }

    void generateScanPoints()
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

        // 优化扫描顺序：使用贪婪算法最小化相邻点距离
        optimizeScanOrder();
    }

    void optimizeScanOrder()
    {
        if (scan_points_.empty()) return;

        std::vector<geometry_msgs::Pose> optimized;
        std::vector<bool> visited(scan_points_.size(), false);

        // 从第一个点开始
        optimized.push_back(scan_points_[0]);
        visited[0] = true;

        for (size_t i = 1; i < scan_points_.size(); ++i)
        {
            size_t best_idx = 0;
            double min_dist = std::numeric_limits<double>::max();

            for (size_t j = 0; j < scan_points_.size(); ++j)
            {
                if (!visited[j])
                {
                    double dist = poseDistance(optimized.back(), scan_points_[j]);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        best_idx = j;
                    }
                }
            }

            optimized.push_back(scan_points_[best_idx]);
            visited[best_idx] = true;
        }

        scan_points_ = optimized;
        ROS_INFO("[scan_controller_with_recorder] optimized scan order for %zu points", scan_points_.size());
    }

    double poseDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        double dz = p1.position.z - p2.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    bool moveToPose(const geometry_msgs::Pose& pose)
    {
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            success = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        return success;
    }

    bool moveToReadyPose()
    {
        ROS_INFO("[scan_controller_with_recorder] moving to ready position using named target");

        move_group_->setNamedTarget("ready");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            success = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        return success;
    }

    void collectDataAtPoint(const geometry_msgs::Pose& pose)
    {
        // Publish not at position before moving
        std_msgs::Bool at_pos_msg;
        at_pos_msg.data = false;
        at_position_pub_.publish(at_pos_msg);

        ROS_INFO("[scan_controller_with_recorder] moving to position: x=%.3f, y=%.3f, z=%.3f", pose.position.x, pose.position.y, pose.position.z);

        if (!moveToPose(pose))
        {
            ROS_ERROR("[scan_controller_with_recorder] failed to move to pose");
            return;
        }

        // Wait for settling
        ros::Duration(wait_time_).sleep();

        // Wait for arm to be stable (joint velocities below threshold)
        ros::Time start_wait = ros::Time::now();
        ros::Duration max_stable_wait(max_stable_wait_time_);  // Max additional wait for stability
        while (!isArmStable() && (ros::Time::now() - start_wait) < max_stable_wait)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        if (isArmStable())
        {
            ROS_INFO("[scan_controller_with_recorder] arm is stable at position");
        }
        else
        {
            ROS_WARN("[scan_controller_with_recorder] arm did not stabilize within timeout, proceeding anyway");
        }

        // Publish at position
        at_pos_msg.data = true;
        at_position_pub_.publish(at_pos_msg);

        // Collect and record raw data samples
        const int num_samples = 10;
        collected_data_.clear();  // Clear previous data

        // Wait for data to accumulate
        for (int i = 0; i < num_samples; ++i)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        // Record the collected raw data
        for (const auto& data : collected_data_)
        {
            // Transform to target frame if needed
            geometry_msgs::Pose pose_in = data.sensor_pose;
            geometry_msgs::Pose pose_w = pose_in;
            bool did_tf = false;
            try {
                if (!frame_id_.empty() && frame_id_ != data.header.frame_id) {
                    geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(frame_id_, data.header.frame_id, ros::Time(0), ros::Duration(0.05));
                    tf2::doTransform(pose_in, pose_w, T);
                    did_tf = true;
                }
            } catch (const std::exception& e) {
                pose_w = pose_in;
                ROS_WARN_THROTTLE(5.0, "TF to '%s' failed: %s; using source frame.", frame_id_.c_str(), e.what());
            }

            double bx = data.mag_x, by = data.mag_y, bz = data.mag_z;
            if (did_tf) {
                try {
                    geometry_msgs::Vector3Stamped vin, vout;
                    vin.header = data.header;
                    vin.vector.x = bx; vin.vector.y = by; vin.vector.z = bz;
                    geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(frame_id_, data.header.frame_id, ros::Time(0), ros::Duration(0.05));
                    tf2::doTransform(vin, vout, T);
                    bx = vout.vector.x; by = vout.vector.y; bz = vout.vector.z;
                } catch (...) { /* ignore */ }
            }

            // Save to file
            std::ofstream file(output_file_, std::ios::app);
            if (file.is_open())
            {
                file << data.header.stamp.toSec() << "," << bx << "," << by << "," << bz << ","
                     << pose_w.position.x << "," << pose_w.position.y << "," << pose_w.position.z << std::endl;
                file.close();
            }
        }
    }

    bool startScan(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        ROS_INFO("[scan_controller_with_recorder] starting scan with %zu points", scan_points_.size());

        // First move to ready position
        if (!moveToReadyPose())
        {
            ROS_ERROR("[scan_controller_with_recorder] failed to move to ready position");
            res.success = false;
            res.message = "Failed to move to ready position";
            return true;
        }

        // Clear the field map aggregator
        std_srvs::Trigger clear_req;
        if (clear_client_.call(clear_req))
        {
            ROS_INFO("[scan_controller_with_recorder] cleared field map aggregator");
        }
        else
        {
            ROS_WARN("[scan_controller_with_recorder] failed to clear field map aggregator");
        }

        // Clear output file
        std::ofstream file(output_file_, std::ios::trunc);
        file << "timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z" << std::endl;
        file.close();

        // Publish not at position initially
        std_msgs::Bool at_pos_msg;
        at_pos_msg.data = false;
        at_position_pub_.publish(at_pos_msg);

        for (const auto& pose : scan_points_)
        {
            collectDataAtPoint(pose);
        }

        // Publish not at position after scan
        at_pos_msg.data = false;
        at_position_pub_.publish(at_pos_msg);

        // 扫描完成后的处理
        finalizeScan();

        res.success = true;
        res.message = "Scan completed";
        return true;
    }

    bool isArmStable(double velocity_threshold = 0.01)
    {
        if (latest_joint_state_.velocity.empty()) return false;
        for (double vel : latest_joint_state_.velocity) {
            if (std::abs(vel) > velocity_threshold) return false;
        }
        return true;
    }

    void magDataCallback(const mag_sensor_node::MagSensorData::ConstPtr& msg)
    {
        collected_data_.push_back(*msg);
        // Keep only recent data
        if (collected_data_.size() > 10)
        {
            collected_data_.erase(collected_data_.begin());
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        latest_joint_state_ = *msg;
    }

    void run()
    {
        if (autostart_)
        {
            // 等待控制器启动
            ROS_INFO("[scan_controller_with_recorder] waiting for controller manager services to be available...");
            while (move_group_->getPlanningFrame().empty())
            {
                ROS_INFO("[scan_controller_with_recorder] waiting for MoveGroup to be ready...");
                ros::Duration(1.0).sleep();
                ros::spinOnce();
            }
            ROS_INFO("[scan_controller_with_recorder] MoveGroup is ready. Waiting additional time for system stabilization...");

            // 等待系统完全启动
            ros::Duration(3.0).sleep();

            ROS_INFO("[scan_controller_with_recorder] starting automatic scan...");
            std_srvs::Trigger::Request req;
            std_srvs::Trigger::Response res;
            startScan(req, res);
        }

        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    ros::ServiceServer start_scan_srv_;
    ros::Subscriber mag_data_sub_;
    ros::ServiceClient clear_client_;
    ros::Publisher at_position_pub_;
    ros::Publisher scan_complete_pub_;
    ros::Subscriber joint_states_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    sensor_msgs::JointState latest_joint_state_;

    std::string frame_id_;
    double yaw_;
    double pitch_;
    bool autostart_;
    std::string mag_topic_;
    double wait_time_;
    double max_stable_wait_time_;
    std::string output_file_;
    std::string output_base_dir_;
    std::vector<double> volume_min_;
    std::vector<double> volume_max_;
    std::vector<double> step_;

    std::string resolveRelativePath(const std::string& path)
    {
        std::string result = path;

        // 处理相对路径（相对于工作空间根目录）
        if (!result.empty() && result[0] != '/') {
            // 获取工作空间根目录（通过当前包的路径向上查找src目录的父目录）
            std::string current_package_path;
            try {
                current_package_path = ros::package::getPath("mag_arm_scan");
                // 从包路径向上查找，直到找到src目录的父目录
                size_t src_pos = current_package_path.find("/src/");
                if (src_pos != std::string::npos) {
                    std::string workspace_root = current_package_path.substr(0, src_pos);
                    result = workspace_root + "/" + result;
                } else {
                    ROS_WARN("[scan_controller_with_recorder] could not determine workspace root, using absolute path");
                }
            } catch (const std::exception& e) {
                ROS_ERROR("[scan_controller_with_recorder] failed to get current package path: %s", e.what());
            }
        }

        return result;
    }    void createTimestampDirectory()
    {
        // 获取当前时间
        std::time_t now = std::time(nullptr);
        std::tm* tm = std::localtime(&now);
        
        // 检查localtime返回值是否有效
        if (!tm) {
            ROS_ERROR("[scan_controller_with_recorder] failed to get local time");
            // 使用默认时间戳
            std::string timestamp_dir = output_base_dir_ + "/scan_default";
            if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
                ROS_INFO("[scan_controller_with_recorder] created default timestamp directory: %s", timestamp_dir.c_str());
            }
            output_file_ = timestamp_dir + "/scan_data.csv";
            return;
        }
        
        // 格式化为 YYYYMMDD_HHMM
        char timestamp[20];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", tm);
        
        // 使用配置的基目录创建时间戳文件夹
        std::string timestamp_dir = output_base_dir_ + "/scan_" + timestamp;
        
        // 创建目录
        if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
            ROS_INFO("[scan_controller_with_recorder] created timestamp directory: %s", timestamp_dir.c_str());
        } else {
            // 检查是否是因为目录已存在
            if (errno == EEXIST) {
                ROS_INFO("[scan_controller_with_recorder] timestamp directory already exists: %s", timestamp_dir.c_str());
            } else {
                ROS_WARN("[scan_controller_with_recorder] failed to create directory: %s (errno: %d)", timestamp_dir.c_str(), errno);
            }
        }
        
        // 设置输出文件路径为文件夹中的scan_data.csv
        output_file_ = timestamp_dir + "/scan_data.csv";
        ROS_INFO("[scan_controller_with_recorder] output file: %s", output_file_.c_str());
    }

    void finalizeScan()
    {
        // 确保输出文件正确关闭
        std::ofstream file_check(output_file_, std::ios::app);
        if (file_check.is_open()) {
            file_check.close();
            ROS_INFO("[scan_controller_with_recorder] output file closed successfully: %s", output_file_.c_str());
        }

        // 计算扫描统计信息
        size_t total_points = scan_points_.size();
        ROS_INFO("[scan_controller_with_recorder] scan completed: %zu points scanned", total_points);

        // 发布扫描完成通知
        std_msgs::String complete_msg;
        complete_msg.data = "Scan completed successfully. Data saved to: " + output_file_;
        scan_complete_pub_.publish(complete_msg);
        ROS_INFO("[scan_controller_with_recorder] published scan completion notification");

        // 清理收集的数据缓冲区
        collected_data_.clear();
        ROS_INFO("[scan_controller_with_recorder] cleaned up collected data buffer");
    }

    std::vector<geometry_msgs::Pose> scan_points_;
    std::vector<mag_sensor_node::MagSensorData> collected_data_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_controller_with_recorder_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ScanControllerWithRecorderNode node(nh, pnh);
    node.run();

    return 0;
}