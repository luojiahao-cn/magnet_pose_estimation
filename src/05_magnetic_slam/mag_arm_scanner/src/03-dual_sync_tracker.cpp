#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mag_core_msgs/MagSensorArray.h>
#include <mag_core_msgs/MagnetPose.h>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <ros/package.h>

// 保存扫描参数的辅助结构体
struct MoveConfig {
    std::string arm1_group;
    std::string arm2_group;
    std::string diana7_group;
    std::string sync_group;

    std::string arm1_link;
    std::string arm2_link;
    std::string diana7_link;

    std::vector<double> diana7_start_pose;
    std::vector<double> diana7_end_pose;
    std::vector<double> arm2_start_pose;
    std::vector<double> arm2_end_pose;

    double velocity_scaling;
    double acceleration_scaling;
    int num_steps;
    double settle_time;
    double step_wait_time;
    std::string output_file;
    std::string estimated_pose_topic;
};

class DualArmScan
{
public:
    DualArmScan(ros::NodeHandle nh) : nh_(nh)
    {
        loadParams();

        spinner_ = std::make_unique<ros::AsyncSpinner>(2);
        spinner_->start();

        // 订阅传感器数据
        mag_sub_ = nh_.subscribe("/mag_device_sensor/data_mT", 10, &DualArmScan::magCallback, this);
        // 订阅估计位姿数据
        mag_pose_sub_ = nh_.subscribe(config_.estimated_pose_topic, 10, &DualArmScan::estimatedPoseCallback, this);

        // 初始化MoveGroups
        arm1_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.arm1_group);
        arm2_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.arm2_group);
        diana7_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.diana7_group);
        sync_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.sync_group);

        setupGroup(*arm2_group_);
        setupGroup(*diana7_group_);
        setupGroup(*sync_group_);

        // 规划组设置
        sync_group_->setPlanningTime(10.0);
        sync_group_->setGoalPositionTolerance(0.01);
        sync_group_->setGoalOrientationTolerance(0.05);
    }

    void run()
    {
        ROS_INFO("Starting Dual Arm Scan Routine...");

        // 初始化输出文件头
        initOutputFile();

        geometry_msgs::Pose d7_start = vectorToPose(config_.diana7_start_pose);
        geometry_msgs::Pose d7_end = vectorToPose(config_.diana7_end_pose);
        geometry_msgs::Pose a2_start = vectorToPose(config_.arm2_start_pose);
        geometry_msgs::Pose a2_end = vectorToPose(config_.arm2_end_pose);

        // 1. 分别将机械臂移动到起始位置
        ROS_INFO("1. Moving Diana7 to Start Position...");
        if (!moveToPose(*diana7_group_, d7_start, config_.diana7_link)) {
            ROS_ERROR("Failed to move Diana7 to start pose.");
            return;
        }

        ROS_INFO("2. Moving Arm2 to Start Position...");
        if (!moveToPose(*arm2_group_, a2_start, config_.arm2_link)) {
            ROS_ERROR("Failed to move Arm2 to start pose.");
            return;
        }

        // 等待一小段时间以确保稳定
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            mag_buffer_.clear();
            is_collecting_ = true;
        }
        ros::Duration(1.0).sleep();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            is_collecting_ = false;
        }

        // 保存起始点数据 (Step 0)
        saveSensorData(0);

        // 2. 同步移动到终止位置
        ROS_INFO("3. Starting Synchronized Movement...");
        performSyncMove(d7_end, a2_end);

        closeOutputFile();
    }

private:
    ros::NodeHandle nh_;
    MoveConfig config_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    ros::Subscriber mag_sub_;
    ros::Subscriber mag_pose_sub_;
    mag_core_msgs::MagSensorArray latest_mag_data_;
    mag_core_msgs::MagnetPose latest_estimated_pose_;
    std::vector<mag_core_msgs::MagSensorArray> mag_buffer_;
    bool is_collecting_ = false;
    std::mutex data_mutex_;
    std::mutex pose_mutex_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm1_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm2_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> diana7_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> sync_group_;

    bool first_entry_ = true;

    void magCallback(const mag_core_msgs::MagSensorArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_mag_data_ = *msg;
        if (is_collecting_) {
            mag_buffer_.push_back(*msg);
        }
    }

    void estimatedPoseCallback(const mag_core_msgs::MagnetPose::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_estimated_pose_ = *msg;
    }

    void loadParams()
    {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("arm1_group", config_.arm1_group, "arm1");
        pnh.param<std::string>("arm2_group", config_.arm2_group, "arm2");
        pnh.param<std::string>("diana7_group", config_.diana7_group, "diana7");
        pnh.param<std::string>("sync_group", config_.sync_group, "triple_arms");

        pnh.param<std::string>("arm1_link", config_.arm1_link, "arm1_ee_link");
        pnh.param<std::string>("arm2_link", config_.arm2_link, "arm2_tool300_tcp_link");
        pnh.param<std::string>("diana7_link", config_.diana7_link, "diana7_bracket_tcp_link");
        pnh.param<std::string>("estimated_pose_topic", config_.estimated_pose_topic, "/magnetic/pose_estimated");

        // 任务特定参数从 dual_arm 子命名空间读取
        ros::NodeHandle mnh("~dual_arm");

        std::vector<double> zero_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        mnh.param<std::vector<double>>("diana7_start_pose", config_.diana7_start_pose, zero_pose);
        mnh.param<std::vector<double>>("diana7_end_pose", config_.diana7_end_pose, zero_pose);
        mnh.param<std::vector<double>>("arm2_start_pose", config_.arm2_start_pose, zero_pose);
        mnh.param<std::vector<double>>("arm2_end_pose", config_.arm2_end_pose, zero_pose);

        mnh.param<double>("velocity_scaling", config_.velocity_scaling, 0.05);
        mnh.param<double>("acceleration_scaling", config_.acceleration_scaling, 0.05);
        mnh.param<int>("num_steps", config_.num_steps, 50);
        mnh.param<double>("settle_time", config_.settle_time, 0.2);
        mnh.param<double>("step_wait_time", config_.step_wait_time, 1.0);
        mnh.param<std::string>("output_file", config_.output_file, "scan_results.json");

        // 如果路径不是绝对路径，则将其相对于包路径
        if (!config_.output_file.empty() && config_.output_file[0] != '/') {
            std::string pkg_path = ros::package::getPath("mag_arm_scanner");
            config_.output_file = pkg_path + "/" + config_.output_file;
        }
    }

    void initOutputFile()
    {
        ROS_INFO("Initializing output file: %s", config_.output_file.c_str());
        std::ofstream ofs(config_.output_file, std::ios::out);
        if (ofs.is_open()) {
            ofs << "[\n";
            ofs.close();
            first_entry_ = true;
            ROS_INFO("Output file initialized as JSON array.");
        } else {
            ROS_ERROR("CRITICAL ERROR: Could not open file for writing: %s", config_.output_file.c_str());
        }
    }

    void closeOutputFile()
    {
        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) {
            ofs << "\n]\n";
            ofs.close();
            ROS_INFO("JSON file finalized.");
        }
    }

    void saveSensorData(int step)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        mag_core_msgs::MagnetPose est_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            est_pose = latest_estimated_pose_;
        }

        if (latest_mag_data_.mag_x.empty()) {
            ROS_WARN("No sensor data received yet, skipping save for step %d", step);
            return;
        }

        // 计算平均值
        size_t num_sensors = latest_mag_data_.mag_x.size();
        std::vector<double> avg_x(num_sensors, 0.0);
        std::vector<double> avg_y(num_sensors, 0.0);
        std::vector<double> avg_z(num_sensors, 0.0);
        double avg_timestamp = latest_mag_data_.header.stamp.toSec();

        if (!mag_buffer_.empty()) {
            double ts_sum = 0;
            for (const auto& msg : mag_buffer_) {
                ts_sum += msg.header.stamp.toSec();
                for (size_t i = 0; i < num_sensors && i < msg.mag_x.size(); ++i) {
                    avg_x[i] += msg.mag_x[i];
                    avg_y[i] += msg.mag_y[i];
                    avg_z[i] += msg.mag_z[i];
                }
            }
            double count = static_cast<double>(mag_buffer_.size());
            avg_timestamp = ts_sum / count;
            for (size_t i = 0; i < num_sensors; ++i) {
                avg_x[i] /= count;
                avg_y[i] /= count;
                avg_z[i] /= count;
            }
            ROS_INFO("Averaged over %zu samples for step %d", mag_buffer_.size(), step);
        } else {
            avg_x = latest_mag_data_.mag_x;
            avg_y = latest_mag_data_.mag_y;
            avg_z = latest_mag_data_.mag_z;
        }

        // 获取当前各臂末端位姿
        geometry_msgs::PoseStamped arm1_pose = arm1_group_->getCurrentPose(config_.arm1_link);
        geometry_msgs::PoseStamped arm2_pose = arm2_group_->getCurrentPose(config_.arm2_link);
        geometry_msgs::PoseStamped diana7_pose = diana7_group_->getCurrentPose(config_.diana7_link);

        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) {
            if (!first_entry_) {
                ofs << ",\n";
            }
            first_entry_ = false;

            ofs << "  {\n";
            ofs << "    \"step\": " << step << ",\n";
            ofs << "    \"timestamp\": " << std::fixed << std::setprecision(6) << avg_timestamp << ",\n";

            auto write_pose = [&](const std::string& name, const geometry_msgs::Pose& p, bool last = false) {
                ofs << "    \"" << name << "\": {\n";
                ofs << "      \"position\": {\"x\": " << p.position.x << ", \"y\": " << p.position.y << ", \"z\": " << p.position.z << "},\n";
                ofs << "      \"orientation\": {\"x\": " << p.orientation.x << ", \"y\": " << p.orientation.y << ", \"z\": " << p.orientation.z << ", \"w\": " << p.orientation.w << "}\n";
                ofs << "    }" << (last ? "" : ",") << "\n";
            };

            ofs << "    \"poses\": {\n";
            write_pose("arm1", arm1_pose.pose);
            write_pose("arm2", arm2_pose.pose);
            write_pose("diana7", diana7_pose.pose, true);
            ofs << "    },\n";

            ofs << "    \"estimated_pose\": {\n";
            ofs << "      \"position\": {\"x\": " << est_pose.position.x << ", \"y\": " << est_pose.position.y << ", \"z\": " << est_pose.position.z << "},\n";
            ofs << "      \"orientation\": {\"x\": " << est_pose.orientation.x << ", \"y\": " << est_pose.orientation.y << ", \"z\": " << est_pose.orientation.z << ", \"w\": " << est_pose.orientation.w << "},\n";
            ofs << "      \"strength\": " << est_pose.magnetic_strength << ",\n";
            ofs << "      \"confidence\": " << est_pose.confidence << "\n";
            ofs << "    },\n";

            ofs << "    \"magnetic_data\": [\n";
            for (size_t i = 0; i < num_sensors; ++i) {
                ofs << "      {\"id\": " << (i + 1) << ", \"x\": " << avg_x[i]
                    << ", \"y\": " << avg_y[i] << ", \"z\": " << avg_z[i] << "}"
                    << (i == num_sensors - 1 ? "" : ",") << "\n";
            }
            ofs << "    ]\n";
            ofs << "  }";

            ofs.close();
            ROS_INFO("Saved JSON entry (step %d)", step);
        } else {
            ROS_ERROR("Failed to open output file: %s", config_.output_file.c_str());
        }
    }

    void setupGroup(moveit::planning_interface::MoveGroupInterface& group)
    {
        group.setMaxVelocityScalingFactor(config_.velocity_scaling);
        group.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
        group.setPlanningTime(5.0);
        group.setNumPlanningAttempts(5);
    }

    geometry_msgs::Pose vectorToPose(const std::vector<double>& v)
    {
        geometry_msgs::Pose p;
        if (v.size() < 6) {
            ROS_ERROR("Pose vector too short, returning identity.");
            return p;
        }
        p.position.x = v[0];
        p.position.y = v[1];
        p.position.z = v[2];
        tf2::Quaternion q;
        q.setRPY(v[3], v[4], v[5]);
        p.orientation = tf2::toMsg(q);
        return p;
    }

    geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, double fraction)
    {
        geometry_msgs::Pose res;
        res.position.x = start.position.x + (end.position.x - start.position.x) * fraction;
        res.position.y = start.position.y + (end.position.y - start.position.y) * fraction;
        res.position.z = start.position.z + (end.position.z - start.position.z) * fraction;

        tf2::Quaternion q_start, q_end;
        tf2::fromMsg(start.orientation, q_start);
        tf2::fromMsg(end.orientation, q_end);
        tf2::Quaternion q_res = q_start.slerp(q_end, fraction);
        res.orientation = tf2::toMsg(q_res);
        return res;
    }

    bool moveToPose(moveit::planning_interface::MoveGroupInterface& group, const geometry_msgs::Pose& target_pose, const std::string& link_name)
    {
        group.setPoseTarget(target_pose, link_name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            group.execute(plan);
        }
        return success;
    }

    void performSyncMove(const geometry_msgs::Pose& d7_target, const geometry_msgs::Pose& a2_target)
    {
        // 1. 获取起始位姿
        geometry_msgs::Pose d7_start = diana7_group_->getCurrentPose(config_.diana7_link).pose;
        geometry_msgs::Pose a2_start = arm2_group_->getCurrentPose(config_.arm2_link).pose;

        // 2. 遍历执行每一个插值点
        ROS_INFO("Executing stepped Cartesian synchronized movement (steps: %d)...", config_.num_steps);

        for (int i = 1; i <= config_.num_steps; ++i) {
            double fraction = static_cast<double>(i) / config_.num_steps;

            geometry_msgs::Pose d7_waypoint = interpolatePose(d7_start, d7_target, fraction);
            geometry_msgs::Pose a2_waypoint = interpolatePose(a2_start, a2_target, fraction);

            // 重新获取当前状态，因为上一步可能已经改变了状态
            moveit::core::RobotStatePtr current_state = sync_group_->getCurrentState();
            const moveit::core::JointModelGroup* arm2_jmg = current_state->getJointModelGroup(config_.arm2_group);
            const moveit::core::JointModelGroup* d7_jmg = current_state->getJointModelGroup(config_.diana7_group);

            moveit::core::RobotStatePtr waypoint_state(new moveit::core::RobotState(*current_state));

            bool ok = true;
            if (!waypoint_state->setFromIK(arm2_jmg, a2_waypoint, config_.arm2_link)) ok = false;
            if (!waypoint_state->setFromIK(d7_jmg, d7_waypoint, config_.diana7_link)) ok = false;

            if (!ok) {
                ROS_ERROR("Failed to compute IK for Cartesian waypoint %d/%d", i, config_.num_steps);
                return;
            }

            // 执行移动到当前点
            sync_group_->setJointValueTarget(*waypoint_state);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (sync_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                sync_group_->execute(plan);

                // 1. 等待机械臂稳定 (settle time)
                if (config_.settle_time > 0) {
                    ros::Duration(config_.settle_time).sleep();
                }

                ROS_INFO("Step %d/%d completed. Collecting data for %.2fs...", i, config_.num_steps, config_.step_wait_time);

                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    mag_buffer_.clear();
                    is_collecting_ = true;
                }
                ros::Duration(config_.step_wait_time).sleep();
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    is_collecting_ = false;
                }

                saveSensorData(i);
            } else {
                ROS_ERROR("Failed to plan to step %d", i);
                return;
            }
        }
        ROS_INFO("Synchronized Cartesian movement completed.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_arm_scan");
    ros::NodeHandle nh;

    DualArmScan scanner(nh);

    // 给连接留出时间
    ros::Duration(1.0).sleep();

    scanner.run();

    ros::waitForShutdown();
    return 0;
}
