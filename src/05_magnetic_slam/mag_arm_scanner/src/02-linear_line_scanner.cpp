#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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
    std::string diana7_group;
    std::string diana7_link;
    std::string arm2_group;

    std::vector<double> diana7_start_pose;
    std::vector<double> diana7_end_pose;

    double velocity_scaling;
    double acceleration_scaling;
    int num_steps;
    double settle_time;
    double step_wait_time;
    std::string output_file;
    std::string estimated_pose_topic;
};

class Diana7Scan
{
public:
    Diana7Scan(ros::NodeHandle nh) : nh_(nh)
    {
        loadParams();

        spinner_ = std::make_unique<ros::AsyncSpinner>(2);
        spinner_->start();

        // 订阅传感器数据
        mag_sub_ = nh_.subscribe("/mag_device_sensor/data_mT", 10, &Diana7Scan::magCallback, this);
        // 订阅估计位姿数据
        mag_pose_sub_ = nh_.subscribe(config_.estimated_pose_topic, 10, &Diana7Scan::estimatedPoseCallback, this);

        // 初始化MoveGroup
        diana7_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.diana7_group);
        arm2_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.arm2_group);

        setupGroup(*diana7_group_);
        setupGroup(*arm2_group_);
    }

    void run()
    {
        ROS_INFO("Starting Diana7 Single Arm Scan Routine...");

        // 0. 将 Arm2 移动到避让位置 (up)
        ROS_INFO("0. Moving Arm2 to 'up' position for clearance...");
        arm2_group_->setNamedTarget("up");
        for (int retry = 1; retry <= 3; ++retry) {
            if (arm2_group_->move()) {
                break;
            }
            ROS_WARN("Attempt %d/3: Failed to move Arm2 to 'up'. Retrying in 1s...", retry);
            ros::Duration(1.0).sleep();
            if (retry == 3) {
                ROS_ERROR("Failed to move Arm2 to 'up' after 3 attempts. Proceeding with caution...");
            }
        }

        // 初始化输出文件头
        initOutputFile();

        geometry_msgs::Pose d7_start = vectorToPose(config_.diana7_start_pose);
        geometry_msgs::Pose d7_end = vectorToPose(config_.diana7_end_pose);

        // 1. 将机械臂移动到起始位置
        ROS_INFO("1. Moving Diana7 to Start Position...");
        if (!moveToPose(*diana7_group_, d7_start, config_.diana7_link)) {
            ROS_ERROR("Failed to move Diana7 to start pose.");
            return;
        }

        // 等待一小段时间以确保稳定并收集基准数据
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

        // 2. 步进移动到终止位置并采集数据
        ROS_INFO("2. Starting Stepped Movement...");
        performSteppedMove(d7_end, ros::Time::now());

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

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> diana7_group_, arm2_group_;

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

        pnh.param<std::string>("diana7_group", config_.diana7_group, "diana7");
        pnh.param<std::string>("diana7_link", config_.diana7_link, "diana7_bracket_tcp_link");
        pnh.param<std::string>("arm2_group", config_.arm2_group, "arm2");
        pnh.param<std::string>("estimated_pose_topic", config_.estimated_pose_topic, "/magnetic/pose_estimated");

        // 任务特定参数从 single_arm 子命名空间读取
        ros::NodeHandle mnh("~single_arm");

        std::vector<double> zero_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        mnh.param<std::vector<double>>("diana7_start_pose", config_.diana7_start_pose, zero_pose);
        mnh.param<std::vector<double>>("diana7_end_pose", config_.diana7_end_pose, zero_pose);

        mnh.param<double>("velocity_scaling", config_.velocity_scaling, 0.1);
        mnh.param<double>("acceleration_scaling", config_.acceleration_scaling, 0.1);
        mnh.param<int>("num_steps", config_.num_steps, 20);
        mnh.param<double>("settle_time", config_.settle_time, 0.2);
        mnh.param<double>("step_wait_time", config_.step_wait_time, 0.5);
        mnh.param<std::string>("output_file", config_.output_file, "diana7_scan_results.json");

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
        } else {
            ROS_ERROR("Could not open file for writing: %s", config_.output_file.c_str());
        }
    }

    void closeOutputFile()
    {
        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) {
            ofs << "\n]\n";
            ofs.close();
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
        }

        geometry_msgs::PoseStamped diana7_pose = diana7_group_->getCurrentPose(config_.diana7_link);

        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) {
            if (!first_entry_) ofs << ",\n";
            first_entry_ = false;

            ofs << "  {\n";
            ofs << "    \"step\": " << step << ",\n";
            ofs << "    \"timestamp\": " << std::fixed << std::setprecision(6) << avg_timestamp << ",\n";

            ofs << "    \"poses\": {\n";
            ofs << "      \"diana7\": {\n";
            ofs << "        \"position\": {\"x\": " << diana7_pose.pose.position.x << ", \"y\": " << diana7_pose.pose.position.y << ", \"z\": " << diana7_pose.pose.position.z << "},\n";
            ofs << "        \"orientation\": {\"x\": " << diana7_pose.pose.orientation.x << ", \"y\": " << diana7_pose.pose.orientation.y << ", \"z\": " << diana7_pose.pose.orientation.z << ", \"w\": " << diana7_pose.pose.orientation.w << "}\n";
            ofs << "      }\n";
            ofs << "    },\n";

            ofs << "    \"estimated_pose\": {\n";
            ofs << "      \"position\": {\"x\": " << est_pose.position.x << ", \"y\": " << est_pose.position.y << ", \"z\": " << est_pose.position.z << "},\n";
            ofs << "      \"orientation\": {\"x\": " << est_pose.orientation.x << ", \"y\": " << est_pose.orientation.y << ", \"z\": " << est_pose.orientation.z << ", \"w\": " << est_pose.orientation.w << "},\n";
            ofs << "      \"strength\": " << est_pose.magnetic_strength << ",\n";
            ofs << "      \"confidence\": " << est_pose.confidence << "\n";
            ofs << "    },\n";

            ofs << "    \"magnetic_data\": [\n";
            for (size_t i = 0; i < num_sensors; ++i) {
                ofs << "      {\"id\": " << (i + 1) << ", \"x\": " << avg_x[i] << ", \"y\": " << avg_y[i] << ", \"z\": " << avg_z[i] << "}"
                    << (i == num_sensors - 1 ? "" : ",") << "\n";
            }
            ofs << "    ]\n";
            ofs << "  }";
            ofs.close();
        }
    }

    void setupGroup(moveit::planning_interface::MoveGroupInterface& group)
    {
        ros::param::set("/move_group/allow_start_state_max_bounds_error", 0.01);
        group.setMaxVelocityScalingFactor(config_.velocity_scaling);
        group.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
        group.setPlanningTime(5.0);
        group.setNumPlanningAttempts(5);
        group.setGoalPositionTolerance(0.001);
        group.setGoalOrientationTolerance(0.005);
    }

    geometry_msgs::Pose vectorToPose(const std::vector<double>& v)
    {
        geometry_msgs::Pose p;
        if (v.size() < 6) return p;
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
        for (int retry = 1; retry <= 3; ++retry) {
            group.setStartStateToCurrentState();
            ROS_INFO("Attempt %d/3 to move...", retry);
            group.setPoseTarget(target_pose, link_name);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                if (group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    return true;
                }
            }
            ros::Duration(0.5).sleep();
        }
        return false;
    }

    void performSteppedMove(const geometry_msgs::Pose& target_pose, ros::Time start_time)
    {
        geometry_msgs::Pose start_pose = diana7_group_->getCurrentPose(config_.diana7_link).pose;

        for (int i = 1; i <= config_.num_steps; ++i) {
            double fraction = static_cast<double>(i) / config_.num_steps;
            geometry_msgs::Pose waypoint = interpolatePose(start_pose, target_pose, fraction);

            ros::Duration elapsed = ros::Time::now() - start_time;
            double avg_time_per_step = elapsed.toSec() / i;
            double remaining_time = avg_time_per_step * (config_.num_steps - i);

            int e_min = (int)elapsed.toSec() / 60;
            int e_sec = (int)elapsed.toSec() % 60;
            int r_min = (int)remaining_time / 60;
            int r_sec = (int)remaining_time % 60;

            ROS_INFO("---------------------------------------------------------");
            ROS_INFO("Progress: [%d/%d] steps (%.1f%%)", i, config_.num_steps, (double)i/config_.num_steps*100.0);
            ROS_INFO("Time: Elapsed: %dm %ds | Est. Remaining: %dm %ds", e_min, e_sec, r_min, r_sec);
            ROS_INFO("---------------------------------------------------------");

            if (moveToPose(*diana7_group_, waypoint, config_.diana7_link)) {
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
                ROS_ERROR("Failed to plan to waypoint at step %d", i);
                return;
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diana7_single_arm_tracking");
    ros::NodeHandle nh;
    Diana7Scan scanner(nh);
    ros::Duration(1.0).sleep();
    scanner.run();
    ros::waitForShutdown();
    return 0;
}
