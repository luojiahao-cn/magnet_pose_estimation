#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
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

// 保存扫描参数的辅助结构体 (与 dual_arm_tracking 逻辑对齐)
struct PlaneScanConfig {
    std::string diana7_group;
    std::string diana7_link;
    std::string arm2_group;

    // 平面扫描参数
    std::vector<double> origin_pose; // [x, y, z, r, p, y]
    double plane_width;              // X轴方向长度 (m)
    double plane_height;             // Y轴方向长度 (m)
    int steps_x;                     // X方向步数
    int steps_y;                     // Y方向步数

    double velocity_scaling;
    double acceleration_scaling;
    double settle_time;
    double step_wait_time;
    std::string output_file;
    std::string estimated_pose_topic;
};

class Diana7PlaneScanner
{
public:
    Diana7PlaneScanner(ros::NodeHandle nh) : nh_(nh)
    {
        loadParams();
        spinner_ = std::make_unique<ros::AsyncSpinner>(2);
        spinner_->start();

        // 订阅传感器数据与估计位姿 (对齐 dual_arm_tracking)
        mag_sub_ = nh_.subscribe("/mag_device_sensor/data_mT", 10, &Diana7PlaneScanner::magCallback, this);
        mag_pose_sub_ = nh_.subscribe(config_.estimated_pose_topic, 10, &Diana7PlaneScanner::estimatedPoseCallback, this);

        // 初始化MoveGroup
        diana7_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.diana7_group);
        arm2_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(config_.arm2_group);

        setupGroup(*diana7_group_);
        setupGroup(*arm2_group_);
    }

    void run()
    {
        ROS_INFO("Starting Planar Scan (Based on dual_arm_tracking's robust logic)...");

        initOutputFile();

        geometry_msgs::Pose origin = vectorToPose(config_.origin_pose);

        // 1. 移动到起始点 (Origin)
        ROS_INFO("1. Moving Diana7 to Plane Origin [x: %.3f, y: %.3f, z: %.3f]...",
                 origin.position.x, origin.position.y, origin.position.z);

        // 使用手动 IK 求解确保成功率
        if (!moveToPoseIK(origin)) {
            ROS_ERROR("CRITICAL: Failed to reach plane origin. Aborting scan.");
            return;
        }

        // 2. 将 Arm2 移动到避让位置 (up)
        ROS_INFO("2. Moving Arm2 to 'up' position for clearance...");
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

        // 3. 网格扫描 (S型路径)
        int total_points = config_.steps_x * config_.steps_y;
        int current_point = 0;
        ros::Time start_time = ros::Time::now();

        for (int j = 0; j < config_.steps_y; ++j) {
            double y_offset = (config_.steps_y > 1) ? (j * config_.plane_height / (config_.steps_y - 1)) : 0;

            // S型路径逻辑：偶数行从左往右，奇数行从右往左
            bool left_to_right = (j % 2 == 0);

            for (int i = 0; i < config_.steps_x; ++i) {
                int x_idx = left_to_right ? i : (config_.steps_x - 1 - i);
                double x_offset = (config_.steps_x > 1) ? (x_idx * config_.plane_width / (config_.steps_x - 1)) : 0;

                geometry_msgs::Pose waypoint = origin;
                waypoint.position.x += x_offset;
                waypoint.position.y += y_offset;

                current_point++;
                ros::Duration elapsed = ros::Time::now() - start_time;
                double avg_time_per_point = elapsed.toSec() / (current_point > 1 ? current_point - 1 : 1);
                double remaining_time = avg_time_per_point * (total_points - current_point + 1);
                if (current_point == 1) remaining_time = 0;

                int e_min = (int)elapsed.toSec() / 60;
                int e_sec = (int)elapsed.toSec() % 60;
                int r_min = (int)remaining_time / 60;
                int r_sec = (int)remaining_time % 60;

                ROS_INFO("---------------------------------------------------------");
                ROS_INFO("Progress: [%d/%d] points (%.1f%%)", current_point, total_points, (double)current_point/total_points*100.0);
                ROS_INFO("Grid Index: [%d, %d]", x_idx, j);
                ROS_INFO("Time: Elapsed: %dm %ds | Est. Remaining: %dm %ds", e_min, e_sec, r_min, r_sec);
                ROS_INFO("---------------------------------------------------------");

                if (moveToPoseIK(waypoint)) {
                    // 1. 等待机械臂稳定 (settle time)
                    if (config_.settle_time > 0) {
                        ros::Duration(config_.settle_time).sleep();
                    }

                    // 2. 数据采集过程 (与 dual_arm_tracking 保持一致)
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

                    saveData(x_idx, j);
                } else {
                    ROS_ERROR("Failed to plan to grid point [%d, %d].", x_idx, j);
                }
            }
        }

        closeOutputFile();
        ROS_INFO("Planar scan routine successfully completed.");
    }

private:
    ros::NodeHandle nh_;
    PlaneScanConfig config_;
    std::unique_ptr<ros::AsyncSpinner> spinner_;
    ros::Subscriber mag_sub_, mag_pose_sub_;
    mag_core_msgs::MagSensorArray latest_mag_data_;
    mag_core_msgs::MagnetPose latest_estimated_pose_;
    std::vector<mag_core_msgs::MagSensorArray> mag_buffer_;
    bool is_collecting_ = false;
    std::mutex data_mutex_, pose_mutex_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> diana7_group_, arm2_group_;
    bool first_entry_ = true;

    void magCallback(const mag_core_msgs::MagSensorArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_mag_data_ = *msg;
        if (is_collecting_) mag_buffer_.push_back(*msg);
    }

    void estimatedPoseCallback(const mag_core_msgs::MagnetPose::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_estimated_pose_ = *msg;
    }

    void loadParams() {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("diana7_group", config_.diana7_group, "diana7");
        pnh.param<std::string>("diana7_link", config_.diana7_link, "diana7_bracket_tcp_link");
        pnh.param<std::string>("arm2_group", config_.arm2_group, "arm2");
        pnh.param<std::string>("estimated_pose_topic", config_.estimated_pose_topic, "/magnetic/pose_estimated");

        // 任务特定参数从 plane_scan 子命名空间读取
        ros::NodeHandle mnh("~plane_scan");

        std::vector<double> def_origin = {0.52, -0.22, 1.0, 0, 3.14159, 1.5708};
        mnh.param<std::vector<double>>("origin_pose", config_.origin_pose, def_origin);
        mnh.param<double>("plane_width", config_.plane_width, 0.35);
        mnh.param<double>("plane_height", config_.plane_height, 0.40);
        mnh.param<int>("steps_x", config_.steps_x, 15);
        mnh.param<int>("steps_y", config_.steps_y, 15);
        mnh.param<double>("velocity_scaling", config_.velocity_scaling, 0.05);
        mnh.param<double>("acceleration_scaling", config_.acceleration_scaling, 0.05);
        mnh.param<double>("settle_time", config_.settle_time, 0.2);
        mnh.param<double>("step_wait_time", config_.step_wait_time, 1.0);
        mnh.param<std::string>("output_file", config_.output_file, "planar_scan_results.json");

        ROS_INFO("Planar Scan Config Loaded (Sub-NS: plane_scan)");
        ROS_INFO("Vel: %.2f, Settle: %.2f, StepWait: %.2f", config_.velocity_scaling, config_.settle_time, config_.step_wait_time);

        if (!config_.output_file.empty() && config_.output_file[0] != '/') {
            config_.output_file = ros::package::getPath("mag_arm_scanner") + "/" + config_.output_file;
        }
    }

    void setupGroup(moveit::planning_interface::MoveGroupInterface& group) {
        ros::param::set("/move_group/allow_start_state_max_bounds_error", 0.01);
        group.setMaxVelocityScalingFactor(config_.velocity_scaling);
        group.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
        group.setPlanningTime(5.0);
        group.setNumPlanningAttempts(5);
        group.setGoalPositionTolerance(0.001);
        group.setGoalOrientationTolerance(0.005);
    }

    void initOutputFile() {
        std::ofstream ofs(config_.output_file, std::ios::out);
        if (ofs.is_open()) { ofs << "[\n"; ofs.close(); first_entry_ = true; }
    }

    void closeOutputFile() {
        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) { ofs << "\n]\n"; ofs.close(); }
    }

    // 与 dual_arm_tracking 核心逻辑一致：手动解算 IK，并设置为关节空间目标
    bool moveToPoseIK(const geometry_msgs::Pose& target) {
        for (int retry = 1; retry <= 3; ++retry) {
            ROS_INFO("Attempt %d/3 to move to pose...", retry);
            diana7_group_->setStartStateToCurrentState();
            diana7_group_->clearPoseTargets();

            // 确保应用参数中的速度/加速度限制
            diana7_group_->setMaxVelocityScalingFactor(config_.velocity_scaling);
            diana7_group_->setMaxAccelerationScalingFactor(config_.acceleration_scaling);

            moveit::core::RobotStatePtr kinematic_state = diana7_group_->getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup(config_.diana7_group);

            // 使用 IK 求解
            bool found_ik = kinematic_state->setFromIK(joint_model_group, target, config_.diana7_link, 10, 0.1);

            if (found_ik) {
                diana7_group_->setJointValueTarget(*kinematic_state);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (diana7_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    if (diana7_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                        return true;
                    }
                }
            } else {
                ROS_ERROR("Failed to compute IK solution for target pose.");
            }
            ros::Duration(0.5).sleep();
        }
        return false;
    }

    void saveData(int ix, int iy) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_mag_data_.mag_x.empty()) return;

        mag_core_msgs::MagnetPose est_pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            est_pose = latest_estimated_pose_;
        }

        size_t n = latest_mag_data_.mag_x.size();
        std::vector<double> ax(n, 0), ay(n, 0), az(n, 0);
        double ts = latest_mag_data_.header.stamp.toSec();

        if (!mag_buffer_.empty()) {
            double ts_sum = 0;
            for (const auto& m : mag_buffer_) {
                ts_sum += m.header.stamp.toSec();
                for (size_t i = 0; i < n && i < m.mag_x.size(); ++i) {
                    ax[i] += m.mag_x[i]; ay[i] += m.mag_y[i]; az[i] += m.mag_z[i];
                }
            }
            double cnt = static_cast<double>(mag_buffer_.size());
            ts = ts_sum / cnt;
            for (size_t i = 0; i < n; ++i) { ax[i] /= cnt; ay[i] /= cnt; az[i] /= cnt; }
        }

        geometry_msgs::PoseStamped curr = diana7_group_->getCurrentPose(config_.diana7_link);
        std::ofstream ofs(config_.output_file, std::ios::app);
        if (ofs.is_open()) {
            if (!first_entry_) ofs << ",\n"; first_entry_ = false;
            ofs << "  {\n";
            ofs << "    \"grid_index\": [" << ix << ", " << iy << "],\n";
            ofs << "    \"timestamp\": " << std::fixed << std::setprecision(6) << ts << ",\n";
            ofs << "    \"pose\": {\n";
            ofs << "      \"position\": {\"x\": " << curr.pose.position.x << ", \"y\": " << curr.pose.position.y << ", \"z\": " << curr.pose.position.z << "},\n";
            ofs << "      \"orientation\": {\"x\": " << curr.pose.orientation.x << ", \"y\": " << curr.pose.orientation.y << ", \"z\": " << curr.pose.orientation.z << ", \"w\": " << curr.pose.orientation.w << "}\n";
            ofs << "    },\n";
            ofs << "    \"estimated_pose\": {\n";
            ofs << "      \"position\": {\"x\": " << est_pose.position.x << ", \"y\": " << est_pose.position.y << ", \"z\": " << est_pose.position.z << "},\n";
            ofs << "      \"orientation\": {\"x\": " << est_pose.orientation.x << ", \"y\": " << est_pose.orientation.y << ", \"z\": " << est_pose.orientation.z << ", \"w\": " << est_pose.orientation.w << "}\n";
            ofs << "    },\n";
            ofs << "    \"magnetic_data\": [\n";
            for (size_t i = 0; i < n; ++i) {
                ofs << "      {\"id\": " << (i + 1) << ", \"x\": " << ax[i] << ", \"y\": " << ay[i] << ", \"z\": " << az[i] << "}" << (i == n - 1 ? "" : ",") << "\n";
            }
            ofs << "    ]\n  }";
            ofs.close();
        }
    }

    geometry_msgs::Pose vectorToPose(const std::vector<double>& v) {
        geometry_msgs::Pose p;
        if (v.size() < 6) return p;
        p.position.x = v[0]; p.position.y = v[1]; p.position.z = v[2];
        tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]); p.orientation = tf2::toMsg(q);
        return p;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "diana7_plane_scan");
    ros::NodeHandle nh;
    Diana7PlaneScanner scanner(nh);
    ros::Duration(1.0).sleep();
    scanner.run();
    ros::waitForShutdown();
    return 0;
}
