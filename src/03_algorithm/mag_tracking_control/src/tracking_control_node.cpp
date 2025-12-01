#include "mag_tracking_control/tracking_control_node.h"
#include "mag_tracking_control/tracking_control_config_loader.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include <locale.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mag_tracking_control {

TrackingControlNode::TrackingControlNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_buffer_(ros::Duration(10.0))
    , tf_listener_(tf_buffer_)
    , has_magnet_pose_(false)
    , is_executing_trajectory_(false)
    , should_stop_thread_(false)
{
    loadParameters();
    
    // 创建策略实例
    strategy_ = createStrategy(config_.strategy_type);
    if (!strategy_ || !strategy_->initialize()) {
        ROS_ERROR("[tracking_control] 策略初始化失败");
        throw std::runtime_error("策略初始化失败");
    }
    ROS_INFO("[tracking_control] 使用策略: %s", strategy_->name().c_str());
    
    // 订阅磁铁位姿
    if (!config_.magnet_pose_topic.empty()) {
        magnet_pose_sub_ = nh_.subscribe(
            config_.magnet_pose_topic,
            10,
            &TrackingControlNode::magnetPoseCallback,
            this
        );
        ROS_INFO("[tracking_control] 订阅磁铁位姿: %s", config_.magnet_pose_topic.c_str());
    }
    
    // 发布目标位姿（用于可视化）
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        config_.target_pose_topic, 10
    );
    ROS_INFO("[tracking_control] 发布目标位姿话题: %s", config_.target_pose_topic.c_str());
    
    // 初始化机械臂服务客户端
    arm_service_client_ = nh_.serviceClient<mag_device_arm::SetEndEffectorPose>(
        config_.arm_service_name
    );
    ROS_INFO("[tracking_control] 等待机械臂服务: %s", config_.arm_service_name.c_str());
    if (!arm_service_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_WARN("[tracking_control] 机械臂服务不可用，将不会执行实际运动");
    }
    
    // 根据配置选择使用连续轨迹模式还是服务模式
    if (config_.use_continuous_trajectory) {
        // 初始化笛卡尔路径服务客户端
        cartesian_path_service_name_ = config_.cartesian_path_service_name;
        
        cartesian_path_client_ = nh_.serviceClient<mag_device_arm::ExecuteCartesianPath>(
            cartesian_path_service_name_
        );
        ROS_INFO("[tracking_control] 等待笛卡尔路径服务: %s", cartesian_path_service_name_.c_str());
        if (!cartesian_path_client_.waitForExistence(ros::Duration(5.0))) {
            ROS_WARN("[tracking_control] 笛卡尔路径服务不可用，连续轨迹模式将无法工作");
        } else {
            ROS_INFO("[tracking_control] 使用连续轨迹模式（通过服务接口）");
        }
        
        // 启动轨迹执行线程
        should_stop_thread_ = false;
        trajectory_execution_thread_ = std::thread(&TrackingControlNode::trajectoryExecutionThread, this);
        ROS_INFO("[tracking_control] 轨迹执行线程已启动");
    }
    
    // 启动控制循环
    if (config_.update_rate > 0) {
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / config_.update_rate),
            &TrackingControlNode::controlLoop,
            this
        );
        ROS_INFO("[tracking_control] 控制循环频率: %.1f Hz", config_.update_rate);
    }
}

TrackingControlNode::~TrackingControlNode() {
    // 停止轨迹执行线程
    if (config_.use_continuous_trajectory) {
        should_stop_thread_.store(true);
        if (trajectory_execution_thread_.joinable()) {
            trajectory_execution_thread_.join();
        }
    }
}

void TrackingControlNode::loadParameters() {
    const std::string ns = "tracking_control_node";
    
    // 从参数服务器加载配置
    XmlRpc::XmlRpcValue config;
    if (!pnh_.getParam("config", config)) {
        ROS_ERROR("[tracking_control] 无法获取配置参数");
        throw std::runtime_error("无法获取配置参数");
    }
    
    // 解析配置
    config_ = loadTrackingControlConfig(config, ns + "/config");
    
    // 连续轨迹模式不再需要 move_group_name 等信息，因为使用服务接口
    
    ROS_INFO("[tracking_control] 配置加载完成");
    ROS_INFO("[tracking_control] 策略类型: %s", config_.strategy_type.c_str());
    ROS_INFO("[tracking_control] 传感器机械臂: %s", config_.sensor_arm_name.c_str());
    ROS_INFO("[tracking_control] 控制频率: %.1f Hz", config_.update_rate);
    if (config_.use_continuous_trajectory) {
        ROS_INFO("[tracking_control] 连续轨迹模式: 启用");
        ROS_INFO("[tracking_control] 轨迹缓冲大小: %zu", config_.trajectory_buffer_size);
    }
}

std::unique_ptr<TrackingControlStrategyBase> TrackingControlNode::createStrategy(const std::string &type) {
    if (type == "fixed_offset") {
        auto strategy = std::make_unique<FixedOffsetStrategy>();
        strategy->initialize(config_.fixed_offset, config_.max_movement_per_step);
        return strategy;
    } else if (type == "adaptive_distance") {
        auto strategy = std::make_unique<AdaptiveDistanceStrategy>();
        strategy->initialize(
            config_.target_field_strength,
            config_.min_field_strength,
            config_.max_field_strength,
            config_.adjustment_gain,
            config_.max_movement_per_step
        );
        return strategy;
    } else {
        ROS_ERROR("[tracking_control] 未知的策略类型: %s", type.c_str());
        return nullptr;
    }
}

void TrackingControlNode::magnetPoseCallback(const mag_core_msgs::MagnetPose::ConstPtr &msg) {
    current_magnet_pose_.position = msg->position;
    current_magnet_pose_.orientation = msg->orientation;
    last_magnet_pose_time_ = ros::Time::now();
    has_magnet_pose_ = true;
}

void TrackingControlNode::controlLoop(const ros::TimerEvent &/*event*/) {
    // 检查是否收到磁铁位姿
    if (!has_magnet_pose_) {
        ROS_WARN_THROTTLE(2.0, "[tracking_control] 尚未收到磁铁位姿");
        return;
    }
    
    // 检查磁铁位姿是否过期
    double time_since_last = (ros::Time::now() - last_magnet_pose_time_).toSec();
    if (time_since_last > 1.0) {
        ROS_WARN_THROTTLE(2.0, "[tracking_control] 磁铁位姿已过期 (%.2f 秒)", time_since_last);
        return;
    }
    
    // 检查策略是否需要更新
    if (!strategy_->needsUpdate()) {
        return;
    }
    
    // 获取当前传感器位姿
    geometry_msgs::Pose current_sensor_pose;
    if (!getCurrentSensorPose(current_sensor_pose)) {
        ROS_WARN_THROTTLE(2.0, "[tracking_control] 无法获取当前传感器位姿");
        return;
    }
    
    // 准备输入
    TrackingControlInput input;
    input.current_sensor_pose = current_sensor_pose;
    input.magnet_pose = current_magnet_pose_;
    input.field_strengths = last_field_strengths_;  // 为 adaptive_distance 策略预留，fixed_offset 策略不使用
    input.timestamp = ros::Time::now();
    
    // 计算目标位姿
    TrackingControlOutput output;
    if (!strategy_->computeTargetPose(input, output)) {
        ROS_WARN("[tracking_control] 策略计算失败: %s", output.message.c_str());
        return;
    }
    
    if (!output.is_valid) {
        ROS_WARN("[tracking_control] 目标位姿无效: %s", output.message.c_str());
        return;
    }
    
    // 发布目标位姿（用于可视化）
    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.header.stamp = ros::Time::now();
    target_pose_stamped.header.frame_id = config_.magnet_frame;
    target_pose_stamped.pose = output.target_pose;
    target_pose_pub_.publish(target_pose_stamped);
    
    // 执行运动
    if (config_.enable_execution) {
        if (config_.use_continuous_trajectory) {
            // 连续轨迹模式：添加到缓冲队列
            addToTrajectoryBuffer(output.target_pose);
        } else {
            // 旧模式：直接执行
            if (executePose(output.target_pose)) {
                ROS_DEBUG("[tracking_control] 成功执行目标位姿，质量评分: %.3f", output.quality_score);
            }
        }
    }
}

bool TrackingControlNode::getCurrentSensorPose(geometry_msgs::Pose &pose) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "world",  // 目标坐标系
            config_.sensor_frame,  // 源坐标系
            ros::Time(0),
            ros::Duration(0.1)
        );
        
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;
        
        return true;
    } catch (const tf2::TransformException &ex) {
        ROS_DEBUG_THROTTLE(1.0, "[tracking_control] TF查询失败: %s", ex.what());
        return false;
    }
}

bool TrackingControlNode::executePose(const geometry_msgs::Pose &target_pose) {
    if (!arm_service_client_.exists()) {
        return false;
    }
    
    mag_device_arm::SetEndEffectorPose srv;
    srv.request.arm = config_.sensor_arm_name;
    srv.request.target = target_pose;
    srv.request.velocity_scaling = config_.velocity_scaling;
    srv.request.acceleration_scaling = config_.acceleration_scaling;
    srv.request.execute = true;
    
    if (arm_service_client_.call(srv)) {
        if (srv.response.success) {
            return true;
        } else {
            ROS_WARN_THROTTLE(1.0, "[tracking_control] 机械臂执行失败: %s", srv.response.message.c_str());
            return false;
        }
    } else {
        ROS_WARN_THROTTLE(1.0, "[tracking_control] 调用机械臂服务失败");
        return false;
    }
}

void TrackingControlNode::addToTrajectoryBuffer(const geometry_msgs::Pose &target_pose) {
    std::lock_guard<std::mutex> lock(trajectory_buffer_mutex_);
    
    // 限制缓冲大小，避免内存无限增长
    if (trajectory_buffer_.size() >= config_.trajectory_buffer_size * 2) {
        ROS_WARN_THROTTLE(1.0, "[tracking_control] 轨迹缓冲已满，丢弃最旧的点");
        trajectory_buffer_.pop();
    }
    
    trajectory_buffer_.push(target_pose);
}

void TrackingControlNode::executeContinuousTrajectory() {
    if (!cartesian_path_client_.exists() || is_executing_trajectory_.load()) {
        return;
    }
    
    // 从缓冲中取出点
    std::vector<geometry_msgs::Pose> waypoints;
    {
        std::lock_guard<std::mutex> lock(trajectory_buffer_mutex_);
        
        if (trajectory_buffer_.size() < config_.trajectory_buffer_size) {
            // 缓冲中的点还不够，等待更多点
            return;
        }
        
        // 取出足够的点
        while (!trajectory_buffer_.empty() && waypoints.size() < config_.trajectory_buffer_size) {
            waypoints.push_back(trajectory_buffer_.front());
            trajectory_buffer_.pop();
        }
    }
    
    if (waypoints.empty()) {
        return;
    }
    
    // 调用笛卡尔路径服务
    is_executing_trajectory_.store(true);
    
    mag_device_arm::ExecuteCartesianPath srv;
    srv.request.arm = config_.sensor_arm_name;
    srv.request.waypoints = waypoints;
    srv.request.step_size = config_.cartesian_path_step_size;
    srv.request.jump_threshold = config_.cartesian_path_jump_threshold;
    srv.request.velocity_scaling = config_.velocity_scaling;
    srv.request.acceleration_scaling = config_.acceleration_scaling;
    srv.request.execute = true;
    
    if (cartesian_path_client_.call(srv)) {
        if (srv.response.success) {
            ROS_DEBUG("[tracking_control] 成功执行连续轨迹，包含 %zu 个点，完成度: %.2f%%", 
                     waypoints.size(), srv.response.fraction * 100.0);
        } else {
            ROS_WARN("[tracking_control] 笛卡尔路径执行失败: %s (完成度: %.2f%%)", 
                    srv.response.message.c_str(), srv.response.fraction * 100.0);
        }
    } else {
        ROS_WARN_THROTTLE(1.0, "[tracking_control] 调用笛卡尔路径服务失败");
    }
    
    is_executing_trajectory_.store(false);
}

void TrackingControlNode::trajectoryExecutionThread() {
    ros::Rate rate(10.0);  // 10 Hz 检查频率
    
    while (ros::ok() && !should_stop_thread_.load()) {
        if (config_.enable_execution && !is_executing_trajectory_.load()) {
            executeContinuousTrajectory();
        }
        rate.sleep();
    }
}

void TrackingControlNode::run() {
    ROS_INFO("[tracking_control] 节点已启动，开始控制循环");
    ros::spin();
    // 析构函数会处理线程清理
}

}  // namespace mag_tracking_control

// 主函数
int main(int argc, char **argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    ros::init(argc, argv, "tracking_control_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try {
        mag_tracking_control::TrackingControlNode node(nh, pnh);
        node.run();
    } catch (const std::exception &e) {
        ROS_ERROR("[tracking_control] 节点异常: %s", e.what());
        return 1;
    }
    
    return 0;
}
