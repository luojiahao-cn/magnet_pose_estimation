#include "mag_tracking_control/tracking_control_node.h"
#include "mag_tracking_control/tracking_control_config_loader.h"

#include <mag_core_utils/logger_utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/console.h>
#include <locale.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <algorithm>

namespace mag_tracking_control {

TrackingControlNode::TrackingControlNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_buffer_(ros::Duration(10.0))
    , tf_listener_(tf_buffer_)
    , has_magnet_pose_(false)
    , current_magnet_confidence_(0.0)
    , is_executing_trajectory_(false)
    , should_stop_thread_(false)
{
    // 设置日志级别
    setLogLevel();
    loadParameters();
    
    // 创建策略实例
    strategy_ = createStrategy(config_.strategy_type);
    if (!strategy_ || !strategy_->initialize()) {
        ROS_ERROR("[tracking_control] 策略初始化失败");
        throw std::runtime_error("策略初始化失败");
    }
    
    // 使用统一的日志格式化工具整合初始化信息
    namespace logger = mag_core_utils::logger;
    std::vector<std::pair<std::string, std::string>> init_items;
    init_items.emplace_back("使用策略", strategy_->name());
    
    // 订阅磁铁位姿
    if (!config_.magnet_pose_topic.empty()) {
        magnet_pose_sub_ = nh_.subscribe(
            config_.magnet_pose_topic,
            10,
            &TrackingControlNode::magnetPoseCallback,
            this
        );
        init_items.emplace_back("订阅磁铁位姿", config_.magnet_pose_topic);
    }
    
    // 发布目标位姿（用于可视化）
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        config_.target_pose_topic, 10
    );
    init_items.emplace_back("发布目标位姿话题", config_.target_pose_topic);
    
    // 初始化机械臂服务客户端
    arm_service_client_ = nh_.serviceClient<mag_device_arm::SetEndEffectorPose>(
        config_.arm_service_name
    );
    bool arm_service_available = arm_service_client_.waitForExistence(ros::Duration(5.0));
    if (arm_service_available) {
        init_items.emplace_back("机械臂服务", config_.arm_service_name + " (可用)");
    } else {
        // 服务暂时不可用，记录状态并继续等待（不阻塞）
        init_items.emplace_back("机械臂服务", config_.arm_service_name + " (等待中...)");
        ROS_INFO_STREAM("[tracking_control] 等待机械臂服务: " << config_.arm_service_name);
        // 在后台等待服务可用（不阻塞构造函数）
        std::thread([this, name = config_.arm_service_name]() {
            arm_service_client_.waitForExistence();
            ROS_INFO_STREAM("[tracking_control] 机械臂服务已可用: " << name);
        }).detach();
    }
    
    // 根据配置选择使用连续轨迹模式还是服务模式
    if (config_.use_continuous_trajectory) {
        // 初始化笛卡尔路径服务客户端
        cartesian_path_service_name_ = config_.cartesian_path_service_name;
        
        cartesian_path_client_ = nh_.serviceClient<mag_device_arm::ExecuteCartesianPath>(
            cartesian_path_service_name_
        );
        bool cartesian_service_available = cartesian_path_client_.waitForExistence(ros::Duration(5.0));
        if (cartesian_service_available) {
            init_items.emplace_back("笛卡尔路径服务", cartesian_path_service_name_ + " (可用)");
            init_items.emplace_back("连续轨迹模式", "已启用（通过服务接口）");
        } else {
            // 服务暂时不可用，记录状态并继续等待（不阻塞）
            init_items.emplace_back("笛卡尔路径服务", cartesian_path_service_name_ + " (等待中...)");
            ROS_INFO_STREAM("[tracking_control] 等待笛卡尔路径服务: " << cartesian_path_service_name_);
            // 在后台等待服务可用（不阻塞构造函数）
            std::thread([this, name = cartesian_path_service_name_]() {
                cartesian_path_client_.waitForExistence();
                ROS_INFO_STREAM("[tracking_control] 笛卡尔路径服务已可用: " << name);
            }).detach();
            init_items.emplace_back("连续轨迹模式", "已启用（通过服务接口）");
        }
        
        // 启动轨迹执行线程
        should_stop_thread_ = false;
        trajectory_execution_thread_ = std::thread(&TrackingControlNode::trajectoryExecutionThread, this);
        init_items.emplace_back("轨迹执行线程", "已启动");
    }
    
    // 启动控制循环
    if (config_.update_rate > 0) {
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / config_.update_rate),
            &TrackingControlNode::controlLoop,
            this
        );
        init_items.emplace_back("控制循环频率", logger::formatFrequency(config_.update_rate));
    }
    
    // 统一输出初始化信息（在所有服务检查完成后）
    ROS_INFO_STREAM("[tracking_control] " << logger::formatInit(init_items));
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

void TrackingControlNode::setLogLevel() {
    std::string log_level_str = "INFO";
    pnh_.param("logging_level", log_level_str, log_level_str);
    
    // 转换为大写
    std::transform(log_level_str.begin(), log_level_str.end(), log_level_str.begin(), ::toupper);
    
    ros::console::Level level = ros::console::levels::Info;
    if (log_level_str == "DEBUG") {
        level = ros::console::levels::Debug;
    } else if (log_level_str == "INFO") {
        level = ros::console::levels::Info;
    } else if (log_level_str == "WARN") {
        level = ros::console::levels::Warn;
    } else if (log_level_str == "ERROR") {
        level = ros::console::levels::Error;
    } else if (log_level_str == "FATAL") {
        level = ros::console::levels::Fatal;
    }
    
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, level);
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
    
    // 使用统一的日志格式化工具
    namespace logger = mag_core_utils::logger;
    std::vector<std::pair<std::string, std::string>> config_items;
    config_items.emplace_back("策略类型", config_.strategy_type);
    config_items.emplace_back("传感器机械臂", config_.sensor_arm_name);
    config_items.emplace_back("控制频率", logger::formatFrequency(config_.update_rate));
    
    if (config_.use_continuous_trajectory) {
        config_items.emplace_back("连续轨迹模式", logger::boolToString(true));
        config_items.emplace_back("轨迹缓冲大小", std::to_string(config_.trajectory_buffer_size));
    }
    
    ROS_INFO_STREAM("[tracking_control] " << logger::formatConfig(config_items));
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
    current_magnet_confidence_ = msg->confidence;
    last_magnet_pose_time_ = ros::Time::now();
    has_magnet_pose_ = true;
}

void TrackingControlNode::controlLoop(const ros::TimerEvent &/*event*/) {
    // 检查是否收到磁铁位姿
    if (!has_magnet_pose_) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[tracking_control] 尚未收到磁铁位姿");
        return;
    }
    
    // 检查磁铁位姿是否过期
    double time_since_last = (ros::Time::now() - last_magnet_pose_time_).toSec();
    if (time_since_last > 1.0) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[tracking_control] 磁铁位姿已过期 (" << time_since_last << " 秒)");
        return;
    }
    
    // 检查位姿估计置信度
    const double min_confidence_threshold = 0.3;  // 最小置信度阈值
    if (current_magnet_confidence_ < min_confidence_threshold) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[tracking_control] 位姿估计置信度过低 (" 
                                 << current_magnet_confidence_ << " < " << min_confidence_threshold << ")，暂停控制");
        return;
    }
    
    // 如果置信度较低但未低于阈值，发出警告但继续控制
    if (current_magnet_confidence_ < 0.5) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[tracking_control] 位姿估计置信度较低 (" 
                                << current_magnet_confidence_ << ")，控制精度可能受影响");
    }
    
    // 检查策略是否需要更新
    if (!strategy_->needsUpdate()) {
        return;
    }
    
    // 获取当前传感器位姿
    geometry_msgs::Pose current_sensor_pose;
    if (!getCurrentSensorPose(current_sensor_pose)) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[tracking_control] 无法获取当前传感器位姿");
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
        ROS_WARN_STREAM("[tracking_control] 策略计算失败: " << output.message);
        return;
    }
    
    if (!output.is_valid) {
        ROS_WARN_STREAM("[tracking_control] 目标位姿无效: " << output.message);
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
                ROS_DEBUG_STREAM("[tracking_control] 成功执行目标位姿，质量评分: " << output.quality_score);
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
        ROS_DEBUG_STREAM_THROTTLE(1.0, "[tracking_control] TF查询失败: " << ex.what());
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
            ROS_WARN_STREAM_THROTTLE(1.0, "[tracking_control] 机械臂执行失败: " << srv.response.message);
            return false;
        }
    } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[tracking_control] 调用机械臂服务失败");
        return false;
    }
}

void TrackingControlNode::addToTrajectoryBuffer(const geometry_msgs::Pose &target_pose) {
    std::lock_guard<std::mutex> lock(trajectory_buffer_mutex_);
    
    // 限制缓冲大小，避免内存无限增长
    if (trajectory_buffer_.size() >= config_.trajectory_buffer_size * 2) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[tracking_control] 轨迹缓冲已满，丢弃最旧的点");
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
            ROS_DEBUG_STREAM("[tracking_control] 成功执行连续轨迹，包含 " << waypoints.size() 
                            << " 个点，完成度: " << srv.response.fraction * 100.0 << "%");
        } else {
            ROS_WARN_STREAM("[tracking_control] 笛卡尔路径执行失败: " << srv.response.message 
                           << " (完成度: " << srv.response.fraction * 100.0 << "%)");
        }
    } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[tracking_control] 调用笛卡尔路径服务失败");
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
    ROS_INFO_STREAM("[tracking_control] 节点已启动，开始控制循环");
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
