/**
 * @file scan_controller_node.cpp
 * @brief 机械臂磁场扫描控制节点实现
 * 
 * 功能概述：
 * - 控制机械臂在三维网格点进行磁场扫描
 * - 采集多传感器磁场数据并求均值
 * - 将数据写入 CSV 文件
 * - 在 RViz 中可视化磁场向量
 * 
 * 主要流程：
 * 1. 加载配置参数和传感器阵列描述
 * 2. 生成三维网格扫描点并优化扫描顺序
 * 3. 通过 mag_device_arm 服务控制机械臂移动
 * 4. 在每个扫描点采集传感器数据
 * 5. 将数据写入 CSV 并发布可视化标记
 */

#include "mag_arm_scan/scan_controller_node.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <cstdio>
#include <sys/stat.h>

#include <filesystem>
#include <map>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mag_core_description/sensor_array_config_loader.hpp>
#include <mag_device_arm/SetEndEffectorPose.h>
#include <mag_device_arm/ExecuteNamedTarget.h>
#include <mag_device_arm/ExecuteCartesianPath.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {

constexpr double kEpsilon = 1e-9;  // 浮点数比较的极小值

}  // namespace

namespace mag_arm_scan {

/**
 * @brief 构造函数：初始化扫描控制节点
 * @param nh 全局节点句柄
 * @param pnh 私有节点句柄
 * 
 * 初始化流程：
 * 1. 加载参数配置
 * 2. 加载传感器阵列配置
 * 3. 生成扫描点网格
 * 4. 加载测试点（如果配置）
 * 5. 初始化 mag_device_arm 服务客户端
 * 6. 注册 ROS 服务和话题
 */
ScanControllerNode::ScanControllerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
		: nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
	// 加载基本参数
	loadParams();
	// 加载传感器阵列配置
	loadSensorConfig();
	// 生成三维网格扫描点
	generateScanPoints();
	// 加载测试点（可选）
	loadTestPoints();
	if (use_test_points_) {
		ROS_INFO_STREAM("[scan_controller] configured with " << scan_points_.size()
					<< " grid points plus " << test_points_.size() << " test points");
	}

	// 初始化 mag_device_arm 服务客户端
	pnh_.param("arm_name", arm_name_, std::string("arm1"));
	std::string arm_service_ns = "/mag_device_arm";
	arm_set_pose_client_ = nh_.serviceClient<mag_device_arm::SetEndEffectorPose>(arm_service_ns + "/set_end_effector_pose");
	arm_execute_named_client_ = nh_.serviceClient<mag_device_arm::ExecuteNamedTarget>(arm_service_ns + "/execute_named_target");
	arm_cartesian_path_client_ = nh_.serviceClient<mag_device_arm::ExecuteCartesianPath>(arm_service_ns + "/execute_cartesian_path");
	
	// 等待服务可用
	ROS_INFO("[scan_controller] waiting for mag_device_arm services...");
	if (!arm_set_pose_client_.waitForExistence(ros::Duration(10.0))) {
		ROS_WARN("[scan_controller] mag_device_arm services not available, will retry on use");
	}
	if (!arm_execute_named_client_.waitForExistence(ros::Duration(10.0))) {
		ROS_WARN("[scan_controller] mag_device_arm execute_named_target service not available");
	}
	if (!arm_cartesian_path_client_.waitForExistence(ros::Duration(10.0))) {
		ROS_WARN("[scan_controller] mag_device_arm execute_cartesian_path service not available");
	}

	start_scan_srv_ = pnh_.advertiseService("start_scan", &ScanControllerNode::startScan, this);
	mag_data_sub_ = nh_.subscribe(mag_topic_, 100, &ScanControllerNode::magDataCallback, this);
	joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ScanControllerNode::jointStateCallback, this);
	at_position_pub_ = nh_.advertise<std_msgs::Bool>("/scan_at_position", 1);
	scan_complete_pub_ = nh_.advertise<std_msgs::String>("/scan_complete", 1);
	if (visualization_enabled_) {
		marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 10);
	}

	if (!use_test_points_) {
		ROS_INFO_STREAM("[scan_controller] initialized with " << scan_points_.size()
					<< " scan points");
	}
}

/**
 * @brief 从参数服务器加载节点配置参数
 * 
 * 加载的参数包括：
 * - 基本参数：frame_id, yaw, pitch, autostart, mag_topic 等
 * - 扫描体积参数：volume_min, volume_max, step
 * - 采样参数：frames_per_sensor, max_sample_wait_time 等
 * - 可视化参数：箭头样式、颜色映射等
 */
void ScanControllerNode::loadParams() {
	// Lambda 函数：要求参数必须存在，否则抛出异常
	auto require = [&](const std::string &key, auto &var) {
		if (!pnh_.getParam(key, var)) throw std::runtime_error(std::string("缺少参数: ~") + key);
	};
	// Lambda 函数：要求三维向量参数必须存在且长度为 3
	auto requireVec3 = [&](const std::string &key, std::vector<double> &out) {
		if (!pnh_.getParam(key, out) || out.size() != 3U) throw std::runtime_error(std::string("缺少或非法参数: ~") + key + "[3]");
	};

	// 基本参数：参考坐标系、姿态、自动启动等
	require("frame_id", frame_id_);  // 数据统一到的世界坐标系
	require("yaw", yaw_);  // 扫描姿态的偏航角（弧度）
	require("pitch", pitch_);  // 扫描姿态的俯仰角（弧度）
	require("autostart", autostart_);  // 是否启动后自动开始扫描
	require("mag_topic", mag_topic_);  // 传感器数据话题
	require("wait_time", wait_time_);  // 移动到目标后的等待时间（秒）
	require("max_stable_wait_time", max_stable_wait_time_);  // 最大稳定等待时间（秒）
	require("output_base_dir", output_base_dir_);  // 输出数据的基础目录

	// 扫描体积参数：定义三维扫描区域
	requireVec3("volume_min", volume_min_);  // 扫描体积的最小边界 [x, y, z]（米）
	requireVec3("volume_max", volume_max_);  // 扫描体积的最大边界 [x, y, z]（米）
	requireVec3("step", step_);  // 扫描步长 [dx, dy, dz]（米）

	for (std::size_t i = 0; i < step_.size(); ++i) {
		if (step_[i] <= 0.0) {
			throw std::runtime_error("~step 必须为正数");
		}
	}

	// 解析输出目录路径并创建时间戳目录
	output_base_dir_ = resolveRelativePath(output_base_dir_);
	createTimestampDirectory();

	// 传感器检测参数：期望的传感器数量（例如 25），以及在自动检测时等待的最长时间
	pnh_.param("required_num_sensors", required_num_sensors_, 0);  // 0 表示不强制检测
	pnh_.param("sensor_detect_time", sensor_detect_time_, 5.0);  // 传感器检测超时时间（秒）

	// 采样参数：每个传感器在每个扫描点采集的帧数
	pnh_.param("frames_per_sensor", frames_per_sensor_, 10);
	if (frames_per_sensor_ < 1) {
		frames_per_sensor_ = 1;
	}
	// 达到采样要求的最大等待时间
	pnh_.param("max_sample_wait_time", max_sample_wait_time_, 5.0);
	if (max_sample_wait_time_ < 0.1) {
		max_sample_wait_time_ = 0.1;
	}
	// 每个传感器缓冲区的最大样本数（设置为 frames_per_sensor 的 3 倍）
	max_samples_per_sensor_ = static_cast<std::size_t>(frames_per_sensor_) * 3U;
	if (max_samples_per_sensor_ < static_cast<std::size_t>(frames_per_sensor_)) {
		max_samples_per_sensor_ = static_cast<std::size_t>(frames_per_sensor_);
	}
	
	// 机械臂运动参数：速度和加速度缩放因子
	pnh_.param("max_velocity_scaling", max_velocity_scaling_, 0.2);
	max_velocity_scaling_ = std::clamp(max_velocity_scaling_, 0.01, 1.0);
	pnh_.param("max_acceleration_scaling", max_acceleration_scaling_, 0.1);
	max_acceleration_scaling_ = std::clamp(max_acceleration_scaling_, 0.01, 1.0);
	pnh_.param("ready_target_name", ready_target_name_, std::string("ready"));  // 就绪姿态的名称
	pnh_.param("end_effector_link", end_effector_link_, std::string("bracket_tcp_link"));  // 末端执行器 link 名称
	pnh_.param("cartesian_path_threshold", cartesian_path_threshold_, 0.1);  // 距离阈值（米），小于此值使用笛卡尔路径
	if (cartesian_path_threshold_ < 0.0) {
		cartesian_path_threshold_ = 0.0;  // 确保非负
	}


	// 可视化参数：从 ~visualization 命名空间加载
	ros::NodeHandle viz_nh(pnh_, "visualization");
	viz_nh.param("enabled", visualization_enabled_, false);  // 是否启用可视化
	viz_nh.param("topic", visualization_topic_, std::string("mag_field_vectors"));  // 可视化话题名称
	viz_nh.param("arrow_length", arrow_length_, 0.05);  // 箭头长度缩放因子
	viz_nh.param("arrow_shaft_ratio", arrow_shaft_ratio_, 0.12);  // 箭头杆直径比例
	viz_nh.param("arrow_head_ratio", arrow_head_ratio_, 0.25);  // 箭头头部直径比例
	viz_nh.param("arrow_head_length_ratio", arrow_head_length_ratio_, 0.35);  // 箭头头部长度比例
	viz_nh.param("magnitude_min", magnitude_min_, 0.0);  // 磁场强度最小值（用于颜色映射）
	viz_nh.param("magnitude_max", magnitude_max_, 10.0);  // 磁场强度最大值（用于颜色映射）
	viz_nh.param("alpha", visualization_alpha_, 1.0);  // 可视化透明度
	viz_nh.param("use_tf_sensor_pose", use_tf_sensor_pose_, true);  // 是否使用 TF 查询传感器位置
	viz_nh.param("sensor_frame_prefix", sensor_frame_prefix_, std::string("sensor_"));  // 传感器坐标系前缀
	viz_nh.param("tf_lookup_timeout", tf_lookup_timeout_, 0.05);  // TF 查询超时时间（秒）
	if (tf_lookup_timeout_ < 0.0) {
		tf_lookup_timeout_ = 0.0;
	}
	// 参数范围检查和限制
	visualization_alpha_ = std::clamp(visualization_alpha_, 0.0, 1.0);
	arrow_length_ = std::max(arrow_length_, 1e-4);
	arrow_shaft_ratio_ = std::clamp(arrow_shaft_ratio_, 1e-3, 0.5);
	arrow_head_ratio_ = std::clamp(arrow_head_ratio_, arrow_shaft_ratio_ * 1.2, 0.8);
	arrow_head_length_ratio_ = std::clamp(arrow_head_length_ratio_, arrow_head_ratio_ * 0.6, 1.0);
	
	// Lambda 函数：加载颜色参数（RGB 或 RGBA）
	auto optColor = [&](const std::string &key, ScanControllerNode::ColorRGBA &out) {
		std::vector<double> v;
		if (viz_nh.getParam(key, v)) {
			if (v.size() == 3U) {
				out.r = v[0]; out.g = v[1]; out.b = v[2];
			} else if (v.size() == 4U) {
				out.r = v[0]; out.g = v[1]; out.b = v[2]; out.a = v[3];
			} else {
				throw std::runtime_error(std::string("非法参数: ~visualization/") + key + " [3|4]");
			}
		}
	};

	optColor("color_low", color_low_);
	optColor("color_high", color_high_);
	color_low_.a = visualization_alpha_;
	color_high_.a = visualization_alpha_;
}

/**
 * @brief 从参数服务器加载测试点配置
 * 
 * 测试点格式：数组，每个元素为 [x, y, z, roll, pitch, yaw]
 * 如果配置了测试点，将覆盖默认的网格扫描顺序
 */
void ScanControllerNode::loadTestPoints() {
	XmlRpc::XmlRpcValue xml_points;
	if (!pnh_.getParam("test_points", xml_points)) {
		// 未配置测试点，使用网格扫描
		use_test_points_ = false;
		test_points_.clear();
		return;
	}

	if (xml_points.getType() != XmlRpc::XmlRpcValue::TypeArray || xml_points.size() == 0) {
		ROS_WARN("[scan_controller] ~test_points must be an array of poses");
		use_test_points_ = false;
		test_points_.clear();
		return;
	}

	auto getNumeric = [](XmlRpc::XmlRpcValue &value, double &out) -> bool {
		switch (value.getType()) {
			case XmlRpc::XmlRpcValue::TypeDouble:
				out = static_cast<double>(value);
				return true;
			case XmlRpc::XmlRpcValue::TypeInt:
				out = static_cast<int>(value);
				return true;
			default:
				return false;
		}
	};

	auto appendPose = [this](double x, double y, double z, double roll, double pitch, double yaw) {
		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		pose.orientation = tf2::toMsg(q);
		test_points_.push_back(pose);
	};

	bool parsed = false;
	if (xml_points[0].getType() == XmlRpc::XmlRpcValue::TypeArray) {
		test_points_.clear();
		test_points_.reserve(xml_points.size());
		for (int i = 0; i < xml_points.size(); ++i) {
			XmlRpc::XmlRpcValue &entry = xml_points[i];
			if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() != 6) {
				ROS_WARN("[scan_controller] each ~test_points entry must be an array with 6 elements (x y z roll pitch yaw)");
				test_points_.clear();
				use_test_points_ = false;
				return;
			}
			double values[6];
			for (int j = 0; j < 6; ++j) {
				if (!getNumeric(entry[j], values[j])) {
					ROS_WARN("[scan_controller] ~test_points values must be numeric");
					test_points_.clear();
					use_test_points_ = false;
					return;
				}
			}
			appendPose(values[0], values[1], values[2], values[3], values[4], values[5]);
		}
		parsed = true;
	} else {
		std::vector<double> flat;
		flat.reserve(static_cast<std::size_t>(xml_points.size()));
		for (int i = 0; i < xml_points.size(); ++i) {
			double value = 0.0;
			if (!getNumeric(xml_points[i], value)) {
				ROS_WARN("[scan_controller] ~test_points values must be numeric");
				test_points_.clear();
				use_test_points_ = false;
				return;
			}
			flat.push_back(value);
		}

		if (flat.size() % 6 != 0 || flat.empty()) {
			ROS_WARN("[scan_controller] ~test_points must contain multiples of 6 values (x y z roll pitch yaw)");
			test_points_.clear();
			use_test_points_ = false;
			return;
		}

		const std::size_t count = flat.size() / 6;
		test_points_.clear();
		test_points_.reserve(count);
		for (std::size_t i = 0; i < count; ++i) {
			appendPose(flat[i * 6 + 0], flat[i * 6 + 1], flat[i * 6 + 2], flat[i * 6 + 3], flat[i * 6 + 4], flat[i * 6 + 5]);
		}
		parsed = true;
	}

	use_test_points_ = parsed && !test_points_.empty();
	if (use_test_points_) {
		ROS_INFO_STREAM("[scan_controller] loaded " << test_points_.size() << " test points from configuration");
	}
}

/**
 * @brief 根据配置的体积和步长生成三维网格扫描点
 * 
 * 生成规则：
 * - 在 volume_min 到 volume_max 的范围内
 * - 按照 step 指定的步长生成网格点
 * - 所有点的姿态使用配置的 yaw 和 pitch（roll 固定为 0）
 * - 生成后优化扫描顺序以减少机械臂运动距离
 */
void ScanControllerNode::generateScanPoints() {
	scan_points_.clear();

	// 设置扫描姿态：roll=0，使用配置的 pitch 和 yaw
	const double roll = 0.0;
	tf2::Quaternion q;
	q.setRPY(roll, pitch_, yaw_);
	const geometry_msgs::Quaternion orientation = tf2::toMsg(q);

	// 生成三维网格点：遍历 x, y, z 三个维度
	for (double x = volume_min_[0]; x <= volume_max_[0] + kEpsilon; x += step_[0]) {
		for (double y = volume_min_[1]; y <= volume_max_[1] + kEpsilon; y += step_[1]) {
			for (double z = volume_min_[2]; z <= volume_max_[2] + kEpsilon; z += step_[2]) {
				geometry_msgs::Pose pose;
				pose.position.x = x;
				pose.position.y = y;
				pose.position.z = z;
				pose.orientation = orientation;
				scan_points_.push_back(pose);
			}
		}
	}

	// 优化扫描顺序：使用最近邻贪心算法减少总运动距离
	optimizeScanOrder();
}

/**
 * @brief 优化扫描点顺序，使用最近邻贪心算法
 * 
 * 算法思路：
 * 1. 从第一个点开始
 * 2. 每次选择距离当前点最近的未访问点
 * 3. 重复直到所有点都被访问
 * 
 * 这样可以减少机械臂的总运动距离，提高扫描效率
 */
void ScanControllerNode::optimizeScanOrder() {
	if (scan_points_.empty()) {
		return;
	}

	std::vector<geometry_msgs::Pose> optimized;
	optimized.reserve(scan_points_.size());
	std::vector<bool> visited(scan_points_.size(), false);

	// 从第一个点开始
	optimized.push_back(scan_points_.front());
	visited[0] = true;

	// 贪心选择：每次选择距离当前点最近的未访问点
	for (std::size_t i = 1; i < scan_points_.size(); ++i) {
		std::size_t best_idx = 0U;
		double min_dist = std::numeric_limits<double>::max();

		// 遍历所有未访问的点，找到距离最近的
		for (std::size_t j = 0; j < scan_points_.size(); ++j) {
			if (visited[j]) {
				continue;
			}
			const double dist = poseDistance(optimized.back(), scan_points_[j]);
			if (dist < min_dist) {
				min_dist = dist;
				best_idx = j;
			}
		}

		// 将最近的点加入优化序列
		optimized.push_back(scan_points_[best_idx]);
		visited[best_idx] = true;
	}

	// 用优化后的顺序替换原始顺序
	scan_points_.swap(optimized);
	ROS_INFO_STREAM("[scan_controller] optimized scan order for " << scan_points_.size()
									<< " points");
}

/**
 * @brief 计算两个位姿之间的欧氏距离（仅考虑位置，不考虑姿态）
 * @param p1 第一个位姿
 * @param p2 第二个位姿
 * @return 位置之间的欧氏距离（米）
 */
double ScanControllerNode::poseDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
	const double dx = p1.position.x - p2.position.x;
	const double dy = p1.position.y - p2.position.y;
	const double dz = p1.position.z - p2.position.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 使用 mag_device_arm 服务控制机械臂移动到指定位姿
 * @param pose 目标位姿（在 frame_id 坐标系下）
 * @return 成功返回 true，失败返回 false
 * 
 * 智能选择规划方式：
 * - 如果当前位置和目标位置距离很近（小于 cartesian_path_threshold_），使用笛卡尔路径规划
 * - 否则使用关节空间规划（set_end_effector_pose）
 * 
 * 这样可以避免近距离移动时出现大角度旋转的问题
 */
bool ScanControllerNode::moveToPose(const geometry_msgs::Pose &pose) {
	// 尝试获取当前末端执行器位姿，用于判断是否使用笛卡尔路径
	geometry_msgs::Pose current_pose;
	bool has_current_pose = false;
	
	if (!end_effector_link_.empty() && !frame_id_.empty() && cartesian_path_threshold_ > 0.0) {
		try {
			// 查询当前末端执行器位姿
			geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
				frame_id_, end_effector_link_, ros::Time(0), ros::Duration(0.5));
			current_pose.position.x = transform.transform.translation.x;
			current_pose.position.y = transform.transform.translation.y;
			current_pose.position.z = transform.transform.translation.z;
			current_pose.orientation = transform.transform.rotation;
			has_current_pose = true;
		} catch (const tf2::TransformException &ex) {
			// TF 查询失败，使用默认的关节空间规划
			ROS_DEBUG("[scan_controller] Failed to get current pose via TF: %s, using joint space planning", ex.what());
		}
	}
	
	// 如果获取到当前位姿，计算距离并决定使用哪种规划方式
	if (has_current_pose) {
		const double distance = poseDistance(current_pose, pose);
		if (distance < cartesian_path_threshold_ && arm_cartesian_path_client_.exists()) {
			// 使用笛卡尔路径规划（直线移动，避免大角度旋转）
			ROS_DEBUG("[scan_controller] Distance %.4f m < threshold %.4f m, using Cartesian path planning", 
			          distance, cartesian_path_threshold_);
			
			mag_device_arm::ExecuteCartesianPath cartesian_srv;
			cartesian_srv.request.arm = arm_name_;
			// 笛卡尔路径需要从当前位置到目标位置的路径点
			// computeCartesianPath 会自动从当前状态开始，所以只需要目标点
			cartesian_srv.request.waypoints.push_back(pose);
			cartesian_srv.request.step_size = 0.01;  // 默认步长（米）
			cartesian_srv.request.jump_threshold = 0.0;  // 禁用跳跃检测
			cartesian_srv.request.velocity_scaling = max_velocity_scaling_;
			cartesian_srv.request.acceleration_scaling = max_acceleration_scaling_;
			cartesian_srv.request.execute = true;
			
			if (!arm_cartesian_path_client_.call(cartesian_srv)) {
				ROS_WARN("[scan_controller] Cartesian path planning service call failed, falling back to joint space planning");
				// 笛卡尔路径失败，回退到关节空间规划
			} else if (cartesian_srv.response.success) {
				ROS_DEBUG("[scan_controller] Cartesian path planning succeeded (fraction: %.2f%%)", 
				         cartesian_srv.response.fraction * 100.0);
				return true;  // 笛卡尔路径成功
			} else {
				ROS_WARN("[scan_controller] Cartesian path planning failed: %s (fraction: %.2f%%), falling back to joint space planning", 
				         cartesian_srv.response.message.c_str(), cartesian_srv.response.fraction * 100.0);
				// 笛卡尔路径失败，回退到关节空间规划
			}
		}
	}
	
	// 使用关节空间规划（默认方式）
	if (!arm_set_pose_client_.exists()) {
		ROS_ERROR("[scan_controller] mag_device_arm service not available");
		return false;
	}

	// 构造服务请求
	mag_device_arm::SetEndEffectorPose srv;
	srv.request.arm = arm_name_;  // 机械臂名称
	srv.request.target = pose;  // 目标位姿
	srv.request.velocity_scaling = max_velocity_scaling_;  // 速度缩放因子
	srv.request.acceleration_scaling = max_acceleration_scaling_;  // 加速度缩放因子
	srv.request.execute = true;  // 执行运动

	// 调用服务
	if (!arm_set_pose_client_.call(srv)) {
		ROS_ERROR("[scan_controller] failed to call set_end_effector_pose service");
		return false;
	}

	// 检查执行结果
	if (!srv.response.success) {
		ROS_WARN("[scan_controller] set_end_effector_pose failed: %s", srv.response.message.c_str());
		return false;
	}

	return true;
}

/**
 * @brief 使用 mag_device_arm 服务控制机械臂移动到命名的就绪姿态
 * @return 成功返回 true，失败返回 false
 * 
 * 就绪姿态通常是一个安全的中间位置，用于扫描开始前和结束后的位置
 */
bool ScanControllerNode::moveToReadyPose() {
	ROS_INFO("[scan_controller] moving to ready position using named target");

	if (!arm_execute_named_client_.exists()) {
		ROS_ERROR("[scan_controller] mag_device_arm execute_named_target service not available");
		return false;
	}

	// 构造服务请求
	mag_device_arm::ExecuteNamedTarget srv;
	srv.request.arm = arm_name_;  // 机械臂名称
	srv.request.target = ready_target_name_;  // 命名目标（如 "ready"）
	srv.request.velocity_scaling = max_velocity_scaling_;  // 速度缩放因子
	srv.request.acceleration_scaling = max_acceleration_scaling_;  // 加速度缩放因子
	srv.request.execute = true;  // 执行运动

	// 调用服务
	if (!arm_execute_named_client_.call(srv)) {
		ROS_ERROR("[scan_controller] failed to call execute_named_target service");
		return false;
	}

	// 检查执行结果
	if (!srv.response.success) {
		ROS_WARN("[scan_controller] execute_named_target failed: %s", srv.response.message.c_str());
		return false;
	}

	return true;
}


/**
 * @brief 在单个扫描点采集数据
 * @param pose 目标扫描点的位姿
 * 
 * 流程：
 * 1. 发布"未到位"信号
 * 2. 控制机械臂移动到目标位姿
 * 3. 等待机械臂稳定（wait_time_）
 * 4. 发布"已到位"信号
 * 5. 清空采样缓冲区
 * 6. 采集传感器数据（等待足够样本或超时）
 * 7. 将数据写入 CSV 文件
 * 8. 发布可视化标记（如果启用）
 */
void ScanControllerNode::collectDataAtPoint(const geometry_msgs::Pose &pose) {
	// 发布"未到位"信号
	std_msgs::Bool at_pos_msg;
	at_pos_msg.data = false;
	at_position_pub_.publish(at_pos_msg);

	ROS_INFO_STREAM("[scan_controller] moving to position: x=" << pose.position.x
									<< ", y=" << pose.position.y << ", z=" << pose.position.z);

	// 控制机械臂移动到目标位姿
	if (!moveToPose(pose)) {
		ROS_ERROR("[scan_controller] failed to move to pose");
		return;
	}

	// 等待机械臂稳定
	ros::Duration(wait_time_).sleep();

	// 发布"已到位"信号
	at_pos_msg.data = true;
	at_position_pub_.publish(at_pos_msg);

	// 清空之前的采样缓冲区，开始新一轮采样
	resetSampleBuffers();

	// 采集传感器数据：等待足够样本或超时
	collected_samples_.clear();
	const ros::Duration max_wait(max_sample_wait_time_);  // 最大等待时间
	const ros::Duration check_interval(0.05);  // 检查间隔（50ms）
	const ros::Time sample_start = ros::Time::now();
	bool full_requirement_met = false;  // 是否满足完整采样要求
	std::size_t captured_sensor_count = 0U;

	// 循环等待，直到满足采样要求或超时
	while ((ros::Time::now() - sample_start) < max_wait) {
		{
			std::lock_guard<std::mutex> lock(data_mutex_);
			captured_sensor_count = sensor_samples_buffer_.size();
			// 检查是否已收集足够的样本
			if (hasEnoughSamplesLocked()) {
				extractSamplesLocked(collected_samples_, false);  // 提取样本（严格模式）
				full_requirement_met = true;
				break;
			}
		}
		check_interval.sleep();  // 短暂等待后再次检查
	}

	// 如果超时仍未满足要求，使用 best-effort 模式提取已有样本
	if (!full_requirement_met) {
		std::lock_guard<std::mutex> lock(data_mutex_);
		captured_sensor_count = sensor_samples_buffer_.size();
		extractSamplesLocked(collected_samples_, true);  // 提取样本（best-effort 模式）
	}

	if (collected_samples_.empty()) {
		ROS_WARN("[scan_controller] no magnetometer data collected at this point");
		return;
	}

	if (!full_requirement_met) {
		if (num_sensors_ > 0) {
			ROS_WARN("[scan_controller] collected partial dataset: %zu sensors captured, expected %d",
			         captured_sensor_count, num_sensors_);
		} else {
			ROS_WARN("[scan_controller] collected partial dataset (sensor count unknown)");
		}
	}

	const std::size_t sample_count = collected_samples_.size();
	if (full_requirement_met) {
		ROS_INFO("[scan_controller] collected %zu samples across %zu sensors", sample_count,
		         expected_sensor_ids_.empty() ? sensor_samples_buffer_.size() : expected_sensor_ids_.size());
	} else {
		ROS_INFO("[scan_controller] proceeding with %zu sampled frames", sample_count);
	}

	std::ofstream file(output_file_, std::ios::app);
	if (!file.is_open()) {
		ROS_ERROR_STREAM("[scan_controller] failed to open output file: " << output_file_);
		return;
	}

	// 用于计算每个传感器磁场向量的平均值
	struct AggregatedSensorAccumulator {
		double sum_x{0.0};  // X 分量累加
		double sum_y{0.0};  // Y 分量累加
		double sum_z{0.0};  // Z 分量累加
		std::size_t count{0U};  // 样本数量
		geometry_msgs::Point fallback_position{};  // 备用位置（用于可视化）
	};
	std::map<std::uint32_t, AggregatedSensorAccumulator> averaged_data;

	// 遍历所有采集的样本，写入 CSV 并累加用于可视化
	for (const auto &data : collected_samples_) {
		// 查找传感器配置，获取传感器在阵列中的位姿
		const mag_core_description::SensorEntry *sensor_entry = sensor_array_loaded_ ? sensor_array_.findSensor(static_cast<int>(data.sensor_id)) : nullptr;
		if (!sensor_entry)
		{
			ROS_WARN_THROTTLE(5.0, "Unknown sensor ID: %d", data.sensor_id);
			continue;
		}
		// 获取传感器在阵列坐标系中的位姿
		geometry_msgs::Pose pose_w = sensor_entry->pose;
		geometry_msgs::TransformStamped transform;
		bool did_tf = false;

		// 如果需要，将传感器位姿转换到目标坐标系（frame_id_）
		if (!frame_id_.empty() && frame_id_ != data.header.frame_id) {
			try {
				// 查询 TF 变换：从传感器坐标系到目标坐标系
				transform = tf_buffer_.lookupTransform(frame_id_, data.header.frame_id, ros::Time(0), ros::Duration(tf_lookup_timeout_));
				tf2::doTransform(pose_w, pose_w, transform);
				did_tf = true;
			} catch (const std::exception &e) {
				ROS_WARN_THROTTLE(5.0, "TF to '%s' failed: %s; using source frame.", frame_id_.c_str(), e.what());
				did_tf = false;
			}
		}

		// 获取磁场向量（在传感器坐标系中）
		double bx = data.mag_x;
		double by = data.mag_y;
		double bz = data.mag_z;

		// 如果需要，将磁场向量也转换到目标坐标系
		if (did_tf) {
			try {
				geometry_msgs::Vector3Stamped vin;
				geometry_msgs::Vector3Stamped vout;
				vin.header = data.header;
				vin.vector.x = bx;
				vin.vector.y = by;
				vin.vector.z = bz;
				tf2::doTransform(vin, vout, transform);
				bx = vout.vector.x;
				by = vout.vector.y;
				bz = vout.vector.z;
			} catch (const std::exception &e) {
				ROS_WARN_THROTTLE(5.0, "Failed to transform magnetic field vector: %s", e.what());
			}
		}

		// 提取传感器位姿信息用于记录
		const geometry_msgs::Point record_point = pose_w.position;
		const geometry_msgs::Quaternion record_ori = pose_w.orientation;
		
		// 写入 CSV 文件：时间戳、磁场向量、传感器位姿、传感器 ID、坐标系
		// 格式：timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,sensor_id,frame_id
		file << data.header.stamp.toSec() << ',' << bx << ',' << by << ',' << bz << ',' << record_point.x << ','
		     << record_point.y << ',' << record_point.z << ',' << record_ori.x << ',' << record_ori.y << ','
		     << record_ori.z << ',' << record_ori.w << ',' << data.sensor_id << ',' << data.header.frame_id << '\n';

		// 累加磁场向量用于计算平均值（用于可视化）
		auto &acc = averaged_data[data.sensor_id];
		if (acc.count == 0U) {
			acc.fallback_position = record_point;  // 保存第一个位置作为备用
		}
		acc.sum_x += bx;
		acc.sum_y += by;
		acc.sum_z += bz;
		acc.count += 1U;
	}
	// 确保数据已写入磁盘，降低崩溃丢失风险
	file.flush();
	file.close();

	// 如果启用可视化，发布磁场向量箭头标记
	if (visualization_enabled_ && marker_pub_) {
		visualization_msgs::MarkerArray markers;
		markers.markers.reserve(averaged_data.size());
		const ros::Time now = ros::Time::now();
		
		// 为每个传感器创建可视化箭头
		for (const auto &entry : averaged_data) {
			const std::uint32_t sensor_id = entry.first;
			const AggregatedSensorAccumulator &acc = entry.second;
			if (acc.count == 0U) {
				continue;
			}
			
			// 获取传感器位置（优先使用 TF，否则使用备用位置）
			geometry_msgs::Point start_point;
			bool has_tf_pose = lookupSensorPosition(sensor_id, start_point);
			if (!has_tf_pose) {
				start_point = acc.fallback_position;
			}

			if (frame_id_.empty()) {
				ROS_WARN_THROTTLE(5.0, "[scan_controller] Missing frame_id for visualization");
				continue;
			}

			// 计算平均磁场向量
			geometry_msgs::Vector3 avg_vector;
			avg_vector.x = acc.sum_x / static_cast<double>(acc.count);
			avg_vector.y = acc.sum_y / static_cast<double>(acc.count);
			avg_vector.z = acc.sum_z / static_cast<double>(acc.count);
			double magnitude = std::sqrt(avg_vector.x * avg_vector.x + avg_vector.y * avg_vector.y + avg_vector.z * avg_vector.z);
			if (magnitude < kEpsilon) {
				continue;  // 磁场强度太小，跳过可视化
			}

			// 计算箭头终点位置
			const double arrow_length_current = std::max(magnitude * arrow_length_, 1e-6);
			geometry_msgs::Point end_point = start_point;
			end_point.x += avg_vector.x * arrow_length_;
			end_point.y += avg_vector.y * arrow_length_;
			end_point.z += avg_vector.z * arrow_length_;

			// 创建箭头标记
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id_;  // 参考坐标系
			marker.header.stamp = now;
			marker.ns = "mag_field";  // 命名空间
			marker.id = static_cast<int>(marker_id_counter_++);  // 唯一 ID
			marker.type = visualization_msgs::Marker::ARROW;  // 箭头类型
			marker.action = visualization_msgs::Marker::ADD;  // 添加标记
			marker.points.push_back(start_point);  // 箭头起点
			marker.points.push_back(end_point);  // 箭头终点
			
			// 计算箭头尺寸：杆直径、头部直径、头部长度
			const double shaft_diameter = std::max(arrow_shaft_ratio_ * arrow_length_current, 1e-4);
			const double head_diameter = std::max(arrow_head_ratio_ * arrow_length_current, shaft_diameter * 1.2);
			const double head_length = std::max(arrow_head_length_ratio_ * arrow_length_current, head_diameter * 0.8);
			marker.scale.x = shaft_diameter;
			marker.scale.y = head_diameter;
			marker.scale.z = head_length;
			marker.pose.orientation.w = 1.0;
			
			// 根据磁场强度插值颜色（从 color_low 到 color_high）
			ColorRGBA color = interpolateColor(magnitude);
			marker.color.r = static_cast<float>(std::clamp(color.r, 0.0, 1.0));
			marker.color.g = static_cast<float>(std::clamp(color.g, 0.0, 1.0));
			marker.color.b = static_cast<float>(std::clamp(color.b, 0.0, 1.0));
			marker.color.a = static_cast<float>(std::clamp(color.a, 0.0, 1.0));
			marker.lifetime = ros::Duration(0.0);  // 永久显示（0 表示不自动删除）
			markers.markers.push_back(marker);
		}

		if (!markers.markers.empty()) {
			marker_pub_.publish(markers);
		}
	}
}

/**
 * @brief 开始扫描服务回调函数
 * @param req 服务请求（未使用）
 * @param res 服务响应
 * @return 总是返回 true
 * 
 * 扫描流程：
 * 1. 自动检测传感器数量（如果配置了 required_num_sensors_）
 * 2. 移动到就绪姿态
 * 3. 清空场地图聚合器（如果服务存在）
 * 4. 创建输出 CSV 文件并写入表头
 * 5. 依次遍历所有扫描点进行数据采集
 * 6. 如果配置了测试点，也进行采集
 * 7. 返回就绪姿态
 * 8. 完成扫描并发布完成消息
 */
bool ScanControllerNode::startScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	(void)req;

	ROS_INFO_STREAM("[scan_controller] starting scan with " << scan_points_.size() << " grid points");

	// 如果没有从配置中得到传感器数量，且用户指定了 required_num_sensors_，则在开始前尝试自动检测
	// 通过监听传感器数据话题，统计出现的传感器 ID
	if (num_sensors_ <= 0 && required_num_sensors_ > 0) {
		ROS_INFO_STREAM("[scan_controller] attempting to detect required_num_sensors=" << required_num_sensors_ << " (timeout=" << sensor_detect_time_ << "s)");
		const ros::Time detect_start = ros::Time::now();
		const ros::Duration timeout(sensor_detect_time_);
		while (ros::ok() && (ros::Time::now() - detect_start) < timeout) {
			{
				std::lock_guard<std::mutex> lock(data_mutex_);
				if (static_cast<int>(sensor_samples_buffer_.size()) >= required_num_sensors_) {
					break;
				}
			}
			ros::Duration(0.1).sleep();
		}
		// 将当前缓冲中出现过的 sensor ids 记录为期望列表（有可能小于 required_num_sensors_）
		{
			std::lock_guard<std::mutex> lock(data_mutex_);
			expected_sensor_ids_.clear();
			for (const auto &entry : sensor_samples_buffer_) {
				expected_sensor_ids_.push_back(static_cast<int>(entry.first));
			}
			std::sort(expected_sensor_ids_.begin(), expected_sensor_ids_.end());
			expected_sensor_ids_.erase(std::unique(expected_sensor_ids_.begin(), expected_sensor_ids_.end()), expected_sensor_ids_.end());
			num_sensors_ = static_cast<int>(expected_sensor_ids_.size());
		}
		if (num_sensors_ < required_num_sensors_) {
			ROS_WARN_STREAM("[scan_controller] detected " << num_sensors_ << " sensors, less than required " << required_num_sensors_);
		} else {
			ROS_INFO_STREAM("[scan_controller] detected required sensors: " << num_sensors_);
		}
	}

	if (!moveToReadyPose()) {
		ROS_ERROR("[scan_controller] failed to move to ready position");
		res.success = false;
		res.message = "Failed to move to ready position";
		return true;
	}

	// Optional: clear field map aggregator if service exists
	ros::ServiceClient clear_client = nh_.serviceClient<std_srvs::Trigger>("/field_map_aggregator/clear_map");
	if (clear_client.exists()) {
		std_srvs::Trigger clear_req;
		if (clear_client.call(clear_req)) {
			ROS_INFO("[scan_controller] cleared field map aggregator");
		}
	}

	std::ofstream file(output_file_, std::ios::trunc);
	if (!file.is_open()) {
		res.success = false;
		res.message = "Failed to open output file";
		return true;
	}
	// 增加 sensor_id 与 frame_id 字段以及传感器姿态四元数，方便后处理区分来源与可视化
	file << "timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w,sensor_id,frame_id\n";
	file.close();

	std_msgs::Bool at_pos_msg;
	at_pos_msg.data = false;
	at_position_pub_.publish(at_pos_msg);

	for (const auto &pose : scan_points_) {
		collectDataAtPoint(pose);
	}

	if (use_test_points_) {
		ROS_INFO_STREAM("[scan_controller] starting test point sequence with "
			<< test_points_.size() << " poses");
		for (const auto &pose : test_points_) {
			collectDataAtPoint(pose);
		}
	}

	at_pos_msg.data = false;
	at_position_pub_.publish(at_pos_msg);

	if (!moveToReadyPose()) {
		ROS_WARN("[scan_controller] failed to return to ready pose after scan");
	}

	finalizeScan();

	res.success = true;
	res.message = "Scan completed";
	return true;
}

// Stability check removed: rely on MoveIt execution completion and optional wait_time

/**
 * @brief 传感器数据回调函数
 * @param msg 传感器数据消息
 * 
 * 功能：
 * - 接收来自传感器话题的磁场数据
 * - 按传感器 ID 分别存储到缓冲区
 * - 每个传感器缓冲区最多保存 max_samples_per_sensor_ 个样本（FIFO）
 */
void ScanControllerNode::magDataCallback(const mag_core_msgs::MagSensorData::ConstPtr &msg) {
	std::lock_guard<std::mutex> lock(data_mutex_);
	// 根据传感器 ID 获取对应的缓冲区
	auto &queue = sensor_samples_buffer_[msg->sensor_id];
	queue.push_back(*msg);
	// 如果缓冲区超过最大容量，移除最旧的样本（FIFO）
	while (queue.size() > max_samples_per_sensor_) {
		queue.pop_front();
	}
}

/**
 * @brief 关节状态回调函数
 * @param msg 关节状态消息
 * 
 * 功能：保存最新的关节状态（用于后续可能的稳定性检查）
 */
void ScanControllerNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	latest_joint_state_ = *msg;
}

/**
 * @brief 从参数服务器加载传感器阵列配置
 * 
 * 功能：
 * - 从 ~config 参数加载传感器阵列描述
 * - 提取所有传感器的 ID 列表
 * - 如果加载失败，尝试使用备用参数或进入 best-effort 模式
 * 
 * 配置路径：~config（包含 frames 和 sensors）
 */
void ScanControllerNode::loadSensorConfig() {
	expected_sensor_ids_.clear();
	num_sensors_ = 0;
	sensor_array_loaded_ = false;
	try
	{
		XmlRpc::XmlRpcValue array_config;
		// 从 ~config 读取传感器阵列配置
		if (pnh_.getParam("config", array_config)) {
			// 解析配置并加载到 SensorArrayDescription
			mag_core_description::SensorArrayConfig config = 
				mag_core_description::loadSensorArrayConfig(array_config, "config");
			sensor_array_.load(config);
			sensor_array_loaded_ = true;
			
			// 提取所有传感器的 ID
			const auto &sensors = sensor_array_.sensors();
			expected_sensor_ids_.reserve(sensors.size());
			for (const auto &entry : sensors)
			{
				expected_sensor_ids_.push_back(entry.id);
			}
			// 排序并去重
			std::sort(expected_sensor_ids_.begin(), expected_sensor_ids_.end());
			expected_sensor_ids_.erase(std::unique(expected_sensor_ids_.begin(), expected_sensor_ids_.end()),
				                     expected_sensor_ids_.end());
			num_sensors_ = static_cast<int>(expected_sensor_ids_.size());
		}
	}
	catch (const std::exception &e)
	{
		ROS_WARN("[scan_controller] 读取 array 配置失败: %s", e.what());
		// 尝试使用备用参数
		int fallback_count = 0;
		if (pnh_.getParam("array/expected_sensor_count", fallback_count) && fallback_count > 0)
		{
			num_sensors_ = fallback_count;
		}
	}

	// 如果无法确定传感器数量，将使用 best-effort 模式
	if (num_sensors_ <= 0) {
		ROS_WARN("[scan_controller] sensor count unavailable, sampling will be best-effort");
	} else {
		ROS_INFO("[scan_controller] expecting %d sensors with %d frames each", num_sensors_,
		         frames_per_sensor_);
	}
}

/**
 * @brief 检查是否已收集足够的样本（需在持锁条件下调用）
 * @return 如果所有期望的传感器都已收集到 frames_per_sensor_ 个样本，返回 true
 * 
 * 检查逻辑：
 * - 如果配置了期望传感器 ID 列表，检查每个传感器是否都有足够样本
 * - 如果未配置 ID 列表，检查缓冲区中的传感器数量是否达到 num_sensors_
 */
bool ScanControllerNode::hasEnoughSamplesLocked() const {
	if (num_sensors_ <= 0) {
		return false;  // 传感器数量未知，无法判断
	}

	// 如果配置了期望传感器 ID 列表，检查每个传感器
	if (!expected_sensor_ids_.empty()) {
		for (int sensor_id : expected_sensor_ids_) {
			auto it = sensor_samples_buffer_.find(static_cast<std::uint32_t>(sensor_id));
			if (it == sensor_samples_buffer_.end()) {
				return false;  // 该传感器还没有数据
			}
			if (static_cast<int>(it->second.size()) < frames_per_sensor_) {
				return false;  // 该传感器样本数不足
			}
		}
		return true;
	}

	// 如果未配置期望传感器 ID 列表，检查缓冲区中的传感器数量和每个传感器的样本数
	if (static_cast<int>(sensor_samples_buffer_.size()) < num_sensors_) {
		return false;  // 传感器数量不足
	}
	for (const auto &entry : sensor_samples_buffer_) {
		if (static_cast<int>(entry.second.size()) < frames_per_sensor_) {
			return false;  // 某个传感器样本数不足
		}
	}
	return true;
}

/**
 * @brief 从各传感器缓冲区提取样本（需在持锁条件下调用）
 * @param out 输出向量，存储提取的样本
 * @param best_effort 如果为 true，尽可能提取所有可用样本；如果为 false，只提取 frames_per_sensor_ 个
 * 
 * 提取策略：
 * - 如果 best_effort=false：每个传感器提取 frames_per_sensor_ 个最新样本
 * - 如果 best_effort=true：每个传感器提取所有可用样本
 * - 优先提取最新的样本（从队列末尾开始）
 */
void ScanControllerNode::extractSamplesLocked(std::vector<mag_core_msgs::MagSensorData> &out,
																 bool best_effort) {
	out.clear();

	// 记录每个传感器实际提取的帧数，便于调试与统计
	std::map<std::uint32_t, std::size_t> per_sensor_counts;

	// Lambda 函数：从队列中提取样本
	auto append_from_queue = [&](const std::deque<mag_core_msgs::MagSensorData> &queue) -> std::size_t {
		const std::size_t target = static_cast<std::size_t>(frames_per_sensor_);  // 目标样本数
		const std::size_t available = queue.size();  // 可用样本数
		// best_effort 模式：提取所有可用样本；否则：提取 min(可用, 目标) 个
		const std::size_t count = best_effort ? available : std::min(available, target);
		if (count == 0) {
			return 0U;
		}
		// 从队列末尾开始提取（最新样本）
		const std::size_t start_index = available > count ? available - count : 0U;
		for (std::size_t i = start_index; i < available; ++i) {
			out.push_back(queue[i]);
		}
		return count;
	};

	if (!expected_sensor_ids_.empty()) {
		for (int sensor_id : expected_sensor_ids_) {
			auto it = sensor_samples_buffer_.find(static_cast<std::uint32_t>(sensor_id));
			if (it == sensor_samples_buffer_.end()) {
				if (!best_effort) {
					return;
				}
				continue;
			}
			const std::size_t c = append_from_queue(it->second);
			per_sensor_counts[static_cast<std::uint32_t>(sensor_id)] = c;
		}
		// 日志每个传感器提取的帧数
		std::size_t total = 0;
		for (const auto &p : per_sensor_counts) total += p.second;
		ROS_INFO_STREAM("[scan_controller] extracted samples (expected list): total=" << total << ", sensors=" << per_sensor_counts.size());
		return;
	}

	for (const auto &entry : sensor_samples_buffer_) {
		const std::size_t c = append_from_queue(entry.second);
		per_sensor_counts[entry.first] = c;
	}

	std::size_t total = 0;
	for (const auto &p : per_sensor_counts) total += p.second;
	ROS_INFO_STREAM("[scan_controller] extracted samples: total=" << total << ", sensors=" << per_sensor_counts.size());
}

/**
 * @brief 清空所有传感器的采样缓冲区
 * 
 * 功能：在每个扫描点开始采集前，清空之前的样本，开启新一轮采样
 */
void ScanControllerNode::resetSampleBuffers() {
	std::lock_guard<std::mutex> lock(data_mutex_);
	for (auto &entry : sensor_samples_buffer_) {
		entry.second.clear();
	}
}

/**
 * @brief 根据磁场强度线性插值颜色
 * @param magnitude 磁场强度
 * @return 插值后的颜色（RGBA）
 * 
 * 插值规则：
 * - magnitude_min_ → color_low_（蓝色，低强度）
 * - magnitude_max_ → color_high_（红色，高强度）
 * - 中间值线性插值
 */
ScanControllerNode::ColorRGBA ScanControllerNode::interpolateColor(double magnitude) const {
	if (magnitude_max_ <= magnitude_min_ + kEpsilon) {
		// 如果最大值和最小值相同，直接返回对应颜色
		return magnitude <= magnitude_min_ ? color_low_ : color_high_;
	}
	// 归一化到 [0, 1] 范围
	const double normalized = std::clamp((magnitude - magnitude_min_) / (magnitude_max_ - magnitude_min_), 0.0, 1.0);
	ColorRGBA color;
	// 线性插值每个颜色分量
	color.r = color_low_.r + (color_high_.r - color_low_.r) * normalized;
	color.g = color_low_.g + (color_high_.g - color_low_.g) * normalized;
	color.b = color_low_.b + (color_high_.b - color_low_.b) * normalized;
	color.a = color_low_.a + (color_high_.a - color_low_.a) * normalized;
	return color;
}

/**
 * @brief 通过 TF 查询传感器位置
 * @param sensor_id 传感器 ID
 * @param out_position 输出位置（在 frame_id_ 坐标系下）
 * @return 成功返回 true，失败返回 false
 * 
 * 功能：查询传感器坐标系到目标坐标系的变换，提取位置信息用于可视化
 */
bool ScanControllerNode::lookupSensorPosition(std::uint32_t sensor_id, geometry_msgs::Point &out_position) const {
	if (!use_tf_sensor_pose_) {
		return false;  // 未启用 TF 查询
	}
	if (sensor_frame_prefix_.empty()) {
		return false;  // 传感器坐标系前缀为空
	}
	try {
		// 构造传感器坐标系名称：sensor_frame_prefix_ + sensor_id
		const std::string sensor_frame = sensor_frame_prefix_ + std::to_string(sensor_id);
		// 查询 TF 变换：从传感器坐标系到目标坐标系
		const geometry_msgs::TransformStamped transform =
				tf_buffer_.lookupTransform(frame_id_, sensor_frame, ros::Time(0), ros::Duration(tf_lookup_timeout_));
		// 提取位置信息
		out_position.x = transform.transform.translation.x;
		out_position.y = transform.transform.translation.y;
		out_position.z = transform.transform.translation.z;
		return true;
	} catch (const tf2::TransformException &ex) {
		ROS_WARN_THROTTLE(5.0, "[scan_controller] TF lookup for sensor %u failed: %s", sensor_id, ex.what());
		return false;
	}
}

/**
 * @brief 节点主运行函数
 * 
 * 功能：
 * - 启动异步旋转器处理回调
 * - 如果配置了 autostart，等待服务就绪后自动开始扫描
 * - 否则等待服务调用或节点关闭
 */
void ScanControllerNode::run() {
	// 启动异步旋转器（2 个线程）
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// 如果配置了自动启动，等待服务就绪后自动开始扫描
	if (autostart_) {
		ROS_INFO("[scan_controller] waiting for mag_device_arm services to become ready...");
		// 等待服务可用
		if (!arm_set_pose_client_.waitForExistence(ros::Duration(10.0))) {
			ROS_ERROR("[scan_controller] mag_device_arm services not available for autostart");
			return;
		}
		
		ROS_INFO("[scan_controller] Services ready. Waiting additional stabilization time...");
		ros::Duration(3.0).sleep();  // 额外等待 3 秒确保系统稳定

		// 调用开始扫描服务
		std_srvs::Trigger::Request req;
		std_srvs::Trigger::Response res;
		startScan(req, res);
		if (!res.success) {
			ROS_ERROR_STREAM("[scan_controller] autostart scan failed: " << res.message);
		}
	}

	// 等待节点关闭
	ros::waitForShutdown();
}

/**
 * @brief 解析相对路径为绝对路径
 * @param path 输入路径（相对或绝对）
 * @return 解析后的绝对路径
 * 
 * 功能：
 * - 如果路径是绝对路径（以 / 开头），直接返回
 * - 如果是相对路径，解析为相对于工作空间根目录的绝对路径
 */
std::string ScanControllerNode::resolveRelativePath(const std::string &path) {
	if (path.empty() || path.front() == '/') {
		return path;  // 已经是绝对路径或空路径
	}

	try {
		// 获取当前包路径，从中提取工作空间根目录
		const std::string current_package_path = ros::package::getPath("mag_arm_scan");
		const std::size_t src_pos = current_package_path.find("/src/");
		if (src_pos != std::string::npos) {
			const std::string workspace_root = current_package_path.substr(0, src_pos);
			return workspace_root + '/' + path;
		}
		ROS_WARN("[scan_controller] could not determine workspace root, using original path");
		return path;
	} catch (const std::exception &e) {
		ROS_ERROR_STREAM("[scan_controller] failed to get package path: " << e.what());
		return path;
	}
}

/**
 * @brief 创建带时间戳的输出目录
 * 
 * 功能：
 * - 根据当前时间生成时间戳（格式：YYYYMMDD_HHMM）
 * - 在输出基础目录下创建子目录：scan_<timestamp>
 * - 设置输出文件路径：<timestamp_dir>/scan_data.csv
 */
void ScanControllerNode::createTimestampDirectory() {
	// 获取当前时间
	const std::time_t now = std::time(nullptr);
	std::tm tm_buffer{};
	std::tm *tm_ptr = std::localtime(&now);
	if (tm_ptr != nullptr) {
		tm_buffer = *tm_ptr;
	}

	// 格式化时间戳：YYYYMMDD_HHMM
	char timestamp[20] = {0};
	if (std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", &tm_buffer) == 0) {
		ROS_WARN("[scan_controller] failed to format timestamp, using default");
		std::snprintf(timestamp, sizeof(timestamp), "default");
	}

	// 创建时间戳目录
	std::string timestamp_dir = output_base_dir_ + "/scan_" + timestamp;
	try {
		// 尝试使用 C++17 filesystem API
		if (std::filesystem::create_directories(timestamp_dir)) {
			ROS_INFO_STREAM("[scan_controller] created timestamp directory: " << timestamp_dir);
		} else {
			ROS_INFO_STREAM("[scan_controller] timestamp directory already exists or no action needed: " << timestamp_dir);
		}
	} catch (const std::exception &e) {
		// 如果 filesystem API 失败，回退到 mkdir
		ROS_WARN_STREAM("[scan_controller] failed to create directory using filesystem: " << timestamp_dir
										<< ", error: " << e.what() << ". Falling back to mkdir.");
		if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
			ROS_INFO_STREAM("[scan_controller] created timestamp directory (mkdir fallback): " << timestamp_dir);
		} else if (errno == EEXIST) {
			ROS_INFO_STREAM("[scan_controller] timestamp directory already exists (mkdir fallback): " << timestamp_dir);
		} else {
			ROS_WARN_STREAM("[scan_controller] failed to create directory (mkdir fallback): " << timestamp_dir
											<< " (errno: " << errno << ")");
		}
	}

	// 设置输出文件路径
	output_file_ = timestamp_dir + "/scan_data.csv";
	ROS_INFO_STREAM("[scan_controller] output file: " << output_file_);
}

/**
 * @brief 完成扫描后的清理工作
 * 
 * 功能：
 * - 确保输出文件已关闭
 * - 发布扫描完成消息
 * - 清空所有数据缓冲区
 */
void ScanControllerNode::finalizeScan() {
	// 确保文件已关闭
	std::ofstream file_check(output_file_, std::ios::app);
	if (file_check.is_open()) {
		file_check.close();
	}

	ROS_INFO_STREAM("[scan_controller] scan completed: " << scan_points_.size() << " points");

	// 发布扫描完成消息
	std_msgs::String complete_msg;
	complete_msg.data = "Scan completed successfully. Data saved to: " + output_file_;
	scan_complete_pub_.publish(complete_msg);

	// 清空所有数据缓冲区
	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		sensor_samples_buffer_.clear();
	}
	collected_samples_.clear();
	ROS_INFO("[scan_controller] cleared collected data buffer");
}

}  // namespace mag_arm_scan

/**
 * @brief 主函数：启动扫描控制节点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码（0 表示成功）
 * 
 * 功能：
 * - 初始化 ROS 节点
 * - 创建 ScanControllerNode 实例
 * - 运行节点主循环
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_controller_node");
	ros::NodeHandle nh;  // 全局节点句柄
	ros::NodeHandle pnh("~");  // 私有节点句柄

	// 创建并运行扫描控制节点
	mag_arm_scan::ScanControllerNode node(nh, pnh);
	node.run();

	return 0;
}
