#include "mag_arm_scan/scan_controller_with_recorder_node.hpp"

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
#include <mag_sensor_node/sensor_config.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

constexpr double kEpsilon = 1e-9;

}  // namespace

namespace mag_arm_scan {

ScanControllerWithRecorderNode::ScanControllerWithRecorderNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
		: nh_(nh), pnh_(pnh), move_group_(nullptr), tf_listener_(tf_buffer_) {
	loadParams();
	loadSensorConfig();
	generateScanPoints();

	// 延迟初始化 MoveGroupInterface，确保 ROS 节点已经完全就绪。
	move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("fr5v6_arm");

	start_scan_srv_ = pnh_.advertiseService("start_scan", &ScanControllerWithRecorderNode::startScan, this);
	mag_data_sub_ = nh_.subscribe(mag_topic_, 100, &ScanControllerWithRecorderNode::magDataCallback, this);
	joint_states_sub_ = nh_.subscribe("/joint_states", 10, &ScanControllerWithRecorderNode::jointStateCallback, this);
	clear_client_ = nh_.serviceClient<std_srvs::Trigger>("/field_map_aggregator/clear_map");
	at_position_pub_ = nh_.advertise<std_msgs::Bool>("/scan_at_position", 1);
	scan_complete_pub_ = nh_.advertise<std_msgs::String>("/scan_complete", 1);
	if (visualization_enabled_) {
		marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 10);
	}

	ROS_INFO_STREAM("[scan_controller_with_recorder] initialized with " << scan_points_.size()
									<< " scan points");
}

void ScanControllerWithRecorderNode::loadParams() {
	if (!pnh_.getParam("frame_id", frame_id_)) {
		throw std::runtime_error("缺少参数: ~frame_id");
	}
	if (!pnh_.getParam("yaw", yaw_)) {
		throw std::runtime_error("缺少参数: ~yaw");
	}
	if (!pnh_.getParam("pitch", pitch_)) {
		throw std::runtime_error("缺少参数: ~pitch");
	}
	if (!pnh_.getParam("autostart", autostart_)) {
		throw std::runtime_error("缺少参数: ~autostart");
	}
	if (!pnh_.getParam("mag_topic", mag_topic_)) {
		throw std::runtime_error("缺少参数: ~mag_topic");
	}
	if (!pnh_.getParam("wait_time", wait_time_)) {
		throw std::runtime_error("缺少参数: ~wait_time");
	}
	if (!pnh_.getParam("max_stable_wait_time", max_stable_wait_time_)) {
		throw std::runtime_error("缺少参数: ~max_stable_wait_time");
	}
	if (!pnh_.getParam("output_base_dir", output_base_dir_)) {
		throw std::runtime_error("缺少参数: ~output_base_dir");
	}

	if (!pnh_.getParam("volume_min", volume_min_) || volume_min_.size() != 3U) {
		throw std::runtime_error("缺少或非法参数: ~volume_min[3]");
	}
	if (!pnh_.getParam("volume_max", volume_max_) || volume_max_.size() != 3U) {
		throw std::runtime_error("缺少或非法参数: ~volume_max[3]");
	}
	if (!pnh_.getParam("step", step_) || step_.size() != 3U) {
		throw std::runtime_error("缺少或非法参数: ~step[3]");
	}

	for (std::size_t i = 0; i < step_.size(); ++i) {
		if (step_[i] <= 0.0) {
			throw std::runtime_error("~step 必须为正数");
		}
	}

	output_base_dir_ = resolveRelativePath(output_base_dir_);
	createTimestampDirectory();

	// 可选参数：期望的传感器数（例如 25），以及在自动检测时等待的最长时间
	pnh_.param("required_num_sensors", required_num_sensors_, 0);
	pnh_.param("sensor_detect_time", sensor_detect_time_, 5.0);

	pnh_.param("frames_per_sensor", frames_per_sensor_, 10);
	if (frames_per_sensor_ < 1) {
		frames_per_sensor_ = 1;
	}
	pnh_.param("max_sample_wait_time", max_sample_wait_time_, 5.0);
	if (max_sample_wait_time_ < 0.1) {
		max_sample_wait_time_ = 0.1;
	}
	max_samples_per_sensor_ = static_cast<std::size_t>(frames_per_sensor_) * 3U;
	if (max_samples_per_sensor_ < static_cast<std::size_t>(frames_per_sensor_)) {
		max_samples_per_sensor_ = static_cast<std::size_t>(frames_per_sensor_);
	}


	ros::NodeHandle viz_nh(pnh_, "visualization");
	viz_nh.param("enabled", visualization_enabled_, false);
	viz_nh.param("topic", visualization_topic_, std::string("mag_field_vectors"));
	viz_nh.param("vector_scale", vector_scale_, 0.05);
	viz_nh.param("arrow_shaft_diameter", marker_shaft_diameter_, 0.003);
	viz_nh.param("arrow_head_diameter", marker_head_diameter_, 0.006);
	viz_nh.param("arrow_head_length", marker_head_length_, 0.01);
	viz_nh.param("magnitude_min", magnitude_min_, 0.0);
	viz_nh.param("magnitude_max", magnitude_max_, 10.0);
	viz_nh.param("alpha", visualization_alpha_, 1.0);
	viz_nh.param("use_tf_sensor_pose", use_tf_sensor_pose_, true);
	viz_nh.param("sensor_frame_prefix", sensor_frame_prefix_, std::string("sensor_"));
	viz_nh.param("tf_lookup_timeout", tf_lookup_timeout_, 0.05);
	if (tf_lookup_timeout_ < 0.0) {
		tf_lookup_timeout_ = 0.0;
	}
	visualization_alpha_ = std::clamp(visualization_alpha_, 0.0, 1.0);

	std::vector<double> color_low_param;
	if (viz_nh.getParam("color_low", color_low_param) && (color_low_param.size() == 3U || color_low_param.size() == 4U)) {
		color_low_.r = color_low_param[0];
		color_low_.g = color_low_param[1];
		color_low_.b = color_low_param[2];
		if (color_low_param.size() == 4U) {
			color_low_.a = color_low_param[3];
		}
	}
	std::vector<double> color_high_param;
	if (viz_nh.getParam("color_high", color_high_param) && (color_high_param.size() == 3U || color_high_param.size() == 4U)) {
		color_high_.r = color_high_param[0];
		color_high_.g = color_high_param[1];
		color_high_.b = color_high_param[2];
		if (color_high_param.size() == 4U) {
			color_high_.a = color_high_param[3];
		}
	}
	color_low_.a = visualization_alpha_;
	color_high_.a = visualization_alpha_;
}

void ScanControllerWithRecorderNode::generateScanPoints() {
	scan_points_.clear();

	const double roll = 0.0;
	tf2::Quaternion q;
	q.setRPY(roll, pitch_, yaw_);
	const geometry_msgs::Quaternion orientation = tf2::toMsg(q);

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

	optimizeScanOrder();
}

void ScanControllerWithRecorderNode::optimizeScanOrder() {
	if (scan_points_.empty()) {
		return;
	}

	std::vector<geometry_msgs::Pose> optimized;
	optimized.reserve(scan_points_.size());
	std::vector<bool> visited(scan_points_.size(), false);

	optimized.push_back(scan_points_.front());
	visited[0] = true;

	for (std::size_t i = 1; i < scan_points_.size(); ++i) {
		std::size_t best_idx = 0U;
		double min_dist = std::numeric_limits<double>::max();

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

		optimized.push_back(scan_points_[best_idx]);
		visited[best_idx] = true;
	}

	scan_points_.swap(optimized);
	ROS_INFO_STREAM("[scan_controller_with_recorder] optimized scan order for " << scan_points_.size()
									<< " points");
}

double ScanControllerWithRecorderNode::poseDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
	const double dx = p1.position.x - p2.position.x;
	const double dy = p1.position.y - p2.position.y;
	const double dz = p1.position.z - p2.position.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 使用 MoveIt 规划并执行到指定位姿；成功返回 true
bool ScanControllerWithRecorderNode::moveToPose(const geometry_msgs::Pose &pose) {
	move_group_->setPoseTarget(pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

	if (success) {
		success = move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
		// Try to force stop any residual motion promptly
		move_group_->stop();
	}

	return success;
}

// 移动到命名的 ready 姿态（由 MoveIt 配置）
bool ScanControllerWithRecorderNode::moveToReadyPose() {
	ROS_INFO("[scan_controller_with_recorder] moving to ready position using named target");

	move_group_->setNamedTarget("ready");

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

	if (success) {
		success = move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
		// 结束后主动 stop，尽快消除尾迹
		move_group_->stop();
	}

	return success;
}

// 单点流程：移动→短等待→采样提取/均值→写 CSV→发布可视化 Marker
void ScanControllerWithRecorderNode::collectDataAtPoint(const geometry_msgs::Pose &pose) {
	std_msgs::Bool at_pos_msg;
	at_pos_msg.data = false;
	at_position_pub_.publish(at_pos_msg);

	ROS_INFO_STREAM("[scan_controller_with_recorder] moving to position: x=" << pose.position.x
									<< ", y=" << pose.position.y << ", z=" << pose.position.z);

	if (!moveToPose(pose)) {
		ROS_ERROR("[scan_controller_with_recorder] failed to move to pose");
		return;
	}

	ros::Duration(wait_time_).sleep();


	at_pos_msg.data = true;
	at_position_pub_.publish(at_pos_msg);

	resetSampleBuffers();

	collected_samples_.clear();
	const ros::Duration max_wait(max_sample_wait_time_);
	const ros::Duration check_interval(0.05);
	const ros::Time sample_start = ros::Time::now();
	bool full_requirement_met = false;
	std::size_t captured_sensor_count = 0U;

	while ((ros::Time::now() - sample_start) < max_wait) {
		{
			std::lock_guard<std::mutex> lock(data_mutex_);
			captured_sensor_count = sensor_samples_buffer_.size();
			if (hasEnoughSamplesLocked()) {
				extractSamplesLocked(collected_samples_, false);
				full_requirement_met = true;
				break;
			}
		}
		check_interval.sleep();
	}

	if (!full_requirement_met) {
		std::lock_guard<std::mutex> lock(data_mutex_);
		captured_sensor_count = sensor_samples_buffer_.size();
		extractSamplesLocked(collected_samples_, true);
	}

	if (collected_samples_.empty()) {
		ROS_WARN("[scan_controller_with_recorder] no magnetometer data collected at this point");
		return;
	}

	if (!full_requirement_met) {
		if (num_sensors_ > 0) {
			ROS_WARN("[scan_controller_with_recorder] collected partial dataset: %zu sensors captured, expected %d",
			         captured_sensor_count, num_sensors_);
		} else {
			ROS_WARN("[scan_controller_with_recorder] collected partial dataset (sensor count unknown)");
		}
	}

	const std::size_t sample_count = collected_samples_.size();
	if (full_requirement_met) {
		ROS_INFO("[scan_controller_with_recorder] collected %zu samples across %zu sensors", sample_count,
		         expected_sensor_ids_.empty() ? sensor_samples_buffer_.size() : expected_sensor_ids_.size());
	} else {
		ROS_INFO("[scan_controller_with_recorder] proceeding with %zu sampled frames", sample_count);
	}

	std::ofstream file(output_file_, std::ios::app);
	if (!file.is_open()) {
		ROS_ERROR_STREAM("[scan_controller_with_recorder] failed to open output file: " << output_file_);
		return;
	}

	struct AggregatedSensorAccumulator {
		double sum_x{0.0};
		double sum_y{0.0};
		double sum_z{0.0};
		std::size_t count{0U};
		geometry_msgs::Point fallback_position{};
	};
	std::map<std::uint32_t, AggregatedSensorAccumulator> averaged_data;

	for (const auto &data : collected_samples_) {
		geometry_msgs::Pose pose_w = data.sensor_pose;
		geometry_msgs::TransformStamped transform;
		bool did_tf = false;

		if (!frame_id_.empty() && frame_id_ != data.header.frame_id) {
			try {
				transform = tf_buffer_.lookupTransform(frame_id_, data.header.frame_id, ros::Time(0), ros::Duration(tf_lookup_timeout_));
				tf2::doTransform(data.sensor_pose, pose_w, transform);
				did_tf = true;
			} catch (const std::exception &e) {
				ROS_WARN_THROTTLE(5.0, "TF to '%s' failed: %s; using source frame.", frame_id_.c_str(), e.what());
				pose_w = data.sensor_pose;
				did_tf = false;
			}
		}

		double bx = data.mag_x;
		double by = data.mag_y;
		double bz = data.mag_z;

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

		const geometry_msgs::Point record_point = pose_w.position;
		const geometry_msgs::Quaternion record_ori = pose_w.orientation;
		// 增加 orientation 与 sensor_id 与 frame_id 字段，便于后续区分和可视化
		file << data.header.stamp.toSec() << ',' << bx << ',' << by << ',' << bz << ',' << record_point.x << ','
		     << record_point.y << ',' << record_point.z << ',' << record_ori.x << ',' << record_ori.y << ','
		     << record_ori.z << ',' << record_ori.w << ',' << data.sensor_id << ',' << data.header.frame_id << '\n';

		auto &acc = averaged_data[data.sensor_id];
		if (acc.count == 0U) {
			acc.fallback_position = record_point;
		}
		acc.sum_x += bx;
		acc.sum_y += by;
		acc.sum_z += bz;
		acc.count += 1U;
	}
	// 确保数据已写入磁盘，降低崩溃丢失风险
	file.flush();
	file.close();

	if (visualization_enabled_ && marker_pub_) {
		visualization_msgs::MarkerArray markers;
		markers.markers.reserve(averaged_data.size());
		const ros::Time now = ros::Time::now();
		for (const auto &entry : averaged_data) {
			const std::uint32_t sensor_id = entry.first;
			const AggregatedSensorAccumulator &acc = entry.second;
			if (acc.count == 0U) {
				continue;
			}
			geometry_msgs::Point start_point;
			bool has_tf_pose = lookupSensorPosition(sensor_id, start_point);
			if (!has_tf_pose) {
				start_point = acc.fallback_position;
			}

			if (frame_id_.empty()) {
				ROS_WARN_THROTTLE(5.0, "[scan_controller_with_recorder] Missing frame_id for visualization");
				continue;
			}

			geometry_msgs::Vector3 avg_vector;
			avg_vector.x = acc.sum_x / static_cast<double>(acc.count);
			avg_vector.y = acc.sum_y / static_cast<double>(acc.count);
			avg_vector.z = acc.sum_z / static_cast<double>(acc.count);
			double magnitude = std::sqrt(avg_vector.x * avg_vector.x + avg_vector.y * avg_vector.y + avg_vector.z * avg_vector.z);
			if (magnitude < kEpsilon) {
				continue;
			}

			geometry_msgs::Point end_point = start_point;
			end_point.x += avg_vector.x * vector_scale_;
			end_point.y += avg_vector.y * vector_scale_;
			end_point.z += avg_vector.z * vector_scale_;

			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id_;
			marker.header.stamp = now;
			marker.ns = "mag_field";
			marker.id = static_cast<int>(marker_id_counter_++);
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.points.push_back(start_point);
			marker.points.push_back(end_point);
			marker.scale.x = marker_shaft_diameter_;
			marker.scale.y = marker_head_diameter_;
			marker.scale.z = marker_head_length_;
			marker.pose.orientation.w = 1.0;
			ColorRGBA color = interpolateColor(magnitude);
			marker.color.r = static_cast<float>(std::clamp(color.r, 0.0, 1.0));
			marker.color.g = static_cast<float>(std::clamp(color.g, 0.0, 1.0));
			marker.color.b = static_cast<float>(std::clamp(color.b, 0.0, 1.0));
			marker.color.a = static_cast<float>(std::clamp(color.a, 0.0, 1.0));
			marker.lifetime = ros::Duration(0.0);
			markers.markers.push_back(marker);
		}

		if (!markers.markers.empty()) {
			marker_pub_.publish(markers);
		}
	}
}

bool ScanControllerWithRecorderNode::startScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	(void)req;

	ROS_INFO_STREAM("[scan_controller_with_recorder] starting scan with " << scan_points_.size() << " points");

	// 如果没有从配置中得到传感器数量，且用户指定了 required_num_sensors_，则在开始前尝试自动检测
	if (num_sensors_ <= 0 && required_num_sensors_ > 0) {
		ROS_INFO_STREAM("[scan_controller_with_recorder] attempting to detect required_num_sensors=" << required_num_sensors_ << " (timeout=" << sensor_detect_time_ << "s)");
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
			ROS_WARN_STREAM("[scan_controller_with_recorder] detected " << num_sensors_ << " sensors, less than required " << required_num_sensors_);
		} else {
			ROS_INFO_STREAM("[scan_controller_with_recorder] detected required sensors: " << num_sensors_);
		}
	}

	if (!moveToReadyPose()) {
		ROS_ERROR("[scan_controller_with_recorder] failed to move to ready position");
		res.success = false;
		res.message = "Failed to move to ready position";
		return true;
	}

	std_srvs::Trigger clear_req;
	if (clear_client_.call(clear_req)) {
		ROS_INFO("[scan_controller_with_recorder] cleared field map aggregator");
	} else {
		ROS_WARN("[scan_controller_with_recorder] failed to clear field map aggregator");
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

	at_pos_msg.data = false;
	at_position_pub_.publish(at_pos_msg);

	finalizeScan();

	res.success = true;
	res.message = "Scan completed";
	return true;
}

// Stability check removed: rely on MoveIt execution completion and optional wait_time

void ScanControllerWithRecorderNode::magDataCallback(const mag_sensor_node::MagSensorData::ConstPtr &msg) {
	std::lock_guard<std::mutex> lock(data_mutex_);
	auto &queue = sensor_samples_buffer_[msg->sensor_id];
	queue.push_back(*msg);
	while (queue.size() > max_samples_per_sensor_) {
		queue.pop_front();
	}
}

void ScanControllerWithRecorderNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
	latest_joint_state_ = *msg;
}

// 加载阵列配置，确定期望的 sensor_id 列表；若失败则 best-effort 采样
void ScanControllerWithRecorderNode::loadSensorConfig() {
	expected_sensor_ids_.clear();
	num_sensors_ = 0;

	mag_sensor_node::SensorConfig &config = mag_sensor_node::SensorConfig::getInstance();
	if (!config.loadConfig(pnh_)) {
		ROS_WARN("[scan_controller_with_recorder] failed to load sensor configuration parameters");
		int param_sensor_count = 0;
		if (pnh_.getParam("sensor_config/num_sensors", param_sensor_count) && param_sensor_count > 0) {
			num_sensors_ = param_sensor_count;
		}
	} else {
		const auto &sensors = config.getAllSensors();
		expected_sensor_ids_.reserve(sensors.size());
		for (const auto &info : sensors) {
			expected_sensor_ids_.push_back(info.id);
		}
		std::sort(expected_sensor_ids_.begin(), expected_sensor_ids_.end());
		expected_sensor_ids_.erase(std::unique(expected_sensor_ids_.begin(), expected_sensor_ids_.end()),
		                         expected_sensor_ids_.end());
		num_sensors_ = static_cast<int>(expected_sensor_ids_.size());
	}

	if (num_sensors_ <= 0) {
		ROS_WARN("[scan_controller_with_recorder] sensor count unavailable, sampling will be best-effort");
	} else {
		ROS_INFO("[scan_controller_with_recorder] expecting %d sensors with %d frames each", num_sensors_,
		         frames_per_sensor_);
	}
}

// 是否已满足每个传感器 frames_per_sensor 的要求（需在持锁条件下调用）
bool ScanControllerWithRecorderNode::hasEnoughSamplesLocked() const {
	if (num_sensors_ <= 0) {
		return false;
	}

	if (!expected_sensor_ids_.empty()) {
		for (int sensor_id : expected_sensor_ids_) {
			auto it = sensor_samples_buffer_.find(static_cast<std::uint32_t>(sensor_id));
			if (it == sensor_samples_buffer_.end()) {
				return false;
			}
			if (static_cast<int>(it->second.size()) < frames_per_sensor_) {
				return false;
			}
		}
		return true;
	}

	if (static_cast<int>(sensor_samples_buffer_.size()) < num_sensors_) {
		return false;
	}
	for (const auto &entry : sensor_samples_buffer_) {
		if (static_cast<int>(entry.second.size()) < frames_per_sensor_) {
			return false;
		}
	}
	return true;
}

// 从各传感器缓冲中提取样本到 out；best_effort=true 时尽可能取可用的最近帧
void ScanControllerWithRecorderNode::extractSamplesLocked(std::vector<mag_sensor_node::MagSensorData> &out,
																 bool best_effort) {
	out.clear();

	// 记录每个 sensor 实际提取的帧数，便于调试与统计
	std::map<std::uint32_t, std::size_t> per_sensor_counts;

	auto append_from_queue = [&](const std::deque<mag_sensor_node::MagSensorData> &queue) -> std::size_t {
		const std::size_t target = static_cast<std::size_t>(frames_per_sensor_);
		const std::size_t available = queue.size();
		const std::size_t count = best_effort ? available : std::min(available, target);
		if (count == 0) {
			return 0U;
		}
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
		ROS_INFO_STREAM("[scan_controller_with_recorder] extracted samples (expected list): total=" << total << ", sensors=" << per_sensor_counts.size());
		return;
	}

	for (const auto &entry : sensor_samples_buffer_) {
		const std::size_t c = append_from_queue(entry.second);
		per_sensor_counts[entry.first] = c;
	}

	std::size_t total = 0;
	for (const auto &p : per_sensor_counts) total += p.second;
	ROS_INFO_STREAM("[scan_controller_with_recorder] extracted samples: total=" << total << ", sensors=" << per_sensor_counts.size());
}

// 清空每个传感器的环形缓冲，开启新一轮采样
void ScanControllerWithRecorderNode::resetSampleBuffers() {
	std::lock_guard<std::mutex> lock(data_mutex_);
	for (auto &entry : sensor_samples_buffer_) {
		entry.second.clear();
	}
}

// 线性插值颜色：magnitude_min_ → color_low_, magnitude_max_ → color_high_
ScanControllerWithRecorderNode::ColorRGBA ScanControllerWithRecorderNode::interpolateColor(double magnitude) const {
	if (magnitude_max_ <= magnitude_min_ + kEpsilon) {
		return magnitude <= magnitude_min_ ? color_low_ : color_high_;
	}
	const double normalized = std::clamp((magnitude - magnitude_min_) / (magnitude_max_ - magnitude_min_), 0.0, 1.0);
	ColorRGBA color;
	color.r = color_low_.r + (color_high_.r - color_low_.r) * normalized;
	color.g = color_low_.g + (color_high_.g - color_low_.g) * normalized;
	color.b = color_low_.b + (color_high_.b - color_low_.b) * normalized;
	color.a = color_low_.a + (color_high_.a - color_low_.a) * normalized;
	return color;
}

// 通过 TF 查询 frame_id_ ← sensor_<id> 的平移，成功填充 out_position
bool ScanControllerWithRecorderNode::lookupSensorPosition(std::uint32_t sensor_id, geometry_msgs::Point &out_position) const {
	if (!use_tf_sensor_pose_) {
		return false;
	}
	if (sensor_frame_prefix_.empty()) {
		return false;
	}
	try {
		const std::string sensor_frame = sensor_frame_prefix_ + std::to_string(sensor_id);
		const geometry_msgs::TransformStamped transform =
				tf_buffer_.lookupTransform(frame_id_, sensor_frame, ros::Time(0), ros::Duration(tf_lookup_timeout_));
		out_position.x = transform.transform.translation.x;
		out_position.y = transform.transform.translation.y;
		out_position.z = transform.transform.translation.z;
		return true;
	} catch (const tf2::TransformException &ex) {
		ROS_WARN_THROTTLE(5.0, "[scan_controller_with_recorder] TF lookup for sensor %u failed: %s", sensor_id, ex.what());
		return false;
	}
}

void ScanControllerWithRecorderNode::run() {
	ros::AsyncSpinner spinner(2);
	spinner.start();

	if (autostart_) {
		ROS_INFO("[scan_controller_with_recorder] waiting for MoveGroup to become ready...");
		while (ros::ok() && move_group_->getPlanningFrame().empty()) {
			ros::Duration(1.0).sleep();
		}
		if (!ros::ok()) {
			return;
		}

		ROS_INFO("[scan_controller_with_recorder] MoveGroup ready. Waiting additional stabilization time...");
		ros::Duration(3.0).sleep();

		std_srvs::Trigger::Request req;
		std_srvs::Trigger::Response res;
		startScan(req, res);
		if (!res.success) {
			ROS_ERROR_STREAM("[scan_controller_with_recorder] autostart scan failed: " << res.message);
		}
	}

	ros::waitForShutdown();
}

std::string ScanControllerWithRecorderNode::resolveRelativePath(const std::string &path) {
	if (path.empty() || path.front() == '/') {
		return path;
	}

	try {
		const std::string current_package_path = ros::package::getPath("mag_arm_scan");
		const std::size_t src_pos = current_package_path.find("/src/");
		if (src_pos != std::string::npos) {
			const std::string workspace_root = current_package_path.substr(0, src_pos);
			return workspace_root + '/' + path;
		}
		ROS_WARN("[scan_controller_with_recorder] could not determine workspace root, using original path");
		return path;
	} catch (const std::exception &e) {
		ROS_ERROR_STREAM("[scan_controller_with_recorder] failed to get package path: " << e.what());
		return path;
	}
}

void ScanControllerWithRecorderNode::createTimestampDirectory() {
	const std::time_t now = std::time(nullptr);
	std::tm tm_buffer{};
	std::tm *tm_ptr = std::localtime(&now);
	if (tm_ptr != nullptr) {
		tm_buffer = *tm_ptr;
	}

	char timestamp[20] = {0};
	if (std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", &tm_buffer) == 0) {
		ROS_WARN("[scan_controller_with_recorder] failed to format timestamp, using default");
		std::snprintf(timestamp, sizeof(timestamp), "default");
	}

	std::string timestamp_dir = output_base_dir_ + "/scan_" + timestamp;
	try {
		if (std::filesystem::create_directories(timestamp_dir)) {
			ROS_INFO_STREAM("[scan_controller_with_recorder] created timestamp directory: " << timestamp_dir);
		} else {
			ROS_INFO_STREAM("[scan_controller_with_recorder] timestamp directory already exists or no action needed: " << timestamp_dir);
		}
	} catch (const std::exception &e) {
		ROS_WARN_STREAM("[scan_controller_with_recorder] failed to create directory using filesystem: " << timestamp_dir
										<< ", error: " << e.what() << ". Falling back to mkdir.");
		if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
			ROS_INFO_STREAM("[scan_controller_with_recorder] created timestamp directory (mkdir fallback): " << timestamp_dir);
		} else if (errno == EEXIST) {
			ROS_INFO_STREAM("[scan_controller_with_recorder] timestamp directory already exists (mkdir fallback): " << timestamp_dir);
		} else {
			ROS_WARN_STREAM("[scan_controller_with_recorder] failed to create directory (mkdir fallback): " << timestamp_dir
											<< " (errno: " << errno << ")");
		}
	}

	output_file_ = timestamp_dir + "/scan_data.csv";
	ROS_INFO_STREAM("[scan_controller_with_recorder] output file: " << output_file_);
}

void ScanControllerWithRecorderNode::finalizeScan() {
	std::ofstream file_check(output_file_, std::ios::app);
	if (file_check.is_open()) {
		file_check.close();
	}

	ROS_INFO_STREAM("[scan_controller_with_recorder] scan completed: " << scan_points_.size() << " points");

	std_msgs::String complete_msg;
	complete_msg.data = "Scan completed successfully. Data saved to: " + output_file_;
	scan_complete_pub_.publish(complete_msg);

	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		sensor_samples_buffer_.clear();
	}
	collected_samples_.clear();
	ROS_INFO("[scan_controller_with_recorder] cleared collected data buffer");
}

}  // namespace mag_arm_scan

int main(int argc, char **argv) {
	ros::init(argc, argv, "scan_controller_with_recorder_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	mag_arm_scan::ScanControllerWithRecorderNode node(nh, pnh);
	node.run();

	return 0;
}
