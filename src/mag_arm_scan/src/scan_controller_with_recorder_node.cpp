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

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mag_sensor_node/sensor_config.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
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

bool ScanControllerWithRecorderNode::moveToPose(const geometry_msgs::Pose &pose) {
	move_group_->setPoseTarget(pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

	if (success) {
		success = move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	return success;
}

bool ScanControllerWithRecorderNode::moveToReadyPose() {
	ROS_INFO("[scan_controller_with_recorder] moving to ready position using named target");

	move_group_->setNamedTarget("ready");

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

	if (success) {
		success = move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	return success;
}

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

	ros::Time start_wait = ros::Time::now();
	const ros::Duration max_stable_wait(max_stable_wait_time_);
	while (!isArmStable() && (ros::Time::now() - start_wait) < max_stable_wait) {
		ros::Duration(0.1).sleep();
	}

	if (isArmStable()) {
		ROS_INFO("[scan_controller_with_recorder] arm is stable at position");
	} else {
		ROS_WARN("[scan_controller_with_recorder] arm did not stabilize within timeout, proceeding anyway");
	}

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

	for (const auto &data : collected_samples_) {
		geometry_msgs::Pose pose_w = data.sensor_pose;
		geometry_msgs::TransformStamped transform;
		bool did_tf = false;

		if (!frame_id_.empty() && frame_id_ != data.header.frame_id) {
			try {
				transform = tf_buffer_.lookupTransform(frame_id_, data.header.frame_id, ros::Time(0), ros::Duration(0.05));
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
		file << data.header.stamp.toSec() << ',' << bx << ',' << by << ',' << bz << ',' << record_point.x << ','
		     << record_point.y << ',' << record_point.z << '\n';
	}
}

bool ScanControllerWithRecorderNode::startScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	(void)req;

	ROS_INFO_STREAM("[scan_controller_with_recorder] starting scan with " << scan_points_.size() << " points");

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
	file << "timestamp,mag_x,mag_y,mag_z,pos_x,pos_y,pos_z\n";
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

bool ScanControllerWithRecorderNode::isArmStable(double velocity_threshold) {
	if (latest_joint_state_.velocity.empty()) {
		return false;
	}
	for (double vel : latest_joint_state_.velocity) {
		if (std::abs(vel) > velocity_threshold) {
			return false;
		}
	}
	return true;
}

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

void ScanControllerWithRecorderNode::extractSamplesLocked(std::vector<mag_sensor_node::MagSensorData> &out,
																 bool best_effort) {
	out.clear();

	auto append_from_queue = [&](const std::deque<mag_sensor_node::MagSensorData> &queue) {
		const std::size_t target = static_cast<std::size_t>(frames_per_sensor_);
		const std::size_t available = queue.size();
		const std::size_t count = best_effort ? available : std::min(available, target);
		if (count == 0) {
			return;
		}
		const std::size_t start_index = available > count ? available - count : 0U;
		for (std::size_t i = start_index; i < available; ++i) {
			out.push_back(queue[i]);
		}
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
			append_from_queue(it->second);
		}
		return;
	}

	for (const auto &entry : sensor_samples_buffer_) {
		append_from_queue(entry.second);
	}
}

void ScanControllerWithRecorderNode::resetSampleBuffers() {
	std::lock_guard<std::mutex> lock(data_mutex_);
	for (auto &entry : sensor_samples_buffer_) {
		entry.second.clear();
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
	if (mkdir(timestamp_dir.c_str(), 0755) == 0) {
		ROS_INFO_STREAM("[scan_controller_with_recorder] created timestamp directory: " << timestamp_dir);
	} else if (errno == EEXIST) {
		ROS_INFO_STREAM("[scan_controller_with_recorder] timestamp directory already exists: " << timestamp_dir);
	} else {
		ROS_WARN_STREAM("[scan_controller_with_recorder] failed to create directory: " << timestamp_dir
										<< " (errno: " << errno << ")");
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
