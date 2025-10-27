#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <mag_sensor_node/MagSensorData.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

namespace moveit::planning_interface {
class MoveGroupInterface;
}

namespace mag_arm_scan {

class ScanControllerWithRecorderNode {
public:
  ScanControllerWithRecorderNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void run();

private:
  void loadParams();
  void generateScanPoints();
  void optimizeScanOrder();
  void loadSensorConfig();
  static double poseDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

  bool moveToPose(const geometry_msgs::Pose &pose);
  bool moveToReadyPose();
  void collectDataAtPoint(const geometry_msgs::Pose &pose);
  void finalizeScan();

  bool startScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool isArmStable(double velocity_threshold = 0.01);

  void magDataCallback(const mag_sensor_node::MagSensorData::ConstPtr &msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

  std::string resolveRelativePath(const std::string &path);
  void createTimestampDirectory();
  bool hasEnoughSamplesLocked() const;
  void extractSamplesLocked(std::vector<mag_sensor_node::MagSensorData> &out, bool best_effort);
  void resetSampleBuffers();
  struct ColorRGBA {
    double r{0.0};
    double g{0.0};
    double b{0.0};
    double a{1.0};
  };
  ColorRGBA interpolateColor(double magnitude) const;
  bool lookupSensorPosition(std::uint32_t sensor_id, geometry_msgs::Point &out_position) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  ros::ServiceServer start_scan_srv_;
  ros::Subscriber mag_data_sub_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher at_position_pub_;
  ros::Publisher scan_complete_pub_;
  ros::ServiceClient clear_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::JointState latest_joint_state_;
  int frames_per_sensor_{10};
  double max_sample_wait_time_{5.0};
  std::size_t max_samples_per_sensor_{30};
  int num_sensors_{0};
  // 如果用户在参数里指定需要的传感器数量（例如 25），可将其设置为 required_num_sensors_
  int required_num_sensors_{0};
  // 在自动检测传感器 id 时等待的最大时间（秒）
  double sensor_detect_time_{5.0};
  std::vector<int> expected_sensor_ids_;

  std::string frame_id_;
  double yaw_{0.0};
  double pitch_{0.0};
  bool autostart_{false};
  std::string mag_topic_;
  double wait_time_{0.0};
  double max_stable_wait_time_{0.0};
  std::string output_file_;
  std::string output_base_dir_;
  std::vector<double> volume_min_;
  std::vector<double> volume_max_;
  std::vector<double> step_;

  std::vector<geometry_msgs::Pose> scan_points_;
  std::unordered_map<std::uint32_t, std::deque<mag_sensor_node::MagSensorData>> sensor_samples_buffer_;
  std::vector<mag_sensor_node::MagSensorData> collected_samples_;
  mutable std::mutex data_mutex_;

  bool visualization_enabled_{false};
  std::string visualization_topic_{"mag_field_vectors"};
  double vector_scale_{0.05};
  double marker_shaft_diameter_{0.003};
  double marker_head_diameter_{0.006};
  double marker_head_length_{0.01};
  double magnitude_min_{0.0};
  double magnitude_max_{10.0};
  double visualization_alpha_{1.0};
  bool use_tf_sensor_pose_{true};
  std::string sensor_frame_prefix_{"sensor_"};
  double tf_lookup_timeout_{0.05};
  ColorRGBA color_low_{0.0, 0.0, 1.0, 1.0};
  ColorRGBA color_high_{1.0, 0.0, 0.0, 1.0};
  ros::Publisher marker_pub_;
  std::uint32_t marker_id_counter_{0U};
};

}  // namespace mag_arm_scan
