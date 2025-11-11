//
// Scan controller with recording and RViz visualization
//
// 职责：
// - 读取节点私有参数（~）与传感器阵列配置
// - 生成三维网格扫描点，调用 MoveIt 逐点运动
// - 在每个扫描点，从 /mag_sensor/data_mT 采集并按传感器分组取均值
// - 将25个传感器的平均磁场向量发布为 MarkerArray（RViz 持续叠加显示）
// - 将原始采样写入 CSV（包含位姿/传感器ID/来源坐标系）
//
// 话题/服务：
// - 订阅:   ~mag_topic (mag_sensor_node/MagSensorData)
// - 发布:   /scan_at_position (std_msgs/Bool) 到位提示
//           /scan_complete (std_msgs/String) 扫描完成提示
//           ~visualization/topic (visualization_msgs/MarkerArray) 磁场箭头
// - 服务:   ~start_scan (std_srvs/Trigger)
//
// 关键参数见 config/scan_controller_node.yaml（frame_id、体积与步长、收集帧数、可视化等）。
#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <magnet_msgs/MagSensorData.h>
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
  // 构造：传入全局与私有 NodeHandle，内部读取参数并初始化 MoveIt、订阅与发布。
  ScanControllerWithRecorderNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  // 运行主循环：可在 autostart=true 时直接触发扫描，否则仅保持 ROS 活跃，等待服务调用。
  void run();

private:
  // 从 ~ 命名空间读取参数与可视化配置，检查必填项。
  void loadParams();
  // 根据 volume_min/max 与 step 生成三维网格扫描点，姿态使用给定 yaw/pitch。
  void generateScanPoints();
  // 近邻贪心优化扫描顺序，减少机械臂运动距离。
  void optimizeScanOrder();
  // 读取传感器阵列配置，得到期望的 sensor_id 列表；若缺失则 best-effort。
  void loadSensorConfig();
  // 计算两姿态的位置距离（m）。
  static double poseDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

  // 使用 MoveIt 规划并执行到指定姿态，返回是否成功。
  bool moveToPose(const geometry_msgs::Pose &pose);
  // 移动到预置的 "ready" 命名姿态。
  bool moveToReadyPose();
  // 计算最接近当前关节状态的 IK 解，若成功将关节目标写入 joint_goal。
  bool computePreferredIK(const geometry_msgs::Pose &pose, std::vector<double> &joint_goal);
  // 在单个扫描点执行：移动→短暂停（wait_time）→采样→写CSV→发布Marker。
  void collectDataAtPoint(const geometry_msgs::Pose &pose);
  // 扫描完成后的收尾：发布完成消息、清缓冲。
  void finalizeScan();

  // 服务回调：按当前网格依次采样，写 CSV 并发布可视化。
  bool startScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  // 传感器与关节回调：维护采样环形缓冲与最新关节态。
  void magDataCallback(const magnet_msgs::MagSensorData::ConstPtr &msg);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

  // 将相对路径解析到工作区根路径（catkin 工作区内）。
  std::string resolveRelativePath(const std::string &path);
  // 创建时间戳输出目录与输出文件名。
  void createTimestampDirectory();
  // 判断是否满足每个传感器 frames_per_sensor 的采样帧数（严格/期望列表）。
  bool hasEnoughSamplesLocked() const;
  // 从缓冲提取样本到 out；best_effort 为真时尽可能提取现有帧。
  void extractSamplesLocked(std::vector<magnet_msgs::MagSensorData> &out, bool best_effort);
  // 清空环形缓冲，开始新一轮采样。
  void resetSampleBuffers();
  // 如果配置提供测试点，加载并启用 test_points_。
  void loadTestPoints();
  struct ColorRGBA {
    double r{0.0};
    double g{0.0};
    double b{0.0};
    double a{1.0};
  };
  ColorRGBA interpolateColor(double magnitude) const;
  // 查找 TF: frame_id_ ← sensor_<id> 的平移，得到 sensor 坐标点位；失败返回 false。
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

  // MoveIt 运动学与规划调参，帮助获得稳定、接近当前姿态的解。
  double max_velocity_scaling_{0.2};
  double max_acceleration_scaling_{0.1};
  double planning_time_{2.0};
  int num_planning_attempts_{1};
  double goal_joint_tolerance_{1e-3};
  double goal_position_tolerance_{1e-4};
  double goal_orientation_tolerance_{1e-3};
  double ik_timeout_{0.1};
  double ik_consistency_limit_{0.2};
  bool use_preferred_ik_{true};
  std::string planner_id_;

  std::vector<geometry_msgs::Pose> scan_points_;
  std::vector<geometry_msgs::Pose> test_points_;
  bool use_test_points_{false};
  std::unordered_map<std::uint32_t, std::deque<magnet_msgs::MagSensorData>> sensor_samples_buffer_;
  std::vector<magnet_msgs::MagSensorData> collected_samples_;
  mutable std::mutex data_mutex_;

  bool visualization_enabled_{false};
  std::string visualization_topic_{"mag_field_vectors"};
  double arrow_length_{0.05};
  double arrow_shaft_ratio_{0.12};
  double arrow_head_ratio_{0.25};
  double arrow_head_length_ratio_{0.35};
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
