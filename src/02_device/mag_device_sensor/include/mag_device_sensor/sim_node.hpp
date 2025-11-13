#pragma once

#include <mag_device_sensor/sensor_node.hpp>

#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_utils/param_reader.hpp>

#include <mag_core_msgs/MagSensorData.h>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <memory>
#include <random>
#include <string>

namespace mag_device_sensor
{

struct CalibrationConfig
{
    double full_scale_mT{3.2};
    double raw_max{32767.0};
};

struct NoiseConfig
{
    bool enable{false};
    std::string type{"gaussian"};
    double mean{0.0};
    double stddev{1.0};
    double amplitude{1.0};
};

struct SimulationConfig
{
    double update_rate_hz{100.0};
    std::string magnet_parent_frame{"world"};
    std::string magnet_child_frame{"magnet"};
    double dipole_strength{10.0};
    Eigen::Vector3d base_direction{0.0, 0.0, 1.0};
};

CalibrationConfig loadCalibrationConfig(const mag_core_utils::param::StructReader &root,
                                        CalibrationConfig defaults = {});
NoiseConfig loadNoiseConfig(const mag_core_utils::param::StructReader &root,
                            NoiseConfig defaults = {});
SimulationConfig loadSimulationConfig(const mag_core_utils::param::StructReader &root,
                                      SimulationConfig defaults = {});

class SensorSimNode
{
public:
    SensorSimNode(ros::NodeHandle nh,
                  ros::NodeHandle pnh,
                  const mag_core_description::SensorArrayDescription &array,
                  TopicConfig topic_config,
                  TfConfig tf_config,
                  CalibrationConfig calibration,
                  NoiseConfig noise,
                  SimulationConfig simulation);

    void start();

private:
    static Eigen::MatrixXd computeField(const Eigen::Matrix<double, Eigen::Dynamic, 3> &sensor_positions,
                                        const Eigen::Vector3d &magnet_position,
                                        const Eigen::Vector3d &magnet_direction,
                                        double dipole_strength);

    void setupPublishers();
    void setupTimers();

    void onSimulationTimer(const ros::TimerEvent &);
    void onTfTimer(const ros::TimerEvent &);

    void publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp);

    Eigen::Vector3d sampleNoise();
    double toMilliTesla(double raw) const { return (raw / calibration_.raw_max) * calibration_.full_scale_mT; }
    double toRaw(double mT) const { return (mT / calibration_.full_scale_mT) * calibration_.raw_max; }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    mag_core_description::SensorArrayDescription array_;
    std::unique_ptr<mag_core_description::SensorArrayTfPublisher> tf_publisher_;

    TopicConfig topic_config_;
    TfConfig tf_config_;
    CalibrationConfig calibration_;
    NoiseConfig noise_;
    SimulationConfig simulation_;

    ros::Publisher raw_pub_;
    ros::Publisher field_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Timer simulation_timer_;
    ros::Timer tf_timer_;

    std::mt19937 rng_{std::random_device{}()};
};

} // namespace mag_device_sensor
