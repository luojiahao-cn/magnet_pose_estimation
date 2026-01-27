#include <mag_device_sensor/sensor_config_loader.hpp>
#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

#include <Eigen/Core>

#include <memory>
#include <random>
#include <string>
#include <cmath>
#include <vector>
#include <locale>

namespace mag_device_sensor
{

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
        ros::Time start_time_;
    };

    namespace
    {
        constexpr double kMu0Div4Pi = 1e-7;
    }

    SensorSimNode::SensorSimNode(ros::NodeHandle nh,
                                 ros::NodeHandle pnh,
                                 const mag_core_description::SensorArrayDescription &array,
                                 TopicConfig topic_config,
                                 TfConfig tf_config,
                                 CalibrationConfig calibration,
                                 NoiseConfig noise,
                                 SimulationConfig simulation)
        : nh_(std::move(nh)),
          pnh_(std::move(pnh)),
          array_(array),
          tf_publisher_(std::make_unique<mag_core_description::SensorArrayTfPublisher>(array_)),
          topic_config_(std::move(topic_config)),
          tf_config_(tf_config),
          calibration_(std::move(calibration)),
          noise_(std::move(noise)),
          simulation_(std::move(simulation)),
          tf_listener_(tf_buffer_)
    {
        if (calibration_.full_scale_mT <= 0.0 || calibration_.raw_max <= 0.0)
        {
            throw std::runtime_error("calibration.* 参数需为正数");
        }
        // 仿真节点也可发布阵列 TF，因此与驱动节点共用发布器
        setupPublishers();
    }

    void SensorSimNode::start()
    {
        // 记录启动时间，用于延迟TF查询失败的警告
        start_time_ = ros::Time::now();
        // 启动仿真更新计时器以及可选的 TF 计时器
        setupTimers();
    }

    Eigen::MatrixXd SensorSimNode::computeField(const Eigen::Matrix<double, Eigen::Dynamic, 3> &sensor_positions,
                                                const Eigen::Vector3d &magnet_position,
                                                const Eigen::Vector3d &magnet_direction,
                                                double dipole_strength)
    {
        // 基于偶极子模型计算每个传感器位置上的磁场，单位 mT
        Eigen::MatrixXd r_vec = sensor_positions.rowwise() - magnet_position.transpose();
        Eigen::VectorXd r_norm = r_vec.rowwise().norm();
        r_norm = r_norm.array().max(1e-12);
        Eigen::Vector3d m = dipole_strength * magnet_direction;
        Eigen::VectorXd m_dot_r = r_vec * m;
        Eigen::MatrixXd field = (kMu0Div4Pi * 1e3) *
                                (3.0 * (r_vec.array().colwise() * (m_dot_r.array() / r_norm.array().pow(5))) -
                                 m.transpose().replicate(r_vec.rows(), 1).array().colwise() / r_norm.array().pow(3));
        return field;
    }

    void SensorSimNode::setupPublishers()
    {
        // 初始化仿真数据的 raw 与 field 发布器
        if (!topic_config_.raw_topic.empty())
        {
            raw_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.raw_topic, 50);
        }
        if (!topic_config_.field_topic.empty())
        {
            field_pub_ = nh_.advertise<mag_core_msgs::MagSensorData>(topic_config_.field_topic, 50);
        }
    }

    void SensorSimNode::setupTimers()
    {
        // 定期触发仿真采样；若需要也周期发送 TF
        simulation_timer_ = nh_.createTimer(ros::Duration(1.0 / simulation_.update_rate_hz),
                                            &SensorSimNode::onSimulationTimer,
                                            this);
        if (tf_config_.enable && tf_publisher_)
        {
            if (tf_config_.publish_rate > 0.0)
            {
                tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_config_.publish_rate),
                                            &SensorSimNode::onTfTimer,
                                            this);
            }
            else
            {
                tf_publisher_->publishStatic();
            }
        }
    }

    void SensorSimNode::onSimulationTimer(const ros::TimerEvent &event)
    {
        // 1. 获取磁铁相对于传感器阵列（array_frame）的位姿
        // simulation_.magnet_parent_frame 指的是磁铁的参考系（例如 world）
        // simulation_.magnet_child_frame 指的是磁铁本身的 frame
        // array_.arrayFrame() 是我们的观测系

        // 我们需要 magnet frame 在 array frame 下的 pose。
        // 即 T_array_magnet。

        // 如果 simulation_.magnet_parent_frame == array_.parentFrame() (e.g. world),
        // 且 array_.parentFrame() -> array_.arrayFrame() 是已知的 (array_pose)，
        // 那么我们可以通过 TF 查找求得 T_array_magnet。

        // 我们直接查询 TF: source=magnet_child_frame, target=array_frame

        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform(array_.arrayFrame(),
                                                   simulation_.magnet_child_frame,
                                                   ros::Time(0));

            publishReadings(transform, transform.header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            // 刚启动时可能查不到 TF，稍微抑制一下警告
            if ((ros::Time::now() - start_time_).toSec() > 2.0)
            {
                ROS_WARN_THROTTLE(1.0, "仿真无法获取磁铁位姿: %s", ex.what());
            }
        }
    }

    void SensorSimNode::onTfTimer(const ros::TimerEvent &)
    {
        if (tf_publisher_)
        {
            tf_publisher_->publishDynamic(ros::Time::now());
        }
    }

    void SensorSimNode::publishReadings(const geometry_msgs::TransformStamped &magnet_tf, const ros::Time &stamp)
    {
        // 1. 提取磁铁位置和方向 (在 array frame 下)
        Eigen::Vector3d mag_pos(magnet_tf.transform.translation.x,
                                magnet_tf.transform.translation.y,
                                magnet_tf.transform.translation.z);

        Eigen::Quaterniond mag_ori(magnet_tf.transform.rotation.w,
                                   magnet_tf.transform.rotation.x,
                                   magnet_tf.transform.rotation.y,
                                   magnet_tf.transform.rotation.z);

        Eigen::Vector3d mag_dir = mag_ori * simulation_.base_direction;

        // 2. 准备传感器位置矩阵 (在 array frame 下)
        const auto &sensors = array_.sensors();
        size_t n_sensors = sensors.size();
        Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(n_sensors, 3);

        for (size_t i = 0; i < n_sensors; ++i)
        {
            sensor_positions.row(i) << sensors[i].pose.position.x,
                sensors[i].pose.position.y,
                sensors[i].pose.position.z;
        }

        // 3. 计算理论磁场
        Eigen::MatrixXd fields = computeField(sensor_positions, mag_pos, mag_dir, simulation_.dipole_strength);

        // 4. 添加噪声并发布
        for (size_t i = 0; i < n_sensors; ++i)
        {
            Eigen::Vector3d B_true = fields.row(i);

            // 噪声
            Eigen::Vector3d noise = sampleNoise();
            Eigen::Vector3d B_meas = B_true + noise;

            // 构造消息
            mag_core_msgs::MagSensorData msg_mT;
            msg_mT.header.stamp = stamp;
            msg_mT.header.frame_id = sensors[i].frame_id;
            msg_mT.sensor_id = sensors[i].id;
            msg_mT.mag_x = B_meas.x();
            msg_mT.mag_y = B_meas.y();
            msg_mT.mag_z = B_meas.z();

            if (field_pub_)
            {
                field_pub_.publish(msg_mT);
            }

            if (raw_pub_)
            {
                mag_core_msgs::MagSensorData msg_raw = msg_mT;
                msg_raw.mag_x = toRaw(B_meas.x());
                msg_raw.mag_y = toRaw(B_meas.y());
                msg_raw.mag_z = toRaw(B_meas.z());
                raw_pub_.publish(msg_raw);
            }
        }
    }

    Eigen::Vector3d SensorSimNode::sampleNoise()
    {
        if (!noise_.enable)
        {
            return Eigen::Vector3d::Zero();
        }

        if (noise_.type == "gaussian")
        {
            std::normal_distribution<double> dist(noise_.mean, noise_.stddev);
            return Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_)) * noise_.amplitude;
        }

        return Eigen::Vector3d::Zero();
    }

} // namespace mag_device_sensor

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "mag_device_sensor_sim");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        namespace rps = rosparam_shortcuts;
        const std::string sim_ns = "mag_device_sensor.sim";
        const std::string array_ns = "mag_device_sensor.array";

        XmlRpc::XmlRpcValue sim_param;
        XmlRpc::XmlRpcValue array_param;

        std::size_t sim_error = 0;
        sim_error += !rps::get(sim_ns, pnh, "config", sim_param);
        rps::shutdownIfError(sim_ns, sim_error);

        std::size_t array_error = 0;
        array_error += !rps::get(array_ns, pnh, "array/config", array_param);
        rps::shutdownIfError(array_ns, array_error);

        auto bundle = mag_device_sensor::loadSensorSimulationConfig(sim_param, sim_ns + ".config");
        auto array_config = mag_core_description::SensorArrayDescription::loadFromParam(array_param, array_ns + ".config");
        mag_core_description::SensorArrayDescription array;
        array.load(array_config);

        mag_device_sensor::SensorSimNode node(nh,
                                              pnh,
                                              array,
                                              bundle.topics,
                                              bundle.tf,
                                              bundle.calibration,
                                              bundle.noise,
                                              bundle.simulation);
        node.start();
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("mag_device_sensor_sim 启动失败: %s", e.what());
        return 1;
    }

    return 0;
}
