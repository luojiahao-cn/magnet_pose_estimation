#include <magnet_msgs/MagSensorData.h>
#include <magnet_msgs/MagnetPose.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <map>
#include <string>

#include "mag_pose_estimation/mag_pose_estimator_kalman.hpp"
#include "mag_pose_estimation/mag_pose_estimator_optimization.hpp"

using mag_pose_estimation::MagneticField;
using mag_pose_estimation::MagnetPose;

class MagnetPoseEstimatorNode
{
public:
    explicit MagnetPoseEstimatorNode(ros::NodeHandle &nh) : nh_(nh)
    {
        ros::NodeHandle pnh("~");
        // 仅从私有命名空间读取配置
        auto require = [&](const std::string &key, auto &var) {
            if (!pnh.getParam(key, var))
                throw std::runtime_error(std::string("缺少参数: ~") + key);
        };

        std::string estimator_type;
        require("estimator_config/estimator_type", estimator_type);

        if (estimator_type == "optimization")
        {
            ROS_INFO("[magnet_pose_estimator] 使用优化算法进行磁场姿态估计");
            estimator_.reset(new mag_pose_estimation::OptimizationMagnetPoseEstimator(pnh));
        }
        else if (estimator_type == "kalman")
        {
            ROS_INFO("[magnet_pose_estimator] 使用卡尔曼滤波器进行磁场姿态估计");
            estimator_.reset(new mag_pose_estimation::KalmanMagnetPoseEstimator(pnh));
        }
        else
        {
            throw std::runtime_error((std::string("未知的estimator_type: ") + estimator_type).c_str());
        }

        // 必需参数：全局坐标系和话题
        require("estimator_config/global_frame", global_frame_);
        require("estimator_config/mag_topic", mag_topic_);
        require("estimator_config/output_topic", output_topic_);

        // 触发所需最少测量数量
        require("estimator_config/min_sensors", min_sensors_);
        if (min_sensors_ < 0)
            min_sensors_ = 0;

        ROS_INFO_STREAM("[magnet_pose_estimator] config: global_frame='" << global_frame_
                        << "', mag_topic='" << mag_topic_ << "', output_topic='" << output_topic_
                        << "', min_sensors=" << min_sensors_);

        // TF listener and broadcaster
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

        // ROS I/O
        error_pub_ = nh_.advertise<std_msgs::Float64>(pnh.getNamespace() + std::string("/error"), 10);
        sub_ = nh_.subscribe(mag_topic_, 50, &MagnetPoseEstimatorNode::magCallback, this);
        reset_srv_ = nh_.advertiseService(pnh.getNamespace() + std::string("/reset_localization"), &MagnetPoseEstimatorNode::onReset, this);
    }

private:
    // 将单条传感器测量转换到 global_frame_
    bool transformMeasurementToGlobal(const MagneticField &in, MagneticField &out)
    {
        if (in.header.frame_id == global_frame_)
        {
            out = in;
            return true;
        }
        try
        {
            // 获取传感器帧的变换 (sensor_array_frame -> sensor_i)
            std::string sensor_frame = "sensor_" + std::to_string(in.sensor_id);
            geometry_msgs::TransformStamped sensor_tf = tf_buffer_.lookupTransform(
                in.header.frame_id, sensor_frame, in.header.stamp, ros::Duration(0.05));

            // 磁场向量在传感器局部坐标系中，转换到sensor_array_frame
            geometry_msgs::Vector3Stamped vin, vout;
            vin.header.frame_id = sensor_frame;
            vin.header.stamp = in.header.stamp;
            vin.vector.x = in.mag_x;
            vin.vector.y = in.mag_y;
            vin.vector.z = in.mag_z;
            tf2::doTransform(vin, vout, sensor_tf);

            // 然后转换到global_frame
            geometry_msgs::TransformStamped array_to_global = tf_buffer_.lookupTransform(
                global_frame_, in.header.frame_id, in.header.stamp, ros::Duration(0.05));

            geometry_msgs::Vector3Stamped vglobal;
            tf2::doTransform(vout, vglobal, array_to_global);

            out = in;
            out.header.frame_id = global_frame_;
            out.mag_x = vglobal.vector.x;
            out.mag_y = vglobal.vector.y;
            out.mag_z = vglobal.vector.z;
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(1.0, "[magnet_pose_estimator] TF 转换失败: %s -> %s（传感器ID=%u）: %s", in.header.frame_id.c_str(),
                              global_frame_.c_str(), in.sensor_id, e.what());
            return false;
        }
    }

    void magCallback(const MagneticField::ConstPtr &msg)
    {
        // Cache by sensor_id; latest wins
        measurements_[msg->sensor_id] = *msg;
        // Trigger when enough sensors, or if sensor_count unknown then when we have at least 1
        const size_t need = min_sensors_ > 0 ? static_cast<size_t>(min_sensors_) : 1u;
        if (measurements_.size() >= need)
        {
            std::map<int, MagneticField> meas_tf;
            std::map<int, geometry_msgs::Pose> sensor_poses;
            for (const auto &kv : measurements_)
            {
                MagneticField tfed;
                if (transformMeasurementToGlobal(kv.second, tfed))
                    meas_tf[kv.first] = tfed;

                // 获取传感器位置
                try {
                    std::string sensor_frame = "sensor_" + std::to_string(kv.second.sensor_id);
                    geometry_msgs::TransformStamped sensor_tf = tf_buffer_.lookupTransform(
                        global_frame_, sensor_frame, kv.second.header.stamp, ros::Duration(0.05));
                    geometry_msgs::Pose sensor_pose;
                    sensor_pose.position.x = sensor_tf.transform.translation.x;
                    sensor_pose.position.y = sensor_tf.transform.translation.y;
                    sensor_pose.position.z = sensor_tf.transform.translation.z;
                    sensor_pose.orientation = sensor_tf.transform.rotation;
                    sensor_poses[kv.first] = sensor_pose;
                } catch (const std::exception &e) {
                    ROS_WARN_THROTTLE(1.0, "[magnet_pose_estimator] 获取传感器位置失败 ID=%d: %s", kv.first, e.what());
                }
            }

            MagnetPose pose;
            double error = 0.0;
            if (!meas_tf.empty() && !sensor_poses.empty() && estimator_->estimate(meas_tf, sensor_poses, pose, &error))
            {
                // 发布磁铁TF变换
                geometry_msgs::TransformStamped magnet_tf;
                magnet_tf.header.stamp = ros::Time::now();
                magnet_tf.header.frame_id = global_frame_;
                magnet_tf.child_frame_id = "magnet_frame";
                magnet_tf.transform.translation.x = pose.position.x;
                magnet_tf.transform.translation.y = pose.position.y;
                magnet_tf.transform.translation.z = pose.position.z;
                magnet_tf.transform.rotation = pose.orientation;
                tf_broadcaster_->sendTransform(magnet_tf);

                std_msgs::Float64 e;
                e.data = error;
                error_pub_.publish(e);
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "[magnet_pose_estimator] 估计失败，等待更多数据");
            }
            measurements_.clear();
        }
    }

    bool onReset(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        estimator_->reset();
        measurements_.clear();
    ROS_INFO("[magnet_pose_estimator] 已重置定位估计器与测量缓存");
        return true;
    }

    ros::NodeHandle nh_;
    std::unique_ptr<mag_pose_estimation::BaseMagnetPoseEstimator> estimator_;
    ros::Subscriber sub_;
    ros::Publisher error_pub_;
    ros::ServiceServer reset_srv_;
    std::map<int, MagneticField> measurements_;
    int min_sensors_{0};

    // TF and params
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string global_frame_;
    std::string mag_topic_;
    std::string output_topic_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnet_pose_estimator");
    ros::NodeHandle nh;
    MagnetPoseEstimatorNode node(nh);
    ros::spin();
    return 0;
}
