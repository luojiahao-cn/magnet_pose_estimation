#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <tf2_ros/transform_listener.h>
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
        // Select algorithm
        std::string estimator_type;
        nh_.param<std::string>("/estimator_config/estimator_type", estimator_type, std::string("optimization"));

        if (estimator_type == "optimization")
        {
            ROS_INFO("使用优化算法进行磁场姿态估计");
            estimator_.reset(new mag_pose_estimation::OptimizationMagnetPoseEstimator(nh_));
        }
        else if (estimator_type == "kalman")
        {
            ROS_INFO("使用卡尔曼滤波器进行磁场姿态估计");
            estimator_.reset(new mag_pose_estimation::KalmanMagnetPoseEstimator(nh_));
        }
        else
        {
            ROS_ERROR("未知的estimator_type: %s，使用默认 optimization", estimator_type.c_str());
            estimator_.reset(new mag_pose_estimation::OptimizationMagnetPoseEstimator(nh_));
        }

        // 全局坐标系和话题配置（优先 ~ 私有参数，其次集中 /estimator_config/*，最后默认）
        pnh.param<std::string>("global_frame", global_frame_, std::string("world"));
        nh_.param<std::string>("/estimator_config/global_frame", global_frame_, global_frame_);
        pnh.param<std::string>("mag_topic", mag_topic_, std::string("/mag_sensor/data_mT"));
        nh_.param<std::string>("/estimator_config/mag_topic", mag_topic_, mag_topic_);
        pnh.param<std::string>("output_topic", output_topic_, std::string("/magnet_pose/estimated"));
        nh_.param<std::string>("/estimator_config/output_topic", output_topic_, output_topic_);

        // 通过参数控制触发所需的最少测量数量
        pnh.param<int>("min_sensors", min_sensors_, 0);
        nh_.param<int>("/estimator_config/min_sensors", min_sensors_, min_sensors_);
        if (min_sensors_ < 0)
            min_sensors_ = 0;

        ROS_INFO_STREAM("[magnet_pose_estimator] config: global_frame='" << global_frame_
                        << "', mag_topic='" << mag_topic_ << "', output_topic='" << output_topic_
                        << "', min_sensors=" << min_sensors_);

        // TF listener
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

        // ROS I/O
        pose_pub_ = nh_.advertise<MagnetPose>(output_topic_, 10);
    error_pub_ = nh_.advertise<std_msgs::Float64>("/magnet_pose/error", 10);
        sub_ = nh_.subscribe(mag_topic_, 50, &MagnetPoseEstimatorNode::magCallback, this);
        reset_srv_ = nh_.advertiseService("/magnet_pose/reset_localization", &MagnetPoseEstimatorNode::onReset, this);
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
            geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(
                global_frame_, in.header.frame_id, in.header.stamp, ros::Duration(0.05));

            geometry_msgs::Pose pose_tf;
            tf2::doTransform(in.sensor_pose, pose_tf, T);

            geometry_msgs::Vector3Stamped vin, vout;
            vin.header = in.header;
            vin.vector.x = in.mag_x;
            vin.vector.y = in.mag_y;
            vin.vector.z = in.mag_z;
            tf2::doTransform(vin, vout, T);

            out = in;
            out.header.frame_id = global_frame_;
            out.sensor_pose = pose_tf;
            out.mag_x = vout.vector.x;
            out.mag_y = vout.vector.y;
            out.mag_z = vout.vector.z;
            return true;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(1.0, "TF 转换失败: %s -> %s（传感器ID=%u）: %s", in.header.frame_id.c_str(),
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
            for (const auto &kv : measurements_)
            {
                MagneticField tfed;
                if (transformMeasurementToGlobal(kv.second, tfed))
                    meas_tf[kv.first] = tfed;
            }

            MagnetPose pose;
            double error = 0.0;
            if (!meas_tf.empty() && estimator_->estimate(meas_tf, pose, &error))
            {
                pose.header.frame_id = global_frame_;
                pose_pub_.publish(pose);
                std_msgs::Float64 e;
                e.data = error;
                error_pub_.publish(e);
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "估计失败，等待更多数据");
            }
            measurements_.clear();
        }
    }

    bool onReset(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
    {
        estimator_->reset();
        measurements_.clear();
        ROS_INFO("已重置定位估计器与测量缓存");
        return true;
    }

    ros::NodeHandle nh_;
    std::unique_ptr<mag_pose_estimation::BaseMagnetPoseEstimator> estimator_;
    ros::Subscriber sub_;
    ros::Publisher pose_pub_;
    ros::Publisher error_pub_;
    ros::ServiceServer reset_srv_;
    std::map<int, MagneticField> measurements_;
    int min_sensors_{0};

    // TF and params
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string global_frame_ {"world"};
    std::string mag_topic_ {"/mag_sensor/data_mT"};
    std::string output_topic_ {"/magnet_pose/estimated"};
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
