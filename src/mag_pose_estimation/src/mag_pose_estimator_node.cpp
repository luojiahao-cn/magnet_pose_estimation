#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

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

        // 通过参数控制触发所需的最少测量数量，避免直接依赖 mag_sensor_node
        nh_.param<int>("/estimator_config/min_sensors", min_sensors_, 0);
        if (min_sensors_ < 0)
            min_sensors_ = 0;

        // ROS I/O
        pose_pub_ = nh_.advertise<MagnetPose>("/magnet_pose/predicted", 10);
        error_pub_ = nh_.advertise<std_msgs::Float64>("/magnet_pose/error", 10);
        sub_ = nh_.subscribe("/magnetic_field/raw_data", 50, &MagnetPoseEstimatorNode::magCallback, this);
        reset_srv_ = nh_.advertiseService("/magnet_pose/reset_localization", &MagnetPoseEstimatorNode::onReset, this);
    }

private:
    void magCallback(const MagneticField::ConstPtr &msg)
    {
        // Cache by sensor_id; latest wins
        measurements_[msg->sensor_id] = *msg;
        // Trigger when enough sensors, or if sensor_count unknown then when we have at least 1
        const size_t need = min_sensors_ > 0 ? static_cast<size_t>(min_sensors_) : 1u;
        if (measurements_.size() >= need)
        {
            MagnetPose pose;
            double error = 0.0;
            if (estimator_->estimate(measurements_, pose, &error))
            {
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
