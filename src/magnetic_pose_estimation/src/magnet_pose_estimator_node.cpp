#include <ros/ros.h>
#include <Eigen/Dense>
#include <map>
#include <tf2/LinearMath/Quaternion.h>           // 添加缺少的tf2头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于转换四元数
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>


class MagnetPoseEstimator {
public:
    MagnetPoseEstimator(ros::NodeHandle& nh) : nh_(nh) {
        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh)) {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 创建发布器和订阅器
        magnet_pose_pub_ = nh_.advertise<magnetic_pose_estimation::MagnetPose>(
            "/magnet_pose/predicted", 10);
        
        magnetic_field_sub_ = nh_.subscribe("/magnetic_field/simulation", 100,
            &MagnetPoseEstimator::magneticFieldCallback, this);

        // 从参数服务器加载磁铁参数
        nh_.param<double>("simulation/magnet/strength", magnet_strength_, 0.1);
        std::vector<double> direction{0, 0, 1};
        nh_.param("simulation/magnet/direction", direction, direction);
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);
        
        // 初始化状态估计
        current_position_ = Eigen::Vector3d(0, 0, 0);
        
        ROS_INFO("磁铁位置估计器已初始化");
    }

private:
    void magneticFieldCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg) {
        // 收集足够的测量数据再进行估计
        measurements_[msg->sensor_id] = *msg;
        
        if (measurements_.size() >= 4) { // 至少需要4个传感器的数据才能进行可靠估计
            estimateMagnetPose();
            measurements_.clear(); // 清空测量数据，准备下一轮估计
        }
    }

    void estimateMagnetPose() {
        const int max_iterations = 10;
        const double convergence_threshold = 1e-6;
        
        // 构建测量矩阵和传感器位置矩阵
        int n = measurements_.size();
        Eigen::MatrixXd sensor_positions(n, 3);
        Eigen::MatrixXd measured_fields(n, 3);
        
        int i = 0;
        for (const auto& measurement : measurements_) {
            const auto& msg = measurement.second;
            sensor_positions(i, 0) = msg.sensor_pose.position.x;
            sensor_positions(i, 1) = msg.sensor_pose.position.y;
            sensor_positions(i, 2) = msg.sensor_pose.position.z;
            
            measured_fields(i, 0) = msg.mag_x;
            measured_fields(i, 1) = msg.mag_y;
            measured_fields(i, 2) = msg.mag_z;
            i++;
        }

        // Gauss-Newton迭代
        Eigen::Vector3d position = current_position_; // 使用上一次的估计作为初始值
        double last_error = std::numeric_limits<double>::max();
        
        for (int iter = 0; iter < max_iterations; ++iter) {
            // 计算预测的磁场
            Eigen::MatrixXd predicted_fields = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, magnetic_direction_, magnet_strength_);
            
            // 计算残差
            Eigen::MatrixXd residuals = measured_fields - predicted_fields;
            
            // 计算Jacobian矩阵
            Eigen::MatrixXd J = calculateJacobian(sensor_positions, position);
            
            // 计算增量
            Eigen::Vector3d delta = (J.transpose() * J).ldlt().solve(J.transpose() * Eigen::Map<Eigen::VectorXd>(residuals.data(), residuals.size()));
            
            // 更新位置估计
            position += delta;
            
            // 检查收敛
            double error = residuals.norm();
            if (std::abs(error - last_error) < convergence_threshold) {
                break;
            }
            last_error = error;
        }

        current_position_ = position;
        publishMagnetPose(position);
    }

    Eigen::MatrixXd calculateJacobian(const Eigen::MatrixXd& sensor_positions, const Eigen::Vector3d& position) {
        const double delta = 1e-6;
        Eigen::MatrixXd J(sensor_positions.rows() * 3, 3);
        
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d pos_plus = position;
            pos_plus(i) += delta;
            
            Eigen::MatrixXd field_plus = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, pos_plus, magnetic_direction_, magnet_strength_);
            
            Eigen::MatrixXd field_center = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
                sensor_positions, position, magnetic_direction_, magnet_strength_);
            
            Eigen::MatrixXd difference = field_plus - field_center;
            J.col(i) = Eigen::Map<Eigen::VectorXd>(difference.data(), difference.size()) / delta;
        }
        
        return J;
    }

    void publishMagnetPose(const Eigen::Vector3d& position) {
        magnetic_pose_estimation::MagnetPose pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";
        
        pose_msg.position.x = position.x();
        pose_msg.position.y = position.y();
        pose_msg.position.z = position.z();
        
        // 设置方向（假设磁铁方向保持固定）
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // 假设磁铁垂直向上
        pose_msg.orientation = tf2::toMsg(q);
        
        pose_msg.magnetic_strength = magnet_strength_;
        
        magnet_pose_pub_.publish(pose_msg);
    }

    ros::NodeHandle& nh_;
    ros::Publisher magnet_pose_pub_;
    ros::Subscriber magnetic_field_sub_;
    
    std::map<int, magnetic_pose_estimation::MagneticField> measurements_;
    Eigen::Vector3d current_position_;
    Eigen::Vector3d magnetic_direction_;
    double magnet_strength_;
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnet_pose_estimator");
    ros::NodeHandle nh;
    
    MagnetPoseEstimator estimator(nh);
    
    ros::spin();
    return 0;
}