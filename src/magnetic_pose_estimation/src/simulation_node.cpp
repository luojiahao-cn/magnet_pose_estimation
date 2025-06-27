#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <random>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @brief 磁场仿真节点
 * 负责模拟磁铁在矩形路径上运动，并计算各传感器位置处的磁场，发布仿真数据。
 */
class SimulationNode
{
public:
    /**
     * @brief 模拟节点构造函数
     * @param nh ROS节点句柄
     */
    SimulationNode(ros::NodeHandle &nh) : nh_(nh)
    {
        // 加载传感器配置
        if (!magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh))
        {
            ROS_ERROR("加载传感器配置失败");
            return;
        }

        // 加载参数
        loadParameters();

        // 初始化发布器和路径
        initializeNode();
    }

private:
    /**
     * @brief 加载仿真参数，包括磁铁参数、路径参数、运动模式参数、噪声参数
     */
    void loadParameters()
    {
        // 加载磁铁参数
        nh_.param<double>("simulation_config/magnet/strength", magnet_strength_, 100.0);
        std::vector<double> direction;
        nh_.param("simulation_config/magnet/direction", direction, std::vector<double>{0, 0, 1});
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);

        // 加载路径参数
        nh_.param<double>("simulation_config/path/width", rect_width_, 0.2);
        nh_.param<double>("simulation_config/path/height", rect_height_, 0.2);
        nh_.param<double>("simulation_config/path/z", rect_z_, 0.1);
        nh_.param<double>("simulation_config/path/center/x", x_center_, 0.2);
        nh_.param<double>("simulation_config/path/center/y", y_center_, 0.2);
        nh_.param<double>("simulation_config/path/update_rate", update_rate_, 10.0);
        nh_.param<int>("simulation_config/path/points_per_side", points_per_side_, 20);

        // 加载运动模式参数
        nh_.param<std::string>("simulation_config/motion/mode", motion_mode_, std::string("translate_only"));
        
        // 旋转参数（仅在需要旋转时加载）
        nh_.param<std::string>("simulation_config/motion/axis", rotation_axis_, std::string("z"));
        nh_.param<double>("simulation_config/motion/angular_velocity", angular_velocity_, 0.1);
        nh_.param<double>("simulation_config/motion/initial_pitch", initial_pitch_, 0.5);
        nh_.param<double>("simulation_config/motion/initial_roll", initial_roll_, 0.0);
        nh_.param<double>("simulation_config/motion/initial_yaw", initial_yaw_, 0.0);
        
        // 静态模式参数
        std::vector<double> static_pos, static_orient;
        nh_.param("simulation_config/motion/static_position", static_pos, std::vector<double>{0.02, 0.02, 0.03});
        nh_.param("simulation_config/motion/static_orientation", static_orient, std::vector<double>{0.0, 0.5, 0.0});
        static_position_ = Eigen::Vector3d(static_pos[0], static_pos[1], static_pos[2]);
        static_orientation_ = Eigen::Vector3d(static_orient[0], static_orient[1], static_orient[2]);

        // 根据运动模式设置启用标志
        updateMotionParameters();

        // 加载噪声参数
        nh_.param<bool>("simulation_config/noise/enable", noise_enable_, false);
        nh_.param<std::string>("simulation_config/noise/type", noise_type_, std::string("gaussian"));
        nh_.param<double>("simulation_config/noise/mean", noise_mean_, 0.0);
        nh_.param<double>("simulation_config/noise/stddev", noise_stddev_, 1.0);
        nh_.param<double>("simulation_config/noise/amplitude", noise_amplitude_, 1.0);
    }

    /**
     * @brief 根据运动模式设置启用标志
     */
    void updateMotionParameters()
    {
        if (motion_mode_ == "translate_and_rotate") {
            translation_enable_ = true;
            rotation_enable_ = true;
            follow_path_ = true;
        } else if (motion_mode_ == "translate_only") {
            translation_enable_ = true;
            rotation_enable_ = false;
            follow_path_ = true;
        } else if (motion_mode_ == "rotate_only") {
            translation_enable_ = false;
            rotation_enable_ = true;
            follow_path_ = false;
        } else if (motion_mode_ == "static") {
            translation_enable_ = false;
            rotation_enable_ = false;
            follow_path_ = false;
        }
        
        ROS_INFO("运动模式: %s, 位移: %s, 旋转: %s", 
                 motion_mode_.c_str(),
                 translation_enable_ ? "启用" : "禁用",
                 rotation_enable_ ? "启用" : "禁用");
    }

    /**
     * @brief 初始化发布器、路径和定时器
     */
    void initializeNode()
    {
        // 创建磁铁位姿和磁场数据发布器
        magnet_pose_pub_ = nh_.advertise<magnetic_pose_estimation::MagnetPose>("/magnet_pose/simulation", 25);
        magnetic_field_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/processed", 25);

        // 生成矩形路径
        generateRectPath();

        // 启动定时器，定期发布数据
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &SimulationNode::timerCallback, this);
    }

    /**
     * @brief 生成矩形运动路径
     */
    void generateRectPath()
    {
        path_points_.clear();

        // 生成矩形路径的顶点
        std::vector<geometry_msgs::Point> corners(4);
        corners[0].x = x_center_ - rect_width_ / 2;
        corners[0].y = y_center_ - rect_height_ / 2;
        corners[1].x = x_center_ + rect_width_ / 2;
        corners[1].y = y_center_ - rect_height_ / 2;
        corners[2].x = x_center_ + rect_width_ / 2;
        corners[2].y = y_center_ + rect_height_ / 2;
        corners[3].x = x_center_ - rect_width_ / 2;
        corners[3].y = y_center_ + rect_height_ / 2;

        // 在顶点之间插值生成路径点
        for (int i = 0; i < 4; i++)
        {
            int next = (i + 1) % 4;
            for (int j = 0; j < points_per_side_; j++)
            {
                double ratio = static_cast<double>(j) / points_per_side_;
                geometry_msgs::Point p;
                p.x = corners[i].x + (corners[next].x - corners[i].x) * ratio;
                p.y = corners[i].y + (corners[next].y - corners[i].y) * ratio;
                p.z = rect_z_;
                path_points_.push_back(p);
            }
        }
    }

    /**
     * @brief 定时器回调函数，用于更新和发布磁铁位置及磁场数据
     * @param event 定时器事件
     */
    void timerCallback(const ros::TimerEvent &)
    {
        // 对于纯旋转模式或静态模式，不需要路径点
        if (path_points_.empty() && (motion_mode_ == "translate_only" || motion_mode_ == "translate_and_rotate"))
            return;

        // 发布磁铁位姿
        magnetic_pose_estimation::MagnetPose magnet_pose;
        magnet_pose.header.stamp = ros::Time::now();
        
        // 根据运动模式设置位置和朝向
        updateMagnetPose(magnet_pose);
        
        magnet_pose.magnetic_strength = magnet_strength_;
        magnet_pose_pub_.publish(magnet_pose);

        // 获取所有传感器
        const auto &sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();

        // 发布每个传感器的磁场数据（保留原有功能）
        publishMagneticFields(sensors, magnet_pose);
    }

    /**
     * @brief 根据运动模式更新磁铁位姿
     * @param magnet_pose 磁铁位姿消息
     */
    void updateMagnetPose(magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        // 更新位置
        if (motion_mode_ == "static") {
            // 静态模式：使用固定位置
            magnet_pose.position.x = static_position_.x();
            magnet_pose.position.y = static_position_.y();
            magnet_pose.position.z = static_position_.z();
        } else if (translation_enable_ && follow_path_ && !path_points_.empty()) {
            // 沿路径移动（保留原有逻辑）
            current_point_index_ = (current_point_index_ + 1) % path_points_.size();
            magnet_pose.position = path_points_[current_point_index_];
        } else if (translation_enable_) {
            // 其他位移模式
            magnet_pose.position = path_points_.empty() ? 
                geometry_msgs::Point() : path_points_[0];
        } else {
            // 不移动，保持在路径中心或固定位置
            magnet_pose.position.x = x_center_;
            magnet_pose.position.y = y_center_;
            magnet_pose.position.z = rect_z_;
        }

        // 更新朝向
        updateMagnetOrientation(magnet_pose);
    }

    /**
     * @brief 更新磁铁朝向
     * @param magnet_pose 磁铁位姿消息
     */
    void updateMagnetOrientation(magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        double roll, pitch, yaw;
        
        if (motion_mode_ == "static") {
            // 静态模式：使用固定朝向
            roll = static_orientation_.x();
            pitch = static_orientation_.y();
            yaw = static_orientation_.z();
        } else if (rotation_enable_) {
            // 旋转模式：基于时间计算动态朝向
            static ros::Time start_time = ros::Time::now();
            double elapsed_time = (ros::Time::now() - start_time).toSec();
            double rotation_angle = elapsed_time * angular_velocity_;
            
            // 设置初始朝向
            roll = initial_roll_;
            pitch = initial_pitch_;
            yaw = initial_yaw_;
            
            // 根据旋转轴应用旋转
            if (rotation_axis_ == "x" || rotation_axis_ == "xyz") {
                roll += rotation_angle;
            }
            if (rotation_axis_ == "y" || rotation_axis_ == "xyz") {
                pitch += rotation_angle;
            }
            if (rotation_axis_ == "z" || rotation_axis_ == "xyz") {
                yaw += rotation_angle;
            }
            
        } else {
            // 不旋转：保持初始朝向
            roll = initial_roll_;
            pitch = initial_pitch_;
            yaw = initial_yaw_;
        }

        // 设置四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        magnet_pose.orientation.x = q.x();
        magnet_pose.orientation.y = q.y();
        magnet_pose.orientation.z = q.z();
        magnet_pose.orientation.w = q.w();
        
        // 调试输出
        static size_t debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            ROS_INFO("磁铁朝向 - 模式: %s, Roll: %.3f, Pitch: %.3f, Yaw: %.3f", 
                     motion_mode_.c_str(), roll, pitch, yaw);
        }
    }

    /**
     * @brief 计算并发布所有传感器的磁场数据（保留原有功能）
     * @param sensors 传感器信息列表
     * @param magnet_pose 当前磁铁位姿
     */
    void publishMagneticFields(const std::vector<magnetic_pose_estimation::SensorInfo> &sensors,
                               const magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        // 构建传感器位置矩阵，单位: m
        Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(sensors.size(), 3);
        for (size_t i = 0; i < sensors.size(); ++i)
        {
            sensor_positions(i, 0) = sensors[i].pose.position.x;
            sensor_positions(i, 1) = sensors[i].pose.position.y;
            sensor_positions(i, 2) = sensors[i].pose.position.z;
        }

        // 构建磁铁位置，单位: m
        Eigen::Vector3d magnetic_position(
            magnet_pose.position.x,
            magnet_pose.position.y,
            magnet_pose.position.z);

        // 由magnet_pose.orientation计算当前磁铁方向
        Eigen::Quaterniond q(
            magnet_pose.orientation.w,
            magnet_pose.orientation.x,
            magnet_pose.orientation.y,
            magnet_pose.orientation.z
        );
        // 假设初始磁铁方向为(0, 0, 1)
        Eigen::Vector3d base_direction(0, 0, 1);
        Eigen::Vector3d current_direction = q * base_direction;

        // 计算磁场，单位: mT
        Eigen::MatrixXd magnetic_fields = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
            sensor_positions, magnetic_position, current_direction, magnet_strength_);

        // 噪声生成器
        std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<double> gaussian_dist(noise_mean_, noise_stddev_);               // 单位: mT
        std::uniform_real_distribution<double> uniform_dist(-noise_amplitude_, noise_amplitude_); // 单位: mT

        for (size_t i = 0; i < sensors.size(); ++i)
        {
            magnetic_pose_estimation::MagneticField field_msg;
            field_msg.header.stamp = ros::Time::now();
            field_msg.sensor_id = sensors[i].id;
            field_msg.sensor_pose = sensors[i].pose;

            double nx = 0, ny = 0, nz = 0; // 噪声，单位: mT
            if (noise_enable_)
            {
                if (noise_type_ == "gaussian")
                {
                    nx = gaussian_dist(generator);
                    ny = gaussian_dist(generator);
                    nz = gaussian_dist(generator);
                }
                else if (noise_type_ == "uniform")
                {
                    nx = uniform_dist(generator);
                    ny = uniform_dist(generator);
                    nz = uniform_dist(generator);
                }
            }

            // 叠加噪声后的磁场，单位: mT
            field_msg.mag_x = magnetic_fields(i, 0) + nx;
            field_msg.mag_y = magnetic_fields(i, 1) + ny;
            field_msg.mag_z = magnetic_fields(i, 2) + nz;
            magnetic_field_pub_.publish(field_msg);
        }
    }

    // 原有成员变量
    ros::NodeHandle nh_;                ///< ROS节点句柄
    ros::Publisher magnet_pose_pub_;    ///< 磁铁位姿发布器
    ros::Publisher magnetic_field_pub_; ///< 磁场数据发布器
    ros::Timer timer_;                  ///< 定时器

    std::vector<geometry_msgs::Point> path_points_; ///< 路径点
    size_t current_point_index_ = 0;                ///< 当前路径点索引
    int points_per_side_ = 20;                      ///< 每条边的路径点数
    double magnet_strength_;                        ///< 磁铁强度，单位: A·m²
    double rect_width_;                             ///< 路径宽度，单位: m
    double rect_height_;                            ///< 路径高度，单位: m
    double rect_z_;                                 ///< 路径高度（z轴），单位: m
    double update_rate_;                            ///< 发布频率，单位: Hz
    double x_center_;                               ///< 路径中心x坐标，单位: m
    double y_center_;                               ///< 路径中心y坐标，单位: m
    Eigen::Vector3d magnetic_direction_;            ///< 磁铁方向（单位: 无量纲）

    bool noise_enable_;      ///< 是否添加噪声
    std::string noise_type_; ///< 噪声类型
    double noise_mean_;      ///< 噪声均值，单位: mT
    double noise_stddev_;    ///< 噪声标准差，单位: mT
    double noise_amplitude_; ///< 噪声幅值，单位: mT

    // 新增运动模式成员变量
    std::string motion_mode_;           ///< 运动模式
    std::string rotation_axis_;         ///< 旋转轴
    double angular_velocity_;           ///< 角速度
    double initial_roll_;               ///< 初始滚转角
    double initial_pitch_;              ///< 初始俯仰角
    double initial_yaw_;                ///< 初始偏航角
    Eigen::Vector3d static_position_;   ///< 静态模式位置
    Eigen::Vector3d static_orientation_; ///< 静态模式朝向
    
    // 运行时状态标志（由motion_mode_控制，不需要配置）
    bool translation_enable_ = false;
    bool rotation_enable_ = false;
    bool follow_path_ = false;
};

/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;

    SimulationNode simulation(nh);

    ros::spin();
    return 0;
}