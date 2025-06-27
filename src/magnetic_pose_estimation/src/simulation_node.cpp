/**
 * @file simulation_node.cpp
 * @brief 磁场仿真节点实现
 * @author lawkaho
 * @date 2025-06-27
 * 
 * 该节点负责模拟磁铁的运动和磁场计算：
 * 1. 支持多种运动模式：静态、仅平移、仅旋转、平移+旋转
 * 2. 基于速度的连续平滑运动控制
 * 3. 实时计算各传感器位置的磁场强度
 * 4. 支持噪声添加和参数化配置
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <random>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @brief 磁场仿真节点类
 * 
 * 功能模块：
 * - 参数加载与配置管理
 * - 基于速度的运动控制
 * - 磁场计算与发布
 * - 多运动模式支持
 */
class SimulationNode
{
public:
    /**
     * @brief 构造函数 - 初始化仿真节点
     * @param nh ROS节点句柄
     */
    SimulationNode(ros::NodeHandle &nh) : nh_(nh)
    {
        if (!loadSensorConfig()) {
            ROS_ERROR("传感器配置加载失败");
            return;
        }
        
        loadSimulationParameters();
        initializeSimulation();
    }

private:
    // ===========================
    // 1. 配置加载模块
    // ===========================
    
    /**
     * @brief 加载传感器配置
     * @return true 加载成功，false 加载失败
     */
    bool loadSensorConfig()
    {
        return magnetic_pose_estimation::SensorConfig::getInstance().loadConfig(nh_);
    }
    /**
     * @brief 加载仿真参数配置
     */
    void loadSimulationParameters()
    {
        loadMagnetParameters();
        loadPathParameters();
        loadMotionParameters();
        loadNoiseParameters();
        
        // 根据运动模式设置运行时标志
        updateMotionFlags();
    }
    
    /**
     * @brief 加载磁铁参数
     */
    void loadMagnetParameters()
    {
        nh_.param<double>("simulation_config/magnet/strength", magnet_strength_, 10.0);
        
        std::vector<double> direction;
        nh_.param("simulation_config/magnet/direction", direction, std::vector<double>{0, 0, 1});
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);
        
        ROS_INFO("磁铁参数 - 强度: %.2f A·m², 方向: [%.2f, %.2f, %.2f]", 
                 magnet_strength_, direction[0], direction[1], direction[2]);
    }
    
    /**
     * @brief 加载路径和运动参数
     */
    void loadPathParameters()
    {
        // 路径几何参数
        nh_.param<double>("simulation_config/path/width", rect_width_, 0.03);
        nh_.param<double>("simulation_config/path/height", rect_height_, 0.03);
        nh_.param<double>("simulation_config/path/z", rect_z_, 0.03);
        nh_.param<double>("simulation_config/path/center/x", x_center_, 0.02);
        nh_.param<double>("simulation_config/path/center/y", y_center_, 0.02);
        
        // 运动控制参数
        nh_.param<double>("simulation_config/path/update_rate", update_rate_, 100.0);
        nh_.param<double>("simulation_config/path/velocity", translation_velocity_, 0.01);
        
        ROS_INFO("路径参数 - 尺寸: %.3f×%.3f m, 中心: (%.3f, %.3f, %.3f) m, 速度: %.4f m/s", 
                 rect_width_, rect_height_, x_center_, y_center_, rect_z_, translation_velocity_);
    }
    
    /**
     * @brief 加载运动模式参数
     */
    void loadMotionParameters()
    {
        nh_.param<std::string>("simulation_config/motion/mode", motion_mode_, "translate_only");
        
        // 旋转参数
        nh_.param<std::string>("simulation_config/motion/axis", rotation_axis_, "z");
        nh_.param<double>("simulation_config/motion/angular_velocity", angular_velocity_, 0.1);
        nh_.param<double>("simulation_config/motion/initial_roll", initial_roll_, 0.0);
        nh_.param<double>("simulation_config/motion/initial_pitch", initial_pitch_, 0.0);
        nh_.param<double>("simulation_config/motion/initial_yaw", initial_yaw_, 0.0);
        
        // 静态模式参数
        std::vector<double> static_pos, static_orient;
        nh_.param("simulation_config/motion/static_position", static_pos, 
                  std::vector<double>{0.02, 0.02, 0.03});
        nh_.param("simulation_config/motion/static_orientation", static_orient, 
                  std::vector<double>{0.0, 0.5, 0.0});
                  
        static_position_ = Eigen::Vector3d(static_pos[0], static_pos[1], static_pos[2]);
        static_orientation_ = Eigen::Vector3d(static_orient[0], static_orient[1], static_orient[2]);
    }
    
    /**
     * @brief 加载噪声参数
     */
    void loadNoiseParameters()
    {
        nh_.param<bool>("simulation_config/noise/enable", noise_enable_, false);
        nh_.param<std::string>("simulation_config/noise/type", noise_type_, "gaussian");
        nh_.param<double>("simulation_config/noise/mean", noise_mean_, 0.0);
        nh_.param<double>("simulation_config/noise/stddev", noise_stddev_, 1.0);
        nh_.param<double>("simulation_config/noise/amplitude", noise_amplitude_, 1.0);
        
        if (noise_enable_) {
            ROS_INFO("噪声配置 - 类型: %s, 均值: %.3f, 标准差: %.3f", 
                     noise_type_.c_str(), noise_mean_, noise_stddev_);
        }
    }

    /**
     * @brief 根据运动模式设置运行时标志
     */
    void updateMotionFlags()
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
        
        ROS_INFO("运动模式: %s | 平移: %s | 旋转: %s", 
                 motion_mode_.c_str(),
                 translation_enable_ ? "启用" : "禁用",
                 rotation_enable_ ? "启用" : "禁用");
    }

    // ===========================
    // 2. 仿真初始化模块
    // ===========================
    
    /**
     * @brief 初始化仿真系统
     */
    void initializeSimulation()
    {
        setupPublishers();
        initializeMotionSystem();
        startSimulationTimer();
    }
    
    /**
     * @brief 设置ROS发布器
     */
    void setupPublishers()
    {
        magnet_pose_pub_ = nh_.advertise<magnetic_pose_estimation::MagnetPose>(
            "/magnet_pose/simulation", 25);
        magnetic_field_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>(
            "/magnetic_field/processed", 25);
            
        ROS_INFO("发布器已初始化 - 磁铁位姿和磁场数据");
    }
    
    /**
     * @brief 初始化运动系统
     */
    void initializeMotionSystem()
    {
        // 计算路径参数
        path_total_length_ = 2.0 * (rect_width_ + rect_height_);
        
        // 设置起始时间和位置
        start_time_ = ros::Time::now();
        current_position_.x = x_center_ - rect_width_ / 2.0;
        current_position_.y = y_center_ - rect_height_ / 2.0;
        current_position_.z = rect_z_;
        
        ROS_INFO("运动系统初始化完成:");
        ROS_INFO("  - 路径总长度: %.4f m", path_total_length_);
        ROS_INFO("  - 运动周期: %.2f s", path_total_length_ / translation_velocity_);
        ROS_INFO("  - 起始位置: (%.4f, %.4f, %.4f) m", 
                 current_position_.x, current_position_.y, current_position_.z);
    }
    
    /**
     * @brief 启动仿真定时器
     */
    void startSimulationTimer()
    {
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                &SimulationNode::simulationLoop, this);
        ROS_INFO("仿真定时器启动 - 频率: %.1f Hz", update_rate_);
    }

    // ===========================
    // 3. 运动计算模块
    // ===========================
    
    /**
     * @brief 根据时间和速度计算矩形路径上的当前位置
     * @param elapsed_time 已经过的时间，单位：s
     * @return 当前位置
     */
    geometry_msgs::Point calculatePositionFromVelocity(double elapsed_time)
    {
        geometry_msgs::Point position;
        
        // 计算在路径上已经移动的距离
        double distance_traveled = translation_velocity_ * elapsed_time;
        double distance_on_path = fmod(distance_traveled, path_total_length_);
        
        // 矩形路径的四条边长度
        double bottom_edge = rect_width_;    // 底边：左下 → 右下
        double right_edge = rect_height_;    // 右边：右下 → 右上
        double top_edge = rect_width_;       // 顶边：右上 → 左上
        double left_edge = rect_height_;     // 左边：左上 → 左下
        
        // 矩形的边界坐标
        double left = x_center_ - rect_width_ / 2.0;
        double right = x_center_ + rect_width_ / 2.0;
        double bottom = y_center_ - rect_height_ / 2.0;
        double top = y_center_ + rect_height_ / 2.0;
        
        position.z = rect_z_;
        
        // 根据距离确定当前在哪条边上
        if (distance_on_path <= bottom_edge) {
            // 底边：从左下向右下移动
            double ratio = distance_on_path / bottom_edge;
            position.x = left + ratio * rect_width_;
            position.y = bottom;
        } else if (distance_on_path <= bottom_edge + right_edge) {
            // 右边：从右下向右上移动
            double ratio = (distance_on_path - bottom_edge) / right_edge;
            position.x = right;
            position.y = bottom + ratio * rect_height_;
        } else if (distance_on_path <= bottom_edge + right_edge + top_edge) {
            // 顶边：从右上向左上移动
            double ratio = (distance_on_path - bottom_edge - right_edge) / top_edge;
            position.x = right - ratio * rect_width_;
            position.y = top;
        } else {
            // 左边：从左上向左下移动
            double ratio = (distance_on_path - bottom_edge - right_edge - top_edge) / left_edge;
            position.x = left;
            position.y = top - ratio * rect_height_;
        }
        
        return position;
    }

    // ===========================
    // 4. 仿真主循环模块
    // ===========================
    
    /**
     * @brief 仿真主循环 - 定时器回调函数
     * @param event 定时器事件
     */
    void simulationLoop(const ros::TimerEvent &)
    {
        // 创建磁铁位姿消息
        magnetic_pose_estimation::MagnetPose magnet_pose;
        magnet_pose.header.stamp = ros::Time::now();
        
        // 更新磁铁位姿
        updateMagnetPose(magnet_pose);
        magnet_pose.magnetic_strength = magnet_strength_;
        
        // 发布磁铁位姿
        magnet_pose_pub_.publish(magnet_pose);
        
        // 计算并发布传感器磁场数据
        publishSensorMagneticFields(magnet_pose);
    }
    
    /**
     * @brief 更新磁铁位姿（位置+朝向）
     * @param magnet_pose 磁铁位姿消息引用
     */
    void updateMagnetPose(magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        updateMagnetPosition(magnet_pose);
        updateMagnetOrientation(magnet_pose);
    }
    
    /**
     * @brief 更新磁铁位置
     * @param magnet_pose 磁铁位姿消息引用
     */
    void updateMagnetPosition(magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        switch (getMotionType()) {
            case MotionType::STATIC:
                // 静态模式：使用配置的固定位置
                magnet_pose.position.x = static_position_.x();
                magnet_pose.position.y = static_position_.y();
                magnet_pose.position.z = static_position_.z();
                break;
                
            case MotionType::TRANSLATE_WITH_PATH:
                // 沿路径移动：基于速度和时间计算
                {
                    double elapsed_time = (ros::Time::now() - start_time_).toSec();
                    current_position_ = calculatePositionFromVelocity(elapsed_time);
                    magnet_pose.position = current_position_;
                }
                break;
                
            case MotionType::TRANSLATE_STATIONARY:
                // 平移启用但无路径：保持在起始位置
                magnet_pose.position.x = x_center_ - rect_width_ / 2.0;
                magnet_pose.position.y = y_center_ - rect_height_ / 2.0;
                magnet_pose.position.z = rect_z_;
                break;
                
            case MotionType::CENTER_FIXED:
            default:
                // 固定在路径中心
                magnet_pose.position.x = x_center_;
                magnet_pose.position.y = y_center_;
                magnet_pose.position.z = rect_z_;
                break;
        }
    }
    
    /**
     * @brief 获取当前运动类型
     * @return 运动类型枚举
     */
    enum class MotionType {
        STATIC,                  // 静态模式
        TRANSLATE_WITH_PATH,     // 沿路径平移
        TRANSLATE_STATIONARY,    // 平移启用但无路径
        CENTER_FIXED            // 固定在中心
    };
    
    MotionType getMotionType() const
    {
        if (motion_mode_ == "static") {
            return MotionType::STATIC;
        } else if (translation_enable_ && follow_path_) {
            return MotionType::TRANSLATE_WITH_PATH;
        } else if (translation_enable_) {
            return MotionType::TRANSLATE_STATIONARY;
        } else {
            return MotionType::CENTER_FIXED;
        }
    }

    /**
     * @brief 更新磁铁朝向
     * @param magnet_pose 磁铁位姿消息引用
     */
    void updateMagnetOrientation(magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        double roll, pitch, yaw;
        
        if (motion_mode_ == "static") {
            // 静态模式：使用配置的固定朝向
            roll = static_orientation_.x();
            pitch = static_orientation_.y();
            yaw = static_orientation_.z();
        } else if (rotation_enable_) {
            // 旋转模式：基于时间计算动态朝向
            calculateDynamicOrientation(roll, pitch, yaw);
        } else {
            // 无旋转：保持初始朝向
            roll = initial_roll_;
            pitch = initial_pitch_;
            yaw = initial_yaw_;
        }

        // 转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        magnet_pose.orientation.x = q.x();
        magnet_pose.orientation.y = q.y();
        magnet_pose.orientation.z = q.z();
        magnet_pose.orientation.w = q.w();
        
        // 定期输出调试信息
        static size_t debug_counter = 0;
        if (++debug_counter % 100 == 0) {
            ROS_DEBUG("磁铁朝向 - 模式: %s, RPY: [%.3f, %.3f, %.3f] rad", 
                     motion_mode_.c_str(), roll, pitch, yaw);
        }
    }
    
    /**
     * @brief 计算动态朝向（旋转模式）
     * @param roll 滚转角输出
     * @param pitch 俯仰角输出
     * @param yaw 偏航角输出
     */
    void calculateDynamicOrientation(double &roll, double &pitch, double &yaw)
    {
        static ros::Time rotation_start_time = ros::Time::now();
        double elapsed_time = (ros::Time::now() - rotation_start_time).toSec();
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
    }

    // ===========================
    // 5. 磁场计算与发布模块
    // ===========================
    
    /**
     * @brief 计算并发布所有传感器的磁场数据
     * @param magnet_pose 当前磁铁位姿
     */
    void publishSensorMagneticFields(const magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        const auto &sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();
        
        if (sensors.empty()) {
            ROS_WARN_THROTTLE(5.0, "未找到传感器配置，跳过磁场计算");
            return;
        }
        
        // 构建传感器位置矩阵
        Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions = buildSensorPositionMatrix(sensors);
        
        // 获取磁铁位置和方向
        Eigen::Vector3d magnet_position = getMagnetPosition(magnet_pose);
        Eigen::Vector3d magnet_direction = getMagnetDirection(magnet_pose);
        
        // 计算磁场
        Eigen::MatrixXd magnetic_fields = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
            sensor_positions, magnet_position, magnet_direction, magnet_strength_);
        
        // 发布每个传感器的磁场数据
        publishIndividualSensorFields(sensors, magnetic_fields, magnet_pose.header.stamp);
    }
    
    /**
     * @brief 构建传感器位置矩阵
     * @param sensors 传感器信息列表
     * @return 传感器位置矩阵 (N×3)
     */
    Eigen::Matrix<double, Eigen::Dynamic, 3> buildSensorPositionMatrix(
        const std::vector<magnetic_pose_estimation::SensorInfo> &sensors)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 3> positions(sensors.size(), 3);
        for (size_t i = 0; i < sensors.size(); ++i) {
            positions(i, 0) = sensors[i].pose.position.x;
            positions(i, 1) = sensors[i].pose.position.y;
            positions(i, 2) = sensors[i].pose.position.z;
        }
        return positions;
    }
    
    /**
     * @brief 获取磁铁位置向量
     * @param magnet_pose 磁铁位姿
     * @return 磁铁位置向量
     */
    Eigen::Vector3d getMagnetPosition(const magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        return Eigen::Vector3d(
            magnet_pose.position.x,
            magnet_pose.position.y,
            magnet_pose.position.z
        );
    }
    
    /**
     * @brief 获取磁铁方向向量（考虑朝向）
     * @param magnet_pose 磁铁位姿
     * @return 当前磁铁方向向量
     */
    Eigen::Vector3d getMagnetDirection(const magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        // 构建旋转四元数
        Eigen::Quaterniond q(
            magnet_pose.orientation.w,
            magnet_pose.orientation.x,
            magnet_pose.orientation.y,
            magnet_pose.orientation.z
        );
        
        // 将基础方向向量旋转到当前朝向
        Eigen::Vector3d base_direction(0, 0, 1);  // 假设初始方向为z轴正方向
        return q * base_direction;
    }

    /**
     * @brief 发布单个传感器的磁场数据（含噪声处理）
     * @param sensors 传感器信息列表
     * @param magnetic_fields 计算得到的磁场矩阵
     * @param timestamp 时间戳
     */
    void publishIndividualSensorFields(
        const std::vector<magnetic_pose_estimation::SensorInfo> &sensors,
        const Eigen::MatrixXd &magnetic_fields,
        const ros::Time &timestamp)
    {
        // 初始化噪声生成器
        static std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<double> gaussian_dist(noise_mean_, noise_stddev_);
        std::uniform_real_distribution<double> uniform_dist(-noise_amplitude_, noise_amplitude_);

        for (size_t i = 0; i < sensors.size(); ++i) {
            magnetic_pose_estimation::MagneticField field_msg;
            field_msg.header.stamp = timestamp;
            field_msg.sensor_id = sensors[i].id;
            field_msg.sensor_pose = sensors[i].pose;

            // 添加噪声
            Eigen::Vector3d noise = generateNoise(gaussian_dist, uniform_dist, generator);
            
            // 设置磁场值（计算值 + 噪声）
            field_msg.mag_x = magnetic_fields(i, 0) + noise.x();
            field_msg.mag_y = magnetic_fields(i, 1) + noise.y();
            field_msg.mag_z = magnetic_fields(i, 2) + noise.z();
            
            magnetic_field_pub_.publish(field_msg);
        }
    }
    
    /**
     * @brief 生成噪声向量
     * @param gaussian_dist 高斯分布生成器
     * @param uniform_dist 均匀分布生成器
     * @param generator 随机数生成器
     * @return 噪声向量
     */
    Eigen::Vector3d generateNoise(
        std::normal_distribution<double> &gaussian_dist,
        std::uniform_real_distribution<double> &uniform_dist,
        std::default_random_engine &generator)
    {
        Eigen::Vector3d noise = Eigen::Vector3d::Zero();
        
        if (noise_enable_) {
            if (noise_type_ == "gaussian") {
                noise.x() = gaussian_dist(generator);
                noise.y() = gaussian_dist(generator);
                noise.z() = gaussian_dist(generator);
            } else if (noise_type_ == "uniform") {
                noise.x() = uniform_dist(generator);
                noise.y() = uniform_dist(generator);
                noise.z() = uniform_dist(generator);
            }
        }
        
        return noise;
    }

    // ===========================
    // 6. 成员变量定义
    // ===========================
    
private:
    // ROS相关成员
    ros::NodeHandle nh_;                                    ///< ROS节点句柄
    ros::Publisher magnet_pose_pub_;                        ///< 磁铁位姿发布器
    ros::Publisher magnetic_field_pub_;                     ///< 磁场数据发布器
    ros::Timer timer_;                                      ///< 仿真定时器

    // 磁铁参数
    double magnet_strength_;                                ///< 磁铁强度，单位: A·m²
    Eigen::Vector3d magnetic_direction_;                    ///< 磁铁方向向量（无量纲）

    // 路径几何参数
    double rect_width_;                                     ///< 矩形路径宽度，单位: m
    double rect_height_;                                    ///< 矩形路径高度，单位: m
    double rect_z_;                                         ///< 路径平面高度，单位: m
    double x_center_;                                       ///< 路径中心x坐标，单位: m
    double y_center_;                                       ///< 路径中心y坐标，单位: m

    // 运动控制参数
    double update_rate_;                                    ///< 仿真更新频率，单位: Hz
    double translation_velocity_;                           ///< 平移速度，单位: m/s
    double path_total_length_;                              ///< 路径总长度，单位: m

    // 运动状态变量
    ros::Time start_time_;                                  ///< 运动开始时间
    geometry_msgs::Point current_position_;                 ///< 当前位置

    // 运动模式参数
    std::string motion_mode_;                               ///< 运动模式字符串
    std::string rotation_axis_;                             ///< 旋转轴标识
    double angular_velocity_;                               ///< 角速度，单位: rad/s
    double initial_roll_;                                   ///< 初始滚转角，单位: rad
    double initial_pitch_;                                  ///< 初始俯仰角，单位: rad
    double initial_yaw_;                                    ///< 初始偏航角，单位: rad
    Eigen::Vector3d static_position_;                       ///< 静态模式位置，单位: m
    Eigen::Vector3d static_orientation_;                    ///< 静态模式朝向，单位: rad

    // 运行时状态标志
    bool translation_enable_;                               ///< 平移使能标志
    bool rotation_enable_;                                  ///< 旋转使能标志
    bool follow_path_;                                      ///< 路径跟随标志

    // 噪声参数
    bool noise_enable_;                                     ///< 噪声使能标志
    std::string noise_type_;                                ///< 噪声类型: "gaussian" / "uniform"
    double noise_mean_;                                     ///< 噪声均值，单位: mT
    double noise_stddev_;                                   ///< 噪声标准差，单位: mT
    double noise_amplitude_;                                ///< 噪声幅值，单位: mT
};

// ===========================
// 7. 主函数
// ===========================

/**
 * @brief 主函数 - 磁场仿真节点入口点
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char **argv)
{
    // 设置中文本地化支持
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    // 初始化ROS节点
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    
    ROS_INFO("========================================");
    ROS_INFO("磁场仿真节点启动");
    ROS_INFO("========================================");
    
    try {
        // 创建仿真节点实例
        SimulationNode simulation_node(nh);
        
        ROS_INFO("仿真节点初始化完成，开始运行...");
        
        // 进入ROS主循环
        ros::spin();
        
    } catch (const std::exception &e) {
        ROS_ERROR("仿真节点运行异常: %s", e.what());
        return -1;
    }
    
    ROS_INFO("磁场仿真节点正常退出");
    return 0;
}