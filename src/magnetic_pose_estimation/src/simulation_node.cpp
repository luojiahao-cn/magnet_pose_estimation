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
     *
     * 初始化模拟节点，包括：
     * - 加载传感器配置
     * - 设置磁铁和路径参数
     * - 创建发布器
     * - 生成矩形运动路径
     * - 启动定时器
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
     * @brief 加载仿真参数，包括磁铁参数、路径参数、噪声参数
     */
    void loadParameters()
    {
        // 加载磁铁参数
        nh_.param<double>("simulation_config/magnet/strength", magnet_strength_, 100.0); // 单位: A·m²
        std::vector<double> direction;
        nh_.param("simulation_config/magnet/direction", direction, std::vector<double>{0, 0, 1}); // 单位: 无量纲
        magnetic_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]);

        // 加载路径参数
        nh_.param<double>("simulation_config/path/width", rect_width_, 0.2);         // 单位: m
        nh_.param<double>("simulation_config/path/height", rect_height_, 0.2);       // 单位: m
        nh_.param<double>("simulation_config/path/z", rect_z_, 0.1);                 // 单位: m
        nh_.param<double>("simulation_config/path/center/x", x_center_, 0.2);        // 单位: m
        nh_.param<double>("simulation_config/path/center/y", y_center_, 0.2);        // 单位: m
        nh_.param<double>("simulation_config/path/update_rate", update_rate_, 10.0); // 单位: Hz
        nh_.param<int>("simulation_config/path/points_per_side", points_per_side_, 20);

        // 加载噪声参数
        nh_.param<bool>("simulation_config/noise/enable", noise_enable_, false);
        nh_.param<std::string>("simulation_config/noise/type", noise_type_, std::string("gaussian"));
        nh_.param<double>("simulation_config/noise/mean", noise_mean_, 0.0);           // 单位: mT
        nh_.param<double>("simulation_config/noise/stddev", noise_stddev_, 1.0);       // 单位: mT
        nh_.param<double>("simulation_config/noise/amplitude", noise_amplitude_, 1.0); // 单位: mT
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
     *
     * 根据设定的宽度和高度参数生成一个矩形路径：
     * - 计算矩形四个顶点坐标
     * - 在顶点之间进行插值，生成密集的路径点
     * - 每条边默认生成points_per_side_个路径点
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
     *
     * 该函数定期执行以下操作：
     * 1. 更新磁铁在预设矩形路径上的位置
     * 2. 发布磁铁的当前位姿信息，包括：
     *    - 位置（来自路径点）
     *    - 朝向（默认为z轴正方向）
     *    - 磁场强度
     * 3. 获取所有传感器的位置信息
     * 4. 计算每个传感器位置处的磁场：
     *    - 构建传感器位置矩阵
     *    - 计算磁铁位置和方向向量
     *    - 使用 MagneticFieldCalculator 计算磁场
     * 5. 发布每个传感器检测到的磁场数据
     */
    void timerCallback(const ros::TimerEvent &)
    {
        if (path_points_.empty())
            return;

        // 更新当前位置
        current_point_index_ = (current_point_index_ + 1) % path_points_.size();

        // 发布磁铁位姿
        magnetic_pose_estimation::MagnetPose magnet_pose;
        magnet_pose.header.stamp = ros::Time::now();
        magnet_pose.position = path_points_[current_point_index_];

        // 先绕x轴偏转pitch，再绕z轴旋转yaw
        double pitch = 0.5; // x轴偏转角度（弧度），可自定义
        double omega_rad_per_step = 0.1; // z轴每步旋转角速度
        double yaw = current_point_index_ * omega_rad_per_step;

        tf2::Quaternion q;
        q.setRPY(0, pitch, yaw); // RPY顺序为(roll, pitch, yaw)，roll为x轴，pitch为y轴，yaw为z轴
        magnet_pose.orientation.x = q.x();
        magnet_pose.orientation.y = q.y();
        magnet_pose.orientation.z = q.z();
        magnet_pose.orientation.w = q.w();

        magnet_pose.magnetic_strength = magnet_strength_;
        magnet_pose_pub_.publish(magnet_pose);

        // 获取所有传感器
        const auto &sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();

        // 发布每个传感器的磁场数据
        publishMagneticFields(sensors, magnet_pose);
    }

    /**
     * @brief 计算并发布所有传感器的磁场数据
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
};

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出状态
 *
 * - 设置中文本地化
 * - 初始化ROS节点
 * - 创建并运行模拟节点
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