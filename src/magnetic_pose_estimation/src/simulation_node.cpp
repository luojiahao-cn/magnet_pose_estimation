#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <magnetic_pose_estimation/sensor_config.hpp>
#include <magnetic_pose_estimation/MagneticField.h>
#include <magnetic_pose_estimation/MagnetPose.h>
#include <magnetic_pose_estimation/magnetic_field_calculator.hpp>
#include <random>

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

        // 加载噪声参数
        nh_.param<bool>("simulation_config/noise/enable", noise_enable_, false);
        nh_.param<std::string>("simulation_config/noise/type", noise_type_, std::string("gaussian"));
        nh_.param<double>("simulation_config/noise/mean", noise_mean_, 0.0);
        nh_.param<double>("simulation_config/noise/stddev", noise_stddev_, 1.0);
        nh_.param<double>("simulation_config/noise/amplitude", noise_amplitude_, 1.0);
    }

    void initializeNode()
    {
        // 创建发布器
        magnet_pose_pub_ = nh_.advertise<magnetic_pose_estimation::MagnetPose>("/magnet_pose/simulation", 100);
        magnetic_field_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/raw_data", 100);

        // 生成路径
        generateRectPath();

        // 启动定时器
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &SimulationNode::timerCallback, this);
    }

    /**
     * @brief 生成矩形运动路径
     *
     * 根据设定的宽度和高度参数生成一个矩形路径：
     * - 计算矩形四个顶点坐标
     * - 在顶点之间进行插值，生成密集的路径点
     * - 每条边默认生成20个路径点
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
        magnet_pose.orientation.w = 1.0; // 默认朝向
        magnet_pose.magnetic_strength = magnet_strength_;
        magnet_pose_pub_.publish(magnet_pose);

        // 获取所有传感器
        const auto &sensors = magnetic_pose_estimation::SensorConfig::getInstance().getAllSensors();

        publishMagneticFields(sensors, magnet_pose);
    }

    void publishMagneticFields(const std::vector<magnetic_pose_estimation::SensorInfo> &sensors,
                               const magnetic_pose_estimation::MagnetPose &magnet_pose)
    {
        // 构建传感器位置矩阵
        Eigen::Matrix<double, Eigen::Dynamic, 3> sensor_positions(sensors.size(), 3);
        for (size_t i = 0; i < sensors.size(); ++i)
        {
            sensor_positions(i, 0) = sensors[i].pose.position.x;
            sensor_positions(i, 1) = sensors[i].pose.position.y;
            sensor_positions(i, 2) = sensors[i].pose.position.z;
        }

        // 构建磁铁位置和方向
        Eigen::Vector3d magnetic_position(
            magnet_pose.position.x,
            magnet_pose.position.y,
            magnet_pose.position.z);

        // 计算磁场
        Eigen::MatrixXd magnetic_fields = magnetic_pose_estimation::MagneticFieldCalculator::calculateMagneticField(
            sensor_positions, magnetic_position, magnetic_direction_, magnet_strength_);

        // 噪声生成器
        std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<double> gaussian_dist(noise_mean_, noise_stddev_);
        std::uniform_real_distribution<double> uniform_dist(-noise_amplitude_, noise_amplitude_);

        for (size_t i = 0; i < sensors.size(); ++i)
        {
            magnetic_pose_estimation::MagneticField field_msg;
            field_msg.header.stamp = ros::Time::now();
            field_msg.sensor_id = sensors[i].id;
            field_msg.sensor_pose = sensors[i].pose;

            double nx = 0, ny = 0, nz = 0;
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

            field_msg.mag_x = magnetic_fields(i, 0) + nx;
            field_msg.mag_y = magnetic_fields(i, 1) + ny;
            field_msg.mag_z = magnetic_fields(i, 2) + nz;
            magnetic_field_pub_.publish(field_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher magnet_pose_pub_;
    ros::Publisher magnetic_field_pub_;
    ros::Timer timer_;

    std::vector<geometry_msgs::Point> path_points_;
    size_t current_point_index_ = 0;
    int points_per_side_ = 20;

    double magnet_strength_;
    double rect_width_;
    double rect_height_;
    double rect_z_;
    double update_rate_;
    double x_center_; // 添加中心x坐标
    double y_center_; // 添加中心y坐标
    Eigen::Vector3d magnetic_direction_;

    bool noise_enable_;
    std::string noise_type_;
    double noise_mean_;
    double noise_stddev_;
    double noise_amplitude_;
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