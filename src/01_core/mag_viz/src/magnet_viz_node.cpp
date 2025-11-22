#include <deque>
#include <string>
#include <cmath>
#include <utility>
#include <sstream>
#include <locale>

#include <ros/ros.h>
#include <mag_core_msgs/MagnetPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mag_core_utils/xmlrpc_utils.hpp>

#include <XmlRpcValue.h>

namespace mag_viz
{

// MagnetVisualizer 类：负责磁铁姿态的可视化显示
class MagnetVisualizer
{
public:
    /**
     * @brief 构造函数：初始化磁铁可视化器
     * 从ROS参数服务器加载配置，设置订阅者和发布者，初始化可视化参数
     * @param nh 全局节点句柄
     * @param pnh 私有节点句柄
     */
    MagnetVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)), pnh_(std::move(pnh))
    {
        // 加载配置参数
        XmlRpc::XmlRpcValue config;
        if (!pnh_.getParam("config", config)) {
            ROS_ERROR("[magnet_viz] ✗ 无法获取配置参数");
            ros::shutdown();
            return;
        }

        namespace xml = mag_core_utils::xmlrpc;

        // 加载坐标系设置
        frame_id_ = xml::requireStringField(xml::requireStructField(config, "frames", "config"), "reference_frame", "config.frames");

        // 加载话题配置
        const auto& topics = xml::requireStructField(config, "topics", "config");
        pose_topic_ = xml::requireStringField(topics, "pose", "config.topics");
        estimated_pose_topic_ = xml::requireStringField(topics, "estimated_pose", "config.topics");

        // 加载功能参数
        const auto& params = xml::requireStructField(config, "params", "config");
        enable_estimated_pose_ = xml::requireBoolField(params, "enable_estimated_pose", "config.params");
        trail_size_ = static_cast<size_t>(xml::readNumber(xml::requireMember(params, "trail_size", "config.params"), "config.params/trail_size"));
        trail_duration_ = xml::readNumber(xml::requireMember(params, "trail_duration", "config.params"), "config.params/trail_duration");
        trail_min_distance_ = xml::readNumber(xml::requireMember(params, "trail_min_distance", "config.params"), "config.params/trail_min_distance");
        trail_width_ = xml::readNumber(xml::requireMember(params, "trail_width", "config.params"), "config.params/trail_width");
        estimated_trail_width_ = xml::readNumber(xml::requireMember(params, "estimated_trail_width", "config.params"), "config.params/estimated_trail_width");
        magnet_length_ = xml::readNumber(xml::requireMember(params, "magnet_length", "config.params"), "config.params/magnet_length");
        magnet_radius_ = xml::readNumber(xml::requireMember(params, "magnet_radius", "config.params"), "config.params/magnet_radius");
        marker_lifetime_ = ros::Duration(xml::readNumber(xml::requireMember(params, "marker_lifetime", "config.params"), "config.params/marker_lifetime"));
        strength_text_size_ = xml::readNumber(xml::requireMember(params, "strength_text_size", "config.params"), "config.params/strength_text_size");
        strength_offset_scale_ = xml::readNumber(xml::requireMember(params, "strength_offset_scale", "config.params"), "config.params/strength_offset_scale");
        show_strength_text_ = xml::requireBoolField(params, "show_strength_text", "config.params");
        strength_label_ = xml::requireStringField(params, "strength_label", "config.params");

        // 加载可视化设置
        const auto& visualization = xml::requireStructField(config, "visualization", "config");
        marker_ns_ = xml::requireStringField(visualization, "marker_ns", "config.visualization");
        estimated_marker_ns_ = xml::requireStringField(visualization, "estimated_marker_ns", "config.visualization");

        // 订阅话题
        pose_sub_ = nh_.subscribe(pose_topic_, 10, &MagnetVisualizer::poseCallback, this);
        if (enable_estimated_pose_)
        {
            estimated_pose_sub_ = nh_.subscribe(estimated_pose_topic_, 10, &MagnetVisualizer::estimatedPoseCallback, this);
            estimated_path_pub_ = pnh_.advertise<nav_msgs::Path>("estimated_trail", 1, true);
        }
        // 发布可视化标记和路径
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
        path_pub_ = pnh_.advertise<nav_msgs::Path>("trail", 1, true);
    }

private:
    /**
     * @brief 磁铁姿态消息回调函数
     * 处理接收到的磁铁姿态消息，更新轨迹数据并发布可视化标记
     * @param msg 接收到的磁铁姿态消息
     */
    void poseCallback(const mag_core_msgs::MagnetPoseConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        if (pose.header.frame_id.empty())
        {
            pose.header.frame_id = frame_id_;
        }
        if (pose.header.stamp == ros::Time())
        {
            pose.header.stamp = ros::Time::now();
        }
        pose.pose.position = msg->position;
        pose.pose.orientation = msg->orientation;

        // 更新轨迹数据
        if (shouldAppendSample(trail_, pose))
        {
            trail_.push_back(pose);
            trimTrail(trail_, pose.header.stamp);
        }
        else if (!trail_.empty())
        {
            trail_.back() = pose;
        }

        // 发布可视化标记和路径
        publishMarkers(pose, msg->magnetic_strength);
        publishTrailPath(trail_, path_pub_);
    }

    // 估算姿态消息回调函数
    /**
     * @brief 估算姿态回调函数
     * 处理接收到的估算姿态消息，更新轨迹数据并发布可视化标记
     * @param msg 接收到的估算姿态消息（MagnetPose，包含磁矩强度）
     */
    void estimatedPoseCallback(const mag_core_msgs::MagnetPoseConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position = msg->position;
        pose.pose.orientation = msg->orientation;
        
        if (pose.header.frame_id.empty())
        {
            pose.header.frame_id = frame_id_;
        }
        if (pose.header.stamp == ros::Time())
        {
            pose.header.stamp = ros::Time::now();
        }

        // 更新估算轨迹数据
        if (shouldAppendSample(estimated_trail_, pose))
        {
            estimated_trail_.push_back(pose);
            trimTrail(estimated_trail_, pose.header.stamp);
        }
        else if (!estimated_trail_.empty())
        {
            estimated_trail_.back() = pose;
        }

        // 发布估算可视化标记和路径（传入估计的磁矩强度）
        publishEstimatedMarkers(pose, msg->magnetic_strength);
        if (estimated_path_pub_)
        {
            publishTrailPath(estimated_trail_, estimated_path_pub_);
        }
    }

    /**
     * @brief 判断是否应该添加新的轨迹采样点
     * 根据最小距离阈值判断新姿态是否足够远离上一个采样点
     * @param trail 当前轨迹队列
     * @param pose 新的姿态采样点
     * @return true 如果应该添加新采样点，false 否则
     */
    bool shouldAppendSample(const std::deque<geometry_msgs::PoseStamped> &trail,
                            const geometry_msgs::PoseStamped &pose) const
    {
        if (trail.empty())
        {
            return true;
        }
        if (trail_min_distance_ <= 0.0)
        {
            return true;
        }
        const geometry_msgs::PoseStamped &last = trail.back();
        const double dx = pose.pose.position.x - last.pose.position.x;
        const double dy = pose.pose.position.y - last.pose.position.y;
        const double dz = pose.pose.position.z - last.pose.position.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        return dist >= trail_min_distance_;
    }

    /**
     * @brief 修剪轨迹数据，保持大小和时间限制
     * 移除超出大小限制或时间限制的旧轨迹点
     * @param trail 要修剪的轨迹队列（引用传递，会被修改）
     * @param latest_stamp 最新的时间戳，用于计算轨迹点年龄
     */
    void trimTrail(std::deque<geometry_msgs::PoseStamped> &trail, const ros::Time &latest_stamp) const
    {
        // 限制轨迹点数量
        while (trail.size() > trail_size_)
        {
            trail.pop_front();
        }
        // 限制轨迹时间长度
        if (trail_duration_ > 0.0)
        {
            while (!trail.empty())
            {
                const double age = (latest_stamp - trail.front().header.stamp).toSec();
                if (age <= trail_duration_)
                {
                    break;
                }
                trail.pop_front();
            }
        }
    }

    // 发布磁铁可视化标记
    void publishMarkers(const geometry_msgs::PoseStamped &pose, double magnetic_strength)
    {
        visualization_msgs::MarkerArray array;
        array.markers.reserve(4);
        const ros::Time stamp = pose.header.stamp;

        // 计算磁铁朝向轴
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        if (q.length2() == 0.0)
        {
            q.setW(1.0);
        }
        q.normalize();
        tf2::Vector3 axis = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
        if (axis.length2() == 0.0)
        {
            axis = tf2::Vector3(0.0, 0.0, 1.0);
        }
        axis.normalize();

        const double cylinder_diameter = magnet_radius_ * 2.0;
        const double segment_length = magnet_length_ * 0.5;
        const double offset_length = segment_length * 0.5;
        const double half_length = magnet_length_ * 0.5;

        visualization_msgs::Marker north;
        north.header = pose.header;
        north.ns = marker_ns_;
        north.id = 0;
        north.type = visualization_msgs::Marker::CYLINDER;
        north.action = visualization_msgs::Marker::ADD;
        north.pose = pose.pose;
        north.pose.position.x += axis.x() * offset_length;
        north.pose.position.y += axis.y() * offset_length;
        north.pose.position.z += axis.z() * offset_length;
        north.scale.x = cylinder_diameter;
        north.scale.y = cylinder_diameter;
        north.scale.z = segment_length;
        north.color.r = 1.0;
        north.color.g = 0.15;
        north.color.b = 0.15;
        north.color.a = 0.95;
        north.lifetime = marker_lifetime_;
        array.markers.push_back(north);

        visualization_msgs::Marker south = north;
        south.id = 1;
        south.pose = pose.pose;
        south.pose.position.x -= axis.x() * offset_length;
        south.pose.position.y -= axis.y() * offset_length;
        south.pose.position.z -= axis.z() * offset_length;
        south.color.r = 0.15;
        south.color.g = 0.2;
        south.color.b = 1.0;
        array.markers.push_back(south);

        if (show_strength_text_)
        {
            visualization_msgs::Marker strength;
            strength.header = pose.header;
            strength.ns = marker_ns_;
            strength.id = 2;
            strength.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            strength.action = visualization_msgs::Marker::ADD;
            strength.scale.z = strength_text_size_;
            strength.color.r = 1.0;
            strength.color.g = 1.0;
            strength.color.b = 1.0;
            strength.color.a = 1.0;
            strength.lifetime = marker_lifetime_;
            strength.pose = pose.pose;
            const double text_offset = half_length + magnet_length_ * strength_offset_scale_;
            strength.pose.position.x += axis.x() * text_offset;
            strength.pose.position.y += axis.y() * text_offset;
            strength.pose.position.z += axis.z() * text_offset;

            std::ostringstream ss;
            if (!strength_label_.empty())
            {
                ss << strength_label_ << ": ";
            }
            ss.setf(std::ios::fixed);
            ss.precision(3);
            ss << magnetic_strength << " Am²";  // 磁矩强度单位：安培·米²
            strength.text = ss.str();
            array.markers.push_back(strength);
        }

        if (trail_.size() >= 2)
        {
            visualization_msgs::Marker trail_marker;
            trail_marker.header.frame_id = pose.header.frame_id;
            trail_marker.header.stamp = stamp;
            trail_marker.ns = marker_ns_;
            trail_marker.id = show_strength_text_ ? 3 : 2;
            trail_marker.type = visualization_msgs::Marker::LINE_STRIP;
            trail_marker.action = visualization_msgs::Marker::ADD;
            trail_marker.pose.orientation.w = 1.0;
            trail_marker.scale.x = trail_width_;
            trail_marker.color.r = 1.0;
            trail_marker.color.g = 1.0;
            trail_marker.color.b = 1.0;
            trail_marker.color.a = 0.8;
            trail_marker.points.reserve(trail_.size());
            for (const auto &sample : trail_)
            {
                trail_marker.points.push_back(sample.pose.position);
            }
            array.markers.push_back(trail_marker);
        }

        marker_pub_.publish(array);
    }

    /**
     * @brief 发布估算姿态的可视化标记
     * 创建并发布估算姿态的3D可视化标记，包括磁铁南北极、磁矩强度文本和轨迹线
     * @param pose 估算姿态数据
     * @param magnetic_strength 估计的磁矩强度 (Am²，安培·米²)
     */
    void publishEstimatedMarkers(const geometry_msgs::PoseStamped &pose, double magnetic_strength)
    {
        visualization_msgs::MarkerArray array;
        array.markers.reserve(4);  // 南北极 + 强度文本 + 轨迹
        const ros::Time stamp = pose.header.stamp;

        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        if (q.length2() == 0.0)
        {
            q.setW(1.0);
        }
        q.normalize();
        tf2::Vector3 axis = tf2::quatRotate(q, tf2::Vector3(0.0, 0.0, 1.0));
        if (axis.length2() == 0.0)
        {
            axis = tf2::Vector3(0.0, 0.0, 1.0);
        }
        axis.normalize();

        const double cylinder_diameter = magnet_radius_ * 2.0;
        const double segment_length = magnet_length_ * 0.5;
        const double offset_length = segment_length * 0.5;

        visualization_msgs::Marker north;
        north.header = pose.header;
        north.ns = estimated_marker_ns_;
        north.id = 0;
        north.type = visualization_msgs::Marker::CYLINDER;
        north.action = visualization_msgs::Marker::ADD;
        north.pose = pose.pose;
        north.pose.position.x += axis.x() * offset_length;
        north.pose.position.y += axis.y() * offset_length;
        north.pose.position.z += axis.z() * offset_length;
        north.scale.x = cylinder_diameter;
        north.scale.y = cylinder_diameter;
        north.scale.z = segment_length;
        north.color.r = 0.2f;
        north.color.g = 0.95f;
        north.color.b = 0.5f;
        north.color.a = 0.65f;
        north.lifetime = marker_lifetime_;
        array.markers.push_back(north);

        visualization_msgs::Marker south = north;
        south.id = 1;
        south.pose = pose.pose;
        south.pose.position.x -= axis.x() * offset_length;
        south.pose.position.y -= axis.y() * offset_length;
        south.pose.position.z -= axis.z() * offset_length;
        south.color.r = 0.1f;
        south.color.g = 0.5f;
        south.color.b = 1.0f;
        array.markers.push_back(south);

        // 显示估计磁矩强度文本
        if (show_strength_text_)
        {
            const double half_length = magnet_length_ * 0.5;
            visualization_msgs::Marker strength;
            strength.header = pose.header;
            strength.ns = estimated_marker_ns_;
            strength.id = 2;
            strength.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            strength.action = visualization_msgs::Marker::ADD;
            strength.scale.z = strength_text_size_;
            strength.color.r = 0.2f;
            strength.color.g = 1.0f;
            strength.color.b = 0.8f;
            strength.color.a = 1.0f;
            strength.lifetime = marker_lifetime_;
            strength.pose = pose.pose;
            const double text_offset = half_length + magnet_length_ * strength_offset_scale_;
            strength.pose.position.x += axis.x() * text_offset;
            strength.pose.position.y += axis.y() * text_offset;
            strength.pose.position.z += axis.z() * text_offset;

            std::ostringstream ss;
            if (!strength_label_.empty())
            {
                ss << strength_label_ << "_est: ";
            }
            else
            {
                ss << "Est: ";
            }
            ss.setf(std::ios::fixed);
            ss.precision(3);
            ss << magnetic_strength << " Am²";  // 磁矩强度单位：安培·米²
            strength.text = ss.str();
            array.markers.push_back(strength);
        }

        if (estimated_trail_.size() >= 2)
        {
            visualization_msgs::Marker trail_marker;
            trail_marker.header.frame_id = pose.header.frame_id;
            trail_marker.header.stamp = stamp;
            trail_marker.ns = estimated_marker_ns_;
            trail_marker.id = show_strength_text_ ? 3 : 2;
            trail_marker.type = visualization_msgs::Marker::LINE_STRIP;
            trail_marker.action = visualization_msgs::Marker::ADD;
            trail_marker.pose.orientation.w = 1.0;
            trail_marker.scale.x = estimated_trail_width_;
            trail_marker.color.r = 0.2f;
            trail_marker.color.g = 1.0f;
            trail_marker.color.b = 0.8f;
            trail_marker.color.a = 0.9f;
            trail_marker.points.reserve(estimated_trail_.size());
            for (const auto &sample : estimated_trail_)
            {
                trail_marker.points.push_back(sample.pose.position);
            }
            array.markers.push_back(trail_marker);
        }

        marker_pub_.publish(array);
    }

    /**
     * @brief 发布轨迹路径消息
     * 将轨迹队列转换为nav_msgs::Path消息并发布
     * @param trail 轨迹队列
     * @param pub 要发布到的发布者
     */
    void publishTrailPath(const std::deque<geometry_msgs::PoseStamped> &trail,
                          const ros::Publisher &pub) const
    {
        if (trail.empty())
        {
            return;
        }
        nav_msgs::Path path;
        path.header.frame_id = trail.back().header.frame_id;
        path.header.stamp = ros::Time::now();
        path.poses.assign(trail.begin(), trail.end());
        pub.publish(path);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber estimated_pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher path_pub_;
    ros::Publisher estimated_path_pub_;

    std::deque<geometry_msgs::PoseStamped> trail_;
    std::deque<geometry_msgs::PoseStamped> estimated_trail_;

    std::string pose_topic_;
    std::string frame_id_;
    std::string marker_ns_;
    std::string estimated_pose_topic_;
    std::string estimated_marker_ns_;
    size_t trail_size_;
    double trail_duration_;
    double trail_min_distance_;
    double trail_width_;
    double estimated_trail_width_;
    double magnet_length_;
    double magnet_radius_;
    ros::Duration marker_lifetime_;
    double strength_text_size_;
    double strength_offset_scale_;
    bool show_strength_text_;
    std::string strength_label_;
    bool enable_estimated_pose_;
};

} // namespace mag_viz

int main(int argc, char **argv)
{
    // 设置本地化，支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    ros::init(argc, argv, "magnet_viz_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    mag_viz::MagnetVisualizer visualizer(nh, pnh);
    ros::spin();
    return 0;
}
