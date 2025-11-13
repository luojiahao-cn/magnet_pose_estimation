#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <magnet_msgs/MagSensorData.h>
#include <mag_viz/mag_sensor_viz_node.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

MagSensorViz::MagSensorViz(ros::NodeHandle &nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    try
    {
        sensor_array_.load(nh_, "array");
        sensor_array_loaded_ = true;
        ROS_INFO_STREAM("[mag_sensor_viz] 已加载阵列配置: " << sensor_array_.sensors().size() << " 个传感器");
    }
    catch (const std::exception &e)
    {
        sensor_array_loaded_ = false;
        ROS_WARN("[mag_sensor_viz] 无法加载 array 配置: %s", e.what());
    }
    // 仅从私有命名空间读取；缺少即报错
    auto require = [&](const std::string &key, auto &var) {
        if (!nh_.getParam(key, var)) throw std::runtime_error(std::string("缺少参数: ~") + key);
    };

    require("topic", topic_);
    require("frame", target_frame_);
    require("marker_topic", marker_topic_);
    require("field_scale", field_scale_);
    require("marker_lifetime", marker_lifetime_);
    require("color_max", color_max_);

    zero_color_ = {0.0, 0.0, 1.0, 1.0};

    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 50);
    sub_ = nh_.subscribe(topic_, 50, &MagSensorViz::onMsg, this);

    ROS_INFO_STREAM("[mag_sensor_viz] topic='" << topic_ << "', marker='" << marker_topic_
                    << "', target_frame='" << (target_frame_.empty()?"<source>":target_frame_) << "'");
}

Color MagSensorViz::lerp(const Color &a, const Color &b, double t)
{
    Color c;
    c.r = a.r + (b.r - a.r) * t;
    c.g = a.g + (b.g - a.g) * t;
    c.b = a.b + (b.b - a.b) * t;
    c.a = a.a + (b.a - a.a) * t;
    return c;
}

Color MagSensorViz::colormap(double mag) const
{
    double m = std::max(0.0, std::min(mag, color_max_));
    double t = (color_max_ > 1e-9) ? (m / color_max_) : 0.0; // 0..1
    if (t <= 0.5)
        return lerp({0, 0, 1, 1}, {0, 1, 0, 1}, t * 2.0);
    else
        return lerp({0, 1, 0, 1}, {1, 0, 0, 1}, (t - 0.5) * 2.0);
}

// 处理单个传感器数据消息，发布对应的可视化 Marker
void MagSensorViz::onMsg(const magnet_msgs::MagSensorData::ConstPtr &msg)
{
    visualization_msgs::Marker m;
    m.header.stamp = msg->header.stamp;
    m.ns = "sensor_field";
    m.id = static_cast<int>(msg->sensor_id);
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.0005;
    m.scale.y = 0.0025;
    m.scale.z = 0.0025;
    m.lifetime = ros::Duration(marker_lifetime_);

    if (!sensor_array_loaded_)
    {
        ROS_WARN_THROTTLE(5.0, "[mag_sensor_viz] 阵列配置未加载，忽略传感器 %u", msg->sensor_id);
        return;
    }

    const auto *entry = sensor_array_.findSensor(static_cast<int>(msg->sensor_id));
    if (!entry)
    {
        ROS_WARN_THROTTLE(5.0, "[mag_sensor_viz] 未知传感器 id=%u", msg->sensor_id);
        return;
    }

    const std::string sensor_frame = entry->frame_id.empty() ? msg->header.frame_id : entry->frame_id;

    geometry_msgs::PoseStamped pose_in;
    pose_in.header.frame_id = sensor_frame;
    pose_in.header.stamp = msg->header.stamp;
    pose_in.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped pose_out = pose_in;
    geometry_msgs::TransformStamped sensor_to_target;
    bool has_transform = false;

    if (!target_frame_.empty())
    {
        try
        {
            sensor_to_target = tf_buffer_.lookupTransform(
                target_frame_, sensor_frame, msg->header.stamp, ros::Duration(0.05));
            tf2::doTransform(pose_in, pose_out, sensor_to_target);
            has_transform = true;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(5.0,
                              "[mag_sensor_viz] 无法获取 %s -> %s 的 TF (%s)，回退到传感器坐标系",
                              target_frame_.c_str(), sensor_frame.c_str(), e.what());
        }
    }

    double fx = msg->mag_x;
    double fy = msg->mag_y;
    double fz = msg->mag_z;

    if (has_transform)
    {
        geometry_msgs::Vector3Stamped vin, vout;
        vin.header.frame_id = sensor_frame;
        vin.header.stamp = msg->header.stamp;
        vin.vector.x = fx;
        vin.vector.y = fy;
        vin.vector.z = fz;

        try
        {
            tf2::doTransform(vin, vout, sensor_to_target);
            fx = vout.vector.x;
            fy = vout.vector.y;
            fz = vout.vector.z;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(5.0, "[mag_sensor_viz] 磁场向量转换失败，使用原始值 (%s)", e.what());
            has_transform = false;
        }
    }

    const double norm = std::sqrt(fx * fx + fy * fy + fz * fz);
    double dx = 1.0;
    double dy = 0.0;
    double dz = 0.0;
    if (norm > 1e-9)
    {
        dx = fx / norm;
        dy = fy / norm;
        dz = fz / norm;
    }

    m.points.resize(2);
    m.points[0].x = 0.0;
    m.points[0].y = 0.0;
    m.points[0].z = 0.0;
    m.points[1].x = dx * field_scale_;
    m.points[1].y = dy * field_scale_;
    m.points[1].z = dz * field_scale_;

    m.header.frame_id = has_transform ? target_frame_ : sensor_frame;
    m.pose = has_transform ? pose_out.pose : pose_in.pose;

    Color c = (norm > 1e-12) ? colormap(norm) : zero_color_;
    m.color.r = c.r;
    m.color.g = c.g;
    m.color.b = c.b;
    m.color.a = c.a;

    visualization_msgs::MarkerArray arr;
    arr.markers.push_back(m);
    pub_.publish(arr);
}

int main(int argc, char** argv)
{
	setlocale(LC_ALL, "zh_CN.UTF-8");
	ros::init(argc, argv, "mag_sensor_viz");
	ros::NodeHandle nh("~");
	MagSensorViz node(nh);
	ros::spin();
	return 0;
}

