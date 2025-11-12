#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mag_sensor_node/sensor_config.hpp>
#include <magnet_msgs/MagSensorData.h>
#include <mag_viz/mag_sensor_viz_node.hpp>

#include <algorithm>
#include <string>
#include <vector>

MagSensorViz::MagSensorViz(ros::NodeHandle &nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    // Load shared sensor configuration so getSensorById() can find IDs.
    if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(nh_)) {
        ROS_WARN("[mag_sensor_viz] 无法加载 sensor_config，后续基于 ID 的查找可能失败");
    } else {
        ROS_INFO_STREAM("[mag_sensor_viz] 已加载 sensor_config: " << mag_sensor_node::SensorConfig::getInstance().getSensorCount() << " 个传感器");
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

void MagSensorViz::onMsg(const magnet_msgs::MagSensorData::ConstPtr &msg)
{
    visualization_msgs::Marker m;
    m.header.stamp = msg->header.stamp;
    m.header.frame_id = target_frame_.empty() ? msg->header.frame_id : target_frame_;
    m.ns = "sensor_field";
    m.id = static_cast<int>(msg->sensor_id);
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.0005;
    m.scale.y = 0.0025;
    m.scale.z = 0.0025;
    m.lifetime = ros::Duration(marker_lifetime_);

    mag_sensor_node::SensorInfo sensor_info;
    if (!mag_sensor_node::SensorConfig::getInstance().getSensorById(msg->sensor_id, sensor_info)) {
        ROS_WARN_THROTTLE(5.0, "Unknown sensor ID: %d", msg->sensor_id);
        return;
    }
    geometry_msgs::Pose pose_in = sensor_info.pose;
    geometry_msgs::Pose pose_tf = pose_in;
    bool did_tf = false;
    try
    {
        if (!target_frame_.empty())
        {
            geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
            tf2::doTransform(pose_in, pose_tf, T);
            did_tf = true;
        }
    }
    catch (const std::exception &e)
    {
        m.header.frame_id = msg->header.frame_id;
        ROS_WARN_THROTTLE(5.0,
                          "TF to target_frame '%s' unavailable (e.g., %s). Falling back to source frame '%s' for sensor %u.",
                          target_frame_.c_str(), e.what(), msg->header.frame_id.c_str(), msg->sensor_id);
    }

    double fx = msg->mag_x, fy = msg->mag_y, fz = msg->mag_z;
    if (did_tf)
    {
        geometry_msgs::Vector3Stamped vin, vout;
        vin.header.frame_id = msg->header.frame_id;
        vin.header.stamp = msg->header.stamp;
        vin.vector.x = fx;
        vin.vector.y = fy;
        vin.vector.z = fz;
        try
        {
            geometry_msgs::TransformStamped T = tf_buffer_.lookupTransform(
                target_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.05));
            tf2::doTransform(vin, vout, T);
            fx = vout.vector.x;
            fy = vout.vector.y;
            fz = vout.vector.z;
        }
        catch (const std::exception &e)
        {
            ROS_WARN_THROTTLE(5.0, "TF for field vector failed; using original vector. (%s)", e.what());
        }
    }
    const double norm = std::sqrt(fx * fx + fy * fy + fz * fz);
    double dx = 1.0, dy = 0.0, dz = 0.0;
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

    m.pose = pose_tf;

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

