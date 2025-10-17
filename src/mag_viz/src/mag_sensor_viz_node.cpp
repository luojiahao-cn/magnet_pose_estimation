#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mag_sensor_node/MagSensorData.h>
#include <mag_viz/mag_sensor_viz_node.hpp>

#include <algorithm>
#include <string>
#include <vector>

MagSensorViz::MagSensorViz(ros::NodeHandle &nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    // 优先读私有参数，其次集中配置 /viz_config/* 与 /frames/global_frame
    nh_.param<std::string>("topic", topic_, std::string("/mag_sensor/data_mT"));
    nh_.param<std::string>("frame", target_frame_, std::string(""));
    nh_.param<std::string>("marker_topic", marker_topic_, std::string("/mag_viz/markers"));
    ros::NodeHandle gnh;
    gnh.param<std::string>("/viz_config/topic", topic_, topic_);
    std::string global_frame_param;
    gnh.param<std::string>("/frames/global_frame", global_frame_param, std::string(""));
    if (target_frame_.empty() && !global_frame_param.empty())
        target_frame_ = global_frame_param;
    nh_.param("field_scale", field_scale_, 0.005);
    nh_.param("marker_lifetime", marker_lifetime_, 0.1);
    nh_.param("color_max", color_max_, 3.2);

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

void MagSensorViz::onMsg(const mag_sensor_node::MagSensorData::ConstPtr &msg)
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

    geometry_msgs::Pose pose_in = msg->sensor_pose;
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

