#include <deque>
#include <string>
#include <cmath>
#include <utility>
#include <sstream>

#include <ros/ros.h>
#include <mag_core_msgs/MagnetPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mag_viz
{

class MagnetVisualizer
{
public:
    MagnetVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)), pnh_(std::move(pnh))
    {
        pose_topic_ = pnh_.param<std::string>("pose_topic", "/magnetic/pose_true");
        frame_id_ = pnh_.param<std::string>("frame_id", "world");
        marker_ns_ = pnh_.param<std::string>("marker_ns", "magnet");
        estimated_pose_topic_ = pnh_.param<std::string>("estimated_pose_topic", "/magnetic/pose_estimated");
        estimated_marker_ns_ = pnh_.param<std::string>("estimated_marker_ns", "magnet_est");
        enable_estimated_pose_ = pnh_.param<bool>("enable_estimated_pose", true);
        trail_size_ = static_cast<size_t>(pnh_.param<int>("trail_size", 400));
        trail_duration_ = pnh_.param<double>("trail_duration", 30.0);
        trail_min_distance_ = pnh_.param<double>("trail_min_distance", 0.002);
        trail_width_ = pnh_.param<double>("trail_width", 0.004);
        estimated_trail_width_ = pnh_.param<double>("estimated_trail_width", 0.003);
        magnet_length_ = pnh_.param<double>("magnet_length", 0.06);
        magnet_radius_ = pnh_.param<double>("magnet_radius", 0.01);
        marker_lifetime_ = ros::Duration(pnh_.param<double>("marker_lifetime", 0.0));
        strength_text_size_ = pnh_.param<double>("strength_text_size", 0.025);
        strength_offset_scale_ = pnh_.param<double>("strength_offset_scale", 0.6);
        show_strength_text_ = pnh_.param<bool>("show_strength_text", true);
        strength_label_ = pnh_.param<std::string>("strength_label", marker_ns_);

        pose_sub_ = nh_.subscribe(pose_topic_, 10, &MagnetVisualizer::poseCallback, this);
        if (enable_estimated_pose_)
        {
            estimated_pose_sub_ = nh_.subscribe(estimated_pose_topic_, 10, &MagnetVisualizer::estimatedPoseCallback, this);
            estimated_path_pub_ = pnh_.advertise<nav_msgs::Path>("estimated_trail", 1, true);
        }
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
        path_pub_ = pnh_.advertise<nav_msgs::Path>("trail", 1, true);
    }

private:
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

        if (shouldAppendSample(trail_, pose))
        {
            trail_.push_back(pose);
            trimTrail(trail_, pose.header.stamp);
        }
        else if (!trail_.empty())
        {
            trail_.back() = pose;
        }

        publishMarkers(pose, msg->magnetic_strength);
        publishTrailPath(trail_, path_pub_);
    }

    void estimatedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose = *msg;
        if (pose.header.frame_id.empty())
        {
            pose.header.frame_id = frame_id_;
        }
        if (pose.header.stamp == ros::Time())
        {
            pose.header.stamp = ros::Time::now();
        }

        if (shouldAppendSample(estimated_trail_, pose))
        {
            estimated_trail_.push_back(pose);
            trimTrail(estimated_trail_, pose.header.stamp);
        }
        else if (!estimated_trail_.empty())
        {
            estimated_trail_.back() = pose;
        }

        publishEstimatedMarkers(pose);
        if (estimated_path_pub_)
        {
            publishTrailPath(estimated_trail_, estimated_path_pub_);
        }
    }

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

    void trimTrail(std::deque<geometry_msgs::PoseStamped> &trail, const ros::Time &latest_stamp) const
    {
        while (trail.size() > trail_size_)
        {
            trail.pop_front();
        }
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

    void publishMarkers(const geometry_msgs::PoseStamped &pose, double magnetic_strength)
    {
        visualization_msgs::MarkerArray array;
        array.markers.reserve(4);
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
            ss.precision(2);
            ss << magnetic_strength << " T";
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

    void publishEstimatedMarkers(const geometry_msgs::PoseStamped &pose)
    {
        visualization_msgs::MarkerArray array;
        array.markers.reserve(3);
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

        if (estimated_trail_.size() >= 2)
        {
            visualization_msgs::Marker trail_marker;
            trail_marker.header.frame_id = pose.header.frame_id;
            trail_marker.header.stamp = stamp;
            trail_marker.ns = estimated_marker_ns_;
            trail_marker.id = 2;
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
    ros::init(argc, argv, "magnet_viz_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    mag_viz::MagnetVisualizer visualizer(nh, pnh);
    ros::spin();
    return 0;
}
