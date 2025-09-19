#include <geometry_msgs/TransformStamped.h>
#include <mag_sensor_node/MagSensorData.h>
#include <mag_sensor_node/MagnetPose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <algorithm>
#include <mag_sensor_node/sensor_config.hpp>
#include <map>

class MagneticFieldVisualizer
{
public:
    explicit MagneticFieldVisualizer(ros::NodeHandle &nh) : nh_(nh)
    {
        if (!mag_sensor_node::SensorConfig::getInstance().loadConfig(nh))
        {
            ROS_ERROR("无法加载传感器配置");
            return;
        }

        // 初始化可视化参数
        nh_.param("/visualization_config/magnetic_field_marker/field_scale", field_scale_, 0.005);
        nh_.param("/visualization_config/magnetic_field_marker/lifetime", lifetime_, 0.1);
        nh_.param("/color_map/max", color_max_, 3.2);

        // 初始化标记
        initializeMarkerTemplates();

        // 等待1s钟以确保所有话题都已发布
        ros::Duration(1.0).sleep();

        // 订阅主题并设置回调
        subscribeToTopics();
    }

private:
    void subscribeToTopics()
    {
        // 获取所有 /magnetic_field/ 下的话题
        ros::master::V_TopicInfo topic_info;
        ros::master::getTopics(topic_info);

        for (const auto &topic : topic_info)
        {
            // 处理磁场话题
            if (topic.name.find("/magnetic_field/") == 0)
            {
                std::string topic_name = topic.name;
                std::string source = topic_name.substr(std::string("/magnetic_field/").length());

                // 为每个源创建发布器
                magnetic_field_pubs_[source] =
                    nh_.advertise<visualization_msgs::MarkerArray>("magnetic_field_markers/" + source, 25);

                // 统一使用 MagSensorData
                magnetic_field_subs_[source] = nh_.subscribe<mag_sensor_node::MagSensorData>(
                    topic_name, 25, boost::bind(&MagneticFieldVisualizer::magneticFieldCallback, this, _1, source));

                ROS_INFO("订阅磁场话题: %s", topic_name.c_str());
            }
            // 处理磁铁位姿话题
            else if (topic.name.find("/magnet_pose/") == 0)
            {
                std::string topic_name = topic.name;
                std::string source = topic_name.substr(std::string("/magnet_pose/").length());

                magnet_marker_pubs_[source] = nh_.advertise<visualization_msgs::MarkerArray>("magnet_marker/" + source, 10);

                magnet_pose_subs_[source] = nh_.subscribe<mag_sensor_node::MagnetPose>(
                    topic_name, 10, boost::bind(&MagneticFieldVisualizer::magnetPoseCallback, this, _1, source));

                ROS_INFO("订阅磁铁位姿话题: %s", topic_name.c_str());
            }
        }
    }

    void initializeMarkerTemplates()
    {
        // 传感器箭头
        sensor_marker_template_.header.frame_id = "world";
        sensor_marker_template_.type = visualization_msgs::Marker::ARROW;
        sensor_marker_template_.action = visualization_msgs::Marker::ADD;

        double shaft_diameter = 0.0005, head_diameter = 0.0025;
        nh_.param("/visualization_config/magnetic_field_marker/arrow/shaft_diameter", shaft_diameter, 0.0005);
        nh_.param("/visualization_config/magnetic_field_marker/arrow/head_diameter", head_diameter, 0.0025);

        sensor_marker_template_.scale.x = shaft_diameter;
        sensor_marker_template_.scale.y = head_diameter;
        sensor_marker_template_.scale.z = head_diameter;

        std::vector<double> color{1.0, 1.0, 0.0, 1.0};
        nh_.param("/visualization_config/magnetic_field_marker/color", color, color);
        sensor_marker_template_.color.r = color[0];
        sensor_marker_template_.color.g = color[1];
        sensor_marker_template_.color.b = color[2];
        sensor_marker_template_.color.a = color[3];

        // 磁铁 marker
        initializeMagnetMarker();
    }

    void initializeMagnetMarker()
    {
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = "magnet";
        magnet_marker_.id = 0;
        magnet_marker_.type = visualization_msgs::Marker::CYLINDER;
        magnet_marker_.action = visualization_msgs::Marker::ADD;

        double scale_x = 0.001, scale_y = 0.001, scale_z = 0.002;
        nh_.param("/visualization_config/magnet_marker/scale/x", scale_x, 0.001);
        nh_.param("/visualization_config/magnet_marker/scale/y", scale_y, 0.001);
        nh_.param("/visualization_config/magnet_marker/scale/z", scale_z, 0.002);
        magnet_marker_.scale.x = scale_x;
        magnet_marker_.scale.y = scale_y;
        magnet_marker_.scale.z = scale_z;

        std::vector<double> color{1.0, 0.0, 0.0, 1.0};
        nh_.param("/visualization_config/magnet_marker/color", color, color);
        magnet_marker_.color.r = color[0];
        magnet_marker_.color.g = color[1];
        magnet_marker_.color.b = color[2];
        magnet_marker_.color.a = color[3];
    }

    void magneticFieldCallback(const mag_sensor_node::MagSensorData::ConstPtr &msg, const std::string &source)
    {
        updateMarkers(msg, source);
    }

    void magnetPoseCallback(const mag_sensor_node::MagnetPose::ConstPtr &msg, const std::string &source)
    {
        updateMagnetMarker(msg, magnet_marker_pubs_[source], "magnet_" + source);
    }

    void updateMarkers(const mag_sensor_node::MagSensorData::ConstPtr &msg, const std::string &source)
    {
        visualization_msgs::Marker marker = sensor_marker_template_;
        marker.header.stamp = ros::Time::now();
        marker.ns = source;
        marker.id = msg->sensor_id;

        Eigen::Vector3d field(msg->mag_x, msg->mag_y, msg->mag_z);
        Eigen::Vector3d direction = field.norm() > 1e-6 ? field.normalized() : Eigen::Vector3d(1, 0, 0);
        marker.points.resize(2);
        marker.points[0].x = marker.points[0].y = marker.points[0].z = 0.0;
        marker.points[1].x = direction.x() * field_scale_;
        marker.points[1].y = direction.y() * field_scale_;
        marker.points[1].z = direction.z() * field_scale_;

        marker.pose.position = msg->sensor_pose.position;
        marker.pose.orientation = msg->sensor_pose.orientation;
        marker.lifetime = ros::Duration(lifetime_);

        double norm_val = std::min(field.norm(), color_max_);
        double ratio = norm_val / color_max_;

        if (ratio <= 0.5)
        {
            marker.color.r = 0.0;
            marker.color.g = ratio * 2.0;
            marker.color.b = 1.0 - ratio * 2.0;
        }
        else
        {
            marker.color.r = (ratio - 0.5) * 2.0;
            marker.color.g = 1.0 - (ratio - 0.5) * 2.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 1.0;

        visualization_msgs::MarkerArray current_markers;
        current_markers.markers.push_back(marker);
        magnetic_field_pubs_[source].publish(current_markers);

        geometry_msgs::TransformStamped sensor_tf;
        sensor_tf.header.stamp = ros::Time::now();
        sensor_tf.header.frame_id = "world";
        sensor_tf.child_frame_id = "sensor_" + std::to_string(msg->sensor_id);

        sensor_tf.transform.translation.x = msg->sensor_pose.position.x;
        sensor_tf.transform.translation.y = msg->sensor_pose.position.y;
        sensor_tf.transform.translation.z = msg->sensor_pose.position.z;
        sensor_tf.transform.rotation = msg->sensor_pose.orientation;

        tf_broadcaster_.sendTransform(sensor_tf);
    }

    void updateMagnetMarker(const mag_sensor_node::MagnetPose::ConstPtr &msg,
                            const ros::Publisher &publisher,
                            const std::string &frame_id)
    {
        magnet_marker_.header.stamp = msg->header.stamp;
        magnet_marker_.header.frame_id = "world";
        magnet_marker_.ns = frame_id;
        magnet_marker_.pose.position = msg->position;
        magnet_marker_.pose.orientation = msg->orientation;

        visualization_msgs::Marker &traj_marker = magnet_trajectories_[frame_id];
        traj_marker.header.stamp = msg->header.stamp;
        traj_marker.header.frame_id = "world";
        traj_marker.ns = frame_id + "_traj";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::Marker::ADD;
        traj_marker.scale.x = 0.0002;
        traj_marker.color.r = 1.0;
        traj_marker.color.g = 0.0;
        traj_marker.color.b = 0.0;
        traj_marker.color.a = 1.0;
        traj_marker.pose.orientation.w = 1.0;
        traj_marker.pose.orientation.x = 0.0;
        traj_marker.pose.orientation.y = 0.0;
        traj_marker.pose.orientation.z = 0.0;
        traj_marker.lifetime = ros::Duration(60);

        traj_marker.points.push_back(msg->position);

        size_t max_traj_length = 500;
        if (traj_marker.points.size() > max_traj_length)
        {
            traj_marker.points.erase(traj_marker.points.begin(),
                                     traj_marker.points.begin() + (traj_marker.points.size() - max_traj_length));
        }

        visualization_msgs::MarkerArray magnet_markers;
        magnet_markers.markers.push_back(magnet_marker_);
        magnet_markers.markers.push_back(traj_marker);
        publisher.publish(magnet_markers);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = "world";
        transform.child_frame_id = frame_id;

        transform.transform.translation.x = msg->position.x;
        transform.transform.translation.y = msg->position.y;
        transform.transform.translation.z = msg->position.z;
        transform.transform.rotation = msg->orientation;

        tf_broadcaster_.sendTransform(transform);
    }

    ros::NodeHandle &nh_;
    std::map<std::string, ros::Publisher> magnetic_field_pubs_;
    std::map<std::string, ros::Publisher> magnet_marker_pubs_;
    std::map<std::string, ros::Subscriber> magnetic_field_subs_;
    std::map<std::string, ros::Subscriber> magnet_pose_subs_;
    std::map<std::string, visualization_msgs::Marker> magnet_trajectories_;

    visualization_msgs::Marker sensor_marker_template_;
    visualization_msgs::Marker magnet_marker_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double field_scale_ = 0.005;
    double lifetime_ = 0.1;
    double color_max_ = 3.2;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "magnetic_field_visualizer");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(50);
    spinner.start();

    MagneticFieldVisualizer visualizer(nh);
    ros::waitForShutdown();
    return 0;
}
