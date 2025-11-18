#include <mag_core_description/sensor_array_config_loader.hpp>
#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mag_viz
{
namespace
{
constexpr double kEpsilon = 1e-6;

double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
}

geometry_msgs::Point toPoint(const tf2::Vector3 &vec)
{
    geometry_msgs::Point point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    return point;
}

std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(clampValue(r, 0.0, 1.0));
    color.g = static_cast<float>(clampValue(g, 0.0, 1.0));
    color.b = static_cast<float>(clampValue(b, 0.0, 1.0));
    color.a = static_cast<float>(clampValue(a, 0.0, 1.0));
    return color;
}

} // namespace

class SensorArrayVisualizer
{
public:
    SensorArrayVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)), pnh_(std::move(pnh))
    {
        loadParameters();
        loadSensorArray();

        measurement_sub_ = nh_.subscribe(measurement_topic_, 100, &SensorArrayVisualizer::onMeasurement, this);
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("sensor_markers", 1, true);

        const double period = publish_rate_ > 0.0 ? (1.0 / publish_rate_) : 0.05;
        timer_ = nh_.createTimer(ros::Duration(period), &SensorArrayVisualizer::onTimer, this);

        publishMarkers(ros::Time::now());
    }

private:
    struct SensorVisual
    {
        int id{0};
        std::string frame_id;
        tf2::Transform tf_parent_sensor;
        tf2::Vector3 up_axis;
    };

    struct MeasurementState
    {
        ros::Time stamp;
        tf2::Vector3 value_sensor;
        tf2::Vector3 value_parent;
    };

    void loadParameters()
    {
        measurement_topic_ = pnh_.param<std::string>("measurement_topic", "/magnetic/sensor/field");
        array_param_key_ = pnh_.param<std::string>("array_param", "array");
        fixed_frame_ = pnh_.param<std::string>("fixed_frame", "world");
        publish_rate_ = pnh_.param<double>("publish_rate", 20.0);

        field_ns_ = pnh_.param<std::string>("field_marker_ns", "sensor_field");
        text_ns_ = pnh_.param<std::string>("text_marker_ns", "sensor_text");

        marker_lifetime_ = ros::Duration(pnh_.param<double>("marker_lifetime", 0.0));

        arrow_shaft_diameter_ = pnh_.param<double>("arrow_shaft_diameter", 0.002);
        arrow_head_diameter_ = pnh_.param<double>("arrow_head_diameter", 0.006);
        arrow_head_length_ = pnh_.param<double>("arrow_head_length", 0.012);
        vector_scale_ = pnh_.param<double>("vector_scale", 0.004);
        max_arrow_length_ = pnh_.param<double>("max_arrow_length", 0.1);
        arrow_origin_offset_ = pnh_.param<double>("arrow_origin_offset", 0.0);
        measurement_timeout_ = pnh_.param<double>("measurement_timeout", 0.3);
        color_min_mT_ = pnh_.param<double>("color_min_mT", 0.0);
        color_max_mT_ = pnh_.param<double>("color_max_mT", 3.0);
        show_text_ = pnh_.param<bool>("show_text", false);
        text_scale_ = pnh_.param<double>("text_scale", 0.018);
        text_offset_ = pnh_.param<double>("text_offset", 0.01);

        stale_color_ = makeColor(0.55, 0.55, 0.55, 0.5);
    }

    void loadSensorArray()
    {
        namespace rps = rosparam_shortcuts;
        const std::string ns = "mag_viz.sensor_array";

        std::string param_key = array_param_key_;
        if (!param_key.empty() && param_key.back() == '/')
        {
            param_key.pop_back();
        }
        const std::string config_key = param_key.empty() ? std::string("config") : param_key + "/config";

        XmlRpc::XmlRpcValue array_param;
        std::size_t error = 0;
        error += !rps::get(ns, pnh_, config_key, array_param);
        rps::shutdownIfError(ns, error);

        auto config = mag_core_description::loadSensorArrayConfig(array_param, ns + ".config");
        mag_core_description::SensorArrayDescription description;
        description.load(config);
        if (fixed_frame_.empty())
        {
            fixed_frame_ = description.parentFrame();
        }
        if (fixed_frame_ != description.parentFrame())
        {
            ROS_WARN_STREAM_THROTTLE(10.0,
                                     "[sensor_array_viz] fixed_frame '" << fixed_frame_
                                                                  << "' 与描述中的 parent_frame '"
                                                                  << description.parentFrame()
                                                                  << "' 不一致，将仍以 parent_frame 发布 marker");
            fixed_frame_ = description.parentFrame();
        }

        tf2::Transform parent_array;
        tf2::fromMsg(description.arrayPose(), parent_array);

        sensors_.clear();
        sensor_index_.clear();
        sensors_.reserve(description.sensors().size());

        for (const auto &sensor_entry : description.sensors())
        {
            SensorVisual visual;
            visual.id = sensor_entry.id;
            visual.frame_id = sensor_entry.frame_id;

            tf2::Transform array_sensor;
            tf2::fromMsg(sensor_entry.pose, array_sensor);
            visual.tf_parent_sensor = parent_array * array_sensor;
            visual.up_axis = visual.tf_parent_sensor.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
            const double up_norm = visual.up_axis.length();
            if (up_norm > kEpsilon)
            {
                visual.up_axis /= up_norm;
            }
            else
            {
                visual.up_axis = tf2::Vector3(0.0, 0.0, 1.0);
            }
            sensors_.push_back(visual);
            sensor_index_.emplace(visual.id, sensors_.size() - 1);
        }
        ROS_INFO_STREAM("[sensor_array_viz] loaded " << sensors_.size() << " sensor poses from parameter '~" << config_key << "'.");
    }

    void onMeasurement(const mag_core_msgs::MagSensorDataConstPtr &msg)
    {
        const auto it = sensor_index_.find(static_cast<int>(msg->sensor_id));
        if (it == sensor_index_.end())
        {
            ROS_WARN_THROTTLE(5.0, "[sensor_array_viz] unknown sensor id %u", msg->sensor_id);
            return;
        }
        const SensorVisual &sensor = sensors_.at(it->second);

        MeasurementState state;
        state.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        state.value_sensor = tf2::Vector3(msg->mag_x, msg->mag_y, msg->mag_z);
        state.value_parent = sensor.tf_parent_sensor.getBasis() * state.value_sensor;
        measurements_[sensor.id] = state;
    }

    void onTimer(const ros::TimerEvent &event)
    {
        publishMarkers(event.current_real);
    }

    const MeasurementState *findMeasurement(int sensor_id, const ros::Time &stamp) const
    {
        const auto it = measurements_.find(sensor_id);
        if (it == measurements_.end())
        {
            return nullptr;
        }
        if (measurement_timeout_ > 0.0 && (stamp - it->second.stamp).toSec() > measurement_timeout_)
        {
            return nullptr;
        }
        return &it->second;
    }

    std_msgs::ColorRGBA colorForMagnitude(double magnitude_mT) const
    {
        if (color_max_mT_ <= color_min_mT_)
        {
            return makeColor(1.0, 0.8, 0.2, 1.0);
        }
        const double t = clampValue((magnitude_mT - color_min_mT_) / (color_max_mT_ - color_min_mT_), 0.0, 1.0);
        // 简单的蓝->红渐变，居中的时候偏黄
        const double r = t;
        const double b = 1.0 - t;
        const double g = clampValue(1.0 - std::fabs(t - 0.5) * 2.0, 0.0, 1.0);
        return makeColor(r, g, b, 1.0);
    }

    std::string formatMeasurementText(int sensor_id, const MeasurementState *state) const
    {
        std::ostringstream ss;
        ss.setf(std::ios::fixed);
        ss.precision(2);
        ss << "S" << sensor_id << " | ";
        if (!state)
        {
            ss << "--";
            return ss.str();
        }
        ss << state->value_sensor.x() << ", "
           << state->value_sensor.y() << ", "
           << state->value_sensor.z() << " mT";
        return ss.str();
    }

    visualization_msgs::Marker makeMeasurementArrow(const SensorVisual &sensor,
                                                    const MeasurementState *state,
                                                    const ros::Time &stamp) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = stamp;
        marker.ns = field_ns_;
        marker.id = sensor.id * marker_stride_;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = arrow_shaft_diameter_;
        marker.scale.y = arrow_head_diameter_;
        marker.scale.z = arrow_head_length_;
        marker.lifetime = marker_lifetime_;

        tf2::Vector3 start_vec = sensor.tf_parent_sensor.getOrigin();
        if (arrow_origin_offset_ > 0.0)
        {
            start_vec += sensor.up_axis * arrow_origin_offset_;
        }
        tf2::Vector3 end_vec = start_vec;
        std_msgs::ColorRGBA color = stale_color_;

        if (state)
        {
            const double magnitude = state->value_parent.length();
            if (magnitude > kEpsilon)
            {
                double length = vector_scale_ * magnitude;
                if (max_arrow_length_ > 0.0)
                {
                    length = std::min(length, max_arrow_length_);
                }
                const tf2::Vector3 direction = state->value_parent.normalized() * length;
                end_vec = start_vec + direction;
                color = colorForMagnitude(magnitude);
            }
            else
            {
                end_vec = start_vec + sensor.up_axis * 0.001;
            }
        }
        else
        {
            end_vec = start_vec + sensor.up_axis * 0.001;
        }

        marker.points.push_back(toPoint(start_vec));
        marker.points.push_back(toPoint(end_vec));
        marker.color = color;
        if (!state)
        {
            marker.color.a = stale_color_.a;
        }
        return marker;
    }

    visualization_msgs::Marker makeMeasurementText(const SensorVisual &sensor,
                                                   const MeasurementState *state,
                                                   const ros::Time &stamp) const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = fixed_frame_;
        marker.header.stamp = stamp;
        marker.ns = text_ns_;
        marker.id = sensor.id * marker_stride_ + 1;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.z = text_scale_;
        marker.lifetime = marker_lifetime_;

        tf2::Vector3 anchor = sensor.tf_parent_sensor.getOrigin();
        const double total_offset = std::max(text_offset_, 0.0) + (arrow_origin_offset_ > 0.0 ? arrow_origin_offset_ : 0.0);
        anchor += sensor.up_axis * (total_offset + text_scale_ * 0.5);
        marker.pose.position = toPoint(anchor);
        marker.pose.orientation.w = 1.0;
        marker.text = formatMeasurementText(sensor.id, state);
        marker.color = state ? makeColor(1.0, 1.0, 1.0, 0.95) : makeColor(0.7, 0.7, 0.7, 0.6);
        return marker;
    }

    void publishMarkers(const ros::Time &stamp)
    {
        if (sensors_.empty())
        {
            ROS_WARN_THROTTLE(5.0, "[sensor_array_viz] no sensor description available, skip visualization");
            return;
        }

        visualization_msgs::MarkerArray array;
        array.markers.reserve(sensors_.size() * (show_text_ ? 2 : 1));

        for (const auto &sensor : sensors_)
        {
            const MeasurementState *state = findMeasurement(sensor.id, stamp);
            array.markers.push_back(makeMeasurementArrow(sensor, state, stamp));
            if (show_text_)
            {
                array.markers.push_back(makeMeasurementText(sensor, state, stamp));
            }
        }

        marker_pub_.publish(array);
    }

    static constexpr int marker_stride_ = 10;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber measurement_sub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;

    std::string measurement_topic_;
    std::string array_param_key_;
    std::string fixed_frame_;
    double publish_rate_{20.0};

    std::string field_ns_;
    std::string text_ns_;

    ros::Duration marker_lifetime_;

    double arrow_shaft_diameter_{0.002};
    double arrow_head_diameter_{0.006};
    double arrow_head_length_{0.012};
    double vector_scale_{0.004};
    double max_arrow_length_{0.1};
    double arrow_origin_offset_{0.0};
    double measurement_timeout_{0.3};
    double color_min_mT_{0.0};
    double color_max_mT_{3.0};
    bool show_text_{false};
    double text_scale_{0.018};
    double text_offset_{0.01};

    std_msgs::ColorRGBA stale_color_;

    std::vector<SensorVisual> sensors_;
    std::unordered_map<int, std::size_t> sensor_index_;
    std::unordered_map<int, MeasurementState> measurements_;
};

} // namespace mag_viz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_array_viz_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    mag_viz::SensorArrayVisualizer visualizer(nh, pnh);
    ros::spin();
    return 0;
}
