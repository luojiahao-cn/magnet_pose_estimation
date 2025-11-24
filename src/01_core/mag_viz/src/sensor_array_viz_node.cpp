#include <mag_core_description/sensor_array_config_loader.hpp>
#include <mag_core_description/sensor_array_description.hpp>
#include <mag_core_msgs/MagSensorData.h>
#include <mag_core_utils/rosparam_shortcuts_extensions.hpp>
#include <mag_core_utils/xmlrpc_utils.hpp>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <locale>
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
    /**
     * @brief 构造函数：初始化传感器阵列可视化器
     * 从ROS参数服务器加载配置，设置订阅者和发布者，初始化可视化参数
     * @param nh 全局节点句柄
     * @param pnh 私有节点句柄
     */
    SensorArrayVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
        : nh_(std::move(nh)), pnh_(std::move(pnh)), tf_buffer_(), tf_listener_(tf_buffer_)
    {
        // 记录启动时间，用于延迟TF查询失败的警告
        start_time_ = ros::Time::now();
        
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
        std::string array_frame;  // 传感器阵列坐标系名称
        tf2::Transform tf_array_sensor;  // 传感器相对于阵列的变换（固定）
        tf2::Vector3 up_axis;  // 传感器上轴方向（在传感器坐标系中）
    };

    struct MeasurementState
    {
        ros::Time stamp;
        tf2::Vector3 value_sensor;
        tf2::Vector3 value_parent;
    };

    void loadParameters()
    {
        // Load configuration
        XmlRpc::XmlRpcValue config;
        if (!pnh_.getParam("config", config)) {
            ROS_ERROR("[sensor_array_viz] ✗ 无法获取配置参数");
            ros::shutdown();
            return;
        }

        namespace xml = mag_core_utils::xmlrpc;

        // Load frames
        const auto& frames = xml::requireStructField(config, "frames", "config");
        fixed_frame_ = xml::requireStringField(frames, "fixed_frame", "config.frames");

        // Load topics
        const auto& topics = xml::requireStructField(config, "topics", "config");
        measurement_topic_ = xml::requireStringField(topics, "measurement", "config.topics");

        // Load params
        const auto& params = xml::requireStructField(config, "params", "config");
        array_param_key_ = xml::requireStringField(params, "array_param", "config.params");
        publish_rate_ = xml::readNumber(xml::requireMember(params, "publish_rate", "config.params"), "config.params/publish_rate");
        marker_lifetime_ = ros::Duration(xml::readNumber(xml::requireMember(params, "marker_lifetime", "config.params"), "config.params/marker_lifetime"));
        measurement_timeout_ = xml::readNumber(xml::requireMember(params, "measurement_timeout", "config.params"), "config.params/measurement_timeout");
        color_min_mT_ = xml::readNumber(xml::requireMember(params, "color_min_mT", "config.params"), "config.params/color_min_mT");
        color_max_mT_ = xml::readNumber(xml::requireMember(params, "color_max_mT", "config.params"), "config.params/color_max_mT");
        show_text_ = xml::requireBoolField(params, "show_text", "config.params");
        text_scale_ = xml::readNumber(xml::requireMember(params, "text_scale", "config.params"), "config.params/text_scale");
        text_offset_ = xml::readNumber(xml::requireMember(params, "text_offset", "config.params"), "config.params/text_offset");

        // Load visualization settings
        const auto& visualization = xml::requireStructField(config, "visualization", "config");
        field_ns_ = xml::requireStringField(visualization, "field_marker_ns", "config.visualization");
        text_ns_ = xml::requireStringField(visualization, "text_marker_ns", "config.visualization");
        arrow_shaft_diameter_ = xml::readNumber(xml::requireMember(visualization, "arrow_shaft_diameter", "config.visualization"), "config.visualization/arrow_shaft_diameter");
        arrow_head_diameter_ = xml::readNumber(xml::requireMember(visualization, "arrow_head_diameter", "config.visualization"), "config.visualization/arrow_head_diameter");
        arrow_head_length_ = xml::readNumber(xml::requireMember(visualization, "arrow_head_length", "config.visualization"), "config.visualization/arrow_head_length");
        vector_scale_ = xml::readNumber(xml::requireMember(visualization, "vector_scale", "config.visualization"), "config.visualization/vector_scale");
        max_arrow_length_ = xml::readNumber(xml::requireMember(visualization, "max_arrow_length", "config.visualization"), "config.visualization/max_arrow_length");
        arrow_origin_offset_ = xml::readNumber(xml::requireMember(visualization, "arrow_origin_offset", "config.visualization"), "config.visualization/arrow_origin_offset");

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
            ROS_WARN_THROTTLE(10.0,
                              "[sensor_array_viz] ⚠ fixed_frame '%s' 与描述中的 parent_frame '%s' 不一致，将仍以 parent_frame 发布 marker",
                              fixed_frame_.c_str(), description.parentFrame().c_str());
            fixed_frame_ = description.parentFrame();
        }

        tf2::Transform parent_array;
        tf2::fromMsg(description.arrayPose(), parent_array);

        sensors_.clear();
        sensor_index_.clear();
        sensors_.reserve(description.sensors().size());

        array_frame_ = description.arrayFrame();
        
        for (const auto &sensor_entry : description.sensors())
        {
            SensorVisual visual;
            visual.id = sensor_entry.id;
            visual.frame_id = sensor_entry.frame_id;
            visual.array_frame = array_frame_;

            tf2::Transform array_sensor;
            tf2::fromMsg(sensor_entry.pose, array_sensor);
            visual.tf_array_sensor = array_sensor;
            
            // 计算传感器上轴方向（在传感器坐标系中，通常是 Z 轴）
            visual.up_axis = tf2::Vector3(0.0, 0.0, 1.0);
            
            sensors_.push_back(visual);
            sensor_index_.emplace(visual.id, sensors_.size() - 1);
        }
        ROS_INFO("[sensor_array_viz] ✓ 已加载 %zu 个传感器位姿 (参数: ~%s)", sensors_.size(), config_key.c_str());
    }

    /**
     * @brief 磁传感器测量数据回调函数
     * 处理接收到的磁传感器测量数据，更新内部状态
     * @param msg 接收到的磁传感器测量消息
     */
    void onMeasurement(const mag_core_msgs::MagSensorDataConstPtr &msg)
    {
        const auto it = sensor_index_.find(static_cast<int>(msg->sensor_id));
        if (it == sensor_index_.end())
        {
            ROS_WARN_THROTTLE(5.0, "[sensor_array_viz] ✗ 未知传感器 ID: %u", msg->sensor_id);
            return;
        }
        
        MeasurementState state;
        state.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        state.value_sensor = tf2::Vector3(msg->mag_x, msg->mag_y, msg->mag_z);
        // value_parent 将在发布标记时从 TF 树查询后计算
        measurements_[msg->sensor_id] = state;
    }

    /**
     * @brief 定时器回调函数
     * 定期发布可视化标记
     * @param event 定时器事件
     */
    void onTimer(const ros::TimerEvent &event)
    {
        publishMarkers(event.current_real);
    }

    /**
     * @brief 查找指定传感器的测量数据
     * 根据传感器ID和时间戳查找有效的测量数据
     * @param sensor_id 传感器ID
     * @param stamp 时间戳
     * @return 测量状态指针，如果没有找到或已过期则返回nullptr
     */
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

    /**
     * @brief 根据磁场强度计算颜色
     * 使用蓝->红渐变色映射磁场强度值
     * @param magnitude_mT 磁场强度（毫特斯拉）
     * @return 颜色RGBA值
     */
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

    /**
     * @brief 格式化测量数据显示文本
     * 创建包含传感器ID和测量值的格式化字符串
     * @param sensor_id 传感器ID
     * @param state 测量状态数据
     * @return 格式化的文本字符串
     */
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

    /**
     * @brief 从 TF 树查询传感器位姿
     * @param sensor 传感器可视化信息
     * @param stamp 时间戳
     * @param tf_parent_sensor 输出：传感器相对于父坐标系的变换
     * @param up_axis_parent 输出：传感器上轴在父坐标系中的方向
     * @return 是否成功查询到 TF
     */
    bool querySensorTf(const SensorVisual &sensor, const ros::Time &stamp,
                       tf2::Transform &tf_parent_sensor, tf2::Vector3 &up_axis_parent) const
    {
        try
        {
            // 查询传感器相对于父坐标系的变换
            geometry_msgs::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
                fixed_frame_, sensor.frame_id, stamp, ros::Duration(0.1));
            tf2::fromMsg(tf_stamped.transform, tf_parent_sensor);
            
            // 计算传感器上轴在父坐标系中的方向
            up_axis_parent = tf_parent_sensor.getBasis() * sensor.up_axis;
            const double up_norm = up_axis_parent.length();
            if (up_norm > kEpsilon)
            {
                up_axis_parent /= up_norm;
            }
            else
            {
                up_axis_parent = tf2::Vector3(0.0, 0.0, 1.0);
            }
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            // 启动后3秒内不报warn，给TF树建立时间
            const ros::Duration time_since_start = ros::Time::now() - start_time_;
            if (time_since_start.toSec() > 3.0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "[sensor_array_viz] ✗ 无法查询传感器 TF (%s -> %s): %s",
                                  fixed_frame_.c_str(),
                                  sensor.frame_id.c_str(),
                                  ex.what());
            }
            return false;
        }
    }

    /**
     * @brief 创建磁场测量箭头标记
     * 根据传感器测量数据创建表示磁场向量的箭头标记
     * @param sensor 传感器可视化信息
     * @param state 测量状态数据
     * @param stamp 时间戳
     * @return 可视化标记
     */
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
        // 初始化四元数为单位四元数，避免 "Uninitialized quaternion" 警告
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        // 从 TF 树查询传感器当前位姿
        tf2::Transform tf_parent_sensor;
        tf2::Vector3 up_axis_parent;
        if (!querySensorTf(sensor, stamp, tf_parent_sensor, up_axis_parent))
        {
            // 如果查询失败，返回一个不可见的标记
            marker.action = visualization_msgs::Marker::DELETE;
            return marker;
        }

        tf2::Vector3 start_vec = tf_parent_sensor.getOrigin();
        if (arrow_origin_offset_ > 0.0)
        {
            start_vec += up_axis_parent * arrow_origin_offset_;
        }
        tf2::Vector3 end_vec = start_vec;
        std_msgs::ColorRGBA color = stale_color_;

        if (state)
        {
            // 将传感器坐标系中的磁场向量转换到父坐标系
            tf2::Vector3 value_parent = tf_parent_sensor.getBasis() * state->value_sensor;
            const double magnitude = value_parent.length();
            if (magnitude > kEpsilon)
            {
                double length = vector_scale_ * magnitude;
                if (max_arrow_length_ > 0.0)
                {
                    length = std::min(length, max_arrow_length_);
                }
                const tf2::Vector3 direction = value_parent.normalized() * length;
                end_vec = start_vec + direction;
                color = colorForMagnitude(magnitude);
            }
            else
            {
                end_vec = start_vec + up_axis_parent * 0.001;
            }
        }
        else
        {
            end_vec = start_vec + up_axis_parent * 0.001;
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

    /**
     * @brief 创建磁场测量文本标记
     * 创建显示传感器ID和测量值的文本标记
     * @param sensor 传感器可视化信息
     * @param state 测量状态数据
     * @param stamp 时间戳
     * @return 可视化标记
     */
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

        // 从 TF 树查询传感器当前位姿
        tf2::Transform tf_parent_sensor;
        tf2::Vector3 up_axis_parent;
        if (!querySensorTf(sensor, stamp, tf_parent_sensor, up_axis_parent))
        {
            // 如果查询失败，返回一个不可见的标记
            marker.action = visualization_msgs::Marker::DELETE;
            return marker;
        }

        tf2::Vector3 anchor = tf_parent_sensor.getOrigin();
        const double total_offset = std::max(text_offset_, 0.0) + (arrow_origin_offset_ > 0.0 ? arrow_origin_offset_ : 0.0);
        anchor += up_axis_parent * (total_offset + text_scale_ * 0.5);
        marker.pose.position = toPoint(anchor);
        // 初始化四元数为单位四元数，避免 "Uninitialized quaternion" 警告
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.text = formatMeasurementText(sensor.id, state);
        marker.color = state ? makeColor(1.0, 1.0, 1.0, 0.95) : makeColor(0.7, 0.7, 0.7, 0.6);
        return marker;
    }

    /**
     * @brief 发布传感器阵列可视化标记
     * 创建并发布所有传感器的3D可视化标记，包括磁场向量箭头和文本标签
     * @param stamp 时间戳
     */
    void publishMarkers(const ros::Time &stamp)
    {
        if (sensors_.empty())
        {
            ROS_WARN_THROTTLE(5.0, "[sensor_array_viz] ✗ 无传感器描述，跳过可视化");
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

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    ros::Time start_time_;  // 启动时间，用于延迟TF查询失败的警告

    std::string measurement_topic_;
    std::string array_param_key_;
    std::string fixed_frame_;
    std::string array_frame_;
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
    // 设置本地化，支持中文输出
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    ros::init(argc, argv, "sensor_array_viz_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    mag_viz::SensorArrayVisualizer visualizer(nh, pnh);
    ros::spin();
    return 0;
}
