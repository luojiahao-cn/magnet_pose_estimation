/**
 * @file array_motion_config_loader.cpp
 * @brief 传感器阵列运动配置加载器实现
 * 
 * 实现从 XML-RPC 配置中解析和加载传感器阵列运动配置的功能
 */

#include <mag_sensor_movable_app/array_motion_config_loader.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace mag_sensor_movable_app
{
namespace
{
namespace xml = mag_core_utils::xmlrpc;

/**
 * @brief 将字符串转换为小写
 * 
 * @param value 输入字符串
 * @return std::string 小写字符串
 */
std::string toLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

/**
 * @brief 解析旋转轴标志字符串
 * 
 * 从字符串中解析出 x、y、z 轴标志，例如 "xyz"、"x"、"yz" 等
 * 
 * @param axes 轴字符串（例如 "xyz"、"x"、"yz"）
 * @param axis_x 输出：是否包含 X 轴
 * @param axis_y 输出：是否包含 Y 轴
 * @param axis_z 输出：是否包含 Z 轴
 * @throws std::runtime_error 如果字符串中不包含任何有效轴
 */
void parseAxisFlags(const std::string &axes, bool &axis_x, bool &axis_y, bool &axis_z)
{
    axis_x = axis_y = axis_z = false;
    for (const auto c_raw : axes)
    {
        const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(c_raw)));
        if (c == 'x')
        {
            axis_x = true;
        }
        else if (c == 'y')
        {
            axis_y = true;
        }
        else if (c == 'z')
        {
            axis_z = true;
        }
    }
    if (!axis_x && !axis_y && !axis_z)
    {
        throw std::runtime_error("spin.axis 至少需要包含 x/y/z 之一");
    }
}

/**
 * @brief 从 XYZ 位置和 RPY 欧拉角创建姿态
 * 
 * @param xyz 位置向量 [x, y, z]
 * @param rpy 欧拉角向量 [roll, pitch, yaw]（弧度）
 * @return geometry_msgs::Pose 姿态消息
 */
geometry_msgs::Pose poseFromXyzRpy(const std::vector<double> &xyz,
                                   const std::vector<double> &rpy)
{
    geometry_msgs::Pose pose;
    pose.position.x = xyz[0];
    pose.position.y = xyz[1];
    pose.position.z = xyz[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.orientation = tf2::toMsg(q);
    return pose;
}

/**
 * @brief 从 RPY 欧拉角创建四元数
 * 
 * @param rpy 欧拉角向量 [roll, pitch, yaw]（弧度）
 * @return geometry_msgs::Quaternion 四元数消息
 */
geometry_msgs::Quaternion quaternionFromRpy(const std::vector<double> &rpy)
{
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    return tf2::toMsg(q);
}

/**
 * @brief 解析 TF 坐标系配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return FrameConfig 坐标系配置
 */
FrameConfig parseFrames(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto frames_ctx = xml::makeContext(context, "frames");
    const auto &frames = xml::requireStructField(root, "frames", context);
    FrameConfig cfg;
    cfg.parent_frame = xml::requireStringField(frames, "parent", frames_ctx);
    cfg.child_frame = xml::requireStringField(frames, "child", frames_ctx);
    return cfg;
}

/**
 * @brief 解析运动参数配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return MotionConfig 运动参数配置
 * @throws std::runtime_error 如果更新频率不是正数
 */
MotionConfig parseMotion(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    MotionConfig cfg;
    if (xml::hasMember(root, "motion"))
    {
        const auto motion_ctx = xml::makeContext(context, "motion");
        const auto &motion = xml::requireStructField(root, "motion", context);
        cfg.update_rate_hz = xml::optionalNumberField(motion, "update_rate_hz", motion_ctx, cfg.update_rate_hz);
    }
    if (cfg.update_rate_hz <= 0.0)
    {
        throw std::runtime_error("motion.update_rate_hz 需为正数");
    }
    return cfg;
}

/**
 * @brief 解析静态轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return StaticTrajectoryConfig 静态轨迹配置
 */
StaticTrajectoryConfig parseStaticConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto static_ctx = xml::makeContext(context, "static");
    const auto &static_block = xml::requireStructField(root, "static", context);
    const auto pose_ctx = xml::makeContext(static_ctx, "initial_pose");
    const auto &pose_node = xml::requireStructField(static_block, "initial_pose", static_ctx);
    const auto xyz = xml::requireVector3Field(pose_node, "xyz", pose_ctx);
    const auto rpy = xml::requireVector3Field(pose_node, "rpy", pose_ctx);
    StaticTrajectoryConfig cfg;
    cfg.pose = poseFromXyzRpy(xyz, rpy);
    return cfg;
}

/**
 * @brief 解析圆形轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return CircularTrajectoryConfig 圆形轨迹配置
 */
CircularTrajectoryConfig parseCircularConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto circular_ctx = xml::makeContext(context, "circular");
    const auto &circular = xml::requireStructField(root, "circular", context);

    CircularTrajectoryConfig cfg;
    const auto center = xml::requireVector3Field(circular, "center_xyz", circular_ctx);
    cfg.center.x = center[0];
    cfg.center.y = center[1];
    cfg.center.z = center[2];
    cfg.radius = xml::optionalNumberField(circular, "radius", circular_ctx, cfg.radius);
    cfg.angular_velocity = xml::optionalNumberField(circular, "angular_velocity", circular_ctx, cfg.angular_velocity);
    cfg.initial_angle = xml::optionalNumberField(circular, "initial_angle", circular_ctx, cfg.initial_angle);
    cfg.orientation_mode = xml::optionalStringField(circular, "orientation_mode", circular_ctx, cfg.orientation_mode);
    
    if (xml::hasMember(circular, "fixed_orientation_rpy"))
    {
        const auto rpy = xml::requireVector3Field(circular, "fixed_orientation_rpy", circular_ctx);
        cfg.fixed_orientation = quaternionFromRpy(rpy);
    }
    
    if (cfg.radius <= 0.0)
    {
        throw std::runtime_error("circular.radius 需为正数");
    }
    return cfg;
}

/**
 * @brief 解析矩形轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return RectangularTrajectoryConfig 矩形轨迹配置
 */
RectangularTrajectoryConfig parseRectangularConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto rect_ctx = xml::makeContext(context, "rectangular");
    const auto &rect = xml::requireStructField(root, "rectangular", context);
    
    RectangularTrajectoryConfig cfg;
    const auto center = xml::requireVector3Field(rect, "center_xyz", rect_ctx);
    cfg.center.x = center[0];
    cfg.center.y = center[1];
    cfg.center.z = center[2];
    cfg.width = xml::optionalNumberField(rect, "width", rect_ctx, cfg.width);
    cfg.height = xml::optionalNumberField(rect, "height", rect_ctx, cfg.height);
    cfg.velocity = xml::optionalNumberField(rect, "velocity", rect_ctx, cfg.velocity);
    cfg.orientation_mode = xml::optionalStringField(rect, "orientation_mode", rect_ctx, cfg.orientation_mode);
    
    if (xml::hasMember(rect, "fixed_orientation_rpy"))
    {
        const auto rpy = xml::requireVector3Field(rect, "fixed_orientation_rpy", rect_ctx);
        cfg.fixed_orientation = quaternionFromRpy(rpy);
    }
    
    if (cfg.width <= 0.0 || cfg.height <= 0.0)
    {
        throw std::runtime_error("rectangular.width/height 需为正数");
    }
    if (cfg.velocity <= 0.0)
    {
        throw std::runtime_error("rectangular.velocity 需为正数");
    }
    cfg.perimeter = 2.0 * (cfg.width + cfg.height);
    return cfg;
}

/**
 * @brief 解析直线轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return LinearTrajectoryConfig 直线轨迹配置
 */
LinearTrajectoryConfig parseLinearConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto linear_ctx = xml::makeContext(context, "linear");
    const auto &linear = xml::requireStructField(root, "linear", context);
    
    LinearTrajectoryConfig cfg;
    const auto start = xml::requireVector3Field(linear, "start_xyz", linear_ctx);
    cfg.start.x = start[0];
    cfg.start.y = start[1];
    cfg.start.z = start[2];
    
    const auto end = xml::requireVector3Field(linear, "end_xyz", linear_ctx);
    cfg.end.x = end[0];
    cfg.end.y = end[1];
    cfg.end.z = end[2];
    
    cfg.velocity = xml::optionalNumberField(linear, "velocity", linear_ctx, cfg.velocity);
    cfg.repeat = xml::optionalBoolField(linear, "repeat", linear_ctx, cfg.repeat);
    cfg.orientation_mode = xml::optionalStringField(linear, "orientation_mode", linear_ctx, cfg.orientation_mode);
    
    if (xml::hasMember(linear, "fixed_orientation_rpy"))
    {
        const auto rpy = xml::requireVector3Field(linear, "fixed_orientation_rpy", linear_ctx);
        cfg.fixed_orientation = quaternionFromRpy(rpy);
    }
    
    // 计算距离
    const double dx = cfg.end.x - cfg.start.x;
    const double dy = cfg.end.y - cfg.start.y;
    const double dz = cfg.end.z - cfg.start.z;
    cfg.distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    
    if (cfg.velocity <= 0.0)
    {
        throw std::runtime_error("linear.velocity 需为正数");
    }
    if (cfg.distance <= 0.0)
    {
        throw std::runtime_error("linear 起点和终点不能相同");
    }
    return cfg;
}

/**
 * @brief 解析自旋轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return SpinTrajectoryConfig 自旋轨迹配置
 */
SpinTrajectoryConfig parseSpinConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto spin_ctx = xml::makeContext(context, "spin");
    const auto &spin = xml::requireStructField(root, "spin", context);
    
    SpinTrajectoryConfig cfg;
    const auto position = xml::requireVector3Field(spin, "position_xyz", spin_ctx);
    cfg.position.x = position[0];
    cfg.position.y = position[1];
    cfg.position.z = position[2];
    
    cfg.angular_velocity = xml::optionalNumberField(spin, "angular_velocity", spin_ctx, cfg.angular_velocity);
    
    // 解析初始姿态
    if (xml::hasMember(spin, "initial_rpy"))
    {
        const auto rpy = xml::requireVector3Field(spin, "initial_rpy", spin_ctx);
        cfg.initial_roll = rpy[0];
        cfg.initial_pitch = rpy[1];
        cfg.initial_yaw = rpy[2];
        cfg.initial_orientation = quaternionFromRpy(rpy);
    }
    
    // 解析旋转轴
    const auto axis = xml::optionalStringField(spin, "axis", spin_ctx, std::string("z"));
    parseAxisFlags(axis, cfg.axis_x, cfg.axis_y, cfg.axis_z);
    
    return cfg;
}

/**
 * @brief 解析轨迹配置
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return TrajectoryConfig 轨迹配置
 */
TrajectoryConfig parseTrajectory(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto traj_ctx = xml::makeContext(context, "motion");
    const auto &motion = xml::requireStructField(root, "motion", context);
    const auto type_raw = xml::requireStringField(motion, "type", traj_ctx);
    const auto type = toLower(type_raw);

    TrajectoryConfig cfg;
    if (type == "static")
    {
        cfg.type = TrajectoryType::Static;
        cfg.static_config = parseStaticConfig(root, context);
    }
    else if (type == "circular")
    {
        cfg.type = TrajectoryType::Circular;
        cfg.circular_config = parseCircularConfig(root, context);
    }
    else if (type == "rectangular")
    {
        cfg.type = TrajectoryType::Rectangular;
        cfg.rectangular_config = parseRectangularConfig(root, context);
    }
    else if (type == "linear")
    {
        cfg.type = TrajectoryType::Linear;
        cfg.linear_config = parseLinearConfig(root, context);
    }
    else if (type == "spin")
    {
        cfg.type = TrajectoryType::Spin;
        cfg.spin_config = parseSpinConfig(root, context);
    }
    else
    {
        throw std::runtime_error("不支持的轨迹类型: " + type_raw + " (支持: static, circular, rectangular, linear, spin)");
    }
    return cfg;
}

/**
 * @brief 解析姿态配置
 * 
 * 解析姿态模式（轨迹、固定或旋转）及相应参数
 * 
 * @param root XML-RPC 配置根节点
 * @param context 配置上下文路径
 * @return OrientationConfig 姿态配置
 * @throws std::runtime_error 如果姿态模式未知
 */
OrientationConfig parseOrientation(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    OrientationConfig cfg;
    cfg.fixed_orientation.w = 1.0;
    if (!xml::hasMember(root, "orientation"))
    {
        cfg.mode = OrientationMode::FromTrajectory;
        return cfg;
    }

    const auto orient_ctx = xml::makeContext(context, "orientation");
    const auto &orientation = xml::requireStructField(root, "orientation", context);
    const auto mode_raw = xml::optionalStringField(orientation, "mode", orient_ctx, std::string("trajectory"));
    const auto mode = toLower(mode_raw);

    if (mode == "trajectory")
    {
        cfg.mode = OrientationMode::FromTrajectory;
        return cfg;
    }

    if (mode == "fixed")
    {
        cfg.mode = OrientationMode::Fixed;
        const auto fixed_ctx = xml::makeContext(orient_ctx, "fixed");
        const auto &fixed = xml::requireStructField(orientation, "fixed", orient_ctx);
        const auto rpy = xml::requireVector3Field(fixed, "rpy", fixed_ctx);
        cfg.fixed_orientation = quaternionFromRpy(rpy);
        return cfg;
    }

    if (mode == "spin")
    {
        cfg.mode = OrientationMode::Spin;
        const auto spin_ctx = xml::makeContext(orient_ctx, "spin");
        const auto &spin = xml::requireStructField(orientation, "spin", orient_ctx);
        
        // initial_rpy 是可选的，如果未配置则使用默认值 [0, 0, 0]，表示使用轨迹中的基础姿态
        if (xml::hasMember(spin, "initial_rpy"))
        {
            const auto initial = xml::requireVector3Field(spin, "initial_rpy", spin_ctx);
            cfg.initial_roll = initial[0];
            cfg.initial_pitch = initial[1];
            cfg.initial_yaw = initial[2];
        }
        // 否则使用默认值 [0, 0, 0]，sampleOrientation 会从基础姿态中提取
        
        cfg.angular_velocity = xml::optionalNumberField(spin, "angular_velocity", spin_ctx, cfg.angular_velocity);
        const auto axis = xml::optionalStringField(spin, "axis", spin_ctx, std::string("z"));
        parseAxisFlags(axis, cfg.axis_x, cfg.axis_y, cfg.axis_z);
        return cfg;
    }

    throw std::runtime_error("未知的 orientation.mode: " + mode_raw);
}

} // namespace

ArrayMotionConfigBundle loadArrayMotionConfig(const XmlRpc::XmlRpcValue &root,
                                               const std::string &context)
{
    ArrayMotionConfigBundle bundle;
    bundle.frame = parseFrames(root, context);
    bundle.motion = parseMotion(root, context);
    bundle.trajectory = parseTrajectory(root, context);
    bundle.orientation = parseOrientation(root, context);
    return bundle;
}

} // namespace mag_sensor_movable_app

