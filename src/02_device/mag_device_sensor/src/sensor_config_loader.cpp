#include <mag_device_sensor/sensor_config_loader.hpp>

#include <mag_core_utils/xmlrpc_utils.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace mag_device_sensor
{
namespace
{
namespace xml = mag_core_utils::xmlrpc;

DriverConfig parseDriverConfig(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto params_ctx = xml::makeContext(context, "params");
    const auto driver_ctx = xml::makeContext(params_ctx, "driver");
    const auto &params = xml::requireStructField(root, "params", context);
    const auto &driver = xml::requireStructField(params, "driver", params_ctx);

    DriverConfig cfg;
    cfg.serial_port = xml::requireStringField(driver, "serial_port", driver_ctx);
    cfg.baud_rate = static_cast<int>(xml::readNumber(xml::requireMember(driver, "baud_rate", driver_ctx),
                                                     xml::makeContext(driver_ctx, "baud_rate")));
    cfg.timeout_ms = static_cast<int>(xml::readNumber(xml::requireMember(driver, "timeout_ms", driver_ctx),
                                                      xml::makeContext(driver_ctx, "timeout_ms")));
    cfg.poll_sleep_ms = static_cast<int>(xml::optionalNumberField(driver, "poll_sleep_ms", driver_ctx, cfg.poll_sleep_ms));
    cfg.freq_stat_period = xml::optionalNumberField(driver, "freq_stat_period", driver_ctx, cfg.freq_stat_period);

    const auto calib_ctx = xml::makeContext(params_ctx, "calibration");
    if (xml::hasMember(params, "calibration"))
    {
        const auto &calibration = xml::requireStructField(params, "calibration", params_ctx);
        cfg.full_scale_mT = xml::optionalNumberField(calibration, "full_scale_mT", calib_ctx, cfg.full_scale_mT);
        cfg.raw_max = xml::optionalNumberField(calibration, "raw_max", calib_ctx, cfg.raw_max);
    }

    if (cfg.full_scale_mT <= 0.0 || cfg.raw_max <= 0.0)
    {
        throw std::runtime_error(context + ": params.calibration 需提供正数 full_scale_mT/raw_max");
    }

    return cfg;
}

TopicConfig parseTopics(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    TopicConfig cfg;
    if (!xml::hasMember(root, "topics"))
    {
        return cfg;
    }
    const auto topics_ctx = xml::makeContext(context, "topics");
    const auto &topics = xml::requireStructField(root, "topics", context);
    cfg.raw_topic = xml::optionalStringField(topics, "raw", topics_ctx, cfg.raw_topic);
    cfg.field_topic = xml::optionalStringField(topics, "field", topics_ctx, cfg.field_topic);
    if (cfg.raw_topic.empty() && cfg.field_topic.empty())
    {
        throw std::runtime_error(topics_ctx + ": 至少需要 raw 或 field 话题");
    }
    return cfg;
}

TfConfig parseTf(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    TfConfig cfg;
    if (!xml::hasMember(root, "tf"))
    {
        return cfg;
    }
    const auto tf_ctx = xml::makeContext(context, "tf");
    const auto &tf = xml::requireStructField(root, "tf", context);
    cfg.enable = xml::optionalBoolField(tf, "enable", tf_ctx, cfg.enable);
    if (xml::hasMember(tf, "publish_rate"))
    {
        cfg.publish_rate = xml::readNumber(xml::requireMember(tf, "publish_rate", tf_ctx),
                                           xml::makeContext(tf_ctx, "publish_rate"));
    }
    return cfg;
}

CalibrationConfig parseCalibration(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    CalibrationConfig cfg;
    if (!xml::hasMember(root, "params"))
    {
        return cfg;
    }
    const auto params_ctx = xml::makeContext(context, "params");
    const auto &params = xml::requireStructField(root, "params", context);
    if (xml::hasMember(params, "calibration"))
    {
        const auto calib_ctx = xml::makeContext(params_ctx, "calibration");
        const auto &calibration = xml::requireStructField(params, "calibration", params_ctx);
        cfg.full_scale_mT = xml::optionalNumberField(calibration, "full_scale_mT", calib_ctx, cfg.full_scale_mT);
        cfg.raw_max = xml::optionalNumberField(calibration, "raw_max", calib_ctx, cfg.raw_max);
    }
    if (cfg.full_scale_mT <= 0.0 || cfg.raw_max <= 0.0)
    {
        throw std::runtime_error(context + ": params.calibration 需提供正数 full_scale_mT/raw_max");
    }
    return cfg;
}

NoiseConfig parseNoise(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    NoiseConfig cfg;
    if (!xml::hasMember(root, "simulation"))
    {
        return cfg;
    }
    const auto sim_ctx = xml::makeContext(context, "simulation");
    const auto &simulation = xml::requireStructField(root, "simulation", context);
    if (!xml::hasMember(simulation, "noise"))
    {
        return cfg;
    }
    const auto noise_ctx = xml::makeContext(sim_ctx, "noise");
    const auto &noise = xml::requireStructField(simulation, "noise", sim_ctx);
    cfg.enable = xml::optionalBoolField(noise, "enable", noise_ctx, cfg.enable);
    cfg.type = xml::optionalStringField(noise, "type", noise_ctx, cfg.type);
    cfg.mean = xml::optionalNumberField(noise, "mean", noise_ctx, cfg.mean);
    cfg.stddev = xml::optionalNumberField(noise, "stddev", noise_ctx, cfg.stddev);
    cfg.amplitude = xml::optionalNumberField(noise, "amplitude", noise_ctx, cfg.amplitude);
    return cfg;
}

SimulationConfig parseSimulation(const XmlRpc::XmlRpcValue &root, const std::string &context)
{
    const auto sim_ctx = xml::makeContext(context, "simulation");
    const auto &simulation = xml::requireStructField(root, "simulation", context);

    SimulationConfig cfg;
    cfg.update_rate_hz = xml::optionalNumberField(simulation, "update_rate_hz", sim_ctx, cfg.update_rate_hz);
    if (cfg.update_rate_hz <= 0.0)
    {
        throw std::runtime_error(sim_ctx + ": update_rate_hz 需为正数");
    }

    if (!xml::hasMember(simulation, "magnet_frame"))
    {
        throw std::runtime_error(sim_ctx + ": 缺少 magnet_frame 配置");
    }
    const auto magnet_frame_ctx = xml::makeContext(sim_ctx, "magnet_frame");
    const auto &magnet_frame = xml::requireStructField(simulation, "magnet_frame", sim_ctx);
    cfg.magnet_parent_frame = xml::requireStringField(magnet_frame, "parent", magnet_frame_ctx);
    cfg.magnet_child_frame = xml::requireStringField(magnet_frame, "child", magnet_frame_ctx);

    if (xml::hasMember(simulation, "magnet"))
    {
        const auto magnet_ctx = xml::makeContext(sim_ctx, "magnet");
        const auto &magnet = xml::requireStructField(simulation, "magnet", sim_ctx);
        cfg.dipole_strength = xml::optionalNumberField(magnet, "dipole_strength", magnet_ctx, cfg.dipole_strength);
        if (cfg.dipole_strength <= 0.0)
        {
            throw std::runtime_error(magnet_ctx + ": dipole_strength 需为正数");
        }
        if (xml::hasMember(magnet, "base_direction"))
        {
            const auto vec = xml::requireVector3Field(magnet, "base_direction", magnet_ctx);
            cfg.base_direction = Eigen::Vector3d(vec[0], vec[1], vec[2]);
        }
    }

    if (cfg.base_direction.norm() == 0.0)
    {
        throw std::runtime_error(sim_ctx + ": magnet.base_direction 不可为零向量");
    }
    cfg.base_direction.normalize();

    return cfg;
}

} // namespace

SensorDriverConfigBundle loadSensorDriverConfig(const XmlRpc::XmlRpcValue &root,
                                                const std::string &context)
{
    const auto &node = xml::asStruct(root, context);
    SensorDriverConfigBundle bundle;
    bundle.driver = parseDriverConfig(node, context);
    bundle.topics = parseTopics(node, context);
    bundle.tf = parseTf(node, context);
    return bundle;
}

SensorSimulationConfigBundle loadSensorSimulationConfig(const XmlRpc::XmlRpcValue &root,
                                                        const std::string &context)
{
    const auto &node = xml::asStruct(root, context);
    SensorSimulationConfigBundle bundle;
    bundle.topics = parseTopics(node, context);
    bundle.tf = parseTf(node, context);
    bundle.calibration = parseCalibration(node, context);
    bundle.noise = parseNoise(node, context);
    bundle.simulation = parseSimulation(node, context);
    return bundle;
}

} // namespace mag_device_sensor
