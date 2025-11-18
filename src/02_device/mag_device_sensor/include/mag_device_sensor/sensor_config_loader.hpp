#pragma once

#include <mag_device_sensor/sensor_node.hpp>
#include <mag_device_sensor/sim_node.hpp>

#include <XmlRpcValue.h>

#include <string>

namespace mag_device_sensor
{

struct SensorDriverConfigBundle
{
    DriverConfig driver;
    TopicConfig topics;
    TfConfig tf;
};

struct SensorSimulationConfigBundle
{
    TopicConfig topics;
    TfConfig tf;
    CalibrationConfig calibration;
    NoiseConfig noise;
    SimulationConfig simulation;
};

SensorDriverConfigBundle loadSensorDriverConfig(const XmlRpc::XmlRpcValue &root,
                                                const std::string &context);

SensorSimulationConfigBundle loadSensorSimulationConfig(const XmlRpc::XmlRpcValue &root,
                                                        const std::string &context);

} // namespace mag_device_sensor
