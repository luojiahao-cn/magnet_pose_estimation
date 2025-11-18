#pragma once

#include <mag_device_magnet/magnet_node.hpp>

#include <XmlRpcValue.h>

#include <string>

namespace mag_device_magnet
{

struct MagnetConfigBundle
{
    FrameConfig frame;
    MotionConfig motion;
    TopicConfig topics;
    TfConfig tf;
    TrajectoryConfig trajectory;
    OrientationConfig orientation;
};

MagnetConfigBundle loadMagnetConfig(const XmlRpc::XmlRpcValue &root,
                                    const std::string &context);

} // namespace mag_device_magnet
