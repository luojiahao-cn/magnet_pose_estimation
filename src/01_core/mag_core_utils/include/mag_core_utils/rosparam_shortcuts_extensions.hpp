#pragma once

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <XmlRpcValue.h>

namespace rosparam_shortcuts {

bool get(const std::string &parent_name,
         const ros::NodeHandle &nh,
         const std::string &param_name,
         XmlRpc::XmlRpcValue &value);

}  // namespace rosparam_shortcuts
