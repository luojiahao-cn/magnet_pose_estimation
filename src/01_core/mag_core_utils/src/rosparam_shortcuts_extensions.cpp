#include "mag_core_utils/rosparam_shortcuts_extensions.hpp"

namespace rosparam_shortcuts {

bool get(const std::string &parent_name,
         const ros::NodeHandle &nh,
         const std::string &param_name,
         XmlRpc::XmlRpcValue &value) {
  if (!nh.getParam(param_name, value)) {
    ROS_ERROR_STREAM_NAMED(parent_name, "Failed to get parameter '" << param_name << "'");
    return false;
  }
  return true;
}

}  // namespace rosparam_shortcuts
