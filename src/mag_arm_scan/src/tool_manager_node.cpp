#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ToolManagerNode {
public:
  ToolManagerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), move_group_("fr5v6_arm"), planning_scene_interface_() {
    // Params
  pnh_.param<std::string>("parent_link", parent_link_, std::string("auto"));
    pnh_.param<std::string>("object_name", object_name_, "mag_sensor_bracket");
    pnh_.param<std::string>("mesh_path", mesh_path_, std::string("package://mag_arm_scan/meshes/magnetic_sensor_bracket.STL"));
    pnh_.param("xyz", xyz_, std::vector<double>({0.0, 0.0, 0.10}));
    pnh_.param("rpy", rpy_, std::vector<double>({0.0, 0.0, 0.0}));
    pnh_.param("scale", scale_, std::vector<double>({1.0, 1.0, 1.0}));
    pnh_.param("touch_links", touch_links_, std::vector<std::string>());
  pnh_.param("auto_attach", auto_attach_, true);

    // Resolve parent_link automatically from MoveGroup if requested
    if (parent_link_ == "auto" || parent_link_.empty()) {
      std::string eef = move_group_.getEndEffectorLink();
      if (!eef.empty()) parent_link_ = eef; else parent_link_ = "ee_link";
    }

  srv_attach_ = pnh_.advertiseService("attach", &ToolManagerNode::attachCb, this);
  srv_detach_ = pnh_.advertiseService("detach", &ToolManagerNode::detachCb, this);

  // Live tuning subscribers
  sub_set_xyz_ = pnh_.subscribe("set_xyz", 1, &ToolManagerNode::onSetXYZ, this);
  sub_set_rpy_ = pnh_.subscribe("set_rpy", 1, &ToolManagerNode::onSetRPY, this);
  sub_set_pose_ = pnh_.subscribe("set_pose", 1, &ToolManagerNode::onSetPose, this);

  ROS_INFO("ToolManagerNode ready. object=%s parent_link=%s mesh=%s", object_name_.c_str(), parent_link_.c_str(), mesh_path_.c_str());
  ROS_INFO("Tool params: xyz=[%.4f, %.4f, %.4f], rpy=[%.4f, %.4f, %.4f], scale=[%.4f, %.4f, %.4f]",
       xyz_.size()>0?xyz_[0]:0.0, xyz_.size()>1?xyz_[1]:0.0, xyz_.size()>2?xyz_[2]:0.0,
       rpy_.size()>0?rpy_[0]:0.0, rpy_.size()>1?rpy_[1]:0.0, rpy_.size()>2?rpy_[2]:0.0,
       scale_.size()>0?scale_[0]:1.0, scale_.size()>1?scale_[1]:1.0, scale_.size()>2?scale_[2]:1.0);

    if (auto_attach_) {
      auto_attach_timer_ = nh_.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&){
        std_srvs::Trigger::Request req; std_srvs::Trigger::Response res;
        this->attachCb(req, res);
        ROS_INFO_STREAM("Auto-attach: " << (res.success ? "success" : (std::string("failed: ") + res.message)));
      }, true); // oneshot
    }
  }

private:
  bool attachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    return attachWithParams(res);
  }

  // Helper to (re)attach object using current parameters
  bool attachWithParams(std_srvs::Trigger::Response &res) {
    // Attempt to remove any previous instance
    try {
      move_group_.detachObject(object_name_);
      ros::Duration(0.1).sleep();
      planning_scene_interface_.removeCollisionObjects({object_name_});
    } catch (...) {}

    // Prepare pose relative to parent_link
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = parent_link_;
    if (xyz_.size() == 3) {
      pose.pose.position.x = xyz_[0];
      pose.pose.position.y = xyz_[1];
      pose.pose.position.z = xyz_[2];
    }
    double rr = rpy_.size() > 0 ? rpy_[0] : 0.0;
    double rp = rpy_.size() > 1 ? rpy_[1] : 0.0;
    double ry = rpy_.size() > 2 ? rpy_[2] : 0.0;
    tf2::Quaternion q; q.setRPY(rr, rp, ry);
    pose.pose.orientation = tf2::toMsg(q);

    // Build collision object with mesh
    moveit_msgs::CollisionObject co;
    co.id = object_name_;
    co.header.frame_id = parent_link_;

    try {
      Eigen::Vector3d s(1.0, 1.0, 1.0);
      if (scale_.size() == 3) s = Eigen::Vector3d(scale_[0], scale_[1], scale_[2]);
      shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path_, s);
      if (!mesh) {
        res.success = false;
        res.message = "Failed to load mesh from resource";
        return true;
      }
      shapes::ShapeMsg shape_msg;
      shapes::constructMsgFromShape(mesh, shape_msg);
      shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
      delete mesh;

      co.meshes.push_back(mesh_msg);
      co.mesh_poses.push_back(pose.pose);
      co.operation = co.ADD;
    } catch (const std::exception &e) {
      res.success = false;
      res.message = std::string("Failed to build mesh collision object: ") + e.what();
      return true;
    }

    // Apply to planning scene
    planning_scene_interface_.applyCollisionObject(co);

    ros::Duration(0.3).sleep();

    // Attach to parent link
    try {
      std::vector<std::string> tl = touch_links_;
      if (tl.empty()) {
        tl.push_back(parent_link_);
      }
      move_group_.attachObject(object_name_, parent_link_, tl);
    } catch (const std::exception &e) {
      res.success = false;
      res.message = std::string("Failed to attach object: ") + e.what();
      return true;
    }

    res.success = true;
    res.message = "attached";
    return true;
  }

  bool detachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    try {
      move_group_.detachObject(object_name_);
      ros::Duration(0.2).sleep();
      planning_scene_interface_.removeCollisionObjects({object_name_});
    } catch (const std::exception &e) {
      res.success = false;
      res.message = std::string("Failed to detach/remove: ") + e.what();
      return true;
    }
    res.success = true;
    res.message = "detached";
    return true;
  }

  // Topic callbacks
  void onSetXYZ(const geometry_msgs::Vector3::ConstPtr& v) {
    if (!v) return;
    if (xyz_.size() != 3) xyz_.assign(3, 0.0);
    xyz_[0] = v->x; xyz_[1] = v->y; xyz_[2] = v->z;
    std_srvs::Trigger::Response res; attachWithParams(res);
    ROS_INFO("~set_xyz -> [%.4f, %.4f, %.4f] | %s", xyz_[0], xyz_[1], xyz_[2], res.success?"reattached":"failed");
  }
  void onSetRPY(const geometry_msgs::Vector3::ConstPtr& v) {
    if (!v) return;
    if (rpy_.size() != 3) rpy_.assign(3, 0.0);
    rpy_[0] = v->x; rpy_[1] = v->y; rpy_[2] = v->z;
    std_srvs::Trigger::Response res; attachWithParams(res);
    ROS_INFO("~set_rpy(rad) -> [%.4f, %.4f, %.4f] | %s", rpy_[0], rpy_[1], rpy_[2], res.success?"reattached":"failed");
  }
  void onSetPose(const geometry_msgs::Pose::ConstPtr& p) {
    if (!p) return;
    if (xyz_.size() != 3) xyz_.assign(3, 0.0);
    xyz_[0] = p->position.x; xyz_[1] = p->position.y; xyz_[2] = p->position.z;
    tf2::Quaternion q; tf2::fromMsg(p->orientation, q);
    double r, pch, y; tf2::Matrix3x3(q).getRPY(r, pch, y);
    if (rpy_.size() != 3) rpy_.assign(3, 0.0);
    rpy_[0] = r; rpy_[1] = pch; rpy_[2] = y;
    std_srvs::Trigger::Response res; attachWithParams(res);
    ROS_INFO("~set_pose -> xyz[%.4f, %.4f, %.4f], rpy(rad)[%.4f, %.4f, %.4f] | %s",
             xyz_[0], xyz_[1], xyz_[2], rpy_[0], rpy_[1], rpy_[2], res.success?"reattached":"failed");
  }

  ros::NodeHandle nh_, pnh_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string parent_link_;
  std::string object_name_;
  std::string mesh_path_;
  std::vector<double> xyz_;
  std::vector<double> rpy_;
  std::vector<double> scale_;
  std::vector<std::string> touch_links_;
  bool auto_attach_ {true};
  ros::Timer auto_attach_timer_;

  ros::ServiceServer srv_attach_;
  ros::ServiceServer srv_detach_;
  ros::Subscriber sub_set_xyz_;
  ros::Subscriber sub_set_rpy_;
  ros::Subscriber sub_set_pose_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tool_manager_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ToolManagerNode node(nh, pnh);
  ros::waitForShutdown();
  return 0;
}
