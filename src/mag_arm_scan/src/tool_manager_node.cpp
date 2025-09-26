#include "mag_arm_scan/tool_manager_node.hpp"
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ToolManagerNode::ToolManagerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh), move_group_("fr5v6_arm"), planning_scene_interface_() {
    // Params
  pnh_.param<std::string>("parent_link", parent_link_, std::string());
  pnh_.param<std::string>("object_name", object_name_, std::string());
  pnh_.param<std::string>("mesh_path", mesh_path_, std::string());
     pnh_.param("xyz", xyz_, std::vector<double>());
     pnh_.param("rpy", rpy_, std::vector<double>());
     pnh_.param("scale", scale_, std::vector<double>());
     pnh_.param("touch_links", touch_links_, std::vector<std::string>());
     pnh_.param("auto_attach", auto_attach_, false);

     // 支架末端参数
     pnh_.param("tip_xyz", tip_xyz_, std::vector<double>());
     pnh_.param("tip_rpy", tip_rpy_, std::vector<double>());

    // Resolve parent_link automatically from MoveGroup if requested
    if (parent_link_ == "auto" || parent_link_.empty()) {
      std::string eef = move_group_.getEndEffectorLink();
      if (!eef.empty()) parent_link_ = eef; else parent_link_ = "ee_link";
    }

  srv_attach_ = pnh_.advertiseService("attach", &ToolManagerNode::attachCb, this);
  srv_detach_ = pnh_.advertiseService("detach", &ToolManagerNode::detachCb, this);


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

    // TF broadcaster timer: publish tool_frame at 20Hz
    tf_timer_ = nh_.createTimer(ros::Duration(0.05), &ToolManagerNode::publishToolTF, this);
  }

bool ToolManagerNode::attachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return attachWithParams(res);
}

// Helper to (re)attach object using current parameters
bool ToolManagerNode::attachWithParams(std_srvs::Trigger::Response &res) {
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

bool ToolManagerNode::detachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
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


void ToolManagerNode::publishToolTF(const ros::TimerEvent&) {
    if (parent_link_.empty() || xyz_.size() != 3 || rpy_.size() != 3) return;
    ros::Time now = ros::Time::now();
    // tool_frame: 支架基座
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = parent_link_;
    tf.child_frame_id = "tool_frame";
    tf.transform.translation.x = xyz_[0];
    tf.transform.translation.y = xyz_[1];
    tf.transform.translation.z = xyz_[2];
    tf2::Quaternion q; q.setRPY(rpy_[0], rpy_[1], rpy_[2]);
    tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(tf);

    // tool_tip_frame: 支架末端
    if (tip_xyz_.size() == 3 && tip_rpy_.size() == 3) {
      geometry_msgs::TransformStamped tf_tip;
      tf_tip.header.stamp = now;
      tf_tip.header.frame_id = "tool_frame";
      tf_tip.child_frame_id = "tool_tip_frame";
      tf_tip.transform.translation.x = tip_xyz_[0];
      tf_tip.transform.translation.y = tip_xyz_[1];
      tf_tip.transform.translation.z = tip_xyz_[2];
      tf2::Quaternion q_tip; q_tip.setRPY(tip_rpy_[0], tip_rpy_[1], tip_rpy_[2]);
      tf_tip.transform.rotation = tf2::toMsg(q_tip);
      tf_broadcaster_.sendTransform(tf_tip);
    }
  }

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
