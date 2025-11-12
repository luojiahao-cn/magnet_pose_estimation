/*
文件: tool_manager_node.cpp
功能概述:
  - 通过 MoveIt PlanningScene 在机械臂末端挂载/卸载工具（网格模型），并发布工具坐标系 TF。
  - 支持服务调用附着/分离；可选开机自动附着；支持设置工具与末端的位姿与尺度。

主要职责:
  - 参数读取: parent_link, object_name, mesh_path, xyz, rpy, scale, touch_links, auto_attach, tip_xyz, tip_rpy
  - 附着流程: 加载网格 → 构造 CollisionObject → 应用到规划场景 → 附着到 parent_link
  - 分离流程: 从末端分离并移除规划场景对象
  - TF 发布: 周期发布 tool_frame 与 tool_tcp（工具末端TCP）坐标系

ROS/MoveIt 接口:
  - 服务: ~attach (std_srvs/Trigger), ~detach (std_srvs/Trigger)
  - 规划组: fr5v6_arm（使用 MoveGroupInterface 与 PlanningSceneInterface）
  - TF: tool_frame、tool_tcp（20 Hz）

参数(私有命名空间 ~):
  - parent_link[string]: 工具附着父连杆；为 "auto"/空时自动取末端执行器链接
  - object_name[string]: 规划场景对象 ID
  - mesh_path[string]: 网格资源 URI，例如 package://pkg/path/to.stl
  - xyz[double[3]], rpy[double[3]]: 工具相对 parent_link 的平移与姿态（弧度）
  - scale[double[3]]: 网格缩放比例
  - touch_links[string[]]: 允许接触的链接集合；为空则默认使用 parent_link
  - auto_attach[bool]: 节点启动后是否自动附着一次
  - tip_xyz[double[3]], tip_rpy[double[3]]: 工具末端(tool_tcp)相对 tool_frame 的位姿

运行流程:
  - 启动 → 可选自动附着 → 周期发布 TF → 根据需要调用 ~attach/~detach 切换工具状态

注意事项:
  - mesh 资源需可读且单位为米；RPY 使用弧度
  - 修改参数后可再次调用 ~attach 以按新参数重建并附着
  - TF 仅描述工具与父连杆的相对位姿；父子关系由 parent_link 决定
*/

#include "mag_arm_scan/tool_manager_node.hpp"
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>

ToolManagerNode::ToolManagerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : nh_(nh), pnh_(pnh), move_group_("fr5v6_arm"), planning_scene_interface_() {
    // local small helpers for required params
    auto require = [&](const std::string &key, auto &var) {
      if (!pnh_.getParam(key, var)) throw std::runtime_error(std::string("缺少参数: ~") + key);
    };
    auto requireVec3 = [&](const std::string &key, std::vector<double> &out) {
      if (!pnh_.getParam(key, out) || out.size() != 3) throw std::runtime_error(std::string("缺少或非法参数: ~") + key + "[3]");
    };
    auto optVec3 = [&](const std::string &key, std::vector<double> &out) {
      if (pnh_.getParam(key, out)) {
        if (out.size() != 3) throw std::runtime_error(std::string("非法参数: ~") + key + "[3]");
      }
    };

    // Params (strict private namespace)
    require("parent_link", parent_link_);
    require("object_name", object_name_);
    require("mesh_path", mesh_path_);
    requireVec3("xyz", xyz_);
    requireVec3("rpy", rpy_);
    requireVec3("scale", scale_);
  pnh_.getParam("touch_links", touch_links_); // 可空
  require("auto_attach", auto_attach_);

  // 支架末端参数（可选）
  optVec3("tip_xyz", tip_xyz_);
  optVec3("tip_rpy", tip_rpy_);
    require("tcp_frame_name", tcp_frame_name_);

    // Resolve parent_link automatically from MoveGroup if requested
    if (parent_link_ == "auto" || parent_link_.empty()) {
      std::string eef = move_group_.getEndEffectorLink();
      if (!eef.empty()) parent_link_ = eef; else parent_link_ = "ee_link";
    }

  srv_attach_ = pnh_.advertiseService("attach", &ToolManagerNode::attachCb, this);
  srv_detach_ = pnh_.advertiseService("detach", &ToolManagerNode::detachCb, this);


  ROS_INFO("[tool_manager] ready. object=%s parent_link=%s mesh=%s", object_name_.c_str(), parent_link_.c_str(), mesh_path_.c_str());
  ROS_INFO("[tool_manager] params: xyz=[%.4f, %.4f, %.4f], rpy=[%.4f, %.4f, %.4f], scale=[%.4f, %.4f, %.4f]",
    xyz_[0], xyz_[1], xyz_[2], rpy_[0], rpy_[1], rpy_[2], scale_[0], scale_[1], scale_[2]);

    if (auto_attach_) {
      // 先行发布一次 TF（基于 xyz/rpy/tip_* 参数），即使后续 mesh/attach 失败也能保证 TF 链路
      publishToolTF(ros::TimerEvent());
      auto_attach_timer_ = nh_.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&){
        std_srvs::Trigger::Request req; std_srvs::Trigger::Response res;
        this->attachCb(req, res);
  ROS_INFO_STREAM("[tool_manager] auto-attach: " << (res.success ? "success" : (std::string("failed: ") + res.message)));
      }, true); // oneshot
    }

  // 按需发布 TF：默认不周期发布，避免 TF_REPEATED_DATA；
  // 在自动附着完成或手动调用 attach 后发布一次静态 TF
  }

bool ToolManagerNode::attachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  return attachWithParams(res);
}

// Helper to (re)attach object using current parameters
bool ToolManagerNode::attachWithParams(std_srvs::Trigger::Response &res) {
    // Attempt to remove any previous instance
    try {
      // Detach only if currently attached
      std::map<std::string, moveit_msgs::AttachedCollisionObject> attached =
          planning_scene_interface_.getAttachedObjects(std::vector<std::string>{object_name_});
      if (!attached.empty()) {
        move_group_.detachObject(object_name_);
        ros::Duration(0.1).sleep();
      }
      // Remove only if the world object exists
      std::vector<std::string> known = planning_scene_interface_.getKnownObjectNames();
      if (std::find(known.begin(), known.end(), object_name_) != known.end()) {
        planning_scene_interface_.removeCollisionObjects({object_name_});
      }
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
        // 即便网格加载失败，也发布 TF，保证链路连贯
        publishToolTF(ros::TimerEvent());
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
      // 构建碰撞体失败时也发布 TF，保证链路连贯
      publishToolTF(ros::TimerEvent());
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
      // 附着失败也发布一次 TF，尽量保持坐标链路可用
      publishToolTF(ros::TimerEvent());
      return true;
    }

    // 发布一次 TF（静态关系），避免高频重复
    publishToolTF(ros::TimerEvent());
    res.success = true;
    res.message = "attached";
    return true;
  }

bool ToolManagerNode::detachCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    try {
      bool did_any = false;
      // Detach if attached
      std::map<std::string, moveit_msgs::AttachedCollisionObject> attached =
          planning_scene_interface_.getAttachedObjects(std::vector<std::string>{object_name_});
      if (!attached.empty()) {
        move_group_.detachObject(object_name_);
        ros::Duration(0.2).sleep();
        did_any = true;
      }
      // Remove if exists in world
      std::vector<std::string> known = planning_scene_interface_.getKnownObjectNames();
      if (std::find(known.begin(), known.end(), object_name_) != known.end()) {
        planning_scene_interface_.removeCollisionObjects({object_name_});
        did_any = true;
      }
      res.success = true;
      res.message = did_any ? "detached" : "object not present";
      return true;
    } catch (const std::exception &e) {
      res.success = false;
      res.message = std::string("Failed to detach/remove: ") + e.what();
      return true;
    }
  }


void ToolManagerNode::publishToolTF(const ros::TimerEvent&) {
    if (parent_link_.empty() || xyz_.size() != 3 || rpy_.size() != 3) return;
  ros::Time now = ros::Time::now();
  // tool_frame: 支架基座（静态 TF）
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = now;
  tf.header.frame_id = parent_link_;
  tf.child_frame_id = "tool_frame";
  tf.transform.translation.x = xyz_[0];
  tf.transform.translation.y = xyz_[1];
  tf.transform.translation.z = xyz_[2];
  tf2::Quaternion q; q.setRPY(rpy_[0], rpy_[1], rpy_[2]);
  tf.transform.rotation = tf2::toMsg(q);

    // tool TCP frame: 支架末端
    std::vector<geometry_msgs::TransformStamped> tfs;
    tfs.push_back(tf);
    if (tip_xyz_.size() == 3 && tip_rpy_.size() == 3) {
      geometry_msgs::TransformStamped tf_tip;
      tf_tip.header.stamp = now;
      tf_tip.header.frame_id = "tool_frame";
      tf_tip.child_frame_id = tcp_frame_name_;
      tf_tip.transform.translation.x = tip_xyz_[0];
      tf_tip.transform.translation.y = tip_xyz_[1];
      tf_tip.transform.translation.z = tip_xyz_[2];
      tf2::Quaternion q_tip; q_tip.setRPY(tip_rpy_[0], tip_rpy_[1], tip_rpy_[2]);
      tf_tip.transform.rotation = tf2::toMsg(q_tip);
      tfs.push_back(tf_tip);
    }
    static_broadcaster_.sendTransform(tfs);
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
