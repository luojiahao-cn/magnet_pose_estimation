#!/usr/bin/env python3
"""Simple demo that drives one arm between two poses or named targets."""

import math
from dataclasses import dataclass
from typing import List, Optional

import rospy
from geometry_msgs.msg import Pose
from mag_device_arm.srv import (
    ExecuteNamedTarget,
    ExecuteNamedTargetRequest,
    SetEndEffectorPose,
    SetEndEffectorPoseRequest,
)


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (rad) to quaternion tuple (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def _pose_from_param(values: dict, default_xyz: List[float]) -> Pose:
    pose = Pose()
    xyz = values.get("xyz", default_xyz)
    if len(xyz) != 3:
        raise ValueError("xyz 必须是长度为 3 的数组")
    pose.position.x, pose.position.y, pose.position.z = xyz

    rpy = values.get("rpy", [0.0, 0.0, 0.0])
    if len(rpy) != 3:
        raise ValueError("rpy 必须是长度为 3 的数组")
    qx, qy, qz, qw = _quaternion_from_rpy(rpy[0], rpy[1], rpy[2])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


@dataclass
class TargetSpec:
    pose: Optional[Pose] = None
    named_target: Optional[str] = None


class ArmPingPong:
    def __init__(self):
        self.arm_name = rospy.get_param("~arm", "arm1")
        self.velocity = rospy.get_param("~velocity_scaling", 0.1)
        self.acceleration = rospy.get_param("~acceleration_scaling", 0.1)
        self.wait_time = rospy.get_param("~wait_time", 2.0)

        pose_a_cfg = rospy.get_param(
            "~pose_a", {"xyz": [0.35, -0.2, 0.2], "rpy": [0.0, math.radians(20), 0.0]}
        )
        pose_b_cfg = rospy.get_param(
            "~pose_b", {"xyz": [0.35, 0.2, 0.2], "rpy": [0.0, math.radians(-20), 0.0]}
        )
        self.targets = [
            self._parse_target(pose_a_cfg, [0.35, -0.2, 0.2]),
            self._parse_target(pose_b_cfg, [0.35, 0.2, 0.2]),
        ]

        self.pose_client = rospy.ServiceProxy(
            "/mag_device_arm/set_end_effector_pose", SetEndEffectorPose
        )
        self.named_client = rospy.ServiceProxy(
            "/mag_device_arm/execute_named_target", ExecuteNamedTarget
        )

        rospy.loginfo("[arm_ping_pong] 等待 mag_device_arm 服务...")
        self.pose_client.wait_for_service()
        self.named_client.wait_for_service()
        rospy.loginfo("[arm_ping_pong] 服务已就绪，开始往复运动")

    @staticmethod
    def _parse_target(values: dict, default_xyz: List[float]) -> TargetSpec:
        if isinstance(values, dict) and "named_target" in values:
            return TargetSpec(named_target=str(values["named_target"]))
        return TargetSpec(pose=_pose_from_param(values, default_xyz))

    def _send_pose(self, pose: Pose) -> bool:
        req = SetEndEffectorPoseRequest()
        req.arm = self.arm_name
        req.target = pose
        req.velocity_scaling = self.velocity
        req.acceleration_scaling = self.acceleration
        req.execute = True
        try:
            resp = self.pose_client(req)
        except rospy.ServiceException as exc:
            rospy.logerr("[arm_ping_pong] 调用 SetEndEffectorPose 失败: %s", exc)
            return False

        if not resp.success:
            rospy.logwarn("[arm_ping_pong] MoveIt 执行失败: %s", resp.message)
            return False
        rospy.loginfo("[arm_ping_pong] %s", resp.message)
        return True

    def _send_named_target(self, target: str) -> bool:
        req = ExecuteNamedTargetRequest()
        req.arm = self.arm_name
        req.target = target
        req.velocity_scaling = self.velocity
        req.acceleration_scaling = self.acceleration
        req.execute = True
        try:
            resp = self.named_client(req)
        except rospy.ServiceException as exc:
            rospy.logerr("[arm_ping_pong] 调用 ExecuteNamedTarget 失败: %s", exc)
            return False

        if not resp.success:
            rospy.logwarn("[arm_ping_pong] MoveIt 执行失败: %s", resp.message)
            return False
        rospy.loginfo("[arm_ping_pong] %s", resp.message)
        return True

    def send_target(self, target: TargetSpec) -> bool:
        if target.named_target:
            return self._send_named_target(target.named_target)
        if target.pose:
            return self._send_pose(target.pose)
        rospy.logerr("[arm_ping_pong] 无效的目标配置")
        return False

    def spin(self):
        index = 0
        while not rospy.is_shutdown():
            self.send_target(self.targets[index])
            rospy.sleep(self.wait_time)
            index = 1 - index


def main():
    rospy.init_node("arm_ping_pong")
    node = ArmPingPong()
    node.spin()


if __name__ == "__main__":
    main()
