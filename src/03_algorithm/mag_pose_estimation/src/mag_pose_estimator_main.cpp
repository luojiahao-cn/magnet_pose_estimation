/**
 * @file mag_pose_estimator_main.cpp
 * @brief 磁体姿态估计节点主程序
 * 
 * ROS 节点入口点，初始化节点并进入事件循环。
 */

#include "mag_pose_estimator/mag_pose_estimator_node.h"

#include <ros/ros.h>

#include <locale>

/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 退出码
 */
int main(int argc, char **argv) {
  // 设置本地化，支持中文输出
  setlocale(LC_ALL, "zh_CN.UTF-8");
  
  // 调用节点的静态 main 函数
  return mag_pose_estimator::MagPoseEstimatorNode::main(argc, argv);
}

