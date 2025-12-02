#pragma once

#include <ros/ros.h>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace mag_core_utils::logger {

/**
 * @brief 格式化浮点数，统一小数位数
 * @param value 浮点数值
 * @param precision 小数位数（默认1位）
 * @return 格式化后的字符串
 */
inline std::string formatFloat(double value, int precision = 1) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

/**
 * @brief 格式化配置信息为多行字符串
 * @param config_items 配置项列表，每项为 {key, value} 对
 * @param indent 缩进字符串（默认 "  "）
 * @return 格式化后的多行字符串
 */
inline std::string formatConfig(const std::vector<std::pair<std::string, std::string>> &config_items,
                                const std::string &indent = "  ") {
  if (config_items.empty()) {
    return "";
  }

  std::ostringstream oss;
  oss << "配置加载完成:";
  for (const auto &item : config_items) {
    oss << "\n" << indent << item.first << ": " << item.second;
  }
  return oss.str();
}

/**
 * @brief 格式化初始化信息为多行字符串
 * @param init_items 初始化项列表，每项为 {key, value} 对
 * @param indent 缩进字符串（默认 "  "）
 * @return 格式化后的多行字符串
 */
inline std::string formatInit(const std::vector<std::pair<std::string, std::string>> &init_items,
                              const std::string &indent = "  ") {
  if (init_items.empty()) {
    return "";
  }

  std::ostringstream oss;
  oss << "初始化完成:";
  for (const auto &item : init_items) {
    oss << "\n" << indent << item.first << ": " << item.second;
  }
  return oss.str();
}

/**
 * @brief 格式化状态信息为多行字符串
 * @param status_items 状态项列表，每项为 {key, value} 对
 * @param indent 缩进字符串（默认 "  "）
 * @return 格式化后的多行字符串
 */
inline std::string formatStatus(const std::vector<std::pair<std::string, std::string>> &status_items,
                                const std::string &indent = "  ") {
  if (status_items.empty()) {
    return "";
  }

  std::ostringstream oss;
  oss << "状态:";
  for (const auto &item : status_items) {
    oss << "\n" << indent << item.first << ": " << item.second;
  }
  return oss.str();
}

/**
 * @brief 将布尔值转换为中文字符串
 */
inline std::string boolToString(bool value) {
  return value ? "启用" : "禁用";
}

/**
 * @brief 格式化频率值（Hz）
 */
inline std::string formatFrequency(double hz) {
  return formatFloat(hz, 1) + " Hz";
}

/**
 * @brief 格式化时间值（秒）
 */
inline std::string formatTime(double seconds, int precision = 3) {
  return formatFloat(seconds, precision) + " 秒";
}

}  // namespace mag_core_utils::logger

