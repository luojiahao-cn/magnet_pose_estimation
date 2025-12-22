#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace mag_sensor_calibration
{

/**
 * @brief 单个传感器的测量数据点
 */
struct MeasurementPoint
{
    double angle;           // 角度（弧度）
    Eigen::Vector3d mag;    // 磁场测量值 (mT)
    Eigen::Vector3d position; // 传感器位置（可选，用于验证）
};

/**
 * @brief 传感器校正参数
 */
struct CalibrationParams
{
    // 硬磁偏移（静态偏移量）
    Eigen::Vector3d hard_iron_offset;
    
    // 软磁干扰矩阵（3x3矩阵，包含比例因子和正交误差）
    Eigen::Matrix3d soft_iron_matrix;
    
    // 校正后的磁场强度范围（用于验证）
    double min_field_strength;
    double max_field_strength;
    
    // 校正质量指标
    double fit_error;       // 拟合误差
    double coverage;        // 角度覆盖度（0-1）
    
    // 是否有效
    bool valid;
    
    CalibrationParams()
        : hard_iron_offset(Eigen::Vector3d::Zero())
        , soft_iron_matrix(Eigen::Matrix3d::Identity())
        , min_field_strength(0.0)
        , max_field_strength(0.0)
        , fit_error(0.0)
        , coverage(0.0)
        , valid(false)
    {}
};

/**
 * @brief 磁场传感器校正算法
 * 
 * 使用椭球拟合方法校正传感器误差：
 * 1. 硬磁偏移（Hard Iron Offset）
 * 2. 软磁干扰（Soft Iron Distortion）- 包含比例因子和正交误差
 * 
 * 理论：理想情况下，传感器在不同角度下测量地磁场时，应该形成一个球面。
 * 但由于硬磁偏移和软磁干扰，实际测量值形成一个椭球。通过椭球拟合可以
 * 计算出校正参数。
 */
class CalibrationAlgorithm
{
public:
    CalibrationAlgorithm();
    ~CalibrationAlgorithm();

    /**
     * @brief 对单个传感器进行校正
     * @param measurements 测量数据点集合
     * @param params 输出的校正参数
     * @return 是否成功
     */
    bool calibrateSensor(const std::vector<MeasurementPoint>& measurements,
                        CalibrationParams& params);

    /**
     * @brief 应用校正参数到测量值
     * @param raw_measurement 原始测量值
     * @param params 校正参数
     * @return 校正后的测量值
     */
    static Eigen::Vector3d applyCalibration(const Eigen::Vector3d& raw_measurement,
                                           const CalibrationParams& params);

    /**
     * @brief 保存校正参数到YAML文件
     * @param params 校正参数
     * @param sensor_id 传感器ID
     * @param filepath 文件路径
     * @return 是否成功
     */
    static bool saveParamsToYAML(const CalibrationParams& params,
                                uint32_t sensor_id,
                                const std::string& filepath);

    /**
     * @brief 从YAML文件加载校正参数
     * @param sensor_id 传感器ID
     * @param filepath 文件路径
     * @param params 输出的校正参数
     * @return 是否成功
     */
    static bool loadParamsFromYAML(uint32_t sensor_id,
                                  const std::string& filepath,
                                  CalibrationParams& params);

private:
    /**
     * @brief 使用最小二乘法进行椭球拟合
     * @param measurements 测量数据点
     * @param center 输出的椭球中心（硬磁偏移）
     * @param transform 输出的变换矩阵（软磁矩阵的逆）
     * @return 是否成功
     */
    bool fitEllipsoid(const std::vector<MeasurementPoint>& measurements,
                     Eigen::Vector3d& center,
                     Eigen::Matrix3d& transform);

    /**
     * @brief 计算拟合误差
     * @param measurements 测量数据点
     * @param params 校正参数
     * @return 平均拟合误差
     */
    double computeFitError(const std::vector<MeasurementPoint>& measurements,
                         const CalibrationParams& params);

    /**
     * @brief 计算角度覆盖度
     * @param measurements 测量数据点
     * @return 覆盖度（0-1）
     */
    double computeCoverage(const std::vector<MeasurementPoint>& measurements);
};

} // namespace mag_sensor_calibration

