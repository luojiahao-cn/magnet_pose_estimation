#include <mag_sensor_calibration/calibration_algorithm.hpp>

#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace mag_sensor_calibration
{

CalibrationAlgorithm::CalibrationAlgorithm()
{
}

CalibrationAlgorithm::~CalibrationAlgorithm()
{
}

bool CalibrationAlgorithm::calibrateSensor(const std::vector<MeasurementPoint>& measurements,
                                           CalibrationParams& params)
{
    if (measurements.size() < 9)
    {
        // 至少需要9个点才能拟合椭球（6个参数 + 3个冗余）
        params.valid = false;
        return false;
    }

    Eigen::Vector3d center;
    Eigen::Matrix3d transform;
    
    if (!fitEllipsoid(measurements, center, transform))
    {
        params.valid = false;
        return false;
    }

    // 硬磁偏移就是椭球中心
    params.hard_iron_offset = center;

    // 软磁矩阵是变换矩阵的逆
    params.soft_iron_matrix = transform.inverse();

    // 计算校正后的磁场强度范围
    double min_field = std::numeric_limits<double>::max();
    double max_field = std::numeric_limits<double>::min();
    
    for (const auto& m : measurements)
    {
        Eigen::Vector3d corrected = applyCalibration(m.mag, params);
        double field_strength = corrected.norm();
        min_field = std::min(min_field, field_strength);
        max_field = std::max(max_field, field_strength);
    }
    
    params.min_field_strength = min_field;
    params.max_field_strength = max_field;

    // 计算拟合误差和覆盖度
    params.fit_error = computeFitError(measurements, params);
    params.coverage = computeCoverage(measurements);

    params.valid = true;
    return true;
}

Eigen::Vector3d CalibrationAlgorithm::applyCalibration(const Eigen::Vector3d& raw_measurement,
                                                       const CalibrationParams& params)
{
    if (!params.valid)
    {
        return raw_measurement;
    }
    
    // 校正公式: corrected = soft_iron_matrix * (raw - hard_iron_offset)
    return params.soft_iron_matrix * (raw_measurement - params.hard_iron_offset);
}

bool CalibrationAlgorithm::fitEllipsoid(const std::vector<MeasurementPoint>& measurements,
                                       Eigen::Vector3d& center,
                                       Eigen::Matrix3d& transform)
{
    // 使用最小二乘法拟合椭球
    // 椭球方程: (x-c)^T * A * (x-c) = 1
    // 其中 c 是中心，A 是正定矩阵
    
    const size_t n = measurements.size();
    
    // 构建设计矩阵 D (n x 9)
    // 每行对应一个测量点，列对应: x^2, y^2, z^2, xy, xz, yz, x, y, z
    Eigen::MatrixXd D(n, 9);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);
    
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector3d& m = measurements[i].mag;
        D(i, 0) = m.x() * m.x();
        D(i, 1) = m.y() * m.y();
        D(i, 2) = m.z() * m.z();
        D(i, 3) = m.x() * m.y();
        D(i, 4) = m.x() * m.z();
        D(i, 5) = m.y() * m.z();
        D(i, 6) = m.x();
        D(i, 7) = m.y();
        D(i, 8) = m.z();
    }
    
    // 求解: D * p = ones
    // 使用SVD求解最小二乘问题
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd p = svd.solve(ones);
    
    // 检查解的有效性
    if (svd.info() != Eigen::Success)
    {
        return false;
    }
    
    // 从参数向量 p 中提取椭球参数
    // p = [a, b, c, d, e, f, g, h, i]
    // 对应: ax^2 + by^2 + cz^2 + dxy + exz + fyz + gx + hy + iz = 1
    
    // 构建二次型矩阵 Q
    Eigen::Matrix3d Q;
    Q << p(0), p(3)/2.0, p(4)/2.0,
         p(3)/2.0, p(1), p(5)/2.0,
         p(4)/2.0, p(5)/2.0, p(2);
    
    // 构建线性项向量
    Eigen::Vector3d linear;
    linear << p(6), p(7), p(8);
    
    // 计算椭球中心: c = -Q^(-1) * linear / 2
    Eigen::Matrix3d Q_inv = Q.inverse();
    center = -0.5 * Q_inv * linear;
    
    // 计算变换矩阵，使得椭球变为单位球
    // 需要特征值分解: Q = V * D * V^T
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(Q);
    if (eigensolver.info() != Eigen::Success)
    {
        return false;
    }
    
    Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
    
    // 检查特征值是否为正（椭球要求）
    if (eigenvalues.minCoeff() <= 0)
    {
        return false;
    }
    
    // 计算变换矩阵: transform = V * diag(1/sqrt(eigenvalues))
    Eigen::Matrix3d scale = Eigen::Vector3d(
        1.0 / std::sqrt(eigenvalues(0)),
        1.0 / std::sqrt(eigenvalues(1)),
        1.0 / std::sqrt(eigenvalues(2))
    ).asDiagonal();
    
    transform = eigenvectors * scale;
    
    return true;
}

double CalibrationAlgorithm::computeFitError(const std::vector<MeasurementPoint>& measurements,
                                            const CalibrationParams& params)
{
    double total_error = 0.0;
    size_t count = 0;
    
    for (const auto& m : measurements)
    {
        Eigen::Vector3d corrected = applyCalibration(m.mag, params);
        double field_strength = corrected.norm();
        
        // 理想情况下，校正后的磁场强度应该接近常数（地磁场强度）
        // 计算与平均值的偏差
        total_error += std::abs(field_strength - params.max_field_strength);
        ++count;
    }
    
    return count > 0 ? total_error / count : 0.0;
}

double CalibrationAlgorithm::computeCoverage(const std::vector<MeasurementPoint>& measurements)
{
    if (measurements.empty())
    {
        return 0.0;
    }
    
    // 计算角度范围
    double min_angle = measurements[0].angle;
    double max_angle = measurements[0].angle;
    
    for (const auto& m : measurements)
    {
        min_angle = std::min(min_angle, m.angle);
        max_angle = std::max(max_angle, m.angle);
    }
    
    double range = max_angle - min_angle;
    
    // 理想情况下，360度覆盖度为1.0
    // 这里计算实际覆盖度（相对于2π）
    return std::min(1.0, range / (2.0 * M_PI));
}

bool CalibrationAlgorithm::saveParamsToYAML(const CalibrationParams& params,
                                            uint32_t sensor_id,
                                            const std::string& filepath)
{
    try
    {
        YAML::Node node;
        
        node["sensor_id"] = sensor_id;
        node["valid"] = params.valid;
        
        // 硬磁偏移
        node["hard_iron_offset"]["x"] = params.hard_iron_offset.x();
        node["hard_iron_offset"]["y"] = params.hard_iron_offset.y();
        node["hard_iron_offset"]["z"] = params.hard_iron_offset.z();
        
        // 软磁矩阵
        node["soft_iron_matrix"]["row0"] = std::vector<double>{
            params.soft_iron_matrix(0, 0), params.soft_iron_matrix(0, 1), params.soft_iron_matrix(0, 2)
        };
        node["soft_iron_matrix"]["row1"] = std::vector<double>{
            params.soft_iron_matrix(1, 0), params.soft_iron_matrix(1, 1), params.soft_iron_matrix(1, 2)
        };
        node["soft_iron_matrix"]["row2"] = std::vector<double>{
            params.soft_iron_matrix(2, 0), params.soft_iron_matrix(2, 1), params.soft_iron_matrix(2, 2)
        };
        
        // 质量指标
        node["min_field_strength"] = params.min_field_strength;
        node["max_field_strength"] = params.max_field_strength;
        node["fit_error"] = params.fit_error;
        node["coverage"] = params.coverage;
        
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }
        
        file << node;
        return true;
    }
    catch (const std::exception& e)
    {
        return false;
    }
}

bool CalibrationAlgorithm::loadParamsFromYAML(uint32_t sensor_id,
                                              const std::string& filepath,
                                              CalibrationParams& params)
{
    try
    {
        YAML::Node node = YAML::LoadFile(filepath);
        
        if (node["sensor_id"].as<uint32_t>() != sensor_id)
        {
            return false;
        }
        
        params.valid = node["valid"].as<bool>();
        
        // 加载硬磁偏移
        params.hard_iron_offset.x() = node["hard_iron_offset"]["x"].as<double>();
        params.hard_iron_offset.y() = node["hard_iron_offset"]["y"].as<double>();
        params.hard_iron_offset.z() = node["hard_iron_offset"]["z"].as<double>();
        
        // 加载软磁矩阵
        auto row0 = node["soft_iron_matrix"]["row0"].as<std::vector<double>>();
        auto row1 = node["soft_iron_matrix"]["row1"].as<std::vector<double>>();
        auto row2 = node["soft_iron_matrix"]["row2"].as<std::vector<double>>();
        
        params.soft_iron_matrix <<
            row0[0], row0[1], row0[2],
            row1[0], row1[1], row1[2],
            row2[0], row2[1], row2[2];
        
        // 加载质量指标
        params.min_field_strength = node["min_field_strength"].as<double>();
        params.max_field_strength = node["max_field_strength"].as<double>();
        params.fit_error = node["fit_error"].as<double>();
        params.coverage = node["coverage"].as<double>();
        
        return true;
    }
    catch (const std::exception& e)
    {
        return false;
    }
}

} // namespace mag_sensor_calibration

