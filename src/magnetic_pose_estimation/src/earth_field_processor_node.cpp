#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <magnetic_pose_estimation/MagneticField.h>
#include <map>
#include <vector>
#include <Eigen/Dense>

class EarthFieldProcessor
{
public:
    EarthFieldProcessor(ros::NodeHandle& nh)
        : nh_(nh)
    {
        raw_sub_ = nh_.subscribe("/magnetic_field/raw_data", 50, &EarthFieldProcessor::rawCallback, this);
        processed_pub_ = nh_.advertise<magnetic_pose_estimation::MagneticField>("/magnetic_field/processed", 100);

        calibrate_srv_ = nh_.advertiseService("/magnetic_field/calibrate_earth_field", &EarthFieldProcessor::calibrateSrv, this);
        reset_srv_ = nh_.advertiseService("/magnetic_field/reset_to_initial", &EarthFieldProcessor::resetSrv, this);

        ROS_INFO("地磁场处理节点已启动");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber raw_sub_;
    ros::Publisher processed_pub_;
    ros::ServiceServer calibrate_srv_;
    ros::ServiceServer reset_srv_;

    bool earth_field_calibrated_ = false;
    std::map<int, Eigen::Vector3d> earth_field_;
    std::map<int, std::vector<Eigen::Vector3d>> calibration_data_;
    int calibration_samples_ = 10;
    bool is_calibrating_ = false;

    void rawCallback(const magnetic_pose_estimation::MagneticField::ConstPtr& msg)
    {
        Eigen::Vector3d raw(msg->mag_x, msg->mag_y, msg->mag_z);

        // 校准流程
        if (is_calibrating_) {
            calibration_data_[msg->sensor_id].push_back(raw);

            bool ready = true;
            for (const auto& pair : calibration_data_) {
                if (pair.second.size() < calibration_samples_) {
                    ready = false;
                    break;
                }
            }
            if (ready) {
                earth_field_.clear();
                for (const auto& pair : calibration_data_) {
                    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
                    for (const auto& v : pair.second) sum += v;
                    earth_field_[pair.first] = sum / pair.second.size();
                }
                earth_field_calibrated_ = true;
                is_calibrating_ = false;
                calibration_data_.clear();
                ROS_INFO("地磁场校准完成！");
            }
            return;
        }

        // 地磁场处理
        magnetic_pose_estimation::MagneticField out = *msg;
        if (earth_field_calibrated_ && earth_field_.count(msg->sensor_id)) {
            Eigen::Vector3d corrected = raw - earth_field_[msg->sensor_id];
            out.mag_x = corrected.x();
            out.mag_y = corrected.y();
            out.mag_z = corrected.z();
        }
        processed_pub_.publish(out);
    }

    bool calibrateSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        calibration_data_.clear();
        is_calibrating_ = true;
        ROS_INFO("开始地磁场校准，请保持磁铁远离传感器...");
        return true;
    }

    bool resetSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        earth_field_calibrated_ = false;
        earth_field_.clear();
        ROS_WARN("地磁场校准已重置，恢复原始数据输出");
        return true;
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "earth_field_processor_node");
    ros::NodeHandle nh;
    EarthFieldProcessor processor(nh);
    ros::spin();
    return 0;
}