#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <mag_core_msgs/MagSensorData.h>
#include <mag_device_arm/SetEndEffectorPose.h>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <sys/stat.h>

struct ScanConfig
{
    double x_min, x_max, x_step;
    double y_min, y_max, y_step;
    double z_fixed;
    double qx, qy, qz, qw;
    int samples_per_point;
    double wait_time;
    std::string output_file;
    std::string arm_name;
};

class PlanarScanner
{
public:
    PlanarScanner(ros::NodeHandle &nh, const ScanConfig &config) : nh_(nh), config_(config)
    {
        sensor_sub_ = nh.subscribe("/magnetic/sensor/field", 100, &PlanarScanner::sensorCallback, this);
        arm_client_ = nh.serviceClient<mag_device_arm::SetEndEffectorPose>("/mag_device_arm/set_end_effector_pose");

        std::string path = ros::package::getPath("mag_planar_scan") + "/data";
        // Create directory if not exists (rudimentary check, mkdir might fail if exists but that's fine)
        mkdir(path.c_str(), 0777);

        std::string full_path = path + "/" + config_.output_file;
        outfile_.open(full_path);
        if (!outfile_.is_open())
        {
            ROS_ERROR("Failed to open output file: %s", full_path.c_str());
        }
        else
        {
            outfile_ << "x,y,z,sensor_id,mag_x,mag_y,mag_z\n";
            ROS_INFO("Saving data to: %s", full_path.c_str());
        }
    }

    ~PlanarScanner()
    {
        if (outfile_.is_open())
            outfile_.close();
    }

    void sensorCallback(const mag_core_msgs::MagSensorData::ConstPtr &msg)
    {
        if (collecting_)
        {
            samples_[msg->sensor_id].push_back(*msg);
        }
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok() && (!arm_client_.exists() || sensor_sub_.getNumPublishers() == 0))
        {
            ROS_INFO_THROTTLE(2, "Waiting for services and topics...");
            ros::spinOnce();
            rate.sleep();
        }

        std::vector<geometry_msgs::Point> points;
        for (double x = config_.x_min; x <= config_.x_max + 1e-5; x += config_.x_step)
        {
            for (double y = config_.y_min; y <= config_.y_max + 1e-5; y += config_.y_step)
            {
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = config_.z_fixed;
                points.push_back(p);
            }
        }

        ROS_INFO("Starting scan with %lu points", points.size());

        for (size_t i = 0; i < points.size() && ros::ok(); ++i)
        {
            ROS_INFO("Moving to point %lu/%lu: (%.3f, %.3f, %.3f)", i + 1, points.size(), points[i].x, points[i].y, points[i].z);
            if (!moveTo(points[i]))
            {
                ROS_WARN("Skipping point due to movement failure");
                continue;
            }

            ros::Duration(config_.wait_time).sleep();

            collectSamples();
            processAndSave(points[i]);
            ros::spinOnce();
        }

        ROS_INFO("Scan complete!");
    }

private:
    bool moveTo(const geometry_msgs::Point &point)
    {
        mag_device_arm::SetEndEffectorPose srv;
        srv.request.arm = config_.arm_name;
        srv.request.target.position = point;
        srv.request.target.orientation.x = config_.qx;
        srv.request.target.orientation.y = config_.qy;
        srv.request.target.orientation.z = config_.qz;
        srv.request.target.orientation.w = config_.qw;

        srv.request.execute = true;
        srv.request.velocity_scaling = 0.2;
        srv.request.acceleration_scaling = 0.2;

        if (arm_client_.call(srv))
        {
            if (srv.response.success)
                return true;
            else
            {
                ROS_WARN("Service returned failure: %s", srv.response.message.c_str());
                return false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service");
            return false;
        }
    }

    void collectSamples()
    {
        collecting_ = true;
        samples_.clear();

        // Wait enough time to collect samples
        // Assuming sensor runs at >100Hz, 0.5s should give >50 samples
        // Or adapt to configured sample count
        double duration = (double)config_.samples_per_point / 50.0; // rough estimate if 50Hz, tweak as needed
        if (duration < 0.2)
            duration = 0.2;

        ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < duration)
        {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        collecting_ = false;
    }

    void processAndSave(const geometry_msgs::Point &p)
    {
        for (auto const &pair : samples_)
        {
            int id = pair.first;
            const auto &msgs = pair.second;
            double avg_x = 0, avg_y = 0, avg_z = 0;
            if (msgs.empty())
                continue;
            for (const auto &m : msgs)
            {
                avg_x += m.mag_x;
                avg_y += m.mag_y;
                avg_z += m.mag_z;
            }
            avg_x /= msgs.size();
            avg_y /= msgs.size();
            avg_z /= msgs.size();

            if (outfile_.is_open())
            {
                outfile_ << p.x << "," << p.y << "," << p.z << "," << id << "," << avg_x << "," << avg_y << "," << avg_z << "\n";
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sensor_sub_;
    ros::ServiceClient arm_client_;
    ScanConfig config_;
    std::ofstream outfile_;
    std::map<int, std::vector<mag_core_msgs::MagSensorData>> samples_;
    bool collecting_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planar_scan_node");
    ros::NodeHandle nh("~");

    ScanConfig config;
    nh.param<double>("x_min", config.x_min, 0.3);
    nh.param<double>("x_max", config.x_max, 0.5);
    nh.param<double>("x_step", config.x_step, 0.05);
    nh.param<double>("y_min", config.y_min, -0.1);
    nh.param<double>("y_max", config.y_max, 0.1);
    nh.param<double>("y_step", config.y_step, 0.05);
    nh.param<double>("z_fixed", config.z_fixed, 0.2);
    nh.param<double>("qx", config.qx, 1.0);
    nh.param<double>("qy", config.qy, 0.0);
    nh.param<double>("qz", config.qz, 0.0);
    nh.param<double>("qw", config.qw, 0.0);
    nh.param<int>("samples_per_point", config.samples_per_point, 50);
    nh.param<double>("wait_time", config.wait_time, 0.5);
    nh.param<std::string>("output_file", config.output_file, "scan_data.csv");
    nh.param<std::string>("arm_name", config.arm_name, "panda_1");

    PlanarScanner scanner(nh, config);
    scanner.run();

    return 0;
}
