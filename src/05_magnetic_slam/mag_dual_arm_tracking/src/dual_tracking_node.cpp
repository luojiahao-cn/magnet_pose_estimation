#include <ros/ros.h>
#include <mag_device_arm/SetEndEffectorPose.h>
#include <mag_core_msgs/MagSensorData.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <cmath>
#include <thread>

class DualArmTracking
{
public:
    DualArmTracking(ros::NodeHandle &nh) : nh_(nh)
    {
        nh.param<std::string>("magnet_arm_name", magnet_arm_name_, "arm1");
        nh.param<std::string>("sensor_arm_name", sensor_arm_name_, "arm2");
        nh.param<double>("radius", radius_, 0.1);
        nh.param<double>("center_x", center_x_, 0.5);
        nh.param<double>("center_y", center_y_, 0.0);
        nh.param<double>("center_z", center_z_, 0.3);
        nh.param<double>("z_offset", z_offset_, 0.3);
        nh.param<std::string>("bag_filename", bag_filename_, "dual_tracking.bag");

        magnet_client_ = nh.serviceClient<mag_device_arm::SetEndEffectorPose>("/mag_device_arm/set_end_effector_pose");
        sensor_client_ = nh.serviceClient<mag_device_arm::SetEndEffectorPose>("/mag_device_arm/set_end_effector_pose");

        bag_.open(bag_filename_, rosbag::bagmode::Write);
        sub_sensor_ = nh.subscribe("/magnetic/sensor/field", 100, &DualArmTracking::sensorCallback, this);

        ROS_INFO("Dual Arm Node Initialized");
    }

    ~DualArmTracking()
    {
        if (bag_.isOpen())
            bag_.close();
    }

    void sensorCallback(const mag_core_msgs::MagSensorData::ConstPtr &msg)
    {
        if (bag_.isOpen())
        {
            std::lock_guard<std::mutex> lock(bag_mutex_);
            bag_.write("/magnetic/sensor/field", ros::Time::now(), *msg);
        }
    }

    void run()
    {
        ros::Rate rate(1.0); // 1 Hz
        double t = 0;

        while (ros::ok())
        {
            if (!magnet_client_.exists())
            {
                ROS_WARN_THROTTLE(2, "Waiting for arm service...");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            double x = center_x_ + radius_ * cos(t);
            double y = center_y_ + radius_ * sin(t);
            double z = center_z_;

            geometry_msgs::Pose magnet_tgt;
            magnet_tgt.position.x = x;
            magnet_tgt.position.y = y;
            magnet_tgt.position.z = z;
            magnet_tgt.orientation.w = 1.0;

            geometry_msgs::Pose sensor_tgt;
            sensor_tgt.position.x = x;
            sensor_tgt.position.y = y;
            sensor_tgt.position.z = z + z_offset_;
            sensor_tgt.orientation.x = 1.0;
            sensor_tgt.orientation.y = 0.0;
            sensor_tgt.orientation.z = 0.0;
            sensor_tgt.orientation.w = 0.0;

            // Move sequentially
            if (moveArm(magnet_arm_name_, magnet_tgt) && moveArm(sensor_arm_name_, sensor_tgt))
            {
                ROS_INFO("Reached step t=%.2f", t);

                // Write ground truth poses to bag
                if (bag_.isOpen())
                {
                    std::lock_guard<std::mutex> lock(bag_mutex_);
                    ros::Time now = ros::Time::now();

                    geometry_msgs::PoseStamped magnet_ps;
                    magnet_ps.header.stamp = now;
                    magnet_ps.header.frame_id = "world";
                    magnet_ps.pose = magnet_tgt;
                    bag_.write("/ground_truth/magnet_pose", now, magnet_ps);

                    geometry_msgs::PoseStamped sensor_ps;
                    sensor_ps.header.stamp = now;
                    sensor_ps.header.frame_id = "world";
                    sensor_ps.pose = sensor_tgt;
                    bag_.write("/ground_truth/sensor_pose", now, sensor_ps);
                }
            }

            t += 0.2;
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    bool moveArm(const std::string &arm, const geometry_msgs::Pose &pose)
    {
        mag_device_arm::SetEndEffectorPose srv;
        srv.request.arm = arm;
        srv.request.target = pose;
        srv.request.execute = true;
        srv.request.velocity_scaling = 0.2;
        srv.request.acceleration_scaling = 0.2;

        // Use local client or shared? Shared is fine if sequential.
        if (magnet_client_.call(srv))
        {
            return srv.response.success;
        }
        return false;
    }

    ros::NodeHandle nh_;
    ros::ServiceClient magnet_client_;
    ros::ServiceClient sensor_client_;
    ros::Subscriber sub_sensor_;
    rosbag::Bag bag_;
    std::mutex bag_mutex_;

    std::string magnet_arm_name_;
    std::string sensor_arm_name_;

    double center_x_, center_y_, center_z_, radius_, z_offset_;
    std::string bag_filename_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_tracking_node");
    ros::NodeHandle nh("~");
    DualArmTracking tracker(nh);
    tracker.run();
    return 0;
}
