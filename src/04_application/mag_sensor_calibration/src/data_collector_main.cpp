#include <mag_sensor_calibration/data_collector_node.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_collector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try
    {
        mag_sensor_calibration::DataCollectorNode node(nh, pnh);
        node.start();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL("[data_collector] 节点异常: %s", e.what());
        return 1;
    }
    
    return 0;
}

